#!/usr/bin/env python3
"""
Host controller for the DIY laser 3D scanner.

Hardware pairing:
  - Creality Ender-3 S1 Plus (Marlin) on PRINTER_PORT (COM8 @ 115200)
  - ESP32 optical head (esp32_sensor.ino) on ESP32_PORT (COM7 @ 115200)
  - Dual photodiodes -> LM358 -> ADS1115 I2C 0x48 (SDA GPIO21, SCL GPIO22)
  - Sensor1 = ADS1115 A0, Sensor2 = ADS1115 A1
  - Laser: AO3400 N-MOSFET GPIO25 Active-HIGH (HIGH=ON, LOW=OFF)

Workflow (v6):
  open ports -> comm test -> G28 X Y -> optical Z home (G92 Z0) -> bed focus
  -> user safe height + laser guide -> rough stop-and-probe map -> fine
  Z-tracking raster -> outputs.

Usage:
  python auto_scanner_v6.py
  python auto_scanner_v6.py --comm-test
"""

from __future__ import annotations

import argparse
import csv
import json
import logging
import math
import os
import re
import sys
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Any, Dict, List, Optional, TextIO, Tuple

# pyserial is optional in DRY_RUN so the file still imports without the package.
try:
    import serial
    from serial.serialutil import SerialException
except ImportError:  # pragma: no cover
    serial = None  # type: ignore
    SerialException = OSError  # type: ignore


# =============================================================================
# CONFIGURATION — all adjustable / hardware-dependent values live here.
# =============================================================================

# Serial ports (Windows). Change if Device Manager shows different COM numbers.
PRINTER_PORT = "COM8"  # Ender-3 S1 Plus USB
ESP32_PORT = "COM7"  # ESP32 USB-UART
BAUD = 115200  # Must match Marlin CONFIG and SERIAL_BAUD in the .ino

# Seconds for each serial read. Keep short so Ctrl+C stays responsive.
SERIAL_TIMEOUT_S = 0.2
# Extra pause after opening Marlin USB (many boards reset on DTR).
PRINTER_BOOT_WAIT_S = 2.5
ESP32_BOOT_WAIT_S = 1.0

# Marlin: how long to wait for a single "ok" (busy/wait extend this).
MARLIN_OK_TIMEOUT_S = 60.0
MARLIN_MOVE_TIMEOUT_S = 180.0
ESP32_REPLY_TIMEOUT_S = 2.0
ESP32_READ_RETRIES = 3

# Printer travel envelope (Ender-3 S1 Plus typical bed; Z max is conservative).
# NEVER command motion outside these machine coordinates.
X_MIN, X_MAX = 0.0, 300.0  # mm
Y_MIN, Y_MAX = 0.0, 300.0  # mm
Z_MIN, Z_MAX = 0.0, 300.0  # mm

# Default bed / object XY. Override after measuring your sensor vs nozzle offset.
BED_CENTER_X = 150.0  # mm — mechanical bed center
BED_CENTER_Y = 150.0  # mm
# Optical head vs nozzle XY offset (add if the sensors are not on the nozzle).
SENSOR_OFFSET_X = 0.0  # mm, +X is right when facing the printer
SENSOR_OFFSET_Y = 0.0  # mm, +Y is toward the back

# Default scan window around the bed center (clipped to printer limits).
# The operator can shrink it at startup; the fine scan then narrows it further
# to the region the rough map found an object in.
SCAN_SIZE_X = 300.0  # mm
SCAN_SIZE_Y = 300.0  # mm

# Motion speeds. Conservative for an unattended optical head.
TRAVEL_FEED_MM_MIN = 1800.0  # rapid XY
Z_FEED_MM_MIN = 300.0  # Z only
SCAN_FEED_MM_MIN = 240.0  # XY during raster
CALIBRATE_FEED_MM_MIN = 120.0  # slow Z probing

# Homing. Set HOME_ON_START False if the machine is already homed and you
# do not want G28 (which can crash if the optical jig blocks the probe).
HOME_ON_START = True
HOME_GCODE = "G28 X Y"  # laterals only — Z is homed optically, not on a microswitch
# Optical Z home: focus is declared when the two net laser signals balance.
FOCUS_BALANCE_THRESHOLD = 150.0  # |signal1 - signal2| counts

# Safe Z used before calibration when firmware Z=0 is unknown / after home.
PRE_CAL_SAFE_Z = 40.0  # mm machine Z after homing, before descending onto the bed

# Stop-and-probe rough map and fine raster geometry.
ROUGH_STEP_MM = 5.0  # coarse stop-and-probe grid pitch
FINE_STEP_MM = 0.5  # fine raster line spacing (replaces LINE_STEP_Y_MM)
BED_EPS_MM = 0.05  # hard floor: Z never goes below z_bed_focus + BED_EPS_MM
BED_FOCUS_STEP_MM = 0.1  # fine Z step used only for the bed focus search
ROUGH_PROBE_STEP_MM = 0.2  # Z step while probing a rough-map cell

# Bed / object optical detection steps.
Z_CALIBRATE_STEP_MM = 0.2  # mm per sample while finding the bed
SAFE_CLEARANCE = 8.0  # mm added above estimated object height for Z_START
Z_LIFT_EMPTY_MM = 6.0  # mm extra lift when a cliff / empty bed is declared

# Sensor math thresholds (ADS1115 counts after las - amb). Tune on a real bed coupon.
# 16-bit full scale is ~32767 (GAIN_ONE +/-4.096 V); far above the old 12-bit 0–4095 range.
SIGNAL_THRESHOLD = 320.0  # minimum per-channel signal1/signal2
MIN_TOTAL_SIGNAL = 640.0  # reject dark / off-surface samples
DETECT_RATIO_THRESHOLD = 960.0  # total_signal required to declare first contact
# (Named "ratio" in the spec list; contact uses total_signal, not the L/R ratio.)

# Live surface following.
RATIO_DEADBAND = 0.08  # |filtered ratio| below this → no Z correction
Z_CORRECTION_STEP = 0.05  # mm per cycle when outside the deadband
MAX_Z_CORRECTION_PER_CYCLE = 0.25  # mm clamp
# Flip this if the head dives into the part instead of tracking it (+1 or -1).
Z_CORRECTION_SIGN = 1
RATIO_FILTER_ALPHA = 0.35  # EMA; higher = snappier, lower = smoother
SIGNAL_FILTER_ALPHA = 0.40

# Continuous raster: short XY segments so Z corrections stay in the planner.
SEGMENT_MM = 0.8  # 0.5–1.0 mm recommended
SAMPLES_PER_SEGMENT = 2  # ESP32 R polls while a short segment is queued

# Cliff / drop search.
DROP_SEARCH_MAX_MM = 20.0  # do not search deeper than this
DROP_SEARCH_STEP_MM = 0.4  # mm
MAX_DROP_SEARCH_STEPS = int(DROP_SEARCH_MAX_MM / DROP_SEARCH_STEP_MM) + 8

# Stop after this many consecutive raster lines with zero valid points.
EMPTY_LINES_TO_STOP = 4

# Max iterations for bounded loops (safety against infinite descent).
MAX_Z_CALIB_STEPS = 2500
MAX_SEGMENTS_PER_LINE = 4000
MAX_SCAN_LINES = 4000

# Simulation / safety switches.
DRY_RUN = False  # True: no serial, no laser, synthetic sensors + log files
# Diagnostics: log raw total_signal and |s1-s2| every 0.2 mm of descent instead
# of scanning. Rows are written to scanner.log in CSV form (z,total,|s1-s2|).
CALIBRATION_MODE = False
# Set False only after COM ports, laser polarity, and travel limits are verified.

# =============================================================================
# End of configuration
# =============================================================================

DATA_RE = re.compile(
    r"^DATA:\s*(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*,\s*"
    r"(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*$"
)
M114_RE = re.compile(
    r"X:(-?\d+(?:\.\d+)?)\s+Y:(-?\d+(?:\.\d+)?)\s+Z:(-?\d+(?:\.\d+)?)"
)


class ScannerError(RuntimeError):
    """Fatal scanner / communication error."""


@dataclass
class SensorSample:
    amb1: float
    amb2: float
    las1: float
    las2: float
    signal1: float
    signal2: float
    total_signal: float
    ratio: float
    valid: bool
    reason: str


@dataclass
class ScanPoint:
    x: float
    y: float
    z: float
    signal1: float
    signal2: float
    total_signal: float
    ratio: float
    relative_height: float
    timestamp: str


def utc_now_iso() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%fZ")


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def finite(*values: float) -> bool:
    return all(math.isfinite(v) for v in values)


def compute_signals(amb1: float, amb2: float, las1: float, las2: float) -> SensorSample:
    signal1 = las1 - amb1
    signal2 = las2 - amb2
    total = signal1 + signal2
    if not finite(amb1, amb2, las1, las2, signal1, signal2, total):
        return SensorSample(amb1, amb2, las1, las2, signal1, signal2, total, 0.0, False, "non_finite")
    if total <= 0.0:
        return SensorSample(amb1, amb2, las1, las2, signal1, signal2, total, 0.0, False, "div_zero_or_dark")
    ratio = (signal1 - signal2) / total
    if not math.isfinite(ratio):
        return SensorSample(amb1, amb2, las1, las2, signal1, signal2, total, ratio, False, "bad_ratio")
    strong = (
        signal1 >= SIGNAL_THRESHOLD
        and signal2 >= SIGNAL_THRESHOLD
        and total >= MIN_TOTAL_SIGNAL
    )
    if not strong:
        return SensorSample(amb1, amb2, las1, las2, signal1, signal2, total, ratio, False, "weak_signal")
    return SensorSample(amb1, amb2, las1, las2, signal1, signal2, total, ratio, True, "ok")


def parse_data_line(line: str) -> Optional[Tuple[float, float, float, float]]:
    m = DATA_RE.match(line.strip())
    if not m:
        return None
    try:
        return tuple(float(m.group(i)) for i in range(1, 5))  # type: ignore[return-value]
    except ValueError:
        return None


def ask_yes_no(prompt: str, default: bool = False) -> bool:
    suffix = " [Y/n]: " if default else " [y/N]: "
    while True:
        try:
            raw = input(prompt + suffix).strip().lower()
        except EOFError:
            return False
        if raw == "" and default:
            return True
        if raw == "" and not default:
            return False
        if raw in ("y", "yes"):
            return True
        if raw in ("n", "no"):
            return False
        print("Please enter y or n.")


def ask_float(prompt: str, lo: float, hi: float, default: Optional[float] = None) -> float:
    while True:
        try:
            raw = input(prompt).strip().replace(",", ".")
        except EOFError as exc:
            raise ScannerError("input closed") from exc
        # Empty input keeps the default when the caller provides one.
        if not raw and default is not None:
            return default
        try:
            value = float(raw)
        except ValueError:
            print("Enter a number.")
            continue
        if not math.isfinite(value):
            print("Value must be finite.")
            continue
        if value < lo or value > hi:
            print(f"Enter a value between {lo} and {hi}.")
            continue
        return value


class DummySerial:
    """Stand-in when DRY_RUN is enabled so the rest of the code stays identical."""

    def __init__(self, name: str) -> None:
        self.name = name
        self.is_open = True
        self._buf: List[bytes] = []

    def write(self, data: bytes) -> int:
        return len(data)

    def readline(self) -> bytes:
        if self._buf:
            return self._buf.pop(0)
        return b""

    def reset_input_buffer(self) -> None:
        self._buf.clear()

    def reset_output_buffer(self) -> None:
        return

    def close(self) -> None:
        self.is_open = False

    def flush(self) -> None:
        return


class MarlinPrinter:
    def __init__(self, ser: Any, logger: logging.Logger, dry_run: bool) -> None:
        self.ser = ser
        self.log = logger
        self.dry_run = dry_run
        self.x = BED_CENTER_X
        self.y = BED_CENTER_Y
        self.z = PRE_CAL_SAFE_Z
        self.absolute = True

    def close(self) -> None:
        try:
            if self.ser is not None and getattr(self.ser, "is_open", False):
                self.ser.close()
        except Exception as exc:
            self.log.warning("Printer close failed: %s", exc)

    def _read_line(self) -> str:
        if self.dry_run:
            return ""
        try:
            raw = self.ser.readline()
        except (SerialException, OSError, TypeError) as exc:
            raise ScannerError(f"printer serial read failed: {exc}") from exc
        if not raw:
            return ""
        try:
            return raw.decode("utf-8", errors="replace").strip()
        except Exception as exc:
            raise ScannerError(f"printer decode failed: {exc}") from exc

    def wait_ok(self, timeout_s: float, context: str) -> None:
        if self.dry_run:
            return
        t0 = time.time()
        while True:
            if time.time() - t0 > timeout_s:
                raise ScannerError(f"Marlin timeout waiting for ok ({context})")
            line = self._read_line()
            if not line:
                continue
            self.log.debug("MARLIN << %s", line)
            low = line.lower()
            if low.startswith("ok"):
                return
            if low.startswith("busy") or low == "wait" or low.startswith("echo:"):
                continue
            if "error" in low or low.startswith("!!") or low.startswith("kill"):
                raise ScannerError(f"Marlin error ({context}): {line}")
            # Temperature / heater lines and other chatter are ignored.

    def send(self, cmd: str, wait: bool = True, timeout_s: Optional[float] = None) -> None:
        cmd = cmd.strip()
        self.log.info("GCODE >> %s", cmd)
        if self.dry_run:
            return
        try:
            self.ser.reset_output_buffer()
            self.ser.write((cmd + "\n").encode("ascii", errors="strict"))
            self.ser.flush()
        except (SerialException, OSError, UnicodeEncodeError) as exc:
            raise ScannerError(f"printer write failed: {exc}") from exc
        if wait:
            self.wait_ok(timeout_s or MARLIN_OK_TIMEOUT_S, cmd)

    def sync_position(self) -> Tuple[float, float, float]:
        if self.dry_run:
            return self.x, self.y, self.z
        self.send("M400", timeout_s=MARLIN_MOVE_TIMEOUT_S)
        # Drain leftover chatter then query.
        deadline = time.time() + 2.0
        while time.time() < deadline:
            if not self._read_line():
                break
        self.send("M114", wait=False)
        t0 = time.time()
        pos = None
        saw_ok = False
        while time.time() - t0 < 10.0:
            line = self._read_line()
            if not line:
                continue
            self.log.debug("MARLIN << %s", line)
            m = M114_RE.search(line)
            if m:
                pos = (float(m.group(1)), float(m.group(2)), float(m.group(3)))
            if line.lower().startswith("ok"):
                saw_ok = True
                break
            if "error" in line.lower() or line.startswith("!!"):
                raise ScannerError(f"Marlin error on M114: {line}")
        if not saw_ok:
            # Some firmwares print ok before the position; try one more wait.
            try:
                self.wait_ok(3.0, "M114-ok")
            except ScannerError:
                pass
        if pos is None:
            raise ScannerError("could not parse M114 position")
        self.x, self.y, self.z = pos
        self.absolute = True
        return pos

    def ensure_absolute(self) -> None:
        self.send("G90")
        self.absolute = True

    def halt_motion(self) -> None:
        """Best-effort stop. Ignores failures so emergency cleanup can continue."""
        for cmd in ("M410", "M84"):
            try:
                if self.dry_run:
                    self.log.info("DRY halt %s", cmd)
                    continue
                self.ser.write((cmd + "\n").encode("ascii"))
                self.ser.flush()
                time.sleep(0.05)
            except Exception as exc:
                self.log.warning("halt %s failed: %s", cmd, exc)
        try:
            self.send("G90", wait=False)
        except Exception as exc:
            self.log.warning("G90 during halt failed: %s", exc)
        self.absolute = True

    def assert_in_bounds(self, x: float, y: float, z: float) -> None:
        if not finite(x, y, z):
            raise ScannerError(f"non-finite target {x},{y},{z}")
        if not (X_MIN <= x <= X_MAX and Y_MIN <= y <= Y_MAX and Z_MIN <= z <= Z_MAX):
            raise ScannerError(
                f"target out of limits: X{x:.3f} Y{y:.3f} Z{z:.3f} "
                f"(allowed X{X_MIN}-{X_MAX} Y{Y_MIN}-{Y_MAX} Z{Z_MIN}-{Z_MAX})"
            )

    def move_abs(
        self,
        x: Optional[float] = None,
        y: Optional[float] = None,
        z: Optional[float] = None,
        feed: float = TRAVEL_FEED_MM_MIN,
        wait: bool = True,
    ) -> None:
        nx = self.x if x is None else x
        ny = self.y if y is None else y
        nz = self.z if z is None else z
        self.assert_in_bounds(nx, ny, nz)
        self.ensure_absolute()
        parts = ["G1"]
        if x is not None:
            parts.append(f"X{nx:.4f}")
        if y is not None:
            parts.append(f"Y{ny:.4f}")
        if z is not None:
            parts.append(f"Z{nz:.4f}")
        parts.append(f"F{feed:.1f}")
        self.send(" ".join(parts), wait=wait, timeout_s=MARLIN_MOVE_TIMEOUT_S)
        self.x, self.y, self.z = nx, ny, nz

    def move_rel(self, dx: float = 0.0, dy: float = 0.0, dz: float = 0.0, feed: float = SCAN_FEED_MM_MIN, wait: bool = True) -> None:
        self.move_abs(self.x + dx, self.y + dy, self.z + dz, feed=feed, wait=wait)


class ESP32Head:
    def __init__(self, ser: Any, logger: logging.Logger, dry_run: bool, sim_height_fn: Any = None) -> None:
        self.ser = ser
        self.log = logger
        self.dry_run = dry_run
        self.sim_height_fn = sim_height_fn
        self._laser_on = False

    def close(self) -> None:
        try:
            self.laser_off()
        except Exception:
            pass
        try:
            if self.ser is not None and getattr(self.ser, "is_open", False):
                self.ser.close()
        except Exception as exc:
            self.log.warning("ESP32 close failed: %s", exc)

    def _write(self, cmd: str) -> None:
        if self.dry_run:
            self.log.debug("ESP32 >> %s (dry)", cmd)
            return
        try:
            self.ser.write((cmd + "\n").encode("ascii"))
            self.ser.flush()
        except (SerialException, OSError) as exc:
            raise ScannerError(f"ESP32 write failed: {exc}") from exc

    def _readline(self) -> str:
        if self.dry_run:
            return ""
        try:
            raw = self.ser.readline()
        except (SerialException, OSError) as exc:
            raise ScannerError(f"ESP32 read failed: {exc}") from exc
        if not raw:
            return ""
        return raw.decode("utf-8", errors="replace").strip()

    def drain_boot_banner(self, wait_s: float = ESP32_BOOT_WAIT_S) -> None:
        """Discard READY / help / ADS1115 status lines after USB reset."""
        if self.dry_run:
            return
        deadline = time.time() + max(wait_s, 0.2)
        while time.time() < deadline:
            line = self._readline()
            if line:
                self.log.debug("ESP32 banner << %s", line)

    def laser_off(self) -> None:
        self._laser_on = False
        if self.dry_run:
            self.log.info("LASER OFF (dry-run)")
            return
        if self.ser is None or not getattr(self.ser, "is_open", False):
            return
        try:
            self.ser.write(b"F\n")
            self.ser.flush()
            t0 = time.time()
            while time.time() - t0 < 0.5:
                line = self._readline()
                if not line:
                    continue
                self.log.debug("ESP32 << %s", line)
                if line.startswith("OK:F") or line.startswith("LASER:OFF") or line.startswith("STREAM:STOP"):
                    break
        except Exception as exc:
            self.log.error("failed to send laser OFF: %s", exc)

    def laser_on(self) -> None:
        """Aiming aid: F then L so the MOSFET toggle starts from a known OFF state."""
        if self.dry_run:
            self._laser_on = True
            self.log.info("LASER ON (dry-run)")
            return
        self.laser_off()
        self._write("L")
        t0 = time.time()
        while time.time() - t0 < ESP32_REPLY_TIMEOUT_S:
            line = self._readline()
            if not line:
                continue
            self.log.debug("ESP32 << %s", line)
            if line.startswith("LASER:ON"):
                self._laser_on = True
                return
            if line.startswith("LASER:OFF"):
                self._write("L")
            if line.startswith("ERR"):
                raise ScannerError(f"ESP32: {line}")
        raise ScannerError("ESP32 did not confirm LASER:ON")

    def _simulated_raw(self, x: float, y: float, z: float, z_bed: float, obj_top: float) -> Tuple[float, float, float, float]:
        """Synthetic dual-photodiode response for DRY_RUN."""
        cx, cy = BED_CENTER_X + SENSOR_OFFSET_X, BED_CENTER_Y + SENSOR_OFFSET_Y
        dx, dy = x - cx, y - cy
        radius = 12.0
        r = math.hypot(dx, dy)
        height = max(0.0, obj_top - z_bed)
        if height > 0.5 and r < radius:
            # Hemisphere sitting on the bed (after the user supplies object height).
            h = math.sqrt(max(0.0, radius * radius - r * r)) * height / radius
            surface_z = z_bed + h
        else:
            surface_z = z_bed
        gap = z - surface_z
        base = 7200.0 * math.exp(-max(gap, 0.0) / 1.8)
        if gap < -0.6:
            base *= 0.15  # too close / crashed optically
        # Left/right imbalance encodes slope.
        tilt = clamp((dx / 20.0), -0.4, 0.4)
        amb1 = 320.0
        amb2 = 336.0
        las1 = amb1 + base * (1.0 - tilt)
        las2 = amb2 + base * (1.0 + tilt)
        return amb1, amb2, las1, las2

    def read_raw(self, x: float, y: float, z: float, z_bed: float = 0.0, obj_top: float = 10.0) -> Tuple[float, float, float, float]:
        if self.dry_run:
            return self._simulated_raw(x, y, z, z_bed, obj_top)
        last_err = "no reply"
        for attempt in range(1, ESP32_READ_RETRIES + 1):
            try:
                self.ser.reset_input_buffer()
            except Exception:
                pass
            self._write("R")
            t0 = time.time()
            while time.time() - t0 < ESP32_REPLY_TIMEOUT_S:
                line = self._readline()
                if not line:
                    continue
                self.log.debug("ESP32 << %s", line)
                parsed = parse_data_line(line)
                if parsed:
                    return parsed
                # Ignore boot banners and diagnostic chatter; only DATA: is the scan payload.
                if line.startswith("ERR:BADCMD") or line.startswith("ERR:OVERFLOW"):
                    last_err = line
                    break
                last_err = f"ignored non-DATA: {line!r}"
            self.log.warning("ESP32 R retry %s/%s (%s)", attempt, ESP32_READ_RETRIES, last_err)
        raise ScannerError(f"invalid or missing ESP32 DATA packet ({last_err})")

    def read_sample(self, x: float, y: float, z: float, z_bed: float, obj_top: float) -> SensorSample:
        amb1, amb2, las1, las2 = self.read_raw(x, y, z, z_bed, obj_top)
        return compute_signals(amb1, amb2, las1, las2)


class ScanWriter:
    def __init__(self, out_dir: str) -> None:
        self.out_dir = out_dir
        os.makedirs(out_dir, exist_ok=True)
        self.xyz_path = os.path.join(out_dir, "scan_result.xyz")
        self.ply_path = os.path.join(out_dir, "scan_result.ply")
        self.csv_path = os.path.join(out_dir, "scan_result.csv")
        self.meta_path = os.path.join(out_dir, "scan_result_metadata.json")
        self.log_path = os.path.join(out_dir, "scanner.log")
        self._xyz: Optional[TextIO] = open(self.xyz_path, "w", encoding="utf-8")
        self._csvf: Optional[TextIO] = open(self.csv_path, "w", newline="", encoding="utf-8")
        self._csv = csv.writer(self._csvf)
        self._csv.writerow(
            ["x", "y", "z", "signal1", "signal2", "total_signal", "ratio", "relative_height", "timestamp"]
        )
        self.points: List[ScanPoint] = []

    def add(self, p: ScanPoint) -> None:
        self.points.append(p)
        assert self._xyz is not None and self._csv is not None
        self._xyz.write(f"{p.x:.4f} {p.y:.4f} {p.z:.4f}\n")
        self._xyz.flush()
        self._csv.writerow(
            [
                f"{p.x:.4f}",
                f"{p.y:.4f}",
                f"{p.z:.4f}",
                f"{p.signal1:.4f}",
                f"{p.signal2:.4f}",
                f"{p.total_signal:.4f}",
                f"{p.ratio:.6f}",
                f"{p.relative_height:.4f}",
                p.timestamp,
            ]
        )
        self._csvf.flush()  # type: ignore[union-attr]

    def write_metadata(self, meta: Dict[str, Any]) -> None:
        with open(self.meta_path, "w", encoding="utf-8") as fh:
            json.dump(meta, fh, indent=2)
            fh.write("\n")

    def write_ply(self) -> None:
        try:
            with open(self.ply_path, "w", encoding="utf-8") as fh:
                fh.write("ply\n")
                fh.write("format ascii 1.0\n")
                fh.write(f"element vertex {len(self.points)}\n")
                fh.write("property float x\n")
                fh.write("property float y\n")
                fh.write("property float z\n")
                fh.write("end_header\n")
                for p in self.points:
                    fh.write(f"{p.x:.4f} {p.y:.4f} {p.z:.4f}\n")
        except OSError:
            pass

    def close_files(self) -> None:
        self.write_ply()
        for fh in (self._xyz, self._csvf):
            if fh:
                try:
                    fh.close()
                except Exception:
                    pass
        self._xyz = None
        self._csvf = None


class LaserScanner:
    def __init__(self, comm_test: bool = False) -> None:
        self.comm_test = comm_test
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.out_dir = os.path.abspath(f"scan_{stamp}")
        self.writer = ScanWriter(self.out_dir)
        self._setup_logging()
        self.printer: Optional[MarlinPrinter] = None
        self.head: Optional[ESP32Head] = None
        # z_bed_focus is the absolute machine Z where the bed was seen; z_safe
        # and every height decision are measured from it, never from machine zero.
        self.z_bed_focus: Optional[float] = None
        self.z_safe: Optional[float] = None
        # Route map built by rough_scan(): (x, y) -> absolute Z or None (empty).
        self.rough_map: Dict[Tuple[float, float], Optional[float]] = {}
        self.object_height_est = 0.0
        # Scan window around the bed center; overridable at startup.
        self.scan_size_x = SCAN_SIZE_X
        self.scan_size_y = SCAN_SIZE_Y
        self.z_start = PRE_CAL_SAFE_Z
        self.ratio_filt = 0.0
        self.total_filt = 0.0
        self.have_filter = False
        self.fatal_message: Optional[str] = None
        self.obj_top_sim = 0.0  # DRY_RUN: flat bed until object height is known
        self.user_cancelled = False

    def _setup_logging(self) -> None:
        self.log = logging.getLogger("scanner")
        self.log.setLevel(logging.DEBUG)
        self.log.handlers.clear()
        fmt = logging.Formatter("%(asctime)s %(levelname)s %(message)s")
        fh = logging.FileHandler(self.writer.log_path, encoding="utf-8")
        fh.setLevel(logging.DEBUG)
        fh.setFormatter(fmt)
        sh = logging.StreamHandler(sys.stdout)
        sh.setLevel(logging.INFO)
        sh.setFormatter(fmt)
        self.log.addHandler(fh)
        self.log.addHandler(sh)

    def open_ports(self) -> None:
        if DRY_RUN:
            self.log.info("DRY_RUN: using dummy serial ports")
            self.printer = MarlinPrinter(DummySerial(PRINTER_PORT), self.log, True)
            self.head = ESP32Head(DummySerial(ESP32_PORT), self.log, True)
            return
        if serial is None:
            raise ScannerError("pyserial is not installed. Run: python -m pip install pyserial")
        try:
            pser = serial.Serial(
                PRINTER_PORT,
                BAUD,
                timeout=SERIAL_TIMEOUT_S,
                write_timeout=SERIAL_TIMEOUT_S,
            )
        except (SerialException, OSError) as exc:
            raise ScannerError(f"cannot open printer {PRINTER_PORT}: {exc}") from exc
        try:
            eser = serial.Serial(
                ESP32_PORT,
                BAUD,
                timeout=SERIAL_TIMEOUT_S,
                write_timeout=SERIAL_TIMEOUT_S,
            )
        except (SerialException, OSError) as exc:
            try:
                pser.close()
            except Exception:
                pass
            raise ScannerError(f"cannot open ESP32 {ESP32_PORT}: {exc}") from exc
        self.printer = MarlinPrinter(pser, self.log, False)
        self.head = ESP32Head(eser, self.log, False)
        time.sleep(PRINTER_BOOT_WAIT_S)
        time.sleep(max(0.0, ESP32_BOOT_WAIT_S - PRINTER_BOOT_WAIT_S))
        try:
            pser.reset_input_buffer()
        except Exception:
            pass
        self.head.drain_boot_banner()
        try:
            eser.reset_input_buffer()
        except Exception:
            pass
        self.head.laser_off()
        # Quiet temperature reports if the firmware supports it.
        try:
            self.printer.send("M155 S0")
        except ScannerError as exc:
            self.log.warning("M155 not accepted (ok to ignore): %s", exc)
        self.printer.ensure_absolute()
        self.printer.send("M17")
        self.log.info("Serial ports open: printer=%s esp32=%s baud=%s", PRINTER_PORT, ESP32_PORT, BAUD)

    def emergency_shutdown(self, reason: str) -> None:
        self.fatal_message = reason
        print("\n*** EMERGENCY SHUTDOWN ***")
        print(reason)
        self.log.error("EMERGENCY SHUTDOWN: %s", reason)
        if self.head is not None:
            try:
                self.head.laser_off()
            except Exception as exc:
                self.log.error("laser off during emergency failed: %s", exc)
        if self.printer is not None:
            try:
                self.printer.halt_motion()
            except Exception as exc:
                self.log.error("printer halt failed: %s", exc)
            try:
                self.printer.ensure_absolute()
            except Exception as exc:
                self.log.error("G90 restore failed: %s", exc)
        self._save_metadata(status="emergency", error=reason)

    def _save_metadata(self, status: str, error: Optional[str] = None) -> None:
        meta = {
            "status": status,
            "error": error,
            "timestamp_utc": utc_now_iso(),
            "dry_run": DRY_RUN,
            "point_count": len(self.writer.points),
            "Z_BED_FOCUS": self.z_bed_focus,
            "user_height_above_bed_mm": self.object_height_est,
            "Z_SAFE": self.z_safe,
            "Z_START": self.z_start,
            "calibration_mode": CALIBRATION_MODE,
            "rough_map_cells": len(self.rough_map),
            "rough_map_hits": sum(1 for v in self.rough_map.values() if v is not None),
            "ports": {"printer": PRINTER_PORT, "esp32": ESP32_PORT, "baud": BAUD},
            "limits": {"X": [X_MIN, X_MAX], "Y": [Y_MIN, Y_MAX], "Z": [Z_MIN, Z_MAX]},
            "scan_window_mm": {"size_x": self.scan_size_x, "size_y": self.scan_size_y},
            "thresholds": {
                "SIGNAL_THRESHOLD": SIGNAL_THRESHOLD,
                "MIN_TOTAL_SIGNAL": MIN_TOTAL_SIGNAL,
                "DETECT_RATIO_THRESHOLD": DETECT_RATIO_THRESHOLD,
                "RATIO_DEADBAND": RATIO_DEADBAND,
                "Z_CORRECTION_STEP": Z_CORRECTION_STEP,
                "MAX_Z_CORRECTION_PER_CYCLE": MAX_Z_CORRECTION_PER_CYCLE,
                "Z_CORRECTION_SIGN": Z_CORRECTION_SIGN,
                "DROP_SEARCH_MAX_MM": DROP_SEARCH_MAX_MM,
                "DROP_SEARCH_STEP_MM": DROP_SEARCH_STEP_MM,
                "FOCUS_BALANCE_THRESHOLD": FOCUS_BALANCE_THRESHOLD,
                "ROUGH_STEP_MM": ROUGH_STEP_MM,
                "FINE_STEP_MM": FINE_STEP_MM,
                "BED_EPS_MM": BED_EPS_MM,
                "EMPTY_LINES_TO_STOP": EMPTY_LINES_TO_STOP,
            },
            "output_dir": self.out_dir,
        }
        try:
            self.writer.write_metadata(meta)
        except Exception as exc:
            self.log.error("metadata write failed: %s", exc)

    def cleanup(self) -> None:
        if self.head is not None:
            try:
                self.head.laser_off()
            except Exception:
                pass
            try:
                self.head.close()
            except Exception:
                pass
        if self.printer is not None:
            try:
                self.printer.close()
            except Exception:
                pass
        try:
            self.writer.close_files()
        except Exception:
            pass
        self.log.info("Serial ports closed. Data directory: %s", self.out_dir)

    def sample(self) -> SensorSample:
        assert self.printer and self.head
        z_bed = self.z_bed_focus if self.z_bed_focus is not None else 0.0
        return self.head.read_sample(self.printer.x, self.printer.y, self.printer.z, z_bed, self.obj_top_sim)

    def contact(self, sample: SensorSample) -> bool:
        return sample.total_signal >= DETECT_RATIO_THRESHOLD and sample.total_signal > 0 and finite(sample.total_signal)

    def record_if_valid(self, sample: SensorSample) -> bool:
        assert self.printer and self.z_bed_focus is not None
        if not sample.valid:
            return False
        p = ScanPoint(
            x=self.printer.x,
            y=self.printer.y,
            z=self.printer.z,
            signal1=sample.signal1,
            signal2=sample.signal2,
            total_signal=sample.total_signal,
            ratio=sample.ratio,
            relative_height=self.printer.z - self.z_bed_focus,
            timestamp=utc_now_iso(),
        )
        self.writer.add(p)
        return True

    def update_filters(self, sample: SensorSample) -> None:
        if not sample.valid:
            return
        if not self.have_filter:
            self.ratio_filt = sample.ratio
            self.total_filt = sample.total_signal
            self.have_filter = True
            return
        a = RATIO_FILTER_ALPHA
        b = SIGNAL_FILTER_ALPHA
        self.ratio_filt = a * sample.ratio + (1.0 - a) * self.ratio_filt
        self.total_filt = b * sample.total_signal + (1.0 - b) * self.total_filt

    def z_correction_mm(self) -> float:
        if not self.have_filter:
            return 0.0
        if abs(self.ratio_filt) <= RATIO_DEADBAND:
            return 0.0
        step = Z_CORRECTION_SIGN * math.copysign(Z_CORRECTION_STEP, self.ratio_filt)
        mag = min(abs(self.ratio_filt) / max(RATIO_DEADBAND, 1e-6), 4.0) * abs(step)
        mag = min(mag, MAX_Z_CORRECTION_PER_CYCLE)
        return math.copysign(mag, step)

    def apply_z_keep_above_bed(self, dz: float) -> None:
        assert self.printer and self.z_bed_focus is not None
        target = self.printer.z + dz
        floor = self.z_bed_focus + 0.05
        if target < floor:
            target = floor
        if abs(target - self.printer.z) < 1e-4:
            return
        self.printer.move_abs(z=target, feed=Z_FEED_MM_MIN, wait=False)

    def run_comm_test(self) -> int:
        assert self.printer and self.head
        self.log.info("=== communication test ===")
        self.head.laser_off()
        if DRY_RUN:
            amb1, amb2, las1, las2 = self.head.read_raw(BED_CENTER_X, BED_CENTER_Y, PRE_CAL_SAFE_Z, 0.0, 10.0)
            self.log.info("DRY_RUN synthetic DATA:%s,%s,%s,%s", amb1, amb2, las1, las2)
            sample = compute_signals(amb1, amb2, las1, las2)
            self.log.info("parsed sample valid=%s total=%.1f", sample.valid, sample.total_signal)
            self.head.laser_off()
            print("Communication test (DRY_RUN) finished. Laser remains OFF.")
            return 0
        try:
            self.printer.send("M115")
        except ScannerError as exc:
            raise ScannerError(f"Marlin M115 failed: {exc}") from exc
        self.head.laser_off()
        self.head.laser_on()
        self.head.laser_off()
        sample = self.sample()
        self.log.info(
            "DATA amb=(%.1f,%.1f) las=(%.1f,%.1f) sig=(%.1f,%.1f) total=%.1f ratio=%.4f valid=%s (%s)",
            sample.amb1,
            sample.amb2,
            sample.las1,
            sample.las2,
            sample.signal1,
            sample.signal2,
            sample.total_signal,
            sample.ratio,
            sample.valid,
            sample.reason,
        )
        self.head.laser_off()
        print("Communication test passed. Laser is OFF. Marlin and ESP32 both replied.")
        return 0

    def log_calibration_row(self, stage: str, sample: SensorSample) -> None:
        """CALIBRATION_MODE diagnostics: CSV row (stage,z,total,|s1-s2|) in the log."""
        assert self.printer
        self.log.info(
            "CALIB,%s,%.3f,%.1f,%.1f",
            stage,
            self.printer.z,
            sample.total_signal,
            abs(sample.signal1 - sample.signal2),
        )

    def optical_z_home(self) -> None:
        """Z home without a microswitch: descend until the photodiodes balance.

        Focus is declared when |signal1 - signal2| < FOCUS_BALANCE_THRESHOLD;
        that height becomes the absolute machine zero via 'G92 Z0' and is never
        re-zeroed afterwards.
        """
        assert self.printer and self.head
        self.head.laser_on()
        last_log_z = self.printer.z
        try:
            for _ in range(MAX_Z_CALIB_STEPS):
                sample = self.sample()
                delta = abs(sample.signal1 - sample.signal2)
                if CALIBRATION_MODE and last_log_z - self.printer.z >= 0.2 - 1e-9:
                    self.log_calibration_row("z_home", sample)
                    last_log_z = self.printer.z
                self.log.info(
                    "optical Z home z=%.3f sig=(%.1f,%.1f) |diff|=%.1f",
                    self.printer.z,
                    sample.signal1,
                    sample.signal2,
                    delta,
                )
                if sample.valid and delta < FOCUS_BALANCE_THRESHOLD:
                    self.printer.send("G92 Z0")
                    self.printer.z = 0.0
                    self.log.info("Optical focus reached — machine Z zero set (G92 Z0)")
                    return
                nxt = self.printer.z - Z_CALIBRATE_STEP_MM
                if nxt < Z_MIN:
                    break
                self.printer.move_abs(z=nxt, feed=CALIBRATE_FEED_MM_MIN)
        finally:
            self.head.laser_off()
        raise ScannerError("optical Z homing failed — laser focus never balanced")

    def home_and_safe(self) -> None:
        """G28 X Y, optical Z home, then park over the bed center at a safe Z."""
        assert self.printer
        if HOME_ON_START:
            self.log.info("Homing laterals with %s — Z homes optically", HOME_GCODE)
            self.printer.send(HOME_GCODE, timeout_s=MARLIN_MOVE_TIMEOUT_S)
            self.printer.ensure_absolute()
            self.printer.sync_position()
            self.optical_z_home()
        else:
            self.printer.ensure_absolute()
            try:
                self.printer.sync_position()
            except ScannerError:
                self.log.warning("M114 failed; using last commanded position")
        z_safe = clamp(max(self.printer.z, PRE_CAL_SAFE_Z), Z_MIN, Z_MAX)
        self.printer.move_abs(z=z_safe, feed=Z_FEED_MM_MIN)
        # Bed center is the reference XY for the bed focus search.
        self.printer.move_abs(
            x=clamp(BED_CENTER_X + SENSOR_OFFSET_X, X_MIN, X_MAX),
            y=clamp(BED_CENTER_Y + SENSOR_OFFSET_Y, Y_MIN, Y_MAX),
            feed=TRAVEL_FEED_MM_MIN,
        )

    def calibrate_bed(self) -> None:
        assert self.printer and self.head
        while True:
            self.log.info("Bed calibration: move to center and descend until optical contact")
            self.head.laser_off()
            cx = clamp(BED_CENTER_X + SENSOR_OFFSET_X, X_MIN, X_MAX)
            cy = clamp(BED_CENTER_Y + SENSOR_OFFSET_Y, Y_MIN, Y_MAX)
            self.printer.move_abs(x=cx, y=cy, feed=TRAVEL_FEED_MM_MIN)
            self.printer.move_abs(z=clamp(PRE_CAL_SAFE_Z, Z_MIN, Z_MAX), feed=Z_FEED_MM_MIN)
            found = False
            z_hit = None
            last_log_z = self.printer.z
            # Bed focus uses the finest Z step of the whole run (0.1 mm).
            for i in range(MAX_Z_CALIB_STEPS):
                if self.printer.z <= Z_MIN + 0.01:
                    break
                sample = self.sample()
                if CALIBRATION_MODE and last_log_z - self.printer.z >= 0.2 - 1e-9:
                    self.log_calibration_row("bed_focus", sample)
                    last_log_z = self.printer.z
                self.log.info(
                    "calib z=%.3f total=%.1f valid=%s",
                    self.printer.z,
                    sample.total_signal,
                    sample.valid,
                )
                if self.contact(sample):
                    found = True
                    # z_bed_focus is the absolute machine Z where the bed was
                    # seen. All height logic is relative to it, never to zero.
                    z_hit = self.printer.z
                    break
                nxt = self.printer.z - BED_FOCUS_STEP_MM
                if nxt < Z_MIN:
                    break
                self.printer.move_abs(z=nxt, feed=CALIBRATE_FEED_MM_MIN)
            if found and z_hit is not None:
                self.z_bed_focus = z_hit
                self.log.info("Z_BED_FOCUS (optical bed, machine Z) = %.4f mm", self.z_bed_focus)
                # Back off so the following object search does not grind the bed.
                lift = clamp(self.z_bed_focus + SAFE_CLEARANCE, Z_MIN, Z_MAX)
                self.printer.move_abs(z=lift, feed=Z_FEED_MM_MIN)
                return
            self.head.laser_off()
            self.log.error("Bed calibration failed — no optical contact")
            print("Calibration failed. Laser is OFF. Motion stopped at last commanded Z.")
            if not ask_yes_no("Retry bed calibration?", default=True):
                raise ScannerError("bed calibration failed and user declined retry")
            self.printer.move_abs(z=clamp(PRE_CAL_SAFE_Z, Z_MIN, Z_MAX), feed=Z_FEED_MM_MIN)

    @property
    def z_floor(self) -> float:
        """Hard mechanical floor: Z never goes below z_bed_focus + BED_EPS_MM."""
        assert self.z_bed_focus is not None
        return self.z_bed_focus + BED_EPS_MM

    def ask_safe_height(self) -> None:
        """Prompt for the working height above the bed, park there, laser ON.

        z_safe is measured from z_bed_focus (the absolute machine Z of the bed),
        never from machine zero.
        """
        assert self.printer and self.head and self.z_bed_focus is not None
        user_height_mm = ask_float(
            "Enter safe Z height above bed (mm): ",
            0.5,
            max(0.5, min(Z_MAX - self.z_bed_focus - 1.0, 350.0)),
        )
        self.z_safe = clamp(self.z_bed_focus + user_height_mm, Z_MIN, Z_MAX)
        self.z_start = self.z_safe
        self.object_height_est = user_height_mm
        self.obj_top_sim = self.z_bed_focus + max(user_height_mm * 0.5, 1.0)
        self.printer.move_abs(z=self.z_safe, feed=Z_FEED_MM_MIN)
        # The laser doubles as a placement pointer while the operator works.
        self.head.laser_on()
        self.log.info("z_safe = %.3f mm (bed %.3f + %.3f)", self.z_safe, self.z_bed_focus, user_height_mm)
        input("Place the object on the bed, then press ENTER to start scanning...")

    def scan_window(self) -> Tuple[float, float, float, float]:
        """Full scan window (bed center +/- half the configured size), clipped."""
        cx = BED_CENTER_X + SENSOR_OFFSET_X
        cy = BED_CENTER_Y + SENSOR_OFFSET_Y
        x0 = clamp(cx - self.scan_size_x / 2.0, X_MIN, X_MAX)
        x1 = clamp(cx + self.scan_size_x / 2.0, X_MIN, X_MAX)
        y0 = clamp(cy - self.scan_size_y / 2.0, Y_MIN, Y_MAX)
        y1 = clamp(cy + self.scan_size_y / 2.0, Y_MIN, Y_MAX)
        return x0, x1, y0, y1

    def probe_cell(self, drop_floor: float) -> Optional[float]:
        """Descend in 0.2 mm steps until contact; None when nothing is found.

        The search never goes below drop_floor, which is itself never below
        z_bed_focus + BED_EPS_MM.
        """
        assert self.printer and self.z_bed_focus is not None
        limit = max(drop_floor, self.z_floor)
        last_log_z = self.printer.z
        for _ in range(MAX_Z_CALIB_STEPS):
            sample = self.sample()
            if CALIBRATION_MODE and last_log_z - self.printer.z >= 0.2 - 1e-9:
                self.log_calibration_row("rough", sample)
                last_log_z = self.printer.z
            if self.contact(sample):
                self.update_filters(sample)
                return self.printer.z
            if self.printer.z <= limit + 1e-6:
                return None
            nxt = max(limit, self.printer.z - ROUGH_PROBE_STEP_MM)
            self.printer.move_abs(z=nxt, feed=CALIBRATE_FEED_MM_MIN)
        return None

    def rough_scan(self) -> None:
        """Stop-and-probe 5 mm grid producing the route map for the fine scan."""
        assert self.printer and self.head and self.z_bed_focus is not None and self.z_safe is not None
        x0, x1, y0, y1 = self.scan_window()
        self.log.info("Rough scan grid X[%.1f..%.1f] Y[%.1f..%.1f] step=%.1f", x0, x1, y0, y1, ROUGH_STEP_MM)
        nx = int((x1 - x0) / ROUGH_STEP_MM) + 1
        ny = int((y1 - y0) / ROUGH_STEP_MM) + 1
        # Neighbourhood surface: the highest hit so far, used to bound how deep
        # the probe may look before declaring a cliff.
        neighbourhood_z = self.z_safe
        for iy in range(ny):
            y = y0 + iy * ROUGH_STEP_MM
            for ix in range(nx):
                x = x0 + ix * ROUGH_STEP_MM
                self.printer.move_abs(z=self.z_safe, feed=Z_FEED_MM_MIN)
                self.printer.move_abs(x=x, y=y, feed=TRAVEL_FEED_MM_MIN)
                # Measure only when the machine is completely stopped.
                self.printer.send("M400", timeout_s=MARLIN_MOVE_TIMEOUT_S)
                drop_floor = max(self.z_floor, neighbourhood_z - DROP_SEARCH_MAX_MM)
                z_hit = self.probe_cell(drop_floor)
                self.rough_map[(round(x, 3), round(y, 3))] = z_hit
                if z_hit is None:
                    self.log.info("rough (%.1f, %.1f) -> empty / cliff", x, y)
                else:
                    neighbourhood_z = z_hit
                    self.log.info("rough (%.1f, %.1f) -> z=%.3f", x, y, z_hit)
        self.printer.move_abs(z=self.z_safe, feed=Z_FEED_MM_MIN)
        hits = sum(1 for v in self.rough_map.values() if v is not None)
        self.log.info("Rough map: %s surfaces of %s probes", hits, len(self.rough_map))
        if hits == 0:
            raise ScannerError("rough scan found no surface — check the object and thresholds")

    def rough_seed_z(self, x: float, y: float) -> Optional[float]:
        """Nearest route-map height for a fine-scan coordinate."""
        best_z: Optional[float] = None
        best_d = float("inf")
        for (gx, gy), gz in self.rough_map.items():
            if gz is None:
                continue
            d = (gx - x) ** 2 + (gy - y) ** 2
            if d < best_d:
                best_d = d
                best_z = gz
        return best_z

    def fine_region(self) -> Tuple[float, float, float, float]:
        """Bounding box of the occupied route-map cells, grown by one rough step."""
        hits = [(gx, gy) for (gx, gy), gz in self.rough_map.items() if gz is not None]
        if not hits:
            raise ScannerError("no valid rough-map cells to refine")
        xs = [p[0] for p in hits]
        ys = [p[1] for p in hits]
        x0 = clamp(min(xs) - ROUGH_STEP_MM, X_MIN, X_MAX)
        x1 = clamp(max(xs) + ROUGH_STEP_MM, X_MIN, X_MAX)
        y0 = clamp(min(ys) - ROUGH_STEP_MM, Y_MIN, Y_MAX)
        y1 = clamp(max(ys) + ROUGH_STEP_MM, Y_MIN, Y_MAX)
        return x0, x1, y0, y1

    def handle_drop(self) -> str:
        """Search downward up to DROP_SEARCH_MAX_MM. Returns 'surface', 'empty', or 'bed'."""
        assert self.printer and self.z_bed_focus is not None
        start_z = self.printer.z
        # Never below the bed floor, never deeper than DROP_SEARCH_MAX_MM (20 mm).
        limit_z = max(self.z_floor, start_z - DROP_SEARCH_MAX_MM)
        self.log.info(
            "Signal lost — drop/cliff search from Z=%.3f down to %.3f (max %.1f mm)",
            start_z,
            limit_z,
            DROP_SEARCH_MAX_MM,
        )
        for _ in range(MAX_DROP_SEARCH_STEPS):
            if self.printer.z <= limit_z + 1e-6:
                break
            nxt = max(limit_z, self.printer.z - DROP_SEARCH_STEP_MM)
            self.printer.move_abs(z=nxt, feed=CALIBRATE_FEED_MM_MIN)
            sample = self.sample()
            if sample.valid and self.contact(sample):
                self.update_filters(sample)
                self.log.info("Lower surface found at Z=%.3f", self.printer.z)
                return "surface"
        if self.printer.z <= self.z_floor + 0.03:
            self.log.info("Drop search reached bed reference — treating as empty/edge")
            return "bed"
        self.log.info("No surface within %.1f mm — empty / cliff", DROP_SEARCH_MAX_MM)
        return "empty"

    def lift_after_empty(self) -> None:
        assert self.printer and self.z_bed_focus is not None
        target = clamp(self.printer.z + Z_LIFT_EMPTY_MM, self.z_bed_focus + 0.05, self.z_start)
        target = max(target, min(self.z_start, Z_MAX))
        self.printer.move_abs(z=target, feed=Z_FEED_MM_MIN)

    def calibration_descent(self) -> None:
        """CALIBRATION_MODE: log raw signals every 0.2 mm of descent, no scanning.

        Rows land in the existing ScanWriter CSV; no extra output files.
        """
        assert self.printer and self.head and self.z_bed_focus is not None
        self.head.laser_on()
        self.printer.move_abs(z=clamp(self.z_start, Z_MIN, Z_MAX), feed=Z_FEED_MM_MIN)
        while self.printer.z > self.z_floor + 1e-6:
            sample = self.sample()
            self.log_calibration_row("descent", sample)
            self.record_if_valid(sample)
            nxt = max(self.z_floor, self.printer.z - 0.2)
            self.printer.move_abs(z=nxt, feed=CALIBRATE_FEED_MM_MIN)
        self.head.laser_off()
        self.printer.move_abs(z=clamp(self.z_start, Z_MIN, Z_MAX), feed=Z_FEED_MM_MIN)

    def fine_scan(self) -> None:
        """Fine raster with live Z tracking over the region found by rough_scan."""
        assert self.printer and self.head and self.z_bed_focus is not None
        # Region = bounding box of the occupied route-map cells, not a fixed window.
        x0, x1, y0, y1 = self.fine_region()
        if x1 - x0 < SEGMENT_MM or y1 - y0 < FINE_STEP_MM:
            raise ScannerError("fine scan region too small after clipping to printer limits")

        self.log.info("Fine raster X[%.2f..%.2f] Y[%.2f..%.2f] seg=%.2f ystep=%.2f", x0, x1, y0, y1, SEGMENT_MM, FINE_STEP_MM)
        y = y0
        direction = 1.0
        empty_lines = 0
        line_index = 0
        while y <= y1 + 1e-6 and line_index < MAX_SCAN_LINES:
            line_index += 1
            start_x = x0 if direction > 0 else x1
            end_x = x1 if direction > 0 else x0
            self.printer.move_abs(z=clamp(self.z_start, Z_MIN, Z_MAX), feed=Z_FEED_MM_MIN)
            self.printer.move_abs(x=start_x, y=y, feed=TRAVEL_FEED_MM_MIN)
            # Seed the line height from the nearest route-map cell; fall back to
            # the last tracking height. Never below the bed floor.
            seed = self.rough_seed_z(start_x, y)
            z_resume = self.printer.z if seed is None else seed + SAFE_CLEARANCE
            z_resume = clamp(z_resume, self.z_floor, Z_MAX)
            self.printer.move_abs(z=z_resume, feed=Z_FEED_MM_MIN)
            points_this_line = 0
            abort_line = False
            for _seg in range(MAX_SEGMENTS_PER_LINE):
                remaining = end_x - self.printer.x
                if abs(remaining) < 0.05:
                    break
                step = direction * min(SEGMENT_MM, abs(remaining))
                target_x = clamp(self.printer.x + step, X_MIN, X_MAX)
                dz = self.z_correction_mm()
                target_z = self.printer.z + dz
                target_z = max(target_z, self.z_floor)
                target_z = clamp(target_z, Z_MIN, Z_MAX)
                # Short segment — do not wait for motion complete before sampling.
                self.printer.move_abs(x=target_x, z=target_z, feed=SCAN_FEED_MM_MIN, wait=False)
                lost = 0
                for _ in range(max(1, SAMPLES_PER_SEGMENT)):
                    sample = self.sample()
                    if sample.valid:
                        self.update_filters(sample)
                        if self.record_if_valid(sample):
                            points_this_line += 1
                        lost = 0
                    else:
                        lost += 1
                if lost >= SAMPLES_PER_SEGMENT:
                    self.printer.send("M400", timeout_s=MARLIN_MOVE_TIMEOUT_S)
                    if self.printer.z <= self.z_floor + 1e-6:
                        # BED: sitting on the bed floor with no signal — empty
                        # area, record nothing and leave this line.
                        self.log.info("Bed area at X=%.2f Y=%.2f — skipping", self.printer.x, y)
                        self.printer.move_abs(z=clamp(self.z_start, Z_MIN, Z_MAX), feed=Z_FEED_MM_MIN)
                        abort_line = True
                        break
                    # CLIFF: tracking lost well above the bed — look for a lower surface.
                    result = self.handle_drop()
                    if result == "surface":
                        if self.record_if_valid(self.sample()):
                            points_this_line += 1
                        continue
                    # 'bed' or 'empty': lift clear and cut the line.
                    self.lift_after_empty()
                    abort_line = True
                    break
            self.printer.send("M400", timeout_s=MARLIN_MOVE_TIMEOUT_S)
            try:
                self.printer.sync_position()
            except ScannerError as exc:
                self.log.warning("M114 resync failed after line: %s", exc)
            if abort_line or points_this_line == 0:
                empty_lines += 1
                self.log.info("Line Y=%.3f empty or cliff-aborted (%s consecutive)", y, empty_lines)
            else:
                empty_lines = 0
                self.log.info("Line Y=%.3f stored %s points", y, points_this_line)
            if empty_lines >= EMPTY_LINES_TO_STOP:
                self.log.info("Stopping: %s consecutive empty lines", empty_lines)
                break
            direction *= -1.0
            y += FINE_STEP_MM
            if y > y1 + 1e-6:
                self.log.info("Y limit of scan window reached")
                break
        self.head.laser_off()
        self.printer.ensure_absolute()
        safe_z = clamp(max(self.z_start, self.printer.z), Z_MIN, Z_MAX)
        self.printer.move_abs(z=safe_z, feed=Z_FEED_MM_MIN)

    def confirm_real_run(self) -> None:
        print("\n=== SAFETY CONFIRMATION ===")
        print(f"DRY_RUN = {DRY_RUN}")
        print(f"Printer {PRINTER_PORT} @ {BAUD}, ESP32 {ESP32_PORT} @ {BAUD}")
        print("Laser GPIO25 is Active-HIGH (HIGH=ON, AO3400). Wear 650 nm eye protection.")
        print(f"Travel limits X {X_MIN}-{X_MAX}, Y {Y_MIN}-{Y_MAX}, Z {Z_MIN}-{Z_MAX} mm")
        if DRY_RUN:
            print("DRY_RUN is on: no real motion or laser.")
            return
        if not ask_yes_no("Type yes to allow REAL motion and laser enable", default=False):
            self.user_cancelled = True
            raise ScannerError("user cancelled before motion")

    def run(self) -> int:
        try:
            self.open_ports()
            if self.comm_test:
                rc = self.run_comm_test()
                self._save_metadata(status="comm_test")
                return rc
            self.confirm_real_run()
            # Scan window: operator value or the default, clipped to the limits.
            self.scan_size_x = ask_float(
                f"Scan window X size in mm [default {SCAN_SIZE_X:.0f}]: ",
                ROUGH_STEP_MM,
                X_MAX - X_MIN,
                default=SCAN_SIZE_X,
            )
            self.scan_size_y = ask_float(
                f"Scan window Y size in mm [default {SCAN_SIZE_Y:.0f}]: ",
                ROUGH_STEP_MM,
                Y_MAX - Y_MIN,
                default=SCAN_SIZE_Y,
            )
            self.home_and_safe()
            self.calibrate_bed()
            self.ask_safe_height()
            if CALIBRATION_MODE:
                # Diagnostics only: log signals during a slow descent, no scan.
                self.calibration_descent()
                self._save_metadata(status="calibration")
                print(f"\nCalibration data written to {self.writer.csv_path}")
                return 0
            self.rough_scan()
            self.fine_scan()
            self._save_metadata(status="completed")
            print(f"\nScan complete. Points: {len(self.writer.points)}")
            print(f"Output: {self.out_dir}")
            return 0
        except KeyboardInterrupt:
            self.emergency_shutdown("Ctrl+C — user interrupt")
            return 130
        except ScannerError as exc:
            self.emergency_shutdown(str(exc))
            return 1
        except Exception as exc:
            self.emergency_shutdown(f"fatal exception: {exc!r}")
            return 1
        finally:
            try:
                if self.head is not None:
                    self.head.laser_off()
            except Exception:
                pass
            self.cleanup()


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="DIY laser 3D scanner host (Ender-3 + ESP32)")
    p.add_argument(
        "--comm-test",
        action="store_true",
        help="Open both serial ports, verify Marlin + ESP32 DATA packets, leave laser OFF",
    )
    return p.parse_args(argv)


def main(argv: Optional[List[str]] = None) -> int:
    args = parse_args(argv)
    app = LaserScanner(comm_test=args.comm_test)
    return app.run()


if __name__ == "__main__":
    sys.exit(main())
