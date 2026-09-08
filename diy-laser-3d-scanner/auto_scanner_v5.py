#!/usr/bin/env python3
"""
Host controller for the DIY laser 3D scanner.

Hardware pairing:
  - Creality Ender-3 S1 Plus (Marlin) on PRINTER_PORT (COM8 @ 115200)
  - ESP32 optical head (esp32_sensor.ino) on ESP32_PORT (COM7 @ 115200)
  - Dual photodiodes -> LM358 -> ADS1115 I2C 0x48 (SDA GPIO21, SCL GPIO22)
  - Sensor1 = ADS1115 A0, Sensor2 = ADS1115 A1
  - Laser: AO3400 N-MOSFET GPIO25 Active-HIGH (HIGH=ON, LOW=OFF)

NOTE (homing strategy):
  The CR-Touch / BLTouch probe was REMOVED together with the original
  nozzle head. Therefore a normal 'G28' would make Marlin wait forever for
  a probe that no longer exists and abort with EMERGENCY SHUTDOWN.

  Instead we home X and Y only ('G28 X Y') and define Z=0 from the laser
  FOCUS POINT on the printer bed:
     1) descend slowly while monitoring total_signal (optical contact),
     2) as soon as the focus/contact is found, send 'G92 Z0' so Marlin
        treats that height as the Z origin (replaces the CR-Touch probe).

Usage:
  python auto_scanner_v5.py
  python auto_scanner_v5.py --comm-test
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

PRINTER_PORT = "COM8"  # Ender-3 S1 Plus USB
ESP32_PORT = "COM7"  # ESP32 USB-UART
BAUD = 115200  # Must match Marlin CONFIG and SERIAL_BAUD in the .ino

SERIAL_TIMEOUT_S = 0.2
PRINTER_BOOT_WAIT_S = 3.0
ESP32_BOOT_WAIT_S = 1.0

MARLIN_OK_TIMEOUT_S = 60.0
MARLIN_MOVE_TIMEOUT_S = 120.0
ESP32_REPLY_TIMEOUT_S = 2.0
ESP32_READ_RETRIES = 3

# Printer travel envelope (Ender-3 S1 Plus)
X_MIN, X_MAX = 0.0, 220.0  # mm
Y_MIN, Y_MAX = 0.0, 220.0  # mm
Z_MIN, Z_MAX = 0.0, 400.0  # mm

# Default bed / object XY.
BED_CENTER_X = 110.0  # mm — mechanical bed center
BED_CENTER_Y = 110.0  # mm
SENSOR_OFFSET_X = 0.0  # mm
SENSOR_OFFSET_Y = 0.0  # mm

# Scan window around the object center
SCAN_SIZE_X = 40.0  # mm
SCAN_SIZE_Y = 40.0  # mm

# Motion speeds
TRAVEL_FEED_MM_MIN = 1800.0  # rapid XY
Z_FEED_MM_MIN = 300.0  # Z only
SCAN_FEED_MM_MIN = 240.0  # XY during raster
CALIBRATE_FEED_MM_MIN = 120.0  # slow Z probing

# -----------------------------------------------------------------------------
# Homing configuration
# CR-Touch removed with the original nozzle head -> we home X/Y only and
# define Z=0 from the laser focus point on the bed (see NOTE in the docstring).
# -----------------------------------------------------------------------------
HOME_ON_START = True
HOME_GCODE = "G28 X Y"  # Home X/Y only — do NOT probe Z (no CR-Touch installed)
RESET_BLTOUCH_BEFORE_HOME = False  # No BLTouch / CR-Touch hardware present
OPTICAL_Z_HOME = True  # Apply G92 Z0 at the laser focus point on the bed
HOME_XY_ORIGIN_GCODE = "G92 X0 Y0"  # Clear negative endstop offsets after G28 X Y
POST_HOME_X = 100.0  # mm — park near bed center once the origin is reset
POST_HOME_Y = 100.0  # mm

PRE_CAL_SAFE_Z = 40.0  # mm machine Z after homing (kept above the bed)

# Bed / object optical detection steps
Z_CALIBRATE_STEP_MM = 0.2
Z_OBJECT_STEP_MM = 0.2
SAFE_CLEARANCE = 8.0
Z_LIFT_EMPTY_MM = 6.0

# Sensor math thresholds
SIGNAL_THRESHOLD = 320.0
MIN_TOTAL_SIGNAL = 640.0
DETECT_RATIO_THRESHOLD = 960.0

# Live surface following
RATIO_DEADBAND = 0.08
Z_CORRECTION_STEP = 0.05
MAX_Z_CORRECTION_PER_CYCLE = 0.25
Z_CORRECTION_SIGN = 1
RATIO_FILTER_ALPHA = 0.35
SIGNAL_FILTER_ALPHA = 0.40

# Continuous raster
SEGMENT_MM = 0.8
LINE_STEP_Y_MM = 0.8
SAMPLES_PER_SEGMENT = 2

# Cliff / drop search
DROP_SEARCH_MAX_MM = 30.0
DROP_SEARCH_STEP_MM = 0.4
MAX_DROP_SEARCH_STEPS = int(DROP_SEARCH_MAX_MM / DROP_SEARCH_STEP_MM) + 8
EMPTY_LINES_TO_STOP = 4

# Max iterations
MAX_Z_CALIB_STEPS = 2500
MAX_OBJECT_SEARCH_STEPS = 2500
MAX_SEGMENTS_PER_LINE = 4000
MAX_SCAN_LINES = 4000

DRY_RUN = False

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


def ask_float(prompt: str, lo: float, hi: float) -> float:
    while True:
        try:
            raw = input(prompt).strip().replace(",", ".")
        except EOFError as exc:
            raise ScannerError("input closed") from exc
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

    def read_raw(self, x: float, y: float, z: float, z_bed: float = 0.0, obj_top: float = 10.0) -> Tuple[float, float, float, float]:
        if self.dry_run:
            return 320.0, 336.0, 7500.0, 7400.0
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
        self._csvf.flush()

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
        self.z_bed_focus: Optional[float] = None
        self.object_height_est = 0.0
        self.z_start = PRE_CAL_SAFE_Z
        self.ratio_filt = 0.0
        self.total_filt = 0.0
        self.have_filter = False
        self.fatal_message: Optional[str] = None
        self.obj_top_sim = 0.0
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
            "object_height_estimate_mm": self.object_height_est,
            "Z_START": self.z_start,
            "homing": {"HOME_GCODE": HOME_GCODE,
                       "OPTICAL_Z_HOME": OPTICAL_Z_HOME,
                       "RESET_BLTOUCH_BEFORE_HOME": RESET_BLTOUCH_BEFORE_HOME},
            "ports": {"printer": PRINTER_PORT, "esp32": ESP32_PORT, "baud": BAUD},
            "limits": {"X": [X_MIN, X_MAX], "Y": [Y_MIN, Y_MAX], "Z": [Z_MIN, Z_MAX]},
            "scan_window_mm": {"size_x": SCAN_SIZE_X, "size_y": SCAN_SIZE_Y},
            "thresholds": {
                "SIGNAL_THRESHOLD": SIGNAL_THRESHOLD,
                "MIN_TOTAL_SIGNAL": MIN_TOTAL_SIGNAL,
                "DETECT_RATIO_THRESHOLD": DETECT_RATIO_THRESHOLD,
                "RATIO_DEADBAND": RATIO_DEADBAND,
                "Z_CORRECTION_STEP": Z_CORRECTION_STEP,
                "MAX_Z_CORRECTION_PER_CYCLE": MAX_Z_CORRECTION_PER_CYCLE,
                "Z_CORRECTION_SIGN": Z_CORRECTION_SIGN,
                "DROP_SEARCH_MAX_MM": DROP_SEARCH_MAX_MM,
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

    def home_and_safe(self) -> None:
        """Home X/Y only, then make sure Z is at a safe height.

        NOTE: the CR-Touch probe was removed with the original nozzle head,
        so a plain 'G28' (which homes Z with the probe) must NOT be sent.
        We home laterals with HOME_GCODE ('G28 X Y') and set Z=0 in
        calibrate_bed() via an optical focus search (OPTICAL_Z_HOME).

        The physical endstops sit outside the bed, so Marlin reports negative
        X/Y right after homing. HOME_XY_ORIGIN_GCODE ('G92 X0 Y0') resets the
        lateral origin before any move, then the head parks near bed center.
        """
        assert self.printer
        if HOME_ON_START:
            self.log.info("Homing lateral axes only (%s) — no Z probe (CR-Touch removed)",
                          HOME_GCODE)
            self.printer.send(HOME_GCODE, timeout_s=MARLIN_MOVE_TIMEOUT_S)
            self.printer.ensure_absolute()
            self.log.info("Resetting lateral origin (%s)", HOME_XY_ORIGIN_GCODE)
            self.printer.send(HOME_XY_ORIGIN_GCODE, timeout_s=MARLIN_MOVE_TIMEOUT_S)
            self.printer.x = 0.0
            self.printer.y = 0.0
            self.printer.sync_position()
            self.printer.move_abs(
                x=clamp(POST_HOME_X, X_MIN, X_MAX),
                y=clamp(POST_HOME_Y, Y_MIN, Y_MAX),
                feed=TRAVEL_FEED_MM_MIN,
            )
        else:
            self.printer.ensure_absolute()
            try:
                self.printer.sync_position()
            except ScannerError:
                self.log.warning("M114 failed; using last commanded position")
        z_safe = clamp(max(self.printer.z, PRE_CAL_SAFE_Z), Z_MIN, Z_MAX)
        self.printer.move_abs(z=z_safe, feed=Z_FEED_MM_MIN)

    def calibrate_bed(self) -> None:
        """Find the laser FOCUS POINT on the bed and define it as Z=0.

        This replaces the removed CR-Touch homing: we lower the head in small
        steps until optical contact (peak focus) is detected, then send
        'G92 Z0' so Marlin treats that height as the Z origin.
        """
        assert self.printer and self.head
        while True:
            self.log.info("Bed calibration: descend to laser focus on the bed")
            self.head.laser_off()
            cx = clamp(BED_CENTER_X + SENSOR_OFFSET_X, X_MIN, X_MAX)
            cy = clamp(BED_CENTER_Y + SENSOR_OFFSET_Y, Y_MIN, Y_MAX)
            self.printer.move_abs(x=cx, y=cy, feed=TRAVEL_FEED_MM_MIN)
            self.printer.move_abs(z=clamp(PRE_CAL_SAFE_Z, Z_MIN, Z_MAX), feed=Z_FEED_MM_MIN)
            found = False
            z_hit = None
            for i in range(MAX_Z_CALIB_STEPS):
                if self.printer.z <= Z_MIN + 0.01:
                    break
                sample = self.sample()
                self.log.info(
                    "calib z=%.3f total=%.1f valid=%s",
                    self.printer.z,
                    sample.total_signal,
                    sample.valid,
                )
                if self.contact(sample):
                    found = True
                    z_hit = self.printer.z
                    break
                nxt = self.printer.z - Z_CALIBRATE_STEP_MM
                if nxt < Z_MIN:
                    break
                self.printer.move_abs(z=nxt, feed=CALIBRATE_FEED_MM_MIN)
            if found and z_hit is not None:
                self.log.info("Optical focus (laser on bed) at machine Z=%.4f mm", z_hit)
                if OPTICAL_Z_HOME:
                    # Define the bed focus point as the Z=0 origin,
                    # exactly replacing the removed CR-Touch probe.
                    self.printer.send("G92 Z0")
                    self.printer.z = 0.0
                    self.z_bed_focus = 0.0
                    self.log.info("G92 Z0 applied — bed focus point is now Z=0 (optical Z-home)")
                else:
                    self.z_bed_focus = z_hit
                lift = clamp(self.z_bed_focus + SAFE_CLEARANCE, Z_MIN, Z_MAX)
                self.printer.move_abs(z=lift, feed=Z_FEED_MM_MIN)
                return
            self.head.laser_off()
            self.log.error("Bed calibration failed — no optical contact")
            print("Calibration failed. Laser is OFF. Motion stopped at last commanded Z.")
            if not ask_yes_no("Retry bed calibration?", default=True):
                raise ScannerError("bed calibration failed and user declined retry")
            self.printer.move_abs(z=clamp(PRE_CAL_SAFE_Z, Z_MIN, Z_MAX), feed=Z_FEED_MM_MIN)

    def find_object(self) -> None:
        assert self.printer and self.head and self.z_bed_focus is not None
        self.z_start = self.z_bed_focus + self.object_height_est + SAFE_CLEARANCE
        self.z_start = clamp(self.z_start, Z_MIN, Z_MAX)
        self.obj_top_sim = self.z_bed_focus + max(self.object_height_est, 1.0)
        cx = clamp(BED_CENTER_X + SENSOR_OFFSET_X, X_MIN, X_MAX)
        cy = clamp(BED_CENTER_Y + SENSOR_OFFSET_Y, Y_MIN, Y_MAX)
        self.log.info("Object search from Z_START=%.3f (bed=%.3f est_h=%.3f)", self.z_start, self.z_bed_focus, self.object_height_est)
        self.printer.move_abs(x=cx, y=cy, z=self.z_start, feed=TRAVEL_FEED_MM_MIN)
        for _ in range(MAX_OBJECT_SEARCH_STEPS):
            sample = self.sample()
            if self.contact(sample) and sample.valid:
                self.update_filters(sample)
                self.log.info("Object surface detected at Z=%.4f", self.printer.z)
                return
            nxt = self.printer.z - Z_OBJECT_STEP_MM
            if nxt <= self.z_bed_focus + 0.05:
                self.head.laser_off()
                raise ScannerError("object not found before reaching Z_BED_FOCUS; stop safely")
            self.printer.move_abs(z=nxt, feed=CALIBRATE_FEED_MM_MIN)
        raise ScannerError("object search exceeded MAX_OBJECT_SEARCH_STEPS")

    def handle_drop(self) -> str:
        assert self.printer and self.z_bed_focus is not None
        start_z = self.printer.z
        limit_z = max(self.z_bed_focus + 0.05, start_z - DROP_SEARCH_MAX_MM)
        self.log.info("Signal lost — drop/cliff search from Z=%.3f down to %.3f", start_z, limit_z)
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
        if self.printer.z <= self.z_bed_focus + 0.08:
            self.log.info("Drop search reached bed reference")
            return "bed"
        self.log.info("No surface within %.1f mm — empty / cliff", DROP_SEARCH_MAX_MM)
        return "empty"

    def lift_after_empty(self) -> None:
        assert self.printer and self.z_bed_focus is not None
        target = clamp(self.printer.z + Z_LIFT_EMPTY_MM, self.z_bed_focus + 0.05, self.z_start)
        target = max(target, min(self.z_start, Z_MAX))
        self.printer.move_abs(z=target, feed=Z_FEED_MM_MIN)

    def raster_scan(self) -> None:
        assert self.printer and self.head and self.z_bed_focus is not None
        cx = BED_CENTER_X + SENSOR_OFFSET_X
        cy = BED_CENTER_Y + SENSOR_OFFSET_Y
        x0 = clamp(cx - SCAN_SIZE_X / 2.0, X_MIN, X_MAX)
        x1 = clamp(cx + SCAN_SIZE_X / 2.0, X_MIN, X_MAX)
        y0 = clamp(cy - SCAN_SIZE_Y / 2.0, Y_MIN, Y_MAX)
        y1 = clamp(cy + SCAN_SIZE_Y / 2.0, Y_MIN, Y_MAX)
        if x1 - x0 < SEGMENT_MM or y1 - y0 < LINE_STEP_Y_MM:
            raise ScannerError("scan window too small after clipping")

        self.log.info("Raster window X[%.2f..%.2f] Y[%.2f..%.2f]", x0, x1, y0, y1)
        y = y0
        direction = 1.0
        empty_lines = 0
        line_index = 0
        while y <= y1 + 1e-6 and line_index < MAX_SCAN_LINES:
            line_index += 1
            start_x = x0 if direction > 0 else x1
            end_x = x1 if direction > 0 else x0
            self.printer.move_abs(x=start_x, y=y, feed=TRAVEL_FEED_MM_MIN)
            z_resume = clamp(self.printer.z, self.z_bed_focus + 0.05, Z_MAX)
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
                target_z = max(target_z, self.z_bed_focus + 0.05)
                target_z = clamp(target_z, Z_MIN, Z_MAX)
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
                    result = self.handle_drop()
                    if result == "surface":
                        if self.record_if_valid(self.sample()):
                            points_this_line += 1
                        continue
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
                self.log.info("Line Y=%.3f empty (%s consecutive)", y, empty_lines)
            else:
                empty_lines = 0
                self.log.info("Line Y=%.3f stored %s points", y, points_this_line)
            if empty_lines >= EMPTY_LINES_TO_STOP:
                self.log.info("Stopping: %s consecutive empty lines", empty_lines)
                break
            direction *= -1.0
            y += LINE_STEP_Y_MM
            if y > y1 + 1e-6:
                break
        self.head.laser_off()
        self.printer.ensure_absolute()
        safe_z = clamp(max(self.z_start, self.printer.z), Z_MIN, Z_MAX)
        self.printer.move_abs(z=safe_z, feed=Z_FEED_MM_MIN)

    def confirm_real_run(self) -> None:
        print("\n=== SAFETY CONFIRMATION ===")
        print(f"DRY_RUN = {DRY_RUN}")
        print(f"Printer {PRINTER_PORT} @ {BAUD}, ESP32 {ESP32_PORT} @ {BAUD}")
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
            self.confirm_real_run()
            self.object_height_est = ask_float(
                "Approximate object height in millimeters: ",
                0.5,
                min(350.0, Z_MAX - 5.0),
            )
            self.home_and_safe()
            self.calibrate_bed()
            self.find_object()
            self.raster_scan()
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


def main() -> int:
    app = LaserScanner()
    return app.run()


if __name__ == "__main__":
    sys.exit(main())
