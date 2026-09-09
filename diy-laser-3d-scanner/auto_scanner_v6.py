#!/usr/bin/env python3
"""DIY laser 3D scanner — v6.

Architecture:
    MarlinPrinter : the "hand"   — serial motion control of the Ender-3 S1 Plus.
    ESP32Head     : the "eye"    — laser + dual photodiode sensor head.
    LaserScanner  : the "brain"  — workflow, geometry and scan algorithms.

Workflow (executed in this exact order):
    1. Startup     — open COM7 (ESP32) and COM8 (Marlin).
    2. Homing      — G28, then travel to the bed center (150, 150).
    3. Bed focus   — descend in small steps until the bed is detected -> z_bed_focus.
    4. User setup  — ask for the safe height, park there, laser on, wait for ENTER.
    5. Rough scan  — 5 mm stop-and-probe grid -> rough_map ("road map").
    6. Fine scan   — 0.5 mm zig-zag with Z tracking -> fine_points.
    7. Cleanup     — park, laser off, write PLY / CSV / XYZ.

Only the Python standard library is required; `pyserial` is needed for real
hardware (the script reports a clear error if it is missing).
"""

from __future__ import annotations

import csv
import math
import os
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from typing import Dict, List, Optional, Tuple

try:
    import serial  # type: ignore
    from serial import SerialException  # type: ignore
except ImportError:  # pyserial is optional at import time, required at connect time
    serial = None  # type: ignore

    class SerialException(Exception):  # type: ignore
        """Fallback so except-blocks stay valid without pyserial installed."""


# =============================================================================
# Section 2 — global constants and settings
# =============================================================================

# Working envelope (machine coordinates, mm). Motion is never commanded outside.
X_MIN, Y_MIN = 0, 0
X_MAX, Y_MAX = 300, 300
Z_MAX = 300

# Bed center — homing / parking position.
HOME_X, HOME_Y = 150, 150

# Serial ports and timeouts.
PRINTER_PORT = "COM8"  # Marlin
SENSOR_PORT = "COM7"  # ESP32
BAUD = 115200
MARLIN_MOVE_TIMEOUT_S = 180  # every motion command may take this long
SERIAL_READ_TIMEOUT_S = 1.0
PRINTER_BOOT_WAIT_S = 3.0
ESP32_BOOT_WAIT_S = 2.0
ESP32_REPLY_TIMEOUT_S = 2.0

# Optical head geometry / polarity.
SENSOR_OFFSET = 0  # mm between the nozzle and the optical head (X and Y)
Z_CORRECTION_SIGN = +1  # flip to -1 if Z tracking runs away from the surface
LASER_GPIO = 25  # ESP32 laser pin, active HIGH

# Sensor thresholds: [per-channel signal, minimum total, surface detect, focus balance]
THRESHOLDS = [320, 640, 960, 150]
SIGNAL_THRESHOLD = THRESHOLDS[0]  # minimum net signal on each photodiode
MIN_TOTAL_SIGNAL = THRESHOLDS[1]  # below this the sample is dark / off surface
DETECT_TOTAL_SIGNAL = THRESHOLDS[2]  # total_signal that declares a surface
FOCUS_BALANCE = THRESHOLDS[3]  # |s1 - s2| below this means "in focus"

# Scan geometry.
DROP_SEARCH_MAX_MM = 20.0  # deepest probe search below the bed focus
ROUGH_SCAN_STEP_MM = 5.0  # stop-and-probe grid pitch
FINE_SCAN_STEP_MM = 0.5  # zig-zag pitch
SAFE_Z_ABOVE_BED_MM: Optional[float] = None  # asked from the user at runtime
CALIBRATION_MODE = False  # True: log raw sensor values every 0.2 mm

# Motion speeds (mm/min) and probing granularity.
TRAVEL_FEED = 1800.0
Z_FEED = 300.0
PROBE_FEED = 120.0
SCAN_FEED = 240.0
PROBE_STEP_MM = 0.1  # Z step while searching for a surface
BED_FOCUS_START_Z_MM = 40.0  # Z the head rises to before the bed focus search
CALIBRATION_LOG_STEP_MM = 0.2  # raw-value logging pitch in calibration mode
BED_GUARD_MM = 0.05  # never descend below z_bed_focus + this value
Z_TRACK_STEP_MM = 0.05  # Z correction per fine-scan sample
MAX_Z_TRACK_PER_POINT_MM = 0.25  # clamp on a single correction

OUTPUT_DIR_PREFIX = "scan_v6"


# =============================================================================
# Data containers
# =============================================================================


@dataclass
class SensorSample:
    """One processed reading of the two photodiodes."""

    amb1: float
    amb2: float
    las1: float
    las2: float
    signal1: float  # las1 - amb1 (net laser signal, ambient removed)
    signal2: float
    total_signal: float
    balance: float  # |signal1 - signal2|, small == in focus


@dataclass
class ScanPoint:
    """One recorded surface point."""

    x: float
    y: float
    z: float
    total_signal: float
    balance: float


class ScannerError(RuntimeError):
    """Fatal scanner / communication error."""


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def log(message: str) -> None:
    """Timestamped console logging (stdout is the only sink we need here)."""
    print(f"[{datetime.now().strftime('%H:%M:%S')}] {message}", flush=True)


# =============================================================================
# Section 1.1 — MarlinPrinter: the hand (mechanical control)
# =============================================================================


class MarlinPrinter:
    """Serial control of the Marlin firmware on PRINTER_PORT."""

    def __init__(self, port: str = PRINTER_PORT, baud: int = BAUD) -> None:
        self.port = port
        self.baud = baud
        self.ser = None
        # Last commanded position; refreshed from M114 by get_position().
        self.x = float(HOME_X)
        self.y = float(HOME_Y)
        self.z = 0.0

    # -- connection ---------------------------------------------------------

    def connect(self) -> None:
        """Open the serial port and wait for the board to finish booting."""
        if serial is None:
            raise ScannerError("pyserial is not installed. Run: python -m pip install pyserial")
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=SERIAL_READ_TIMEOUT_S)
        except (SerialException, OSError) as exc:
            raise ScannerError(f"cannot open printer port {self.port}: {exc}") from exc
        time.sleep(PRINTER_BOOT_WAIT_S)
        try:
            self.ser.reset_input_buffer()
        except Exception:
            pass
        self.send_gcode("G21")  # millimetres
        self.send_gcode("G90")  # absolute positioning
        log(f"Printer connected on {self.port}")

    def disconnect(self) -> None:
        """Close the port; never raises so cleanup can always run."""
        try:
            if self.ser is not None and getattr(self.ser, "is_open", False):
                self.ser.close()
                log("Printer disconnected")
        except Exception as exc:
            log(f"WARNING: printer close failed: {exc}")

    # -- low level ----------------------------------------------------------

    def _readline(self) -> str:
        if self.ser is None:
            return ""
        try:
            raw = self.ser.readline()
        except (SerialException, OSError) as exc:
            raise ScannerError(f"printer serial read failed: {exc}") from exc
        if not raw:
            return ""
        return raw.decode("utf-8", errors="replace").strip()

    def _wait_ok(self, timeout_s: float, context: str) -> List[str]:
        """Collect Marlin replies until 'ok'; raise on error or timeout."""
        deadline = time.time() + timeout_s
        lines: List[str] = []
        while True:
            if time.time() > deadline:
                raise ScannerError(f"Marlin timeout waiting for ok ({context})")
            line = self._readline()
            if not line:
                continue
            lines.append(line)
            low = line.lower()
            if low.startswith("ok"):
                return lines
            if low.startswith("busy") or low == "wait" or low.startswith("echo:"):
                continue
            if "error" in low or low.startswith("!!") or low.startswith("kill"):
                raise ScannerError(f"Marlin error ({context}): {line}")

    def send_gcode(self, cmd: str, wait: bool = True, timeout_s: Optional[float] = None) -> List[str]:
        """Send one G-code line and (optionally) wait for the 'ok' reply."""
        if self.ser is None:
            raise ScannerError("printer is not connected")
        cmd = cmd.strip()
        try:
            self.ser.write((cmd + "\n").encode("ascii", errors="strict"))
            self.ser.flush()
        except (SerialException, OSError, UnicodeEncodeError) as exc:
            raise ScannerError(f"printer write failed ({cmd}): {exc}") from exc
        if not wait:
            return []
        return self._wait_ok(timeout_s or MARLIN_MOVE_TIMEOUT_S, cmd)

    # -- motion -------------------------------------------------------------

    def _assert_in_bounds(self, x: float, y: float, z: float) -> None:
        if not all(math.isfinite(v) for v in (x, y, z)):
            raise ScannerError(f"non-finite target X{x} Y{y} Z{z}")
        if not (X_MIN <= x <= X_MAX and Y_MIN <= y <= Y_MAX and 0 <= z <= Z_MAX):
            raise ScannerError(
                f"target out of limits: X{x:.3f} Y{y:.3f} Z{z:.3f} "
                f"(allowed X{X_MIN}-{X_MAX} Y{Y_MIN}-{Y_MAX} Z0-{Z_MAX})"
            )

    def move_abs(
        self,
        x: Optional[float] = None,
        y: Optional[float] = None,
        z: Optional[float] = None,
        feed: float = TRAVEL_FEED,
    ) -> None:
        """Absolute move; axes left as None keep their current target."""
        nx = self.x if x is None else float(x)
        ny = self.y if y is None else float(y)
        nz = self.z if z is None else float(z)
        self._assert_in_bounds(nx, ny, nz)
        parts = ["G1"]
        if x is not None:
            parts.append(f"X{nx:.4f}")
        if y is not None:
            parts.append(f"Y{ny:.4f}")
        if z is not None:
            parts.append(f"Z{nz:.4f}")
        parts.append(f"F{feed:.1f}")
        self.send_gcode(" ".join(parts))
        self.x, self.y, self.z = nx, ny, nz

    def home_xy(self) -> None:
        """Home the machine (G28) — this also defines the machine Z zero."""
        log("Homing X/Y (G28)")
        self.send_gcode("G28", timeout_s=MARLIN_MOVE_TIMEOUT_S)
        self.send_gcode("G90")
        self.get_position()

    def wait_for_idle(self) -> None:
        """Block until the planner buffer is empty (M400)."""
        self.send_gcode("M400", timeout_s=MARLIN_MOVE_TIMEOUT_S)

    def get_position(self) -> Tuple[float, float, float]:
        """Read the current position with M114 and cache it."""
        self.wait_for_idle()
        lines = self.send_gcode("M114", timeout_s=MARLIN_MOVE_TIMEOUT_S)
        for line in lines:
            if "X:" in line and "Y:" in line and "Z:" in line:
                try:
                    fields = line.replace(":", " ").split()
                    values = {}
                    for i, token in enumerate(fields[:-1]):
                        if token in ("X", "Y", "Z"):
                            values[token] = float(fields[i + 1])
                    if {"X", "Y", "Z"} <= set(values):
                        self.x, self.y, self.z = values["X"], values["Y"], values["Z"]
                        return self.x, self.y, self.z
                except ValueError:
                    continue
        # No parsable M114 reply: fall back to the last commanded position.
        log("WARNING: could not parse M114, using last commanded position")
        return self.x, self.y, self.z


# =============================================================================
# Section 1.2 — ESP32Head: the eye and the pencil (sensor + laser)
# =============================================================================


class ESP32Head:
    """Serial control of the ESP32 optical head on SENSOR_PORT."""

    def __init__(self, port: str = SENSOR_PORT, baud: int = BAUD) -> None:
        self.port = port
        self.baud = baud
        self.ser = None
        self.laser_is_on = False
        self.last_sample: Optional[SensorSample] = None

    # -- connection ---------------------------------------------------------

    def connect(self) -> None:
        if serial is None:
            raise ScannerError("pyserial is not installed. Run: python -m pip install pyserial")
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=SERIAL_READ_TIMEOUT_S)
        except (SerialException, OSError) as exc:
            raise ScannerError(f"cannot open sensor port {self.port}: {exc}") from exc
        time.sleep(ESP32_BOOT_WAIT_S)
        try:
            self.ser.reset_input_buffer()
        except Exception:
            pass
        log(f"ESP32 head connected on {self.port} (laser on GPIO{LASER_GPIO}, active HIGH)")

    def disconnect(self) -> None:
        """Always leave the laser off, then close the port."""
        try:
            self.laser_off()
        except Exception:
            pass
        try:
            if self.ser is not None and getattr(self.ser, "is_open", False):
                self.ser.close()
                log("ESP32 head disconnected")
        except Exception as exc:
            log(f"WARNING: ESP32 close failed: {exc}")

    # -- low level ----------------------------------------------------------

    def send_cmd(self, cmd: str) -> str:
        """Send a single-letter command and return the first reply line."""
        if self.ser is None:
            raise ScannerError("ESP32 head is not connected")
        try:
            self.ser.reset_input_buffer()
            self.ser.write((cmd + "\n").encode("ascii"))
            self.ser.flush()
        except (SerialException, OSError) as exc:
            raise ScannerError(f"ESP32 write failed ({cmd}): {exc}") from exc
        deadline = time.time() + ESP32_REPLY_TIMEOUT_S
        while time.time() < deadline:
            try:
                raw = self.ser.readline()
            except (SerialException, OSError) as exc:
                raise ScannerError(f"ESP32 read failed ({cmd}): {exc}") from exc
            if not raw:
                continue
            line = raw.decode("utf-8", errors="replace").strip()
            if line:
                return line
        raise ScannerError(f"ESP32 timeout waiting for a reply to {cmd!r}")

    # -- laser --------------------------------------------------------------

    def laser_on(self) -> None:
        """'N' switches the laser MOSFET on (active HIGH)."""
        self.send_cmd("N")
        self.laser_is_on = True
        log("LASER ON — wear 650 nm eye protection")

    def laser_off(self) -> None:
        """'F' switches the laser off; failures are logged, never raised."""
        try:
            self.send_cmd("F")
        except ScannerError as exc:
            log(f"WARNING: laser off failed: {exc}")
        self.laser_is_on = False

    # -- sensing ------------------------------------------------------------

    def read_raw(self) -> Tuple[float, float, float, float]:
        """'R' returns 'DATA:amb1,amb2,las1,las2' — parsed into four floats."""
        reply = self.send_cmd("R")
        payload = reply.split(":", 1)[1] if reply.upper().startswith("DATA") else reply
        parts = payload.split(",")
        if len(parts) != 4:
            raise ScannerError(f"malformed ESP32 data packet: {reply!r}")
        try:
            amb1, amb2, las1, las2 = (float(p) for p in parts)
        except ValueError as exc:
            raise ScannerError(f"non-numeric ESP32 data packet: {reply!r}") from exc
        return amb1, amb2, las1, las2

    def compute_signals(self) -> SensorSample:
        """Read the head and derive net signals, total_signal and |s1 - s2|."""
        amb1, amb2, las1, las2 = self.read_raw()
        signal1 = las1 - amb1  # ambient light removed
        signal2 = las2 - amb2
        sample = SensorSample(
            amb1=amb1,
            amb2=amb2,
            las1=las1,
            las2=las2,
            signal1=signal1,
            signal2=signal2,
            total_signal=signal1 + signal2,
            balance=abs(signal1 - signal2),
        )
        self.last_sample = sample
        return sample

    def is_surface_detected(self, sample: Optional[SensorSample] = None) -> bool:
        """True when both diodes see a strong, balanced (in-focus) return."""
        s = sample if sample is not None else self.compute_signals()
        return (
            s.signal1 >= SIGNAL_THRESHOLD
            and s.signal2 >= SIGNAL_THRESHOLD
            and s.total_signal >= MIN_TOTAL_SIGNAL
            and s.total_signal >= DETECT_TOTAL_SIGNAL
            and s.balance < FOCUS_BALANCE
        )


# =============================================================================
# Section 1.3 — LaserScanner: the brain (workflow and algorithms)
# =============================================================================


class LaserScanner:
    """Coordinates the hand and the eye and runs the seven workflow steps."""

    def __init__(self) -> None:
        self.printer = MarlinPrinter()
        self.head = ESP32Head()
        # Absolute machine Z of the bed focus point — the reference for every
        # height comparison in this script (never the machine zero).
        self.z_bed_focus: Optional[float] = None
        self.z_safe: Optional[float] = None
        self.rough_map: Dict[Tuple[float, float], Optional[float]] = {}
        self.fine_points: List[ScanPoint] = []
        self.out_dir = f"{OUTPUT_DIR_PREFIX}_{datetime.now().strftime('%Y%m%d_%H%M%S')}"

    # -- helpers ------------------------------------------------------------

    @property
    def z_floor(self) -> float:
        """Hard mechanical guard: never descend below this Z."""
        assert self.z_bed_focus is not None
        return self.z_bed_focus + BED_GUARD_MM

    def _probe_down(self, z_from: float, z_limit: float) -> Optional[float]:
        """Step Z down from z_from until a surface is detected.

        Returns the absolute Z where the surface was found, or None when the
        search reached z_limit without any detection (a drop / empty bed).
        """
        z = z_from
        last_log_z = z_from
        while z > z_limit:
            sample = self.head.compute_signals()
            if CALIBRATION_MODE and (last_log_z - z) >= CALIBRATION_LOG_STEP_MM - 1e-9:
                log(f"  calib z={z:.3f} total={sample.total_signal:.1f} |s1-s2|={sample.balance:.1f}")
                last_log_z = z
            if self.head.is_surface_detected(sample):
                return self.printer.get_position()[2]
            z = max(z - PROBE_STEP_MM, z_limit)
            self.printer.move_abs(z=z, feed=PROBE_FEED)
        return None

    def _park_safe(self) -> None:
        assert self.z_safe is not None
        self.printer.move_abs(z=self.z_safe, feed=Z_FEED)

    # -- step 1 -------------------------------------------------------------

    def startup(self) -> None:
        """Step 1 — open both serial links."""
        log("=== Step 1: startup ===")
        self.head.connect()
        self.printer.connect()

    # -- step 2 -------------------------------------------------------------

    def homing(self) -> None:
        """Step 2 — home the machine (defines machine zero) and go to center."""
        log("=== Step 2: homing ===")
        self.printer.home_xy()
        self.printer.move_abs(
            x=HOME_X + SENSOR_OFFSET,
            y=HOME_Y + SENSOR_OFFSET,
            feed=TRAVEL_FEED,
        )
        log(f"At bed center ({HOME_X}, {HOME_Y})")

    # -- step 3 -------------------------------------------------------------

    def bed_focus(self) -> None:
        """Step 3 — descend at the center until the bed is in optical focus."""
        log("=== Step 3: bed focus ===")
        self.head.laser_on()
        start_z = max(self.printer.get_position()[2], BED_FOCUS_START_Z_MM)
        self.printer.move_abs(z=start_z, feed=Z_FEED)
        z_hit = self._probe_down(start_z, z_limit=0.0)
        if z_hit is None:
            raise ScannerError("bed focus failed — no surface detected above Z0")
        # This absolute Z marks the bed: it is our reference zero, not machine 0.
        self.z_bed_focus = z_hit
        log(f"z_bed_focus = {self.z_bed_focus:.4f} mm (absolute machine Z of the bed)")

    # -- step 4 -------------------------------------------------------------

    def user_setup(self) -> None:
        """Step 4 — ask for the safe height, park there and wait for the user."""
        log("=== Step 4: user setup ===")
        assert self.z_bed_focus is not None
        global SAFE_Z_ABOVE_BED_MM
        while True:
            raw = input("Enter safe Z height above bed (mm): ").strip()
            try:
                user_height_mm = float(raw)
            except ValueError:
                print("Please enter a number, for example 20")
                continue
            if user_height_mm <= 0 or self.z_bed_focus + user_height_mm > Z_MAX:
                print(f"Height must be > 0 and keep Z below {Z_MAX} mm")
                continue
            break
        SAFE_Z_ABOVE_BED_MM = user_height_mm
        self.z_safe = self.z_bed_focus + user_height_mm
        self.printer.move_abs(z=self.z_safe, feed=Z_FEED)
        self.head.laser_on()  # the laser doubles as a placement pointer
        log(f"z_safe = {self.z_safe:.3f} mm")
        input("Place the object on the bed, then press ENTER to start scanning...")
        if CALIBRATION_MODE:
            log("CALIBRATION_MODE is on: raw sensor values are logged every 0.2 mm")

    # -- step 5 -------------------------------------------------------------

    def rough_scan(self) -> None:
        """Step 5 — 5 mm stop-and-probe grid producing the road map."""
        log("=== Step 5: rough scan (stop-and-probe, 5 mm grid) ===")
        assert self.z_bed_focus is not None and self.z_safe is not None
        xs = self._axis_points(X_MIN, X_MAX, ROUGH_SCAN_STEP_MM)
        ys = self._axis_points(Y_MIN, Y_MAX, ROUGH_SCAN_STEP_MM)
        drop_limit = max(self.z_floor, self.z_bed_focus - DROP_SEARCH_MAX_MM)
        for y in ys:
            for x in xs:
                self._park_safe()
                self.printer.move_abs(x=x + SENSOR_OFFSET, y=y + SENSOR_OFFSET, feed=TRAVEL_FEED)
                self.printer.wait_for_idle()  # measure only while fully stopped
                z_hit = self._probe_down(self.z_safe, z_limit=drop_limit)
                self.rough_map[(x, y)] = z_hit
                if z_hit is None:
                    log(f"rough ({x:.1f}, {y:.1f}) -> empty / drop")
                else:
                    log(f"rough ({x:.1f}, {y:.1f}) -> z={z_hit:.3f}")
        self._park_safe()
        hits = sum(1 for v in self.rough_map.values() if v is not None)
        log(f"rough map built: {hits} surface points of {len(self.rough_map)} probes")

    # -- step 6 -------------------------------------------------------------

    def fine_scan(self) -> None:
        """Step 6 — 0.5 mm zig-zag with continuous Z tracking (the pencil)."""
        log("=== Step 6: fine scan (Z-tracking, 0.5 mm zig-zag) ===")
        assert self.z_bed_focus is not None and self.z_safe is not None
        xs = self._axis_points(X_MIN, X_MAX, FINE_SCAN_STEP_MM)
        ys = self._axis_points(Y_MIN, Y_MAX, FINE_SCAN_STEP_MM)
        for row, y in enumerate(ys):
            line = xs if row % 2 == 0 else list(reversed(xs))  # zig-zag
            z_track = self._row_start_z(line[0], y)
            if z_track is None:
                continue  # no road-map height for this row: nothing to follow
            self.printer.move_abs(z=self.z_safe, feed=Z_FEED)
            self.printer.move_abs(x=line[0] + SENSOR_OFFSET, y=y + SENSOR_OFFSET, feed=TRAVEL_FEED)
            self.printer.move_abs(z=max(z_track, self.z_floor), feed=Z_FEED)
            for x in line:
                self.printer.move_abs(x=x + SENSOR_OFFSET, feed=SCAN_FEED)
                sample = self.head.compute_signals()
                z_now = self.printer.z
                if self.head.is_surface_detected(sample):
                    # Case C — on the object: record the point.
                    self.fine_points.append(
                        ScanPoint(x, y, z_now, sample.total_signal, sample.balance)
                    )
                    z_track = z_now
                    continue
                if z_now <= self.z_floor:
                    # Case A — bare bed: nothing to scan here, lift and move on.
                    self.printer.move_abs(z=self.z_safe, feed=Z_FEED)
                    z_track = self._row_start_z(x, y) or self.z_safe
                    self.printer.move_abs(z=max(z_track, self.z_floor), feed=Z_FEED)
                    continue
                # Case B — cliff (object edge): follow the surface downwards
                # while staying above the bed guard.
                step = Z_CORRECTION_SIGN * min(Z_TRACK_STEP_MM, MAX_Z_TRACK_PER_POINT_MM)
                z_track = clamp(z_now - step, self.z_floor, self.z_safe)
                self.printer.move_abs(z=z_track, feed=Z_FEED)
            log(f"row y={y:.1f} done — {len(self.fine_points)} points so far")
        self._park_safe()

    def _row_start_z(self, x: float, y: float) -> Optional[float]:
        """Nearest road-map height for a fine-scan coordinate."""
        gx = round(x / ROUGH_SCAN_STEP_MM) * ROUGH_SCAN_STEP_MM
        gy = round(y / ROUGH_SCAN_STEP_MM) * ROUGH_SCAN_STEP_MM
        gx = clamp(gx, X_MIN, X_MAX)
        gy = clamp(gy, Y_MIN, Y_MAX)
        return self.rough_map.get((gx, gy))

    @staticmethod
    def _axis_points(low: float, high: float, step: float) -> List[float]:
        """Inclusive axis sampling grid."""
        count = int(round((high - low) / step)) + 1
        return [round(low + i * step, 4) for i in range(count)]

    # -- step 7 -------------------------------------------------------------

    def cleanup(self) -> None:
        """Step 7 — park at the center, laser off, write the three files."""
        log("=== Step 7: cleanup ===")
        try:
            if self.z_safe is not None:
                self.printer.move_abs(z=self.z_safe, feed=Z_FEED)
                self.printer.move_abs(x=HOME_X, y=HOME_Y, feed=TRAVEL_FEED)
        except ScannerError as exc:
            log(f"WARNING: parking failed: {exc}")
        self.head.laser_off()
        self.head.disconnect()
        self.printer.disconnect()
        self.write_outputs()

    def write_outputs(self) -> None:
        """Write scan_output.ply / .csv / .xyz into the run directory."""
        os.makedirs(self.out_dir, exist_ok=True)
        ply_path = os.path.join(self.out_dir, "scan_output.ply")
        csv_path = os.path.join(self.out_dir, "scan_output.csv")
        xyz_path = os.path.join(self.out_dir, "scan_output.xyz")
        try:
            with open(ply_path, "w", encoding="utf-8") as f:
                f.write("ply\nformat ascii 1.0\n")
                f.write(f"element vertex {len(self.fine_points)}\n")
                f.write("property float x\nproperty float y\nproperty float z\n")
                f.write("end_header\n")
                for p in self.fine_points:
                    f.write(f"{p.x:.4f} {p.y:.4f} {p.z:.4f}\n")
            with open(csv_path, "w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerow(["x", "y", "z", "total_signal", "s1_s2"])
                for p in self.fine_points:
                    writer.writerow(
                        [f"{p.x:.4f}", f"{p.y:.4f}", f"{p.z:.4f}", f"{p.total_signal:.1f}", f"{p.balance:.1f}"]
                    )
            with open(xyz_path, "w", encoding="utf-8") as f:
                for p in self.fine_points:
                    f.write(f"{p.x:.4f} {p.y:.4f} {p.z:.4f}\n")
        except OSError as exc:
            log(f"ERROR: writing output files failed: {exc}")
            return
        log(f"Saved {len(self.fine_points)} points to {self.out_dir}")

    # -- orchestration ------------------------------------------------------

    def run(self) -> int:
        """Execute the workflow; every failure ends in a clean shutdown."""
        try:
            self.startup()
        except ScannerError as exc:
            log(f"ERROR: startup failed: {exc}")
            self.head.disconnect()
            self.printer.disconnect()
            return 1
        try:
            self.homing()
            self.bed_focus()
            self.user_setup()
            self.rough_scan()
            self.fine_scan()
            return 0
        except ScannerError as exc:
            log(f"ERROR: {exc}")
            return 1
        except KeyboardInterrupt:
            log("Interrupted by the user")
            return 1
        finally:
            self.cleanup()


def main() -> int:
    print("DIY laser 3D scanner v6")
    print(f"Printer {PRINTER_PORT}, ESP32 {SENSOR_PORT}, laser on GPIO{LASER_GPIO} (active HIGH)")
    return LaserScanner().run()


if __name__ == "__main__":
    sys.exit(main())
