# DIY Laser 3D Scanner (Ender-3 S1 Plus + ESP32)

Continuous zigzag raster scanner for a Creality Ender-3 S1 Plus (Marlin) with an ESP32 optical head: dual photodiodes, LM358 preamp, ADS1115 16-bit ADC, and a 650 nm laser driven by an AO3400 N-MOSFET.

## Laser safety (read first)

- This project uses a **class 2/3R-class 650 nm red laser**. Treat it as hazardous to eyes.
- **Never look into the beam** or at specular reflections from metal, glass, or glossy plastic.
- Wear **650 nm OD2+ laser safety glasses** while the laser is powered.
- Keep the beam pointed at the printer bed. Block windows/mirrors in the beam path.
- Firmware and Python **default the laser OFF** (GPIO25 LOW). Confirm MOSFET polarity before first power-up.
- Do not leave `DRY_RUN = False` unattended. Keep a hardware kill switch on the laser supply.

## Hardware (verify before connecting the real laser)

| Item | Value |
| --- | --- |
| Printer | Creality Ender-3 S1 Plus, Marlin, USB serial |
| Printer port | `COM8`, `115200` 8N1 |
| ESP32 | ESP32 DevKit, USB-UART `COM7`, `115200` 8N1 |
| ADC | ADS1115 16-bit, I2C address **0x48** |
| I2C | SDA **GPIO 21**, SCL **GPIO 22** |
| Sensor 1 | ADS1115 **A0** (photodiode → LM358) |
| Sensor 2 | ADS1115 **A1** (photodiode → LM358) |
| ADS1115 gain | `GAIN_ONE` ±4.096 V (1 bit = 0.125 mV). Optional: `GAIN_TWOTHIRDS` ±6.144 V (1 bit = 0.1875 mV) |
| Laser driver | AO3400 N-channel MOSFET on **GPIO 25** |
| Laser polarity | **Active-HIGH** (HIGH = ON, LOW = OFF, 10 kΩ gate pull-down) |
| Bed travel | X 0–220 mm, Y 0–220 mm, Z 0–400 mm |
| Laser wavelength | 650 nm |

### Wiring

```
ESP32 3V3  ---------------- ADS1115 VDD, LM358 VCC (use 5 V for the op-amp only if analog out stays within ADS1115 VDD)
ESP32 GND  ---------------- ADS1115 GND, LM358 GND, MOSFET source, laser module GND, printer USB GND
GPIO21     ---------------- ADS1115 SDA
GPIO22     ---------------- ADS1115 SCL
ADS1115 A0 ---------------- LM358 channel 1 output (sensor 1)
ADS1115 A1 ---------------- LM358 channel 2 output (sensor 2)
GPIO25     ---------------- AO3400 gate (10 kΩ pull-down to GND; HIGH = laser ON)
AO3400 drain ------------- laser cathode / module enable (per your diode wiring)
USB COM7   ---------------- ESP32 USB-UART
USB COM8   ---------------- Ender-3 S1 Plus USB (Marlin)
```

Startup, host timeout, `F`/`X`, and the end of every `R` pulse leave GPIO25 **LOW** (laser OFF).

## Arduino libraries

In Library Manager install:

- **Adafruit ADS1X15** (`Adafruit_ADS1X15.h`)
- **Adafruit BusIO** (dependency)

## Upload the firmware

1. Arduino IDE or `arduino-cli`, board **ESP32 Dev Module**.
2. Open `esp32_sensor/esp32_sensor.ino`.
3. Port `COM7`, upload.
4. Serial Monitor **115200**. You should see a short banner (`READY`, ADS1115 status) with the laser **off**.
5. Send `?` or `H` for the diagnostic menu.

## Serial diagnostic suite (Serial Monitor)

Commands are **single characters** (newline optional). Case-insensitive.

| Key | Function |
| --- | --- |
| `?` / `H` | Help: pinout, commands, laser state, stored offset, ADS1115 status |
| `I` | Probe I2C `0x48`, verify ADS1115, print `ADS1115 Status: OK` or `ERROR (Check SDA/SCL wiring)` |
| `L` | Toggle laser ON/OFF for beam aiming, optics, and MOSFET test |
| `N` | 50 ambient samples, laser OFF, A0 and A1: Min / Max / Avg / peak-to-peak + rating (`EXCELLENT` … `HIGH NOISE`) |
| `S` | Live alignment stream ~15–20 Hz. Stop with **any character**, `F`, or `X` |
| `C` | Measure A0/A1 hardware offset; store in RAM; applied to `S` stream `Diff` |
| `R` | Python scan pulse. Reply: `DATA:amb1,amb2,las1,las2` |
| `F` / `X` | Immediate laser OFF, idle |

### Live stream (`S`) — Serial Plotter friendly

Each cycle: ambient A0/A1 → laser ON 3 ms → laser A0/A1 → laser OFF.

```
Net = las − amb
Diff = Net1 − Net2 − offset
Sum  = Net1 + Net2
Ratio = (Sum > 50) ? Diff/Sum : 0
```

Line format:

```text
Amb1:%d Amb2:%d Net1:%d Net2:%d Diff:%d Ratio:%.3f Status:%s
```

Status:

- `GOLDEN_FOCUS` when `|Diff| < 150`
- `SATURATED` if either Net `> 26000` (turn LM358 trimmers down)
- `LOW_SIGNAL` if `Sum < 100`
- `TRACKING` otherwise

Use `S` to set LM358 gain without clipping, then jog Z until `Diff ≈ 0` (golden focus).

## Serial settings (Python)

- Printer: `PRINTER_PORT = "COM8"`, `BAUD = 115200`
- ESP32: `ESP32_PORT = "COM7"`, `BAUD = 115200`
- The host ignores ESP32 boot banners and only accepts `DATA:...` during a scan. It never sends `N` (noise test). Laser OFF is `F`. Scan samples are `R`.

## Python install

```text
python -m pip install pyserial
```

Requires Python 3.9+.

## Run the Python program

```text
python auto_scanner_v4.py
```

Confirms before real motion/laser unless you use `--comm-test` or `DRY_RUN`.

The controller keeps:

- Differential null / ratio tracking (`(sig1 − sig2) / (sig1 + sig2)`)
- Bounded travel limits and safe Z floor at `Z_BED_FOCUS`
- Zigzag raster with short XY segments and live Z correction
- Cliff / empty-bed drop search
- Point clouds: `.xyz` and `.ply` plus CSV / metadata / log

## DRY_RUN

In `auto_scanner_v4.py`:

```python
DRY_RUN = True
```

No serial ports, no laser, simulated sensors and G-code.

## Communication test

```text
python auto_scanner_v4.py --comm-test
```

Opens both ports, Marlin `M115`, ESP32 `F` / `L` / `F` / `R`, verifies `DATA:`, leaves the laser OFF.

## Calibrate `Z_BED_FOCUS`

1. Clear the bed. Confirm jig clearance.
2. `DRY_RUN = False`, confirm the safety prompt.
3. Home (configurable), move to bed center, step Z down.
4. Optical contact (`DETECT_RATIO_THRESHOLD` / `MIN_TOTAL_SIGNAL`) stores machine Z as `Z_BED_FOCUS`.
5. This is an optical reference, not necessarily firmware `Z=0`.

## Reverse `Z_CORRECTION_SIGN`

If the head drives **into** the surface:

```python
Z_CORRECTION_SIGN = -1  # or +1
```

Depends on which photodiode is on +X vs −X.

## Adjust sensor thresholds

ADS1115 counts (after `las − amb`), in the Python **CONFIGURATION** block:

- `SIGNAL_THRESHOLD` — minimum per-channel net
- `MIN_TOTAL_SIGNAL` — reject dark / off-surface samples
- `DETECT_RATIO_THRESHOLD` — first contact (uses `total_signal`)
- `RATIO_DEADBAND` — ignore small left/right imbalance
- `Z_CORRECTION_STEP` / `MAX_Z_CORRECTION_PER_CYCLE` — live Z servo
- `DROP_SEARCH_MAX_MM` — cliff search
- `EMPTY_LINES_TO_STOP` — consecutive empty raster lines end the scan

## Emergency stop

- **Ctrl+C** or a Marlin/serial error runs `emergency_shutdown()`:
  1. Laser OFF (`F` on ESP32)
  2. Printer halt (`M410` / `M84`) and `G90`
  3. Serial ports closed
  4. Points flushed to the timestamped folder (`.xyz` / `.ply` / CSV)
  5. Error in `scanner.log`
- Keep the printer reset and laser PSU switch within reach.

## Output

Each run writes `scan_YYYYMMDD_HHMMSS/` with:

- `scan_result.xyz`
- `scan_result.ply`
- `scan_result.csv`
- `scan_result_metadata.json`
- `scanner.log`

`relative_height = z - Z_BED_FOCUS`.
