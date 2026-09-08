/*
 * ESP32 optical head for the DIY laser 3D scanner.
 *
 * Hardware:
 *   Dual photodiodes -> LM358 preamp -> ADS1115 (I2C 0x48) A0 / A1
 *   I2C: SDA GPIO21, SCL GPIO22
 *   Laser: N-channel MOSFET AO3400 on GPIO25, Active-HIGH
 *          HIGH = laser ON, LOW = laser OFF (external 10k pull-down)
 *
 * Default: laser OFF at boot, after F/X, after R, on host timeout, and on error.
 *
 * Serial diagnostic suite (single-character, case-insensitive):
 *   ? / H  help menu
 *   I      I2C / ADS1115 probe
 *   L      toggle laser (aiming / MOSFET test)
 *   N      ambient noise floor (laser OFF)
 *   S      live alignment stream (~15-20 Hz)
 *   C      A0/A1 offset calibration (RAM)
 *   R      pulsed read for Python: DATA:amb1,amb2,las1,las2
 *   F / X  laser OFF, idle
 */

#include <Wire.h>
#include <Adafruit_ADS1X15.h>

// =============================================================================
// CONFIGURATION
// =============================================================================

static const uint32_t SERIAL_BAUD = 115200;

static const int PIN_LASER = 25;
static const int PIN_SDA = 21;
static const int PIN_SCL = 22;
static const uint8_t ADS1115_ADDR = 0x48;

// Active-HIGH MOSFET driver (AO3400). LOW = off (10k pull-down on the gate).
static const int LASER_ON_LEVEL = LOW;
static const int LASER_OFF_LEVEL = HIGH;

// GAIN_ONE: +/-4.096 V, 1 bit = 0.125 mV (LM358 0–3.3/5 V).
// GAIN_TWOTHIRDS: +/-6.144 V, 1 bit = 0.1875 mV if the preamp can exceed 4.096 V.
static const adsGain_t ADS_GAIN = GAIN_ONE;

static const uint16_t LASER_PULSE_MS = 3;
static const uint16_t AMBIENT_SETTLE_MS = 2;

// Alignment stream period: 50–66 ms → ~15–20 Hz.
static const uint16_t STREAM_PERIOD_MS = 55;

static const uint8_t NOISE_SAMPLES = 50;
static const uint8_t CAL_SAMPLES = 16;

static const int32_t GOLDEN_DIFF_ABS = 150;
static const int32_t SATURATION_NET = 26000;
static const int32_t LOW_SIGNAL_SUM = 100;
static const int32_t RATIO_SUM_MIN = 50;

static const uint32_t HOST_TIMEOUT_MS = 2000;

// =============================================================================

Adafruit_ADS1115 ads;

static bool ads_ok = false;
static bool laser_is_on = false;
static bool streaming = false;
static uint32_t last_command_ms = 0;
static int32_t sensor_offset = 0;  // Net1 - Net2 hardware bias, RAM only

static void laser_off() {
  digitalWrite(PIN_LASER, LASER_OFF_LEVEL);
  laser_is_on = false;
}

static void laser_on() {
  digitalWrite(PIN_LASER, LASER_ON_LEVEL);
  laser_is_on = true;
}

static bool probe_ads1115() {
  Wire.beginTransmission(ADS1115_ADDR);
  if (Wire.endTransmission() != 0) {
    ads_ok = false;
    return false;
  }
  if (!ads.begin(ADS1115_ADDR)) {
    ads_ok = false;
    return false;
  }
  ads.setGain(ADS_GAIN);
  ads.setDataRate(RATE_ADS1115_860SPS);
  (void)ads.readADC_SingleEnded(0);
  ads_ok = true;
  return true;
}

static int16_t read_ch(uint8_t channel) {
  if (!ads_ok) {
    return 0;
  }
  return ads.readADC_SingleEnded(channel);
}

static void print_ads_status_line() {
  Serial.print(F("ADS1115 Status: "));
  if (ads_ok) {
    Serial.println(F("OK"));
  } else {
    Serial.println(F("ERROR (Check SDA/SCL wiring)"));
  }
}

static void print_help() {
  Serial.println(F("======== ESP32 Scanner Diagnostic Suite ========"));
  Serial.println(F("MCU: ESP32 DevKit"));
  Serial.println(F("ADC: ADS1115 16-bit @ I2C 0x48"));
  Serial.println(F("  SDA=GPIO21  SCL=GPIO22"));
  Serial.println(F("  Sensor1=A0  Sensor2=A1  (LM358 preamp)"));
  Serial.print(F("  Gain: "));
  if (ADS_GAIN == GAIN_ONE) {
    Serial.println(F("GAIN_ONE +/-4.096V (0.125 mV/bit)"));
  } else {
    Serial.println(F("GAIN_TWOTHIRDS +/-6.144V (0.1875 mV/bit)"));
  }
  Serial.println(F("Laser: GPIO25 AO3400 N-MOSFET Active-HIGH"));
  Serial.println(F("  HIGH=ON  LOW=OFF (10k gate pull-down)"));
  Serial.println(F("------------------------------------------------"));
  Serial.println(F("? / H  This help / pinout / status"));
  Serial.println(F("I      I2C & ADS1115 bus scanner"));
  Serial.println(F("L      Toggle laser ON/OFF (aiming)"));
  Serial.println(F("N      Ambient noise & baseline (laser OFF)"));
  Serial.println(F("S      Live alignment stream ~15-20 Hz"));
  Serial.println(F("         Stop: any character, F, or X"));
  Serial.println(F("C      Sensor A0/A1 offset calibration (RAM)"));
  Serial.println(F("R      Python scan pulse: DATA:amb1,amb2,las1,las2"));
  Serial.println(F("F / X  Safe shutdown — laser OFF, idle"));
  Serial.println(F("------------------------------------------------"));
  Serial.print(F("Laser: "));
  Serial.println(laser_is_on ? F("ON") : F("OFF"));
  Serial.print(F("Offset (Net1-Net2): "));
  Serial.println(sensor_offset);
  print_ads_status_line();
  Serial.println(F("================================================"));
}

static void handle_i2c_scan() {
  Serial.println(F("--- I2C / ADS1115 ---"));
  Serial.println(F("Probing 0x48 on SDA=21 SCL=22 ..."));
  Wire.beginTransmission(ADS1115_ADDR);
  uint8_t err = Wire.endTransmission();
  Serial.print(F("I2C endTransmission code: "));
  Serial.println(err);
  if (probe_ads1115()) {
    int16_t a0 = read_ch(0);
    int16_t a1 = read_ch(1);
    Serial.print(F("A0 raw="));
    Serial.print(a0);
    Serial.print(F("  A1 raw="));
    Serial.println(a1);
    print_ads_status_line();
  } else {
    print_ads_status_line();
  }
}

static void handle_laser_toggle() {
  if (laser_is_on) {
    laser_off();
    Serial.println(F("LASER:OFF"));
  } else {
    laser_on();
    Serial.println(F("LASER:ON"));
  }
}

static const char *noise_rating(int32_t pkpk) {
  if (pkpk < 50) {
    return "EXCELLENT";
  }
  if (pkpk < 200) {
    return "GOOD";
  }
  if (pkpk < 500) {
    return "MODERATE";
  }
  return "HIGH NOISE";
}

static void handle_noise() {
  laser_off();
  delay(AMBIENT_SETTLE_MS);

  while (Serial.available() > 0) {
    (void)Serial.read();
  }

  int32_t min1 = 32767, max1 = -32768, sum1 = 0;
  int32_t min2 = 32767, max2 = -32768, sum2 = 0;
  uint8_t n = 0;

  for (uint8_t i = 0; i < NOISE_SAMPLES; i++) {
    if (Serial.available() > 0) {
      break;
    }
    int16_t v1 = read_ch(0);
    int16_t v2 = read_ch(1);
    if (v1 < min1) {
      min1 = v1;
    }
    if (v1 > max1) {
      max1 = v1;
    }
    if (v2 < min2) {
      min2 = v2;
    }
    if (v2 > max2) {
      max2 = v2;
    }
    sum1 += v1;
    sum2 += v2;
    n++;
  }

  if (n == 0) {
    Serial.println(F("Ambient Noise: aborted"));
    return;
  }
  int32_t avg1 = sum1 / (int32_t)n;
  int32_t avg2 = sum2 / (int32_t)n;
  int32_t pk1 = max1 - min1;
  int32_t pk2 = max2 - min2;
  int32_t pk = (pk1 > pk2) ? pk1 : pk2;

  Serial.println(F("--- Ambient Noise (laser OFF, 50 samples) ---"));
  Serial.print(F("A0 Min="));
  Serial.print(min1);
  Serial.print(F(" Max="));
  Serial.print(max1);
  Serial.print(F(" Avg="));
  Serial.print(avg1);
  Serial.print(F(" PkPk="));
  Serial.println(pk1);
  Serial.print(F("A1 Min="));
  Serial.print(min2);
  Serial.print(F(" Max="));
  Serial.print(max2);
  Serial.print(F(" Avg="));
  Serial.print(avg2);
  Serial.print(F(" PkPk="));
  Serial.println(pk2);
  Serial.print(F("Ambient Noise: "));
  Serial.println(noise_rating(pk));
}

static void pulsed_sample(int16_t *amb1, int16_t *amb2, int16_t *las1, int16_t *las2) {
  laser_off();
  delay(AMBIENT_SETTLE_MS);
  *amb1 = read_ch(0);
  *amb2 = read_ch(1);
  laser_on();
  delay(LASER_PULSE_MS);
  *las1 = read_ch(0);
  *las2 = read_ch(1);
  laser_off();
}

static const char *stream_status(int32_t net1, int32_t net2, int32_t diff, int32_t sum) {
  if (net1 > SATURATION_NET || net2 > SATURATION_NET) {
    return "SATURATED";
  }
  if (sum < LOW_SIGNAL_SUM) {
    return "LOW_SIGNAL";
  }
  if (diff < 0) {
    diff = -diff;
  }
  if (diff < GOLDEN_DIFF_ABS) {
    return "GOLDEN_FOCUS";
  }
  return "TRACKING";
}

static void handle_stream() {
  Serial.println(F("--- Live Alignment (any key / F / X to stop) ---"));
  Serial.println(F("Jog Z until Diff~0 = GOLDEN_FOCUS. Trim LM358 to avoid SATURATED.")); 
    while (Serial.available() > 0) {
    (void)Serial.read();
  }
  streaming = true;
  uint32_t next_ms = millis();

  while (streaming) {
    if (Serial.available() > 0) {
      break;
    }
    uint32_t now = millis();
    if ((int32_t)(now - next_ms) < 0) {
      continue;
    }
    next_ms = now + STREAM_PERIOD_MS;

    int16_t amb1, amb2, las1, las2;
    pulsed_sample(&amb1, &amb2, &las1, &las2);

    int32_t net1 = (int32_t)las1 - (int32_t)amb1;
    int32_t net2 = (int32_t)las2 - (int32_t)amb2;
    int32_t diff = (net1 - net2) - sensor_offset;
    int32_t sum = net1 + net2;
    float ratio = 0.0f;
    if (sum > RATIO_SUM_MIN) {
      ratio = (float)diff / (float)sum;
    }
    const char *st = stream_status(net1, net2, diff, sum);

    Serial.printf(
        "Amb1:%d Amb2:%d Net1:%d Net2:%d Diff:%d Ratio:%.3f Status:%s\n",
        (int)amb1,
        (int)amb2,
        (int)net1,
        (int)net2,
        (int)diff,
        ratio,
        st);
  }

  while (Serial.available() > 0) {
    (void)Serial.read();
  }
  laser_off();
  streaming = false;
  Serial.println(F("STREAM:STOP LASER:OFF"));
}

static void handle_calibrate() {
  laser_off();
  int32_t acc = 0;
  uint8_t n = 0;
  for (uint8_t i = 0; i < CAL_SAMPLES; i++) {
    if (Serial.available() > 0) {
      break;
    }
    int16_t amb1, amb2, las1, las2;
    pulsed_sample(&amb1, &amb2, &las1, &las2);
    int32_t net1 = (int32_t)las1 - (int32_t)amb1;
    int32_t net2 = (int32_t)las2 - (int32_t)amb2;
    acc += (net1 - net2);
    n++;
  }
  laser_off();
  if (n == 0) {
    Serial.println(F("CAL:ERROR"));
    return;
  }
  sensor_offset = acc / (int32_t)n;
  Serial.print(F("CAL:offset="));
  Serial.print(sensor_offset);
  Serial.print(F(" samples="));
  Serial.println(n);
  Serial.println(F("Offset stored in RAM (lost on reset). Applied to S-stream Diff."));
}

static void handle_read() {
  int16_t amb1, amb2, las1, las2;
  pulsed_sample(&amb1, &amb2, &las1, &las2);
  Serial.print(F("DATA:"));
  Serial.print(amb1);
  Serial.print(',');
  Serial.print(amb2);
  Serial.print(',');
  Serial.print(las1);
  Serial.print(',');
  Serial.println(las2);
}

static void handle_shutdown() {
  laser_off();
  streaming = false;
  Serial.println(F("OK:F"));
}

static void process_command(char cmd) {
  last_command_ms = millis();
  if (cmd >= 'a' && cmd <= 'z') {
    cmd = (char)(cmd - 'a' + 'A');
  }

  switch (cmd) {
    case '?':
    case 'H':
      print_help();
      break;
    case 'I':
      handle_i2c_scan();
      break;
    case 'L':
      handle_laser_toggle();
      break;
    case 'N':
      handle_noise();
      break;
    case 'S':
      handle_stream();
      break;
    case 'C':
      handle_calibrate();
      break;
    case 'R':
      handle_read();
      break;
    case 'F':
    case 'X':
      handle_shutdown();
      break;
    case '\r':
    case '\n':
    case ' ':
    case '\t':
      break;
    default:
      laser_off();
      Serial.print(F("ERR:BADCMD"));
      Serial.println();
      break;
  }
}

void setup() {
  pinMode(PIN_LASER, OUTPUT);
  laser_off();

  Serial.begin(SERIAL_BAUD);
  Serial.setTimeout(20);

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);

  last_command_ms = millis();

  Serial.println(F("READY"));
  Serial.println(F("ESP32 laser scanner — laser OFF"));
  Serial.println(F("ADS1115 A0/A1 + AO3400 GPIO25 Active-HIGH"));
  Serial.println(F("Type ? or H for diagnostic menu"));

  if (!probe_ads1115()) {
    print_ads_status_line();
  } else {
    print_ads_status_line();
  }
}

void loop() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    process_command(c);
  }

  if (laser_is_on && (millis() - last_command_ms) > HOST_TIMEOUT_MS) {
    laser_off();
    Serial.println(F("ERR:TIMEOUT"));
  }
}
