// ============================================================
// chokeSLAM STM32 IMU BRIDGE  -  STM32F411CEU6 "Black Pill"
//
// Sensor bridge only (decision #47): streams the BNO08x heading and the
// drive-motor hall encoder to the Raspberry Pi over native USB (CDC,
// /dev/ttyACM*), in the format agreed in docs/CHANGES.md section 9.1:
//
//     $IMU,<seq>,<t_ms>,<enc>,<yaw>\n        100 Hz
//
//   seq   uint32  +1 every line (a line skipped because USB was busy still
//                 uses up its seq, so the Pi counts it as lost)
//   t_ms  uint32  HAL_GetTick() when sampled
//   enc   int32   cumulative encoder count since power-on, forward = +,
//                 never reset
//   yaw   float   Game Rotation Vector yaw in degrees, 2 decimals, AS THE
//                 CHIP REPORTS IT: no zeroing, no sign flip. The Pi applies
//                 IMU_YAW_SIGN = -1 ("clockwise reads negative").
//
// Nothing else goes over USB: no '#' debug lines (decision #49). Status is
// shown on the Black Pill's on-board LED (PC13) only (decision #51).
//
// The robot does NOT drive under this firmware: the motor is held off and
// the steering servo is held straight. Push or place the robot by hand.
//
// PINOUT (taken from the open-round sketch, bench-verified there)
//   encoder   PA0 / PA1   TIM5 encoder mode TI12, AF2, pull-ups; negated so
//                         driving forward counts up
//   IMU       BNO08x on SPI1: SCK PA5, MISO PA6, MOSI PA7,
//             CS PA4, INT PB0, RST PB1, 3 MHz
//   motor     PA2 forward, PA3 reverse      -> held LOW (off)
//   servo     PA8, 500-2500 us, straight 76.5 -> held straight
//   status    PC13  on-board LED, lit when the pin is LOW (see below)
//   LED1/2/3  PB12 / PB13 / PB14  held off
//   USB       PA11 / PA12 (native USB FS)
//   unused    PB6/PB7 (I2C colour sensor), PB8 (TCA reset), PB15 (button)
//             are left untouched.
//   NOTE: the Black Pill's on-board KEY button is also wired to PA0
//         (encoder channel A). Don't press it while running.
//
// STATUS LED (on-board, PC13)
//   solid          streaming: IMU reports arriving, lines going out
//   slow (500 ms)  IMU fine, but lines can't go out (Pi not connected /
//                  not reading)
//   fast (100 ms)  IMU fault: not found at boot, or no report for 100 ms
//
// ARDUINO IDE (STM32duino core)
//   Board:        Generic STM32F4 series -> Board part number: BlackPill F411CE
//   USB support:  CDC (generic 'Serial' supersede U(S)ART)
//   Upload:       STM32CubeProgrammer (DFU)  [hold BOOT0, tap NRST]
//   Libraries:    SparkFun BNO08x Cortex Based IMU (SparkFun_BNO08x_Arduino_Library),
//                 Servo (bundled with the core)
// ============================================================
#include <Arduino.h>
#include <Servo.h>
#include <SPI.h>
#include <SparkFun_BNO08x_Arduino_Library.h>

// ---- pins ----
const int MOT_RPWM_PIN = PA2;
const int MOT_LPWM_PIN = PA3;
const int SERVO_PIN    = PA8;
const int IMU_CS_PIN   = PA4;
const int IMU_INT_PIN  = PB0;
const int IMU_RST_PIN  = PB1;
const int LED1_PIN     = PB12;
const int LED2_PIN     = PB13;
const int LED3_PIN     = PB14;
const int STATUS_LED_PIN = PC13;             // on-board LED, active LOW

// ---- servo (from the open-round sketch) ----
const int   SERVO_MIN_PULSE_US  = 500;
const int   SERVO_MAX_PULSE_US  = 2500;
const float SERVO_TRUE_STRAIGHT = 76.5;

// ---- stream ----
const uint32_t PERIOD_MS          = 10;    // 100 Hz
const uint16_t IMU_REPORT_MS      = 10;    // Game Rotation Vector interval
const uint32_t IMU_STALE_MS       = 100;   // no report for this long = IMU fault
const uint32_t HOST_OK_MS         = 200;   // a line went out within this = streaming

SPIClass SPI_IMU(PA7, PA6, PA5);           // MOSI, MISO, SCLK
Servo    steeringServo;
BNO08x   imu;

bool     imuFound    = false;
bool     haveYaw     = false;
float    lastYawDeg  = 0.0f;
uint32_t lastYawMs   = 0;
uint32_t seq         = 0;
uint32_t nextSendMs  = 0;
uint32_t lastSentMs  = 0;

// ------------------------------------------------------------
// encoder: TIM5 on PA0/PA1, exactly as in the open-round sketch
// ------------------------------------------------------------
void initEncoder() {
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_TIM5_CLK_ENABLE();
  GPIO_InitTypeDef g = {0};
  g.Pin       = GPIO_PIN_0 | GPIO_PIN_1;
  g.Mode      = GPIO_MODE_AF_PP;
  g.Pull      = GPIO_PULLUP;
  g.Speed     = GPIO_SPEED_FREQ_HIGH;
  g.Alternate = GPIO_AF2_TIM5;
  HAL_GPIO_Init(GPIOA, &g);

  static TIM_HandleTypeDef htim5 = {0};
  TIM_Encoder_InitTypeDef s = {0};
  htim5.Instance         = TIM5;
  htim5.Init.Prescaler   = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period      = 0xFFFFFFFF;
  s.EncoderMode  = TIM_ENCODERMODE_TI12;
  s.IC1Polarity  = TIM_ICPOLARITY_RISING;
  s.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  s.IC2Polarity  = TIM_ICPOLARITY_RISING;
  s.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  HAL_TIM_Encoder_Init(&htim5, &s);
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
  TIM5->CNT = 0;
}

// forward = +; never reset after boot (a lost line loses no distance)
int32_t readEncoder() { return -(int32_t)TIM5->CNT; }

// ------------------------------------------------------------
// IMU: yaw from the Game Rotation Vector quaternion (same formula as the
// open-round sketch), raw - no offset, no sign
// ------------------------------------------------------------
void serviceImu() {
  if (!imuFound) return;
  if (imu.wasReset()) imu.enableGameRotationVector(IMU_REPORT_MS);
  while (imu.getSensorEvent()) {
    if (imu.getSensorEventID() != SENSOR_REPORTID_GAME_ROTATION_VECTOR) continue;
    float qi = imu.getQuatI(), qj = imu.getQuatJ(), qk = imu.getQuatK(), qr = imu.getQuatReal();
    if (qi == 0.0f && qj == 0.0f && qk == 0.0f && qr == 0.0f) continue;   // not a real reading
    lastYawDeg = atan2f(2.0f * (qi * qj + qr * qk), qr * qr + qi * qi - qj * qj - qk * qk) * (180.0f / PI);
    lastYawMs  = millis();
    haveYaw    = true;
  }
}

bool imuHealthy() { return imuFound && haveYaw && (millis() - lastYawMs) <= IMU_STALE_MS; }

// ------------------------------------------------------------
// one line, formatted without printf-float (not enabled in nano libc)
// ------------------------------------------------------------
void sendSample() {
  uint32_t t_ms = HAL_GetTick();
  int32_t  enc  = readEncoder();
  int32_t  y100 = (int32_t)lroundf(lastYawDeg * 100.0f);
  const char *sign = (y100 < 0) ? "-" : "";
  uint32_t ay = (uint32_t)(y100 < 0 ? -y100 : y100);

  seq++;                                     // used even if the line is skipped
  char line[64];
  int n = snprintf(line, sizeof(line), "$IMU,%lu,%lu,%ld,%s%lu.%02lu\n",
                   (unsigned long)seq, (unsigned long)t_ms, (long)enc, sign,
                   (unsigned long)(ay / 100), (unsigned long)(ay % 100));
  if (n <= 0 || n >= (int)sizeof(line)) return;
  // never block: if the Pi isn't reading, drop this line (the seq gap shows it)
  if (Serial && Serial.availableForWrite() >= n) {
    Serial.write((const uint8_t *)line, n);
    lastSentMs = millis();
  }
}

// ------------------------------------------------------------
void updateLed() {
  uint32_t now = millis();
  bool on;
  if (!imuHealthy())                   on = (now / 100) & 1;   // fast: IMU fault
  else if (now - lastSentMs > HOST_OK_MS) on = (now / 500) & 1; // slow: no host
  else                                 on = true;              // solid: streaming
  digitalWrite(STATUS_LED_PIN, on ? LOW : HIGH);   // active LOW
}

void setup() {
  // safe state first: motor off, servo straight, colour LEDs off
  pinMode(MOT_RPWM_PIN, OUTPUT);  digitalWrite(MOT_RPWM_PIN, LOW);
  pinMode(MOT_LPWM_PIN, OUTPUT);  digitalWrite(MOT_LPWM_PIN, LOW);
  pinMode(LED1_PIN, OUTPUT);      digitalWrite(LED1_PIN, LOW);
  pinMode(LED2_PIN, OUTPUT);      digitalWrite(LED2_PIN, LOW);
  pinMode(LED3_PIN, OUTPUT);      digitalWrite(LED3_PIN, LOW);
  pinMode(STATUS_LED_PIN, OUTPUT); digitalWrite(STATUS_LED_PIN, HIGH);   // off
  steeringServo.attach(SERVO_PIN, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US);
  steeringServo.writeMicroseconds(
      (int)((SERVO_TRUE_STRAIGHT / 180.0f) * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US)) + SERVO_MIN_PULSE_US);

  Serial.begin(115200);            // USB CDC: the baud rate is ignored

  initEncoder();

  SPI_IMU.begin();
  imuFound = imu.beginSPI(IMU_CS_PIN, IMU_INT_PIN, IMU_RST_PIN, 3000000, SPI_IMU);
  if (imuFound) {
    delay(500);
    imu.enableGameRotationVector(IMU_REPORT_MS);
  }
  nextSendMs = millis();
}

void loop() {
  serviceImu();

  // 100 Hz, paced by the clock; a late loop catches up without bursting
  uint32_t now = millis();
  if ((int32_t)(now - nextSendMs) >= 0) {
    nextSendMs += PERIOD_MS;
    if ((int32_t)(now - nextSendMs) > (int32_t)PERIOD_MS) nextSendMs = now + PERIOD_MS;
    // only real headings go out: with the IMU stale the Pi sees no lines
    // (link STALE) instead of a frozen yaw it would integrate as "not turning"
    if (imuHealthy()) sendSample();
  }
  updateLed();
}
