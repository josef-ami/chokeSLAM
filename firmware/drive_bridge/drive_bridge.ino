// ============================================================
// chokeSLAM DRIVE BRIDGE  -  STM32F411CEU6 "Black Pill"   (checkpoint E, #77)
//
// The obstacle-round firmware for the Pi-driven car. The Pi plans and closes
// the path loop; this sketch only EXECUTES and REPORTS:
//
//   STM32 -> Pi   $IMU,<seq>,<t_ms>,<enc>,<yaw>\n   100 Hz  (unchanged, section 9.1)
//                 $STA,<seq_ack>,<status>\n         20 Hz   (status bits, drive_protocol.h)
//   Pi -> STM32   11-byte DRIVE frames (the owner's stm_link.py spec), ~50 Hz
//
//   DIRECT        servo = the road-wheel angle the Pi names (+ = LEFT), mapped
//                 to servo degrees (drive_protocol.h SteerMap -- PLACEHOLDER linear
//                 map until the wheel angle is measured at both stops)
//                 motor = speed loop on the encoder (CLOSED_LOOP flag) or feed-forward only
//   STOP          motor off, servo straight
//   HEADING_HOLD  not used by the Pi; treated as STOP
//   WATCHDOG      no valid DRIVE frame for 250 ms -> motor off, servo straight,
//                 status bit WATCHDOG, until frames come back
//   BUTTON        PB12 to GND (pull-up), latched on the first press, reported in
//                 $STA; the Pi starts the round on it (the one start button, 9.11)
//
// Everything not in drive_protocol.h is the approved IMU bridge (#47-#51):
// same pins, encoder, IMU, $IMU formatting, status LED.
//
// PINOUT
//   encoder PA0/PA1 (TIM5), IMU BNO08x SPI1 (PA5/6/7, CS PA4, INT PB0, RST PB1),
//   motor PA2 fwd / PA3 rev (BTS7960), servo PA8 (500-2500 us), button PB12,
//   LED PC13 (active LOW)
//
// STATUS LED (PC13)
//   solid      frames arriving, motor enabled
//   slow       streaming, no DRIVE frames (waiting for the Pi)
//   fast       IMU fault
//
// NOT COMPILED HERE (no STM32duino toolchain in the sandbox, as for section 14.4);
// drive_protocol.h is host-tested by test_drive_firmware.py.
// ============================================================
#include <Arduino.h>
#include <Servo.h>
#include <SPI.h>
#include <SparkFun_BNO08x_Arduino_Library.h>
#include "drive_protocol.h"

const int MOT_RPWM_PIN = PA2;
const int MOT_LPWM_PIN = PA3;
const int SERVO_PIN    = PA8;
const int IMU_CS_PIN   = PA4;
const int IMU_INT_PIN  = PB0;
const int IMU_RST_PIN  = PB1;
const int BTN_PIN      = PB12;
const int STATUS_LED_PIN = PC13;

const int   SERVO_MIN_PULSE_US = 500;
const int   SERVO_MAX_PULSE_US = 2500;
const float TICKS_PER_MM       = 1.4853f;      // config.ENCODER_TICKS_PER_CM / 10

const uint32_t PERIOD_MS     = 10;             // $IMU 100 Hz, control 100 Hz
const uint32_t STA_PERIOD_MS = 50;             // $STA 20 Hz
const uint16_t IMU_REPORT_MS = 10;
const uint32_t IMU_STALE_MS  = 100;

SPIClass SPI_IMU(PA7, PA6, PA5);
Servo    steeringServo;
BNO08x   imu;

drive::Parser   parser;
drive::Command  cmd;
drive::SteerMap steerMap;
drive::SpeedPI  speedPI;
bool     haveCmd = false;
uint32_t lastCmdMs = 0;
bool     buttonLatched = false;
uint8_t  btnCount = 0;

bool     imuFound = false, haveYaw = false;
float    lastYawDeg = 0.0f;
uint32_t lastYawMs = 0, seq = 0, nextMs = 0, nextStaMs = 0;
int32_t  lastEnc = 0;
float    speedMeas = 0.0f;                      // mm/s, filtered
uint8_t  statusBits = 0;

void initEncoder() {
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_TIM5_CLK_ENABLE();
  GPIO_InitTypeDef g = {0};
  g.Pin = GPIO_PIN_0 | GPIO_PIN_1; g.Mode = GPIO_MODE_AF_PP; g.Pull = GPIO_PULLUP;
  g.Speed = GPIO_SPEED_FREQ_HIGH; g.Alternate = GPIO_AF2_TIM5;
  HAL_GPIO_Init(GPIOA, &g);
  static TIM_HandleTypeDef htim5 = {0};
  TIM_Encoder_InitTypeDef s = {0};
  htim5.Instance = TIM5; htim5.Init.Prescaler = 0; htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 0xFFFFFFFF;
  s.EncoderMode = TIM_ENCODERMODE_TI12;
  s.IC1Polarity = TIM_ICPOLARITY_RISING; s.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  s.IC2Polarity = TIM_ICPOLARITY_RISING; s.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  HAL_TIM_Encoder_Init(&htim5, &s);
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
  TIM5->CNT = 0;
}
int32_t readEncoder() { return -(int32_t)TIM5->CNT; }

void serviceImu() {
  if (!imuFound) return;
  if (imu.wasReset()) imu.enableGameRotationVector(IMU_REPORT_MS);
  while (imu.getSensorEvent()) {
    if (imu.getSensorEventID() != SENSOR_REPORTID_GAME_ROTATION_VECTOR) continue;
    float qi = imu.getQuatI(), qj = imu.getQuatJ(), qk = imu.getQuatK(), qr = imu.getQuatReal();
    if (qi == 0.0f && qj == 0.0f && qk == 0.0f && qr == 0.0f) continue;
    lastYawDeg = atan2f(2.0f * (qi * qj + qr * qk), qr * qr + qi * qi - qj * qj - qk * qk) * (180.0f / PI);
    lastYawMs = millis();
    haveYaw = true;
  }
}
bool imuHealthy() { return imuFound && haveYaw && (millis() - lastYawMs) <= IMU_STALE_MS; }

void sendLine(const char *line, int n) {
  if (n > 0 && Serial && Serial.availableForWrite() >= n) Serial.write((const uint8_t *)line, n);
}

void sendImu() {
  uint32_t t_ms = HAL_GetTick();
  int32_t enc = readEncoder();
  int32_t y100 = (int32_t)lroundf(lastYawDeg * 100.0f);
  uint32_t ay = (uint32_t)(y100 < 0 ? -y100 : y100);
  seq++;
  char line[64];
  int n = snprintf(line, sizeof(line), "$IMU,%lu,%lu,%ld,%s%lu.%02lu\n", (unsigned long)seq,
                   (unsigned long)t_ms, (long)enc, y100 < 0 ? "-" : "", (unsigned long)(ay / 100),
                   (unsigned long)(ay % 100));
  if (n < (int)sizeof(line)) sendLine(line, n);
}

void setMotor(int pwm) {
  if (pwm > 0)      { analogWrite(MOT_RPWM_PIN, pwm); analogWrite(MOT_LPWM_PIN, 0); }
  else if (pwm < 0) { analogWrite(MOT_RPWM_PIN, 0);   analogWrite(MOT_LPWM_PIN, -pwm); }
  else              { analogWrite(MOT_RPWM_PIN, 0);   analogWrite(MOT_LPWM_PIN, 0); }
}

void setServo(float deg) {
  int pulse = (int)((deg / 180.0f) * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US)) + SERVO_MIN_PULSE_US;
  steeringServo.writeMicroseconds(pulse);
}

void readPi() {
  while (Serial.available() > 0) {
    drive::Command c;
    if (parser.feed((uint8_t)Serial.read(), c)) { cmd = c; haveCmd = true; lastCmdMs = millis(); }
  }
}

void control(float dt) {
  int32_t e = readEncoder();
  float v = (e - lastEnc) / TICKS_PER_MM / dt;
  lastEnc = e;
  speedMeas += 0.3f * (v - speedMeas);
  drive::Output o = drive::decide(cmd, haveCmd, millis() - lastCmdMs);
  if (o.motorOn) {
    setServo(steerMap.servoFor(o.wheelDeg));
    setMotor(speedPI.step(o.speedMmps, speedMeas, dt, o.closedLoop));
  } else {
    setServo(steerMap.straight);
    speedPI.step(0.0f, speedMeas, dt, false);
    setMotor(0);
  }
  statusBits = (o.motorOn ? drive::ST_ENABLED : 0) | (o.watchdog ? drive::ST_WATCHDOG : 0) |
               (buttonLatched ? drive::ST_BUTTON : 0) | (imuHealthy() ? drive::ST_IMU_OK : 0);
}

void updateButton() {
  if (digitalRead(BTN_PIN) == LOW) { if (btnCount < 5 && ++btnCount == 5) buttonLatched = true; }
  else btnCount = 0;
}

void updateLed() {
  uint32_t now = millis();
  bool on;
  if (!imuHealthy())                       on = (now / 100) & 1;
  else if (!(statusBits & drive::ST_ENABLED)) on = (now / 500) & 1;
  else                                     on = true;
  digitalWrite(STATUS_LED_PIN, on ? LOW : HIGH);
}

void setup() {
  pinMode(MOT_RPWM_PIN, OUTPUT); digitalWrite(MOT_RPWM_PIN, LOW);
  pinMode(MOT_LPWM_PIN, OUTPUT); digitalWrite(MOT_LPWM_PIN, LOW);
  pinMode(BTN_PIN, INPUT_PULLUP);
  pinMode(STATUS_LED_PIN, OUTPUT); digitalWrite(STATUS_LED_PIN, HIGH);
  steeringServo.attach(SERVO_PIN, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US);
  setServo(steerMap.straight);
  Serial.begin(115200);
  initEncoder();
  SPI_IMU.begin();
  imuFound = imu.beginSPI(IMU_CS_PIN, IMU_INT_PIN, IMU_RST_PIN, 3000000, SPI_IMU);
  if (imuFound) { delay(500); imu.enableGameRotationVector(IMU_REPORT_MS); }
  nextMs = nextStaMs = millis();
}

void loop() {
  serviceImu();
  readPi();
  uint32_t now = millis();
  if ((int32_t)(now - nextMs) >= 0) {
    nextMs += PERIOD_MS;
    if ((int32_t)(now - nextMs) > (int32_t)PERIOD_MS) nextMs = now + PERIOD_MS;
    updateButton();
    control(PERIOD_MS / 1000.0f);
    if (imuHealthy()) sendImu();
  }
  if ((int32_t)(now - nextStaMs) >= 0) {
    nextStaMs += STA_PERIOD_MS;
    char line[32];
    sendLine(line, drive::formatStatus(line, sizeof(line), cmd.seq, statusBits));
  }
  updateLed();
}
