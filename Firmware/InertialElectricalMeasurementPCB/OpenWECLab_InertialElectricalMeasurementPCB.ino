#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <Adafruit_Sensor.h>
#include "esp_timer.h"
#include <math.h>

/* ================= CONFIG I2C ================= */
#define I2C_SDA 23
#define I2C_SCL 22
#define TCA_ADDR 0x70

#define INA1_CHANNEL 0
#define INA2_CHANNEL 2
#define MPU_CHANNEL 6

/* ================= MPU9250 ================= */
#define MPU9250_ADDR 0x68
#define MAG_ADDR     0x0C

/* ================= ENCODER MOTOR 1 ================= */
#define ENCODER_A 19
#define ENCODER_B 18

/* ================= ENCODER MOTOR 2 ================= */
#define Encoder_C3 26   // Señal A motor 2
#define Encoder_C4 25   // Señal B motor 2

const int Numpulso = 410;

/* ================= VARIABLES ENCODERS ================= */
volatile long contador_pulsos = 0;
volatile long contador_pulsos_2 = 0;

volatile float ang_encoder_deg = 0;
volatile float ang_encoder2_deg = 0;

/* ================= OBJETOS ================= */
Adafruit_INA219 ina1(0x40);
Adafruit_INA219 ina2(0x40);

/* ================= MUTEX ================= */
SemaphoreHandle_t i2cMutex;

/* ================= IMU ================= */
float roll = 0, pitch = 0, yaw = 0;

/* ================= ELÉCTRICOS ================= */
volatile float V1 = 0, I1 = 0, P1 = 0;
volatile float V2 = 0, I2 = 0, P2 = 0;

/* ================= MADGWICK ================= */
float q0 = 1, q1 = 0, q2 = 0, q3 = 0;
float beta = 0.1;
unsigned long lastIMUTime;

/* ================= MAG CAL ================= */
float magAdj[3];

/* ================= TCA ================= */
void selectTCA(uint8_t bus) {
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << bus);
  Wire.endTransmission();
}

/* ================= I2C AUX ================= */
void I2CwriteByte(uint8_t addr, uint8_t reg, uint8_t data) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

void I2Cread(uint8_t addr, uint8_t reg, uint8_t n, uint8_t *buf) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(addr, n);
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
}

/* ================= ISR ENCODER MOTOR 1 ================= */
void IRAM_ATTR encoderISR() {
  int A = digitalRead(ENCODER_A);
  int B = digitalRead(ENCODER_B);
  if (A == B) contador_pulsos++;
  else contador_pulsos--;
}

/* ================= ISR ENCODER MOTOR 2 ================= */
void IRAM_ATTR encoderISR_2() {
  int A = digitalRead(Encoder_C3);
  int B = digitalRead(Encoder_C4);
  if (A == B) contador_pulsos_2++;
  else contador_pulsos_2--;
}

/* ================= CORE 0: ENCODERS + INA ================= */
void Task_Encoders_INA(void *pv) {
  while (true) {

    long pulsos1, pulsos2;

    noInterrupts();
    pulsos1 = contador_pulsos;
    pulsos2 = contador_pulsos_2;
    interrupts();

    long mod1 = pulsos1 % Numpulso;
    if (mod1 < 0) mod1 += Numpulso;
    ang_encoder_deg = (mod1 * 360.0) / Numpulso;

    long mod2 = pulsos2 % Numpulso;
    if (mod2 < 0) mod2 += Numpulso;
    ang_encoder2_deg = (mod2 * 360.0) / Numpulso;

    if (xSemaphoreTake(i2cMutex, portMAX_DELAY)) {
      selectTCA(INA1_CHANNEL);
      V1 = ina1.getBusVoltage_V();
      I1 = ina1.getCurrent_mA() ;
      P1 = V1 * I1;

      selectTCA(INA2_CHANNEL);
      V2 = ina2.getBusVoltage_V();
      I2 = ina2.getCurrent_mA();
      P2 = V2 * I2;
      xSemaphoreGive(i2cMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

/* ================= MADGWICK ================= */
void MadgwickUpdate(float ax, float ay, float az,
                    float gx, float gy, float gz,
                    float mx, float my, float mz,
                    float dt) {

  float qDot0 = 0.5 * (-q1*gx - q2*gy - q3*gz);
  float qDot1 = 0.5 * ( q0*gx + q2*gz - q3*gy);
  float qDot2 = 0.5 * ( q0*gy - q1*gz + q3*gx);
  float qDot3 = 0.5 * ( q0*gz + q1*gy - q2*gx);

  q0 += qDot0 * dt;
  q1 += qDot1 * dt;
  q2 += qDot2 * dt;
  q3 += qDot3 * dt;

  float norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 /= norm; q1 /= norm; q2 /= norm; q3 /= norm;
}

/* ================= CORE 1: IMU ================= */
void Task_IMU(void *pv) {
  lastIMUTime = micros();

  while (true) {
    float ax, ay, az, gx, gy, gz, mx, my, mz;

    if (xSemaphoreTake(i2cMutex, portMAX_DELAY)) {
      selectTCA(MPU_CHANNEL);

      uint8_t buf[14];
      I2Cread(MPU9250_ADDR, 0x3B, 14, buf);

      ax = ((int16_t)(buf[0]<<8 | buf[1])) / 2048.0;
      ay = ((int16_t)(buf[2]<<8 | buf[3])) / 2048.0;
      az = ((int16_t)(buf[4]<<8 | buf[5])) / 2048.0;

      gx = ((int16_t)(buf[8]<<8 | buf[9])) * DEG_TO_RAD / 16.4;
      gy = ((int16_t)(buf[10]<<8 | buf[11])) * DEG_TO_RAD / 16.4;
      gz = ((int16_t)(buf[12]<<8 | buf[13])) * DEG_TO_RAD / 16.4;

      uint8_t magBuf[7];
      I2Cread(MAG_ADDR, 0x03, 7, magBuf);

      mx = ((int16_t)(magBuf[1]<<8 | magBuf[0])) * magAdj[0] * 0.15;
      my = ((int16_t)(magBuf[3]<<8 | magBuf[2])) * magAdj[1] * 0.15;
      mz = ((int16_t)(magBuf[5]<<8 | magBuf[4])) * magAdj[2] * 0.15;

      xSemaphoreGive(i2cMutex);
    }

    unsigned long now = micros();
    float dt = (now - lastIMUTime) * 1e-6;
    lastIMUTime = now;

    MadgwickUpdate(ax, ay, az, gx, gy, gz, mx, my, mz, dt);

    roll  = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2)) * RAD_TO_DEG;
    pitch = asin (2*(q0*q2 - q3*q1)) * RAD_TO_DEG;
    yaw   = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3)) * RAD_TO_DEG;

    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

/* ================= CORE 1: LOGGER ================= */
void Task_Logger(void *pv) {
  while (true) {
    Serial.printf(
      "%lld,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
      esp_timer_get_time(),
      roll, pitch, yaw,
      ang_encoder_deg,
      ang_encoder2_deg,
      V1, I1, P1,
      V2, I2, P2
    );
    vTaskDelay(pdMS_TO_TICKS(40));
  }
}

/* ================= SETUP ================= */
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println(
    "angle1_deg,angle2_deg,V1,I1,P1,V2,I2,P2"
  );

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  i2cMutex = xSemaphoreCreateMutex();

  xSemaphoreTake(i2cMutex, portMAX_DELAY);

  selectTCA(INA1_CHANNEL); ina1.begin();
  selectTCA(INA2_CHANNEL); ina2.begin();

  selectTCA(MPU_CHANNEL);
  I2CwriteByte(MPU9250_ADDR, 0x6B, 0x00);
  I2CwriteByte(MPU9250_ADDR, 0x1C, 0x18);
  I2CwriteByte(MPU9250_ADDR, 0x1B, 0x18);
  I2CwriteByte(MPU9250_ADDR, 0x37, 0x02);

  I2CwriteByte(MAG_ADDR, 0x0A, 0x0F);
  uint8_t asa[3];
  I2Cread(MAG_ADDR, 0x10, 3, asa);
  magAdj[0] = ((asa[0] - 128) * 0.5 / 128) + 1.0;
  magAdj[1] = ((asa[1] - 128) * 0.5 / 128) + 1.0;
  magAdj[2] = ((asa[2] - 128) * 0.5 / 128) + 1.0;
  I2CwriteByte(MAG_ADDR, 0x0A, 0x16);

  xSemaphoreGive(i2cMutex);

  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  pinMode(Encoder_C3, INPUT_PULLUP);
  pinMode(Encoder_C4, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Encoder_C3), encoderISR_2, CHANGE);

  xTaskCreatePinnedToCore(Task_Encoders_INA, "Enc+INA", 4096, NULL, 5, NULL, 0);
  xTaskCreatePinnedToCore(Task_IMU,          "IMU",     4096, NULL, 6, NULL, 1);
  xTaskCreatePinnedToCore(Task_Logger,       "Logger",  4096, NULL, 4, NULL, 1);
}

void loop() {}
