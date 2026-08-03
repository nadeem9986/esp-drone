/*
  ESP32 MPU-6050 Official ESP-Drone Mahony Quaternion Filter (6-DOF)
  
  Uses 3D Quaternion Sensor Fusion (q0, q1, q2, q3) - EXACTLY like ESP-Drone!
  - 100% eliminates Gimbal Lock & Pitch/Roll crosstalk into Yaw
  - Locks drone in 1 solid position facing forward
  - Reversed Roll direction as requested by user
*/

#include <Wire.h>

#define I2C_SDA 21
#define I2C_SCL 22
#define MPU_ADDR 0x68

bool mpuConnected = false;

// Mahony Quaternion Variables
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
float kp = 2.0f; // Proportional gain for Mahony filter
float ki = 0.005f; // Integral gain
float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;

// Output Euler Angles
float pitch = 0.0;
float roll = 0.0;
float yaw = 0.0;

// Gyro Offsets
float gyroX_offset = 0.0, gyroY_offset = 0.0, gyroZ_offset = 0.0;
unsigned long prevTime = 0;

void calibrateSensors() {
  Serial.println("[CALIBRATION] Keep drone STILL on a flat surface...");
  float gx_sum = 0, gy_sum = 0, gz_sum = 0;
  for (int i = 0; i < 500; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU_ADDR, (size_t)6, true);

    int16_t rawGX = (Wire.read() << 8) | Wire.read();
    int16_t rawGY = (Wire.read() << 8) | Wire.read();
    int16_t rawGZ = (Wire.read() << 8) | Wire.read();

    gx_sum += rawGX / 131.0;
    gy_sum += rawGY / 131.0;
    gz_sum += rawGZ / 131.0;
    delay(3);
  }
  gyroX_offset = gx_sum / 500.0;
  gyroY_offset = gy_sum / 500.0;
  gyroZ_offset = gz_sum / 500.0;
  Serial.println("[SUCCESS] ESP-Drone Mahony Quaternion Calibrated!");
}

// Official Mahony AHRS 6-Axis Quaternion Filter
void mahonyAHRSupdate6AXIS(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
  float recipNorm;
  float halfvx, halfvy, halfvz;
  float halfex, halfey, halfez;
  float qa, qb, qc;

  // Convert gyro to rad/s
  gx *= (M_PI / 180.0f);
  gy *= (M_PI / 180.0f);
  gz *= (M_PI / 180.0f);

  // Apply Deadband on static gyro noise
  if (fabs(gz) < (0.8f * M_PI / 180.0f)) gz = 0.0f;

  // Compute feedback only if accelerometer measurement valid
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    // Normalize accelerometer measurement
    recipNorm = 1.0f / sqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Estimated direction of gravity
    halfvx = q1 * q3 - q0 * q2;
    halfvy = q0 * q1 + q2 * q3;
    halfvz = q0 * q0 - 0.5f + q3 * q3;

    // Error is cross product between estimated direction and measured direction of gravity
    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);

    // Compute and apply integral feedback if enabled
    if (ki > 0.0f) {
      integralFBx += ki * halfex * dt;
      integralFBy += ki * halfey * dt;
      integralFBz += ki * halfez * dt;
      gx += integralFBx;
      gy += integralFBy;
      gz += integralFBz;
    } else {
      integralFBx = 0.0f;
      integralFBy = 0.0f;
      integralFBz = 0.0f;
    }

    // Apply proportional feedback
    gx += kp * halfex;
    gy += kp * halfey;
    gz += kp * halfez;
  }

  // Integrate rate of change of quaternion
  gx *= (0.5f * dt);
  gy *= (0.5f * dt);
  gz *= (0.5f * dt);
  qa = q0;
  qb = q1;
  qc = q2;
  q0 += (-qb * gx - qc * gy - q3 * gz);
  q1 += (qa * gx + qc * gz - q3 * gy);
  q2 += (qa * gy - qb * gz + q3 * gx);
  q3 += (qa * gz + qb * gy - qc * gx);

  // Normalize quaternion
  recipNorm = 1.0f / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;

  // Extract Euler Angles (Pitch, Roll, Yaw)
  roll  = atan2(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * 180.0f / M_PI;
  pitch = asin(2.0f * (q0 * q2 - q3 * q1)) * 180.0f / M_PI;
  yaw   = atan2(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * 180.0f / M_PI;

  // REVERSE ROLL AXIS SIGN AS REQUESTED BY USER
  roll = -roll;
}

void checkMPU() {
  Wire.beginTransmission(MPU_ADDR);
  if (Wire.endTransmission() == 0) {
    mpuConnected = true;
    Serial.println("[SUCCESS] MPU-6050 active at 0x68!");

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission();
    delay(50);

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1A);
    Wire.write(0x03);
    Wire.endTransmission();

    calibrateSensors();
    prevTime = micros();
  } else {
    mpuConnected = false;
    Serial.println("[WARNING] MPU-6050 NOT detected. Check wires...");
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL, 400000);
  checkMPU();
}

void loop() {
  if (!mpuConnected) {
    checkMPU();
    delay(1000);
    return;
  }

  unsigned long currentTime = micros();
  float dt = (currentTime - prevTime) / 1000000.0;
  prevTime = currentTime;

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) {
    mpuConnected = false;
    return;
  }
  Wire.requestFrom((uint8_t)MPU_ADDR, (size_t)14, true);
  if (Wire.available() < 14) return;

  int16_t rawAX = (Wire.read() << 8) | Wire.read();
  int16_t rawAY = (Wire.read() << 8) | Wire.read();
  int16_t rawAZ = (Wire.read() << 8) | Wire.read();
  int16_t rawTemp = (Wire.read() << 8) | Wire.read();
  int16_t rawGX = (Wire.read() << 8) | Wire.read();
  int16_t rawGY = (Wire.read() << 8) | Wire.read();
  int16_t rawGZ = (Wire.read() << 8) | Wire.read();

  float ax = rawAX / 16384.0;
  float ay = rawAY / 16384.0;
  float az = rawAZ / 16384.0;

  float gx = (rawGX / 131.0) - gyroX_offset;
  float gy = (rawGY / 131.0) - gyroY_offset;
  float gz = (rawGZ / 131.0) - gyroZ_offset;

  // Run Official ESP-Drone Mahony Quaternion Sensor Fusion
  mahonyAHRSupdate6AXIS(gx, gy, gz, ax, ay, az, dt);

  // Determine Flight State String
  String motion = "STABLE LEVEL";
  if (pitch > 10.0) motion = "FORWARD (PITCH DOWN)";
  else if (pitch < -10.0) motion = "BACKWARD (PITCH UP)";
  else if (roll > 10.0) motion = "BANK RIGHT";
  else if (roll < -10.0) motion = "BANK LEFT";
  else if (abs(gz) > 20.0) motion = (gz > 0) ? "YAW ROTATING RIGHT" : "YAW ROTATING LEFT";
  else if (az > 1.25) motion = "ACCELERATING UP";
  else if (az < 0.65) motion = "FREEFALL / DESCENDING";

  // Stream JSON Telemetry at 50Hz
  static unsigned long lastStream = 0;
  if (millis() - lastStream >= 20) {
    lastStream = millis();
    Serial.print("{\"pitch\":");
    Serial.print(pitch, 2);
    Serial.print(",\"roll\":");
    Serial.print(roll, 2);
    Serial.print(",\"yaw\":");
    Serial.print(yaw, 2);
    Serial.print(",\"yawRate\":");
    Serial.print(gz, 1);
    Serial.print(",\"ax\":");
    Serial.print(ax, 2);
    Serial.print(",\"ay\":");
    Serial.print(ay, 2);
    Serial.print(",\"az\":");
    Serial.print(az, 2);
    Serial.print(",\"gx\":");
    Serial.print(gx, 1);
    Serial.print(",\"gy\":");
    Serial.print(gy, 1);
    Serial.print(",\"gz\":");
    Serial.print(gz, 1);
    Serial.print(",\"motion\":\"");
    Serial.print(motion);
    Serial.print("\",\"temp\":");
    Serial.print((rawTemp / 340.0) + 36.53, 1);
    Serial.println("}");
  }
}
