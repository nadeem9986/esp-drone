/*
  ESP32 + MPU-6050 RC Drift Car Gyro Steering Stabilizer
  
  How it works:
  - Measures car yaw rate (Z-axis rotation speed in deg/s).
  - Automatically calculates Counter-Steering angle to keep the car drifting smoothly
    without spinning out (360 spin).
  - Outputs PWM to Steering Servo on GPIO 13.
  
  Connections:
    MPU-6050 VCC -> ESP32 3V3
    MPU-6050 GND -> ESP32 GND
    MPU-6050 SDA -> ESP32 GPIO 21
    MPU-6050 SCL -> ESP32 GPIO 22
    Steering Servo Signal -> ESP32 GPIO 13
*/

#include <Wire.h>
#include <ESP32Servo.h>

#define I2C_SDA 21
#define I2C_SCL 22
#define MPU_ADDR 0x68
#define SERVO_PIN 13

Servo steeringServo;

// Gyro settings
float gyroZ_offset = 0.0;
float yawAngle = 0.0;
unsigned long prevMicros = 0;

// RC Drift Settings
int driverSteering = 90; // Center position (0-180 deg)
float gyroGain = 0.35;    // Gyro assist sensitivity (35% counter-steer strength)
int servoOutput = 90;

bool mpuOK = false;

void calibrateYawGyro() {
  Serial.println("[CALIBRATING] Hold car STILL to calibrate Gyro...");
  float gz_sum = 0;
  for (int i = 0; i < 300; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x47); // Gyro Z High Byte
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU_ADDR, (size_t)2, true);
    
    int16_t rawGZ = (Wire.read() << 8) | Wire.read();
    gz_sum += rawGZ / 131.0;
    delay(3);
  }
  gyroZ_offset = gz_sum / 300.0;
  Serial.print("[SUCCESS] Gyro Z Offset: ");
  Serial.println(gyroZ_offset);
}

void setup() {
  Serial.begin(115200);
  
  // Attach Steering Servo
  steeringServo.attach(SERVO_PIN, 500, 2500);
  steeringServo.write(90); // Center front wheels

  Wire.begin(I2C_SDA, I2C_SCL, 400000);

  // Initialize MPU-6050
  Wire.beginTransmission(MPU_ADDR);
  if (Wire.endTransmission() == 0) {
    mpuOK = true;
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B);
    Wire.write(0x00); // Wake up
    Wire.endTransmission();
    
    calibrateYawGyro();
    prevMicros = micros();
  } else {
    Serial.println("[ERROR] MPU-6050 not found at 0x68!");
  }
}

void loop() {
  if (!mpuOK) {
    delay(1000);
    return;
  }

  // Calculate DT
  unsigned long now = micros();
  float dt = (now - prevMicros) / 1000000.0;
  prevMicros = now;

  // Read Gyro Z (Yaw rate) and Accel X/Y
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (size_t)14, true);

  if (Wire.available() >= 14) {
    int16_t rawAX = (Wire.read() << 8) | Wire.read();
    int16_t rawAY = (Wire.read() << 8) | Wire.read();
    int16_t rawAZ = (Wire.read() << 8) | Wire.read();
    Wire.read(); Wire.read(); // Skip temp
    int16_t rawGX = (Wire.read() << 8) | Wire.read();
    int16_t rawGY = (Wire.read() << 8) | Wire.read();
    int16_t rawGZ = (Wire.read() << 8) | Wire.read();

    float gz = (rawGZ / 131.0) - gyroZ_offset; // deg/s
    float ay = rawAY / 16384.0; // Lateral G-force (sideways slide)

    // Integrate Yaw Angle
    yawAngle += gz * dt;

    // COUNTER-STEER FORMULA:
    // CounterSteer = DriverInput - (Gain * YawRate)
    float correction = gz * gyroGain;
    servoOutput = constrain(driverSteering - (int)correction, 30, 150);

    // Apply correction to front wheels steering servo
    steeringServo.write(servoOutput);

    // Stream telemetry to Web Dashboard
    static unsigned long lastLog = 0;
    if (millis() - lastLog >= 20) { // 50Hz update
      lastLog = millis();
      Serial.print("{\"yawRate\":");
      Serial.print(gz, 1);
      Serial.print(",\"yawAngle\":");
      Serial.print(yawAngle, 1);
      Serial.print(",\"latG\":");
      Serial.print(ay, 2);
      Serial.print(",\"servo\":");
      Serial.print(servoOutput);
      Serial.print(",\"driver\":");
      Serial.print(driverSteering);
      Serial.println("}");
    }
  }

  delay(5);
}
