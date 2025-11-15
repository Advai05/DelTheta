#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

float yaw = 0;              
float gyroZBias = 0;         
float yawOffset = 0;         
unsigned long lastTime = 0;
unsigned long lastPrint = 0; 
const float LEAST_COUNT = 2.0; 
const unsigned long PRINT_INTERVAL = 1000; 

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("Initializing MPU6050...");

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) delay(10);
  }

  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  delay(100);

  Serial.println("MPU6050 initialized successfully!");

  // Calibrate gyro bias
  calibrateGyro();

  // Set initial yaw as zero
  yawOffset = 0;
  lastTime = millis();
  Serial.println("Starting yaw measurement...");
}

void loop() {
  sensors_event_t accelEvent, gyroEvent, tempEvent;
  mpu.getEvent(&accelEvent, &gyroEvent, &tempEvent);

  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0; // seconds
  lastTime = currentTime;

 
  yaw += (gyroEvent.gyro.z - gyroZBias) * dt;


  float yaw_deg = yaw * 180.0 / PI;

  yaw_deg -= yawOffset;


  if (yaw_deg > 360) yaw_deg -= 360;
  if (yaw_deg < 0)   yaw_deg += 360;


  yaw_deg = round(yaw_deg / LEAST_COUNT) * LEAST_COUNT;


  if (currentTime - lastPrint >= PRINT_INTERVAL) {
    Serial.print("Torpedo Tilt Angle: ");
    Serial.print(yaw_deg, 1); // 1 decimal place
    Serial.println("°");
    lastPrint = currentTime;
  }

  delay(20); }

// ---------------- Gyro calibration function ----------------
void calibrateGyro() {
  sensors_event_t accelEvent, gyroEvent, tempEvent;
  int samples = 500;
  float sum = 0;

  Serial.println("Keep sensor stationary for a few seconds!");
  delay(2000);

  for (int i = 0; i < samples; i++) {
    mpu.getEvent(&accelEvent, &gyroEvent, &tempEvent);
    sum += gyroEvent.gyro.z;
    delay(5);
  }

  gyroZBias = sum / samples;
  Serial.print("Gyro Z bias: "); Serial.println(gyroZBias, 6);
  Serial.println("Calibration complete.");
}
