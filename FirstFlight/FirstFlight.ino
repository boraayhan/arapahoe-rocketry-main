#include <Adafruit_MPU6050.h>
#include <SD.h>
#include <SFE_BMP180.h>

#include <SPI.h>
#include <Wire.h>
int state = 0;
double aX, aY, aZ, wX, wY, wZ, temp, altitude, baseline, pressure;
//acceleration x, y, z, angular velocity x, y, z, rocket internal temperature (from accelerometer), pressure based altitude, ground level pressure, 
//and current pressure (used for altitude calculation)

SFE_BMP180 bmp;
Adafruit_MPU6050 mpu;

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10);
  Serial.println("Adafruit MPU6050 test!");
  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  if (bmp.begin())
    Serial.println("BMP180 init success");
  else {
    Serial.println("BMP180 init fail (disconnected?)\n\n");
  }
  baseline = getPressure();
  delay(100);
}

void loop() {
  // put your main code here, to run repeatedly:
  /* Get new sensor events with the readings */
  PrintInfo();
}

void EvaluateState() {
  switch (state) {
    case 0:  // doing nothing
      Serial.println("Lazy rocket");
      break;
    case 1:  // powered flight
      // code block
      break;
    case 2:  // flight
      // code block
      break;
    case 3:  // falling
      // code block
      break;
  }
}

void GetSensors() {
  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);
  aX = a.acceleration.x;
  aY = a.acceleration.y;
  aZ = a.acceleration.z;
  wX = g.gyro.x;
  wY = g.gyro.y;
  wZ = g.gyro.z;
  temp = t.temperature;
  pressure = getPressure();
  altitude = bmp.altitude(pressure,baseline);
}

void PrintInfo() {
  GetSensors();
  Serial.print("Temperature: ");
  Serial.print(temp);
  Serial.println(" degC");
  Serial.print("Total Acceleration: ");
  Serial.println(pow(pow(aX, 2) + pow(aY, 2) + pow(aZ, 2), .5));
  Serial.print("Relative Altitude (meters): ");
  Serial.println(altitude);
  delay(500);
}

void LogData() {
  int sec = millis() / 1000;
  Serial.print(", seconds: ");
  Serial.print(sec);
  int min = sec / 60;
  Serial.print(", minutes: ");
  Serial.print(min);
}

double getPressure() {
  char status;
  double T, P, p0, a;
  status = bmp.startTemperature();
  if (status != 0) {
    delay(status);
    status = bmp.getTemperature(T);
    if (status != 0) {
      status = bmp.startPressure(3);
      if (status != 0) {
        delay(status);
        status = bmp.getPressure(P, T);
        if (status != 0) {
          return (P);
        }
      }
    }
  }
}
