#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_BMP085.h>

int state = 0;
double aX, aY, aZ, wX, wY, wZ, temp, altitude;

Adafruit_MPU6050 mpu;

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10);  // will pause Zero, Leonardo, etc until serial console opens

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

  Serial.println("");
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
}

void PrintInfo() {
  double a,P;
  P = getPressure();
  a = pressure.altitude(P,baseline);
  
  Serial.print("relative altitude: ");
  if (a >= 0.0) Serial.print(" "); // add a space for positive numbers
  Serial.print(a,1);
  Serial.print(" meters, ");
    
  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);

  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");

  Serial.print("Temperature: ");
  Serial.print(t.temperature);
  Serial.println(" degC");
  Serial.print("Total Accal: ");
  Serial.println(pow(pow(a.acceleration.x, 2) + pow(a.acceleration.y, 2) + pow(a.acceleration.z, 2), .5));
  Serial.println("");
  delay(500);
}

void LogData() {
  
  int sec = millis() / 1000;
  Serial.print(", seconds: ");
  Serial.print(sec);
  int min = sec / 60 Serial.print(", minutes: "); 
  Serial.print(min); 
}