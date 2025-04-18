//Servo Constants
#define SERVO1_PIN 5 //for Parachute
#define SERVO2_PIN 2 
#define INITPOS1 120 //0 < angle < 180
#define INITPOS2 0
#define FINALPOS1 30 //0 < angle < 180
#define FINALPOS2 90

//Rocket Constants
#define BURNTIME 1000 //in ms
#define PARACHUTE_DEPLOYMENT_DELAY 3000 // Calculate using kinematics after making motor or use altitude check (ie 20 meters)
#define LAUNCH_ACCEL_THRESHOLD 50
#define ALTITUDE_ERROR 1

//Modules and Libraries
#include <Adafruit_MPU6050.h>
#include <SD.h>
#include <SFE_BMP180.h>
#include <Servo.h>
#include <SPI.h>
#include <Wire.h>

int state = 0, arm = 1;
double aX, aY, aZ, wX, wY, wZ, temp, altitude, baseline, pressure;
double prevalt1 = 0, prevalt2 = 0, prevalt3 = 0;
double burn_time, initTime = 0, maxHeightTimestamp = 0;
double agX[4] = {0,0,0,0}, agY[4] = {0,0,0,0}, agZ[4] = {0,0,0,0};
double agMag[4] = {0,0,0,0};

bool record = true;

Servo servo1;
Servo servo2;
//acceleration x, y, z, angular velocity x, y, z, rocket internal temperature (from accelerometer), pressure based altitude, ground level pressure, 
//and current pressure (used for altitude calculation)

SFE_BMP180 bmp;
Adafruit_MPU6050 mpu;
File file;

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

void ReportFail(String cause)
{
  Serial.println("[ERROR]: " + cause + " has failed!");
  //Write "{cause} has failed!" onto data card along with timestamp, etc too
}

void setup(void) {
  Serial.begin(115200);
  initServo();
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
  {
    Serial.println("BMP180 init success");
  }
  else {
    Serial.println("BMP180 init fail (disconnected?)\n\n");
  }
  baseline = getPressure();
  /*if (!SD.begin(4)) {
    ReportFail("SD Hardware");
    while (1); //
  }*/


  delay(100);
}

void EvaluateState() {
  switch (state) {
    case 0:  //On the ground, not armed
      Serial.println("Rocket Initialized. State: 1");
      if(arm == 1)
      {
        for(int i = 0; i < 4; i++){
          GetSensors();
          agX[i] = aX;
          agY[i] = aY;
          agZ[i] = aZ;
          agMag[i] = pow((pow(agX[i],2) + pow(agY[i],2) + pow(agZ[i],2)),0.5);
          delay(200);
        }

        Serial.println("Gravity Acceleration Magnitude :");
        Serial.println((agMag[0] + agMag[1] + agMag[2] + agMag[3])/4);

        baseline = getPressure();
        state = 1;
      }
      break;
    case 1:  // Armed
      Serial.println("Armed! State: 1");
      //ResetAltitude();
      if(((pow(pow(aX, 2) + pow(aY, 2) + pow(aZ, 2), .5)) - ((agMag[0] + agMag[1] + agMag[2] + agMag[3])/4)) > LAUNCH_ACCEL_THRESHOLD)
      {
        initTime = millis();
        state = 2;
      }
      break;
    case 2:  // Engine active
        Serial.println("State: 2");
       // eng is burning fuel at delta(time)=k
      if(millis() > (initTime + BURNTIME))
      {
        state = 3;
      }
      break;
    case 3:  // Non-powered flight upward
       //flying and keep stable 
        Serial.println("State: 3");
      if(prevalt3 > altitude)
      {
        state = 4;
        maxHeightTimestamp = millis();
      }
      break;
    case 4: //Falling without parachute
    //x is alt target to parassute
      Serial.println("State: 4");
      if(millis() > (maxHeightTimestamp + PARACHUTE_DEPLOYMENT_DELAY))
      {
        parachuteOpen();
        state = 5;
      }
      //check alt
      break;
    case 5: //Falling with parachute
    // keep track of rec and get drive ready
    // fins can guild the rocket still!
      Serial.println("State: 5");
      if((altitude > (prevalt3 - ALTITUDE_ERROR)) && (altitude < (prevalt3 + ALTITUDE_ERROR))){
        state = 6;
      }
      break; 
    case 6: //Just landed
      Serial.println("State: 6");
      delay(50);
      state = 7;
      break;
    case 7: //ready to turn off
      Serial.println("State: 7");
      record = false;
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
  prevalt3 = prevalt2;
  prevalt2 = prevalt1;
  prevalt1 = altitude;  
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

void WriteToCard() {
  File file = SD.open("data.csv", FILE_WRITE);
  double entry[10] = {millis()/1000, aX, aY, aZ, altitude, temp, pressure, wX, wY, wZ};
  if (file) {
    for (int i = 0; i < 10; i++) {
      file.print(entry[i]);
      if (i < 9) file.print(",");
    }
    file.println();
    file.close();

    for (int i = 0; i < 10; i++) {
      Serial.print(entry[i]);
      if (i < 9) Serial.print(",");
    }
    Serial.println();
  } else {
    Serial.println("error opening data.csv");
  }
}

void WipeData() //Be careful with this one lol
{
  
}

/* void ResetAltitude(pressure) //sets the 0m altitude to current altitude
{
  //alex
  //Use the barometric formula to estimate the altitude: h = (1 - (P / P0)^(1/5.257)) × 44330 Here, 
  // h is the altitude in meters, P is the pressure reading from the barometer
  // P0 is the reference pressure reading at sea level.
  float P0 = 1013.25 millibars (mb);
  altitude = (1 - (pressure / P0)^(1/5.257)) * 44330;
}*/

void initServo() {
    // Servo for parachute 

  servo1.attach(SERVO1_PIN);  // Attaches the servo to the specified pin
  servo1.write(INITPOS1);


  servo2.attach(SERVO2_PIN);  // Attaches the servo to the specified pin
  servo2.write(INITPOS2);

  delay(10);

  Serial.println(servo1.read());
  Serial.println(servo2.read());
}

void parachuteOpen(){
  servo1.write(FINALPOS1);
  servo2.write(FINALPOS2);
}

void loop() {
  // put your main code here, to run repeatedly:
  /* Get new sensor events with the readings */
  if(record) {
    PrintInfo();
  }
  EvaluateState();
  //WriteToCard();
}