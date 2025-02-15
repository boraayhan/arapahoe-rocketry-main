#define BURNTIME 1000
#define PARACHUTE_DEPLOYMENT_DELAY 1000
#define ERROR 1
#define SERVO1_PIN 1 //for Parachute


#include <Adafruit_MPU6050.h>
#include <SD.h>
#include <SFE_BMP180.h>
#include <Servo.h>
#include <SPI.h>
#include <Wire.h>
//gets fixed when in adneo runner

int state = 0, arm = 1;
double aX, aY, aZ, wX, wY, wZ, temp, altitude, baseline, pressure;
double prevalt1 = 0, prevalt2 = 0, prevalt3 = 0;
double burn_time, initTime = 0, maxHeightTimestamp = 0;
double agX[4] = {0,0,0,0}, agY[4] = {0,0,0,0}, agZ[4] = {0,0,0,0};
double agAvgX, agAvgY, agAvgZ;
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

  initServo();

  delay(100);
}

void EvaluateState() {
  switch (state) {
    case 0:  //On the ground, not armed
      Serial.println("Rocket Initialized");
      //wipe data?
      if(arm == 1)
      {
        for(int i = 0; i < 4; i++){
          GetSensors();
          agX[i] = aX;
          agY[i] = aY;
          agZ[i] = aZ;
          delay(30);
        }
        agAvgX = ((agX[0] + agX[1] + agX[2] + agX[3])/4);
        agAvgY = ((agY[0] + agY[1] + agY[2] + agY[3])/4);
        agAvgZ = ((agZ[0] + agZ[1] + agZ[2] + agZ[3])/4);
        baseline = getPressure();



        state = 1;
      }
      break;
    case 1:  // Armed
      Serial.println("armed");
      ResetAltitude()
      if((pow(aX - agAvgX,2) + pow(aY - agAvgY,2) + pow(aZ - agAvgZ,2)) > 1)
      {
        initTime = millis();
        state = 2;
      }
      break;
    case 2:  // Engine active
       // eng is burning fuel at delta(time)=k
       state = 3;//No way to know if rocket is unpowered right now
      break;
    case 3:  // Non-powered fligh t upward
       //flying and keep stable 
      Serial.println("Engine Ended");
      //pa=pereveas alt, ca=current alt
      if(prevalt3>altitude)
      {
        state = 4;
        maxHeightTimestamp = millis();
      }
      else
      {
        //keep flying      
      }
      break;
    case 4: //Falling without parachut
    //x is alt target to parassute
      if(millis()>maxHeightTimestamp + PARACHUTE_DEPLOYMENT_DELAY)
      {
        
        //turn moter of paratute
      }
      //check alt
      break;
    case 5: //Falling witth parachut
    // keep track of rec and get drive ready
    // fins can guild the rocket still!
      if((a)titude > (prevalt3 - ERROR)) && (altitude < (prevalt3 + ERROR)){
        state = 6;
      }
      break; 
    case 6: //Just land
    //stop rec
      delay(50);
      state = 7;
      break;
    case 7: //ready to turn off
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
  Serial.println(agX);
  Serial.println(agY);
  Serial.println(agZ);
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

void ResetAltitude(pressure) //sets the 0m altitude to current altitude
{
  //alex
  //Use the barometric formula to estimate the altitude: h = (1 - (P / P0)^(1/5.257)) × 44330 Here, 
  // h is the altitude in meters, P is the pressure reading from the barometer
  // P0 is the reference pressure reading at sea level.
  float P0 = 1013.25 millibars (mb);
  altitude = (1 - (pressure / P0)^(1/5.257)) × 44330;
}





void initServo() {
  static Servo servo1;  // Servo for parachute  
  myServo.attach(SERVO1_PIN);  // Attaches the servo to the specified pin
  servo1.write(0);
}
void incrementServo(Servo toChange, int anl) {
  toChange.write(toChange);
}



 
  
  
  
  
  //#include <Servo.h>: Includes the Servo library, necessary for controlling servo motors.
  //Servo myServo;: Creates a Servo object named myServo. You can name it differently if needed.
  //myServo.attach(pin);: Attaches the servo to a specified digital pin (pin).
  //myServo.write(angle);: Sets the servo position to a specified angle (angle), ranging from 0 to 180 degrees.
  //myServo.writeMicroseconds(microseconds);: Controls the servo position by specifying the pulse width in microseconds. Typically, 1000 µs is 0 degrees, 2000 µs is 180 degrees, and 1500 µs is 90 degrees.
  //myServo.read();: Reads the current angle of the servo (the last value written).
  //myServo.attached();: Checks if the servo is attached to a pin, returning true or false.
  //myServo.detach();: Detaches the servo from its pin, disabling control until it's attached again.



void loop() {
  // put your main code here, to run repeatedly:
  /* Get new sensor events with the readings */
  PrintInfo();
  EvaluateState();
  //WriteToCard();
}
