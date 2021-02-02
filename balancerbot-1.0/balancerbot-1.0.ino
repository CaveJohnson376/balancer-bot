//libs
#include "MPU6050.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

//constants
#define motorLF 26 //left motor, forward
#define motorLB 27 //...backward
#define motorLS 14 //...speed (PWM)
#define motorRF 33 //right...
#define motorRB 25 //...
#define motorRS 32 //...

//vars'n'objects
MPU6050 accelgyro;//our accelerometer+gyro sensor we are going to use

int16_t ax, ay, az;//we only going to use X and maybe Y, as it clearly shows an "UP" direction we gonna need...
int16_t gx, gy, gz;//...and we ain't going to use any of these
//because these only show rotation acceleration, but not "UP" or "DOWN" direction

void setup() {
  //set mode for motor direction control pins
  pinMode(motorLF, OUTPUT);
  pinMode(motorLB, OUTPUT);
  pinMode(motorRF, OUTPUT);
  pinMode(motorRB, OUTPUT);

  //we might want to turn off the motors to not to mess things up
  digitalWrite(motorLF, 0);
  digitalWrite(motorLB, 0);
  digitalWrite(motorRF, 0);
  digitalWrite(motorRB, 0);

  //initialize PWM channels
  ledcSetup(0, 5000, 8);//left
  ledcSetup(1, 5000, 8);//right
  //connect PWM channels to corresponding motor speed control pins
  ledcAttachPin(motorLS, 0);
  ledcAttachPin(motorRS, 1);
  //limit motor speed to 100% of max speed
  ledcWrite(0, 255);
  ledcWrite(1, 255);

  //init i2c interface
  Wire.begin(21, 22);
  //then gyroaccel sensor
  accelgyro.initialize();
  //we ain't gonna use serial interface, because it slow af
  //and it requires us to use WIRES, so we gonna use
  //wifi instead (ofc later)
  //btw, external bluetooth thing is considered because why not
}

void loop() {
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  if (ax < 100) {
    digitalWrite(motorLF, 1);
    digitalWrite(motorLB, 0);
    digitalWrite(motorRF, 1);
    digitalWrite(motorRB, 0);
  } else if (ax > 100) {
    digitalWrite(motorLF, 0);
    digitalWrite(motorLB, 1);
    digitalWrite(motorRF, 0);
    digitalWrite(motorRB, 1);
  } else {
    digitalWrite(motorLF, 0);
    digitalWrite(motorLB, 0);
    digitalWrite(motorRF, 0);
    digitalWrite(motorRB, 0);
  }
}
