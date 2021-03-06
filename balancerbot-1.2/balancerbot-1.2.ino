//libs
#include "MPU6050.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

//constants
//swap numbers in there if things are messed up
#define motorLF 26 //left motor, forward
#define motorLB 27 //...backward
#define motorLS 14 //...speed (PWM)
#define motorRF 25 //right...
#define motorRB 33 //...
#define motorRS 32 //...


const float Pk = 4; //Proportional
const float Ik = 0; //Integral
const float Dk = 0.2; //Differential

//vars'n'objects
MPU6050 accelgyro;//our accelerometer+gyro sensor we are going to use

int16_t ax, ay, az;//we only going to use X and maybe Y, as it clearly shows an "UP" direction we gonna need...
int16_t gx, gy, gz;//...and we ain't going to use any of these
//because these only show rotation acceleration, but not "UP" or "DOWN" direction
float error_prev;
float Isum = 0;

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
  ledcSetup(0, 20000, 16);//left
  ledcSetup(1, 20000, 16);//right
  //connect PWM channels to corresponding motor speed control pins
  ledcAttachPin(motorLS, 0);
  ledcAttachPin(motorRS, 1);
  //limit motor speed to 100% of max speed

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

  float motion = PID(ax);

  int PWM = constrain(abs(motion), 0, 65536);

  ledcWrite(0, PWM);
  ledcWrite(1, PWM);
  if (motion <= 0) {
    digitalWrite(motorLF, 1);
    digitalWrite(motorLB, 0);
    digitalWrite(motorRF, 1);
    digitalWrite(motorRB, 0);
  } else {
    digitalWrite(motorLF, 0);
    digitalWrite(motorLB, 1);
    digitalWrite(motorRF, 0);
    digitalWrite(motorRB, 1);
  }
}
//PID regulator function
float PID (int error) {
  //P
  float P = error * Pk;

  //I
  Isum = Isum + error; // Накапливаем (суммируем)
  Isum = constrain(Isum, -65536, 65536); // Проверяем граничные значение
  float I = Ik * Isum;

  //D
  float D = (error - error_prev) * Dk;
  error_prev = error;

  //output
  return P + I + D;
}
