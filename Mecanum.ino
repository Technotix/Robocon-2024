#include <XBOXRECV.h>
#include <Cytron_SmartDriveDuo.h>
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>
#define mdds_2_1 30
#define mdds_4_3 31
#define maxSpeed 35
#define minSpeed 0
int m1_speed ;
int m2_speed ;
int m3_speed ;
int m4_speed ;

Cytron_SmartDriveDuo motor2motor1(SERIAL_SIMPLIFIED, mdds_2_1, 115200);
Cytron_SmartDriveDuo motor4motor3(SERIAL_SIMPLIFIED, mdds_4_3, 115200);

void updateMotors(int x_speed, int y_speed, int r_speed){
  if (x_speed < minSpeed && x_speed > -minSpeed) {
    x_speed = 0;
  }
  if (y_speed < minSpeed && y_speed > -minSpeed) {
    y_speed = 0;
  }
  if (r_speed < minSpeed && r_speed > -minSpeed) {
    r_speed = 0;
  }

  m1_speed = constrain((+x_speed + y_speed - r_speed), -maxSpeed, maxSpeed );
  m2_speed = constrain((-x_speed + y_speed + r_speed), -maxSpeed, maxSpeed );
  m3_speed = constrain((+x_speed + y_speed + r_speed), -maxSpeed, maxSpeed);
  m4_speed = constrain((-x_speed + y_speed - r_speed), -maxSpeed, maxSpeed);
  
  motor2motor1.control(m2_speed, m1_speed);
  motor4motor3.control(m4_speed, m3_speed);

}


USB Usb;
XBOXRECV Xbox(&Usb);
void setup() {
  Serial.begin(115200);
#if !defined(__MIPSEL__)
  while (!Serial)
    ;  // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1)
      ;  //halt
  }
  Serial.print(F("\r\nXbox Wireless Receiver Library Started"));
}

void loop(){
m1_speed = 0;
m2_speed = 0;
m3_speed = 0;
m4_speed = 0;
  Usb.Task();
  if (Xbox.XboxReceiverConnected) {
      int i = 0;
      int x_speed = 0, y_speed = 0, r_speed = 0;
      int leftHatX = Xbox.getAnalogHat(LeftHatX, i);
      int rightHatX = Xbox.getAnalogHat(RightHatX, i);
      int leftHatY = Xbox.getAnalogHat(LeftHatY, i);
      if (leftHatX > 7500) {
        x_speed = map(leftHatX, 7500, 32768, 0, 100);
      } else if (leftHatX < -7500) {
        x_speed = map(leftHatX, -32768, -7500, -100, 0);
      }
      if (leftHatY > 7500) {
        y_speed = map(leftHatY, 7500, 32768, 0, 100);
      } else if (leftHatY < -7500) {
        y_speed = map(leftHatY, -32768, -7500, -100, 0);
      }
      if (rightHatX > 7500) {
        r_speed = map(rightHatX, 7500, 32768, 0, 100);
      } else if (rightHatX < -7500) {
        r_speed = map(rightHatX, -32768, -7500, -100, 0);
      }
      updateMotors(x_speed, y_speed, r_speed);
      Serial.print("  Motor 1: ");
      Serial.print(m1_speed);
      Serial.print("  Motor 2: ");
      Serial.print(m2_speed);
      Serial.print("  Motor 3: ");
      Serial.print(m3_speed);
      Serial.print("  Motor 4: ");
      Serial.println(m4_speed);
  }
}
    
