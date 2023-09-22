/*
 Example sketch for the Xbox Wireless Reciver library - developed by Kristian Lauszus
 It supports up to four controllers wirelessly
 For more information see the blog post: http://blog.tkjelectronics.dk/2012/12/xbox-360-receiver-added-to-the-usb-host-library/ or
 send me an e-mail:  kristianl@tkjelectronics.com
 */

#include <XBOXRECV.h>
int ena1=3;
int in1=48;
int in2=49;
int ena2=2;
int in3=51;
int in4=50;
int ena3=5;
int in5=52;
int in6=53;
int ena4=4;
int in7=46;
int in8=47;

// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>

USB Usb;
XBOXRECV Xbox(&Usb);
void motor1f()
          {
          digitalWrite(in1, HIGH);//on
          digitalWrite(in2, LOW);//off,
          analogWrite(ena1, 100);// speed of motor
          }
void motor2f()
          {
          digitalWrite(in3, HIGH);//on
          digitalWrite(in4, LOW);//off,
          analogWrite(ena2, 100);// speed of motor
          }
void motor3f()
          {
          digitalWrite(in5, HIGH);//on
          digitalWrite(in6, LOW);//off,
          analogWrite(ena3, 100);// speed of motor
          }
void motor4f()
          {
          dsigitalWrite(in7, HIGH);//on
          digitalWrite(in8, LOW);//off,
          analogWrite(ena4, 100);// speed of motor
          }
void motor1r()
          {
          digitalWrite(in1, LOW);//on
          digitalWrite(in2, HIGH);//off,
          analogWrite(ena1, 100);// speed of motor
          }
void motor2r()
          {
          digitalWrite(in3, LOW);//on
          digitalWrite(in4, HIGH);//off,
          analogWrite(ena2, 100);// speed of motor
          }
void motor3r()
          {
          digitalWrite(in5, LOW);//on
          digitalWrite(in6, HIGH);//off,
          analogWrite(ena3, 100);// speed of motor
          }
void motor4r()
          {
          digitalWrite(in7, LOW);//on
          digitalWrite(in8, HIGH);//off,
          analogWrite(ena4, 100);// speed of motor
          }
void motor1s()
          {
          digitalWrite(in1, LOW);//on
          digitalWrite(in2, LOW);//off,
          analogWrite(ena1, 0);// speed of motor
          }
void motor2s()
          {
          digitalWrite(in3, LOW);//on
          digitalWrite(in4, LOW);//off,
          analogWrite(ena2, 0);// speed of motor
          }
void motor3s()
          {
          digitalWrite(in5, LOW);//on
          digitalWrite(in6, LOW);//off,
          analogWrite(ena3, 0);// speed of motor
          }
void motor4s()
          {
          digitalWrite(in7, LOW);//on
          digitalWrite(in8, LOW);//off,
          analogWrite(ena4, 0);// speed of motor
          }
void setup() {
  pinMode(ena1, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(ena2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(ena3, OUTPUT);
  pinMode(in5, OUTPUT);
  pinMode(in6, OUTPUT);
  pinMode(ena4, OUTPUT);
  pinMode(in7, OUTPUT);
  pinMode(in8, OUTPUT);

  
  Serial.begin(115200);
#if !defined(__MIPSEL__)
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); //halt
  }
  Serial.print(F("\r\nXbox Wireless Receiver Library Started"));
}
void loop() {
  
  Usb.Task();
  if (Xbox.XboxReceiverConnected) {
    for (uint8_t i = 0; i < 4; i++) {
      if (Xbox.Xbox360Connected[i]) {
          if (Xbox.getAnalogHat(LeftHatX, i) > 7500 || Xbox.getAnalogHat(LeftHatX, i) < -7500 || Xbox.getAnalogHat(LeftHatY, i) > 7500 || Xbox.getAnalogHat(LeftHatY, i) < -7500 || Xbox.getAnalogHat(RightHatX, i) > 7500 || Xbox.getAnalogHat(RightHatX, i) < -7500 || Xbox.getAnalogHat(RightHatY, i) > 7500 || Xbox.getAnalogHat(RightHatY, i) < -7500) {
          if (Xbox.getAnalogHat(LeftHatX, i) > 7500 || Xbox.getAnalogHat(LeftHatX, i) < -7500 ) {
            Serial.print(F("LeftHatX: "));
            Serial.print(Xbox.getAnalogHat(LeftHatX, i));
            Serial.print("\t");
          }
          if (Xbox.getAnalogHat(LeftHatY, i) > 7500 || Xbox.getAnalogHat(LeftHatY, i) < -7500) {
            Serial.print(F("LeftHatY: "));
            Serial.print(Xbox.getAnalogHat(LeftHatY, i));
            Serial.print("\t");
          }
          Serial.println();
        }

        else if (Xbox.getAnalogHat(LeftHatY, i) > 7500) {
          motor1f();
          motor3f();
          motor2s();
          motor4s();
        }
        else if (Xbox.getAnalogHat(LeftHatY, i) < -7500) {
          motor1r();
          motor3r();
          motor2s();
          motor4s();
        }
        else if (Xbox.getAnalogHat(LeftHatX, i) > 7500) {
          motor2f();
          motor4f();
          motor1s();
          motor3s();
        }
        else if (Xbox.getAnalogHat(LeftHatX, i) < -7500) {
          motor2r();
          motor4r();
          motor1s();
          motor3s();
        }
        else
        {
          motor1s();
          motor2s();
          motor3s();
          motor4s();
        }
      }
    }
  }
}