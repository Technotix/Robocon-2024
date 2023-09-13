#include <Cytron_SmartDriveDuo.h>
//#include "CytronMotorDriver.h"
#include <HardwareSerial.h>
#include <SPI.h>
#include <Wire.h>
#include <XBOXRECV.h>

// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
USB Usb;
XBOXRECV Xbox(&Usb);
int in1=36;
int in2=46;
Cytron_SmartDriveDuo motor2motor1(SERIAL_SIMPLIFIED, in1, 115200);
Cytron_SmartDriveDuo motor4motor3(SERIAL_SIMPLIFIED, in2, 115200);
int xspeed;
int yspeed;
int rspeed;
int m1speed;
int m2speed;
int m3speed;
int m4speed;
void setup() 
{
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    Serial.begin(115200);
    #if !defined(__MIPSEL__)
    while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
    #endif
    if (Usb.Init() == -1) 
    {
        Serial.print(F("\r\nOSC did not start"));
        while (1); //halt
    }
  Serial.print(F("\r\nXbox Wireless Receiver Library Started"));
}
void loop() 
{


   Usb.Task();
    if (Xbox.XboxReceiverConnected) 
    {
      for (uint8_t i = 0; i < 4; i++) 
      {
          if (Xbox.Xbox360Connected[i]) 
          {
              if(Xbox.getAnalogHat(LeftHatX, i) > 7500 || Xbox.getAnalogHat(LeftHatX, i) < -7500 || Xbox.getAnalogHat(LeftHatY, i) > 7500 || Xbox.getAnalogHat(LeftHatY, i) < -7500 || Xbox.getAnalogHat(RightHatX, i) > 7500 || Xbox.getAnalogHat(RightHatX, i) < -7500 || Xbox.getAnalogHat(RightHatY, i) > 7500 || Xbox.getAnalogHat(RightHatY, i) < -7500) {
              {
                  xspeed=map(Xbox.getAnalogHat(LeftHatX,i),-32768,32768,-255,255);
                  yspeed=map(Xbox.getAnalogHat(LeftHatY,i),-32768,32768,-255,255);
                  rspeed=map(Xbox.getAnalogHat(RightHatX,i),-32768,32768,-255,255); 
                  m2speed=constrain(xspeed-rspeed+yspeed,-50,50);
                  m3speed=constrain(yspeed-rspeed-xspeed,-50,50);
                  m4speed=constrain(-yspeed-rspeed-xspeed,-50,50);
                  m1speed=constrain(+xspeed-rspeed-yspeed,-50,50);
                  motor2motor1.control(m2speed, m1speed);
                  motor4motor3.control(m4speed, m3speed);
        /*Serial.print("Xspeed=");
        Serial.print(xspeed);
        Serial.print(" Yspeed=");
        Serial.print(yspeed);
        Serial.print(" Rspeed=");
        Serial.print(rspeed);
        */
                  Serial.print("m1speed=");
                  Serial.print(m1speed);
                  Serial.print(" m2speed=");
                  Serial.print(m2speed);
                  Serial.print(" m3speed=");
                  Serial.print(m3speed);
                  Serial.println(" m4speed=");
                  Serial.print(m4speed);
                  Serial.println();
                  motor2motor1.control(m2speed, m1speed);
                  motor4motor3.control(m4speed, m3speed);
                  }
              }
              else
              {
                  motor2motor1.control(0, 0);
                  motor4motor3.control(0, 0);
              }
          }
        }
    }
  }

