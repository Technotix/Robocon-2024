#include <XBOXRECV.h>

// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>

USB Usb;
XBOXRECV Xbox(&Usb);
int ena1=3;
int in1=48;
int in2=49;
int ena2=2;
int in3=39;
int in4=38;
int ena3=5;
int in5=30;
int in6=31;
int ena4=6;
int in7=46;
int in8=47;
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
  int xspeed;
  int yspeed;
  int rspeed;
  int m1speed;
  int m2speed;
  int m3speed;
  int m4speed;
  
  Usb.Task();
  if (Xbox.XboxReceiverConnected) {
    for (uint8_t i = 0; i < 4; i++) {
            if (Xbox.Xbox360Connected[i]) {
              if(Xbox.getAnalogHat(LeftHatX, i) > 7500 || Xbox.getAnalogHat(LeftHatX, i) < -7500 || Xbox.getAnalogHat(LeftHatY, i) > 7500 || Xbox.getAnalogHat(LeftHatY, i) < -7500 || Xbox.getAnalogHat(RightHatX, i) > 7500 || Xbox.getAnalogHat(RightHatX, i) < -7500 || Xbox.getAnalogHat(RightHatY, i) > 7500 || Xbox.getAnalogHat(RightHatY, i) < -7500) {
 {
        xspeed=map(Xbox.getAnalogHat(LeftHatX,i),-32768,32768,-255,255);
        yspeed=map(Xbox.getAnalogHat(LeftHatY,i),-32768,32768,-255,255);
        rspeed=map(Xbox.getAnalogHat(RightHatX,i),-32768,32768,-255,255); 
        m2speed=constrain(xspeed+rspeed+yspeed,-255,255);
        m3speed=constrain(yspeed+rspeed-xspeed,-255,255);
        m4speed=constrain(-yspeed+rspeed-xspeed,-255,255);
        m1speed=constrain(+xspeed+rspeed-yspeed,-255,255);
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
        if(m1speed>0)
        {
          digitalWrite(in1, HIGH);//on
          digitalWrite(in2, LOW);//off,
          analogWrite(ena1, m1speed);// speed of motor
        }
        else if(m1speed<0)
        {
          digitalWrite(in1, LOW);//on
          digitalWrite(in2, HIGH);//off,
          analogWrite(ena1, abs(m1speed));// speed of motor
        }
        else
        {
         digitalWrite(in1, LOW);//on
          digitalWrite(in2, LOW);//off
          analogWrite(ena1,0);
        }
        if(m2speed>0)
        {
          digitalWrite(in3, HIGH);//on
          digitalWrite(in4, LOW);//off,
          analogWrite(ena2, m2speed);// speed of motor
        }
        else if(m2speed<0)
        {
          digitalWrite(in3, LOW);//on
          digitalWrite(in4, HIGH);//off,
          analogWrite(ena2, abs(m2speed));// speed of motor
        }
        else
        {
         digitalWrite(in3, LOW);//on
          digitalWrite(in4, LOW);//off
          analogWrite(ena2,0);
        }
        if(m3speed>0)
        {
          digitalWrite(in5, HIGH);//on
          digitalWrite(in6, LOW);//off,
          analogWrite(ena3, m3speed);// speed of motor
        }
        else if(m3speed<0)
        {
          digitalWrite(in5, LOW);//on
          digitalWrite(in6, HIGH);//off,
          analogWrite(ena3, abs(m3speed));// speed of motor
        }
        else
        {
         digitalWrite(in5, LOW);//on
          digitalWrite(in6, LOW);//off
          analogWrite(ena3,0);
        }
        if(m4speed>0)
        {
          digitalWrite(in7, HIGH);//on
          digitalWrite(in8, LOW);//off,
          analogWrite(ena4, m4speed);// speed of motor
        }
        else if(m4speed<0)
        {
          digitalWrite(in7, LOW);//on
          digitalWrite(in8, HIGH);//off,
          analogWrite(ena4, abs(m4speed));// speed of motor
        }
        else
        {
         digitalWrite(in7, LOW);//on
          digitalWrite(in8, LOW);//off
          analogWrite(ena4,0);
        }
        }
        }
       else
        {
         digitalWrite(in1, LOW);//on
          digitalWrite(in2, LOW);//off
          analogWrite(ena1,0);
         digitalWrite(in3, LOW);//on
          digitalWrite(in4, LOW);//off
          analogWrite(ena2,0);
         digitalWrite(in5, LOW);//on
          digitalWrite(in6, LOW);//off
          analogWrite(ena3,0);
         digitalWrite(in7, LOW);//on
          digitalWrite(in8, LOW);//off
          analogWrite(ena4,0);
              }
              }
      }
    }
  }

