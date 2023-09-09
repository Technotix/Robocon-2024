#include <XBOXRECV.h>

// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>
#define in1_1 48
#define in1_2 49
#define pwm_1 3
#define in2_1 39
#define in2_2 38
#define pwm_2 2
#define in3_1 30
#define in3_2 31
#define pwm_3 5
#define in4_1 46
#define in4_2 47
#define pwm_4 6

int m1_speed = 0;
int m2_speed = 0;
int m3_speed = 0;
int m4_speed = 0;



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
  pinMode(in1_1, OUTPUT);
  pinMode(in1_2, OUTPUT);
  pinMode(pwm_1, OUTPUT);
  pinMode(in2_1, OUTPUT);
  pinMode(in2_2, OUTPUT);
  pinMode(pwm_2, OUTPUT);
  pinMode(in3_1, OUTPUT);
  pinMode(in3_2, OUTPUT);
  pinMode(pwm_3, OUTPUT);
  pinMode(in4_1, OUTPUT);
  pinMode(in4_2, OUTPUT);
  pinMode(pwm_4, OUTPUT);
}
void loop() {
  Usb.Task();
  if (Xbox.XboxReceiverConnected) {
    for (uint8_t i = 0; i < 4; i++) {
      int x_speed = 0, y_speed = 0, r_speed = 0;
      int LeftHatX = Xbox.getAnalogHat(LeftHatX, i);
      int RightHatX = Xbox.getAnalogHat(RightHatX, i);
      int LeftHatY = Xbox.getAnalogHat(LeftHatY, i);
      if (LeftHatX > 7500) {
        x_speed = map(Xbox.getAnalogHat(LeftHatX, i), 7500, 32768, 0, 255);
      } else if (LeftHatX < -7500) {
        x_speed = map(Xbox.getAnalogHat(LeftHatX, i), -32768, 7500, -255, 0);
      }
      if (LeftHatY > 7500) {
        y_speed = map(Xbox.getAnalogHat(LeftHatY, i), 7500, 32768, 0, 255);
      } else if (LeftHatY < -7500) {
        y_speed = map(Xbox.getAnalogHat(LeftHatY, i), -32768, 7500, -255, 0);
      }
      if (RightHatX > 7500) {
        r_speed = map(Xbox.getAnalogHat(RightHatX, i), 7500, 32768, 0, 255);
      } else if (RightHatX < -7500) {
        r_speed = map(Xbox.getAnalogHat(RightHatX, i), -32768, 7500, -255, 0);
      }
      
      m1_speed = constrain((-x_speed + y_speed + r_speed), -255, 255);
      m2_speed = constrain((-x_speed - y_speed + r_speed), -255, 255);
      m3_speed = constrain((x_speed - y_speed + r_speed), -255, 255);
      m4_speed = constrain((x_speed + y_speed + r_speed), -255, 255);

      if (m1_speed > 0) {
        digitalWrite(in1_1, HIGH);
        digitalWrite(in1_2, LOW);
        analogWrite(pwm_1, m1_speed);
        Serial.println(m1_speed);
      } else if (m1_speed < 0) {
        digitalWrite(in1_1, LOW);
        digitalWrite(in1_2, HIGH);
        analogWrite(pwm_1, -m1_speed);
        Serial.println(m1_speed);
      } else {
        digitalWrite(in1_1, LOW);
        digitalWrite(in1_2, LOW);
        analogWrite(pwm_1, 0);
        Serial.println(m1_speed);
      }

      if (m2_speed > 0) {
        digitalWrite(in2_1, HIGH);
        digitalWrite(in2_2, LOW);
        analogWrite(pwm_2, m2_speed);
        Serial.println(m2_speed);
      } else if (m2_speed < 0) {
        digitalWrite(in2_1, LOW);
        digitalWrite(in2_2, HIGH);
        analogWrite(pwm_2, -m2_speed);
        Serial.println(m2_speed);
      } else {
        digitalWrite(in2_1, LOW);
        digitalWrite(in2_2, LOW);
        analogWrite(pwm_2, 0);
        Serial.println(m2_speed);
      }

      if (m3_speed > 0) {
        digitalWrite(in3_1, HIGH);
        digitalWrite(in3_2, LOW);
        analogWrite(pwm_3, m3_speed);
        Serial.println(m3_speed);
      } else if (m3_speed < 0) {
        digitalWrite(in3_1, LOW);
        digitalWrite(in3_2, HIGH);
        analogWrite(pwm_3, -m3_speed);
        Serial.println(m3_speed);
      } else {
        digitalWrite(in3_1, LOW);
        digitalWrite(in3_2, LOW);
        analogWrite(pwm_3, 0);
        Serial.println(m3_speed);
      }

      if (m4_speed > 0) {
        digitalWrite(in4_1, HIGH);
        digitalWrite(in4_2, LOW);
        analogWrite(pwm_4, m4_speed);
        Serial.println(m4_speed);
      } else if (m4_speed < 0) {
        digitalWrite(in4_1, LOW);
        digitalWrite(in4_2, HIGH);
        analogWrite(pwm_4, -m4_speed);
        Serial.println(m4_speed);
      } else {
        digitalWrite(in4_1, LOW);
        digitalWrite(in4_2, LOW);
        analogWrite(pwm_4, 0);
        Serial.println(m4_speed);
      }
    }
  } else {
    m1_speed = 0;
    m2_speed = 0;
    m3_speed = 0;
    m4_speed = 0;
  }
}
