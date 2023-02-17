#include <WiFiManager.h>  // 引用「WiFi管理員」程式庫
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_I2CDevice.h>
#include "HUSKYLENS.h"
#include "SoftwareSerial.h"
HUSKYLENS huskylens;
//HUSKYLENS green line >> SDA; blue line >> SCL
void printResult(HUSKYLENSResult result);


int dataPiny = 13;   // Latch pin (STCP腳位)
int latchPiny = 12;  // Clock pin (SHCP腳位)
int clockPiny = 14;  // Data pin (DS腳位)
int dataPinx = 27;   // Latch pin (STCP腳位)
int latchPinx = 26;  // Clock pin (SHCP腳位)
int clockPinx = 25;  // Data pin (DS腳位)

#define limitXHIGHRIGHT 17
#define limitXLOWRIGHT 16

#define limitYLOWLEFT 19
#define limitYLOWRIGHT 18


#define xstepdist 2200
#define ystepdist 1150


// Current time

void xmoveleft() {
  for (int i = 0; i < xstepdist; i++) {
    digitalWrite(latchPinx, LOW);  // x軸向左
    shiftOut(dataPinx, clockPinx, LSBFIRST, B11110100);
    digitalWrite(latchPinx, HIGH);
    delay(0.01);
    digitalWrite(latchPinx, LOW);
    shiftOut(dataPinx, clockPinx, LSBFIRST, B11010000);
    digitalWrite(latchPinx, HIGH);
    delay(0.01);
  }

  Serial.println("xmoveleft");
}
void xmoveright() {
  for (int i = 0; i < xstepdist; i++) {
    digitalWrite(latchPinx, LOW);  // x軸向右
    shiftOut(dataPinx, clockPinx, LSBFIRST, B10111100);
    digitalWrite(latchPinx, HIGH);
    delay(0.01);
    digitalWrite(latchPinx, LOW);
    shiftOut(dataPinx, clockPinx, LSBFIRST, B10011000);
    digitalWrite(latchPinx, HIGH);
    delay(0.01);
  }
  Serial.println("xmoveright");
}
void ymoveup() {
  for (int i = 0; i < ystepdist; i++) {
    digitalWrite(latchPiny, LOW);  // y軸向上
    shiftOut(dataPiny, clockPiny, LSBFIRST, B10111100);
    digitalWrite(latchPiny, HIGH);
    delay(0.01);
    digitalWrite(latchPiny, LOW);
    shiftOut(dataPiny, clockPiny, LSBFIRST, B10011000);
    digitalWrite(latchPiny, HIGH);
    delay(0.01);
  }

  Serial.println("ymoveup");
}
void ymovedown() {
  for (int i = 0; i < ystepdist; i++) {
    digitalWrite(latchPiny, LOW);  // y軸向下
    shiftOut(dataPiny, clockPiny, LSBFIRST, B11110100);
    digitalWrite(latchPiny, HIGH);
    delay(0.01);
    digitalWrite(latchPiny, LOW);
    shiftOut(dataPiny, clockPiny, LSBFIRST, B11010000);
    digitalWrite(latchPiny, HIGH);
    delay(0.01);
  }

  Serial.println("ymovedown");
}
void reset() {
  do {
    xmoveright();
  } while (digitalRead(limitXLOWRIGHT) == HIGH || digitalRead(limitXHIGHRIGHT) == HIGH); /*直到x軸左極限被按壓*/

  do {
    ymovedown();
  } while (digitalRead(limitYLOWRIGHT) == HIGH || digitalRead(limitYLOWLEFT) == HIGH);
  delay(1000);
}



void setup() {


  Serial.begin(115200);

  pinMode(limitYLOWLEFT, INPUT_PULLUP);  //x軸左下極限
  digitalWrite(limitYLOWLEFT, HIGH);

  pinMode(limitYLOWRIGHT, INPUT_PULLUP);  //y軸右下極限
  digitalWrite(limitYLOWRIGHT, HIGH);

  pinMode(limitXLOWRIGHT, INPUT_PULLUP);  //x軸右下極限
  digitalWrite(limitXLOWRIGHT, HIGH);

  pinMode(limitXHIGHRIGHT, INPUT_PULLUP);  //x軸右上極限
  digitalWrite(limitXHIGHRIGHT, HIGH);

  Wire.begin();
  while (!huskylens.begin(Wire)) {
    Serial.println(F("Begin failed!"));
    Serial.println(F("1.Please recheck the Protocol Type in HUSKYLENS (General Settings>>Protocol Type>>I2C)"));
    Serial.println(F("2.Please recheck the connection."));
    delay(100);
  }


  pinMode(latchPinx, OUTPUT);
  pinMode(dataPinx, OUTPUT);
  pinMode(clockPinx, OUTPUT);
  pinMode(latchPiny, OUTPUT);
  pinMode(dataPiny, OUTPUT);
  pinMode(clockPiny, OUTPUT);

  reset();
}
void currentdetect() {
  int adc = analogRead(15);
  float voltage = adc * 5 / 1023.0;
  float current = (voltage - 2.5) / 0.185;
  if (current < 0.16) {
    current = 0;
  }
  Serial.print("Current : ");
  Serial.println(current);
  delay(300);
}
void loop() {
  if (!huskylens.request()) Serial.println(F("Fail to request data from HUSKYLENS, recheck the connection!"));
  else if (!huskylens.isLearned()) Serial.println(F("Nothing learned, press learn button on HUSKYLENS to learn one!"));
  else if (!huskylens.available()) Serial.println(F("No block or arrow appears on the screen!"));
  else {
    Serial.println(F("###########"));
    while (huskylens.available()) {
      HUSKYLENSResult result = huskylens.read();
      delay(3000);
      printResult(result);
      int revolutionx = result.xCenter;
      int revolutiony = result.yCenter;
      int adc = analogRead(15);
      float voltage = adc * 5 / 1023.0;
      float current = (voltage - 2.5) / 0.185;

      for (int i = 0; i < revolutiony; i++) {
        ymoveup();
      }

      for (int i = 0; i < revolutionx; i++) {
        xmoveleft();
      }
      delay(5000);
      if (current < 58) {
        while (current > 58) {
          for (int i = 0; i < +10; i++) {
            xmoveright();
          }
          delay(5000); 

          for (int i = 0; i < +10; i++) {
            ymoveup();
          }
          delay(5000);

          for (int i = 0; i < +10; i++) {
            xmoveleft();
          }
          delay(5000);

          for (int i = 0; i < +10; i++) {
            xmoveleft();
          }
          delay(5000);

          for (int i = 0; i < +10; i++) {
            ymovedown();
          }
          delay(5000);

          for (int i = 0; i < +10; i++) {
            ymovedown();
          }
          delay(5000);

          for (int i = 0; i < +10; i++) {
            xmoveright();
          }
          delay(5000);

          for (int i = 0; i < +10; i++) {
            xmoveright();
          }
          delay(5000);

          for (int i = 0; i < +10; i++) {
            ymoveup();
          }
          delay(5000);

          for (int i = 0; i < +10; i++) {
            xmoveleft();
          }
          delay(5000);
        }
        while (current < 58) {
          delay(1800000);
        }
      } else {
        while (current < 58) {
          delay(1800000);
        }
      }
      reset();
    }
  }
}

void printResult(HUSKYLENSResult result) {
  if (result.command == COMMAND_RETURN_BLOCK) {
    Serial.println(String() + F("Block:xCenter=") + result.xCenter + F(",yCenter=") + result.yCenter + F(",width=") + result.width + F(",height=") + result.height + F(",ID=") + result.ID);
  } else if (result.command == COMMAND_RETURN_ARROW) {
    Serial.println(String() + F("Arrow:xOrigin=") + result.xOrigin + F(",yOrigin=") + result.yOrigin + F(",xTarget=") + result.xTarget + F(",yTarget=") + result.yTarget + F(",ID=") + result.ID);
  } else {
    Serial.println("Object unknown!");
  }
}
