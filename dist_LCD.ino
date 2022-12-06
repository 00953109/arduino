#include <Wire.h>
#include "rgb_lcd.h"
#include <Ultrasonic.h>

long duration, dist, average;
long aver[3];
Ultrasonic ultrasonic(5, 6);

rgb_lcd lcd;
const int colorR = 255;
const int colorG = 165;
const int colorB = 0;

void setup() {
  Serial.begin(9600);
  lcd.begin(16, 2);
  lcd.setRGB(colorR , colorG , colorB);
}

void loop() {
  for (int i = 0; i <= 2; i++) {
    measure();
    aver[i] = dist;
    delay(10);
  }
  dist = (aver[0] + aver[1] + aver[2]) / 3;

  if ( dist > 30 )
  {
    lcd.clear();
    lcd.setRGB(800 , 500 , 300);
    lcd.print(dist);
    lcd.print(" EMPTY :)");
    delay(1000);
  }
  else if ( dist < 10 )
  {
    lcd.clear();
    lcd.setRGB(255 , 0 , 166);
    lcd.print(dist);
    lcd.print(" FULL!!CLEAR IT!\n");
    delay(1000);
  }
  else
  {
    lcd.clear();
    lcd.setRGB(500 , 165 , 0);
    lcd.print(dist);
    lcd.print("  ......");
    delay(1000);
  }
}

void measure() {
  dist = ultrasonic.read();

  Serial.print("Distance in CM: ");
  Serial.println(dist);
}
