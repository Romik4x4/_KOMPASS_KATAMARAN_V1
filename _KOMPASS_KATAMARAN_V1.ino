// ***************************************
// Arduino 1.0.6 @ 31.10.2016
// ***************************************

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <TinyGPS++.h>
#include <BMP085.h>
#include <EEPROM.h>
#include <EEPROMAnything.h>
#include <Sunrise.h>
#include <I2C_eeprom.h>
#include <RTClib.h>
#include <Average.h>

#define OLED_MOSI   5
#define OLED_CLK    7
#define OLED_DC     1
#define OLED_CS     2
#define OLED_RESET  0

Adafruit_SSD1306 display(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);

long a;
long b;
float v;

TinyGPSPlus gps;  // GPS Serial(1)4800
char nmea;        // NMEA input

// ====================== Setup ================================================

void setup() {

  pinMode(23, OUTPUT);
  Serial1.begin(4800);  // GPS
  display.begin();

}

void loop() {

  if (Serial1.available()) {
    nmea = Serial1.read();
    gps.encode(nmea);
  }

  a = random(1,99);
  b = random(10,50);

  digitalWrite(23, HIGH);   
 // delay(100);              
  digitalWrite(23, LOW);    
 // delay(100);         

  if (a > b) { 
    v = a/b; 
  } 
  else { 
    v = float(b/a); 
  }

  display.cp437(true); // Для русских букв  
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.setTextWrap(0);
  display.println(utf8rus("Скорость:"));
  display.setTextColor(WHITE);
  display.setTextSize(4);
  display.println(gps.charsProcessed());
  display.setTextSize(2);
  display.println(utf8rus("км/ч"));
  display.display();


}

String utf8rus(String source)
{
  int i,k;
  String target;
  unsigned char n;
  char m[2] = { 
    '0', '\0'   };

  k = source.length(); 
  i = 0;

  while (i < k) {
    n = source[i]; 
    i++;

    if (n >= 0xC0) {
      switch (n) {
      case 0xD0: 
        {
          n = source[i]; 
          i++;
          if (n == 0x81) { 
            n = 0xA8; 
            break; 
          }
          if (n >= 0x90 && n <= 0xBF) n = n + 0x30;
          break;
        }
      case 0xD1: 
        {
          n = source[i]; 
          i++;
          if (n == 0x91) { 
            n = 0xB8; 
            break; 
          }
          if (n >= 0x80 && n <= 0x8F) n = n + 0x70;
          break;
        }
      }
    }
    m[0] = n; 
    target = target + String(m);
  }
  return target;
}


