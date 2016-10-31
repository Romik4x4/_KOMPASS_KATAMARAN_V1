// ***************************************
// Arduino 1.0.6 @ 31.10.2016
// ***************************************

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define OLED_MOSI   5
#define OLED_CLK    7
#define OLED_DC     1
#define OLED_CS     2
#define OLED_RESET  0

Adafruit_SSD1306 display(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);

void setup() {

  pinMode(23, OUTPUT);
  Serial1.begin(4800);  // GPS
  display.begin();
 
}

void loop() {
  
  digitalWrite(23, HIGH);   
  delay(500);              
  digitalWrite(23, LOW);    
  delay(500);             
  
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.println("12:15 21/10");
  display.setTextColor(WHITE); // 'inverted' text
  display.println(3.141592);
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.print("0x"); display.println(320, HEX);
  display.display();
  
  
}
