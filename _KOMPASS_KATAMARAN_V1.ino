// ***************************************
// Arduino 1.0.6 @ 31.10.2016
// ***************************************
// I2C device found at address 0x1E  !
// I2C device found at address 0x50  !
// I2C device found at address 0x51  !
// I2C device found at address 0x68  !
// I2C device found at address 0x77  !

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
#include <Average.h>
#include <RTClib.h>
#include <Rtc_Pcf8563.h>

#define OLED_MOSI   5
#define OLED_CLK    7
#define OLED_CS     30
#define OLED_DC     29
#define OLED_RESET  28

unsigned long BAR_EEPROM_POS = 0;

#define EEPROM_ADDRESS      (0x50) // 24LC512

#define EE24LC512MAXBYTES 524288/8 // 65536 Байт [0-655365]

I2C_eeprom eeprom(EEPROM_ADDRESS,EE24LC512MAXBYTES);

// ----------------------- BMP085 ---------------------------------

struct bmp085_t // Данные о давлении,высоте и температуре
{    
  double Press,Alt,Temp;
  unsigned long unix_time; 

} 
bmp085_data;

struct bmp085_out // Данные о давлении,высоте и температуре
{    
  double Press,Alt,Temp;
  unsigned long unix_time; 

} 
bmp085_data_out;


Adafruit_SSD1306 display(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);

unsigned long currentMillis;
unsigned long PreviousInterval = 0;  
unsigned long onePreviousInterval = 0; 
unsigned long barPreviousInterval = 0;

TinyGPSPlus gps;  // GPS Serial(1)4800
char nmea;        // NMEA input

Rtc_Pcf8563 rtc;  // PCF8563

BMP085 bmp = BMP085();  // BMP085

long Temperature = 0, Pressure = 0, Altitude = 0;

#define FIVE_MINUT 300000
#define TWO_DAYS 172800

#define DEBUG 0

#define LED 23

// ====================== Setup ================================================

void setup() {

  Serial.begin(9600);   // Debug
  pinMode(LED, OUTPUT);  // LED
  Serial1.begin(4800);  // GPS
  display.begin();      // Turn On Display
  Wire.begin();
  delay(500);
  bmp.init();  

  // rtc.initClock();  // Erase Clock

  rtc.setSquareWave(SQW_1HZ);

  // rtc.clearSquareWave();
  
  // day, weekday, month, century(1=1900, 0=2000), year(0-99)
  //  rtc.setDate(10, 4, 11, 0, 16);
  // hr, min, sec
  //  rtc.setTime(16,14,00);
  
    display.clearDisplay();
    display.display();

}

// ================================ Main =====================================


void loop() {

   
  currentMillis = millis();

  if(currentMillis - PreviousInterval > (FIVE_MINUT*3) ) {  // 15 Минут
    PreviousInterval = currentMillis;  

    bmp.getTemperature(&Temperature);  // Температура
    bmp.getPressure(&Pressure);        // Давление
    bmp.getAltitude(&Altitude);        // Высота 

    bmp085_data.Press = Pressure/133.3;
    bmp085_data.Alt   = Altitude/100.0;
    bmp085_data.Temp  = Temperature/10.0;

    Save_Bar_Data();

  }


  if (Serial1.available()) {
    nmea = Serial1.read();
    gps.encode(nmea);
  }

  if(currentMillis - onePreviousInterval > 1000 ) {  // 1 Секунда
    onePreviousInterval = currentMillis;  
    Display_Test();
    if (digitalRead(LED) == 1) digitalWrite(LED,LOW); 
    else digitalWrite(LED,HIGH);
  }

}

// =============================== Functions ================================


void Display_Test() {

  display.cp437(true); // Для русских букв  
  //display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setTextWrap(0);
  display.fillRect(0,0,127,16,BLACK);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.print(rtc.formatTime());

  //  display.println(utf8rus("Скорость:"));
  //  display.print(" ");
  //  display.println(Pressure/133.3);
  //  display.setTextColor(WHITE);
  //  display.setTextSize(4);
  //  display.println(gps.charsProcessed());
  //  display.setTextSize(2);
  //  display.println(utf8rus("км/ч"));
  //  display.print(rtc.formatTime());
  //  virtual void drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color);
  //  virtual void drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color);

  //  virtual void drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color);
  //  virtual void drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color);

  ShowBMP085();
  display.display();

}

String utf8rus(String source)
{
  int i,k;
  String target;
  unsigned char n;
  char m[2] = {'0', '\0'};

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

// =============== Functions ==================

void Turn_On_MPU( void ) {

  // Bypass Mode
  Wire.beginTransmission(0x68);
  Wire.write(0x37);
  Wire.write(0x02);
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x6A);
  Wire.write(0x00);
  Wire.endTransmission();

  // Disable Sleep Mode
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

}

// --------------------------- Save Barometer Data to EEPROM -------------------------------------

void Save_Bar_Data( void ) {

  // 48 часов * 60 минут = 2880 Минут
  // 2880 минут / 30 минут = 96 Ячеек
  // (UnixTime / 1800) % 96 = номер ячейки

  bmp.getTemperature(&Temperature);  // Температура
  bmp.getPressure(&Pressure);        // Давление
  bmp.getAltitude(&Altitude);        // Высота 

  DateTime now = DateTime (rtc.getYear(), 
  rtc.getMonth(), 
  rtc.getDay(),
  rtc.getHour(), 
  rtc.getMinute(), 
  rtc.getSecond());

  bmp085_data.unix_time = now.unixtime(); 

  if (DEBUG) {
    Serial.println(bmp085_data.unix_time);
  }

  BAR_EEPROM_POS = ( (bmp085_data.unix_time/1800)%96 ) * sizeof(bmp085_data); // Номер ячейки памяти.

  bmp085_data.Press = Pressure/133.3;
  bmp085_data.Alt   = Altitude/100.0;
  bmp085_data.Temp  = Temperature/10.0;

  const byte* p = (const byte*)(const void*)&bmp085_data;
  for (unsigned int i = 0; i < sizeof(bmp085_data); i++) 
    eeprom.writeByte(BAR_EEPROM_POS++,*p++);

}


// ---------------- Barometer Graphics ------------------------

void ShowBMP085() {

  int x;

   display.drawFastVLine(0,20,43, WHITE);
   display.drawFastHLine(0,63,127, WHITE);
    
  if (currentMillis - barPreviousInterval > FIVE_MINUT/2) {  
    barPreviousInterval = currentMillis;      

    DateTime now = DateTime (rtc.getYear(), 
    rtc.getMonth(), 
    rtc.getDay(),
    rtc.getHour(), 
    rtc.getMinute(), 
    rtc.getSecond());  

    byte current_position = (now.unixtime()/1800)%96;  

    Average<double> bar_data(96); // Вычисление максимального и минимального значения

    double barArray[96];   

    BAR_EEPROM_POS = 0;

    for(byte j = 0;j < 96; j++) {           

      byte* pp = (byte*)(void*)&bmp085_data_out; 

      for (unsigned int i = 0; i < sizeof(bmp085_data); i++)
        *pp++ = eeprom.readByte(BAR_EEPROM_POS++); 

      if ((now.unixtime() - bmp085_data.unix_time) < TWO_DAYS) {

        barArray[j] = bmp085_data.Press; 
        bar_data.push(bmp085_data.Press);
        
        Serial.print(bmp085_data.Press);
        Serial.print(" ");
        Serial.print(bmp085_data.Temp);
        Serial.print(" ");        
        Serial.println(bmp085_data.unix_time);
        Serial.println("----");        

      } 
      else {

        barArray[j] = 0.0;

      }
      
    }

    BAR_EEPROM_POS = 0;

    int y_pres = 127;

    for(byte j=0;j<96;j++) {

      if (j != 0) {
        x = map(barArray[current_position],bar_data.minimum()-1,bar_data.maximum()+1,43,1);
      } 
      else {
        x = map(Pressure,bar_data.minimum()-1,bar_data.maximum()+1,43,1); // Текущие значение
      }

      display.drawLine(0,y_pres,42,y_pres, BLACK); // Стереть линию

      if (barArray[current_position] != 0.0) {     
       display.drawLine(x,y_pres,42,y_pres, WHITE); // Нарисовать данные    
      }

      if (current_position == 0) current_position = 96;

      current_position--; 

      y_pres--;

    } 

  }  

}








