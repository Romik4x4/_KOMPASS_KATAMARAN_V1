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
#include <RTClib.h>
#include <Average.h>
#include <Rtc_Pcf8563.h>

#define OLED_MOSI   5
#define OLED_CLK    7
#define OLED_CS     2
#define OLED_DC     1
#define OLED_RESET  28


#define EEPROM_ADDRESS_256      (0x51) // 24LC256
#define EEPROM_ADDRESS_32       (0x50) // EEPROM on RTC 

#define BMP085_ADDRESS  0x77 // BMP085
#define DS1307_ADDRESS  0x68 // DS1307

#define EE24LC32MAXBYTES   32768/8
#define EE24LC256MAXBYTES 262144/8 // 32768 Байт [0-32767]

I2C_eeprom  eeprom32(EEPROM_ADDRESS_32 ,EE24LC32MAXBYTES);
I2C_eeprom eeprom256(EEPROM_ADDRESS_256,EE24LC256MAXBYTES);

// ----------------------- BMP085 ---------------------------------

struct bmp085_t // Данные о давлении,высоте и температуре
{    
    double Press,Alt,Temp;
    unsigned long unix_time; 
    
} bmp085_data;

struct bmp085_out // Данные о давлении,высоте и температуре
{    
    double Press,Alt,Temp;
    unsigned long unix_time; 
    
} bmp085_data_out;


Adafruit_SSD1306 display(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);

long a;
long b;
float v;


unsigned long currentMillis;
unsigned long PreviousInterval = 0;     


TinyGPSPlus gps;  // GPS Serial(1)4800
char nmea;        // NMEA input

Rtc_Pcf8563 rtc;  // PCF8563

BMP085 bmp = BMP085();

long Temperature = 0, Pressure = 0, Altitude = 0;


// ====================== Setup ================================================

void setup() {

  pinMode(23, OUTPUT);  // LED
  Serial1.begin(4800);  // GPS
  display.begin();      // Turn On Display
  Wire.begin();
  delay(500);

  // rtc.initClock();  // Erase Clock
  
  // rtc.setSquareWave(SQW_1HZ);
  
  rtc.clearSquareWave();
  
  bmp.init();  
  

}

void loop() {

  
     
   currentMillis = millis();

  if(currentMillis - updatePreviousInterval > 10000) {  // Каждые 10 секунд
   updatePreviousInterval = currentMillis;  

   dps.getTemperature(&Temperature);  // Температура
   dps.getPressure(&Pressure);        // Давление
   dps.getAltitude(&Altitude);        // Высота 

   sensors.requestTemperatures(); 
   tempC = sensors.getTempC(EXT_Thermometer);

   bmp085_data.Press = ( bmp085_data.Press + Pressure/133.3 ) / 2.0;
   bmp085_data.Alt   = ( bmp085_data.Alt + Altitude/100.0 ) / 2.0;
   bmp085_data.Temp  = ( bmp085_data.Temp + tempC ) / 2.0;

  }



  
   bmp085_data.Press = Pressure/133.3;
  bmp085_data.Alt   = Altitude/100.0;
  bmp085_data.Temp  = tempC;
  
  
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

  bmp.getTemperature(&Temperature); 
  bmp.getPressure(&Pressure);
  bmp.getAltitude(&Altitude);
  
  display.cp437(true); // Для русских букв  
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.setTextWrap(0);
  // display.println(utf8rus("Скорость:"));
  display.print(Temperature/10.0);
  display.print(" ");
  display.println(Pressure/133.3);
  
  display.setTextColor(WHITE);
  display.setTextSize(4);
  display.println(gps.charsProcessed());
  display.setTextSize(2);
//  display.println(utf8rus("км/ч"));
  display.print(rtc.formatTime());
  display.display();

}

String utf8rus(String source)
{
  int i,k;
  String target;
  unsigned char n;
  char m[2] = { 
    '0', '\0'         };

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

   dps.getTemperature(&Temperature);  // Температура
   dps.getPressure(&Pressure);        // Давление
   dps.getAltitude(&Altitude);        // Высота 
   
   DateTime now = rtc.now();
  
   bmp085_data.unix_time = now.unixtime(); 
   
   // DEBUG
   
   if (DEBUG) {
     bt.print("h:"); bt.println(now.hour());
     bt.print("m:"); bt.println(now.minute());
     bt.print("p:"); bt.println((bmp085_data.unix_time/1800)%96);
   }
   
   BAR_EEPROM_POS = ( (bmp085_data.unix_time/1800)%96 ) * sizeof(bmp085_data); // Номер ячейки памяти.
  
   sensors.requestTemperatures(); 
   tempC = sensors.getTempC(EXT_Thermometer);  
       
   bmp085_data.Press = ( bmp085_data.Press + Pressure/133.3 ) / 2.0;
   bmp085_data.Alt   = ( bmp085_data.Alt + Altitude/100.0 ) / 2.0;
   bmp085_data.Temp  = ( bmp085_data.Temp + tempC ) / 2.0;
     
   if (DEBUG) {
    bt.println(BAR_EEPROM_POS/sizeof(bmp085_data));
    bt.println(bmp085_data.unix_time);
    bt.println(now.hour());
    bt.println(now.minute());   
    bt.println("-------");
   }
   
   const byte* p = (const byte*)(const void*)&bmp085_data;
   for (unsigned int i = 0; i < sizeof(bmp085_data); i++) 
    eeprom32.writeByte(BAR_EEPROM_POS++,*p++);
   
}


