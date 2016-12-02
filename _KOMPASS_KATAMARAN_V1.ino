// ***************************************
// Arduino 1.0.6 @ 31.10.2016 *
// ***************************************
// I2C device found at address 0x1E  ! HMC5883
// I2C device found at address 0x50  ! EEPROM 24LC512
// I2C device found at address 0x51  ! PCF8563
// I2C device found at address 0x68  ! MPU6050 
// I2C device found at address 0x77  ! BMP085

/*
 Tilt compensated HMC5883L + MPU6050 (GY-86 / GY-87). Output for HMC5883L_compensation_processing.pde
 Read more: http://www.jarzebski.pl/arduino/czujniki-i-sensory/3-osiowy-magnetometr-hmc5883l.html
 GIT: https://github.com/jarzebski/Arduino-HMC5883L
 Web: http://www.jarzebski.pl
 (c) 2014 by Korneliusz Jarzebski
 */

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
#include <HMC5883L.h>
#include <MPU6050.h>

#define OLED_MOSI   5
#define OLED_CLK    7
#define OLED_CS     2
#define OLED_DC     3
#define OLED_RESET  4

Adafruit_SSD1306 display0(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);  // main display
Adafruit_SSD1306 display1(OLED_MOSI, OLED_CLK, A2,A1,A3);  
Adafruit_SSD1306 display2(OLED_MOSI, OLED_CLK, 21,20,22);  
Adafruit_SSD1306 display3(OLED_MOSI, OLED_CLK, 13,14,12);  
Adafruit_SSD1306 display4(OLED_MOSI, OLED_CLK, 18,19,15); 
Adafruit_SSD1306 display5(OLED_MOSI, OLED_CLK, A7,23,A6); 

#define FIVE_MINUT 300000
#define TWO_DAYS 172800
#define DEBUG 0
#define LED 23 //  Do not Usage !!!
#define UTC 3

#define TIMECTL_MAXTICS 4294967295L
#define TIMECTL_INIT          0

unsigned long barshowTimeMark = 0;
unsigned long barshowTimeInterval = 1000*60*2;  // 2 Минуты

unsigned long tripTimeMark     = 0;
unsigned long tripTimeInterval = 1000*60;       // 1 Минутa

unsigned long savebarTimeMark     = 0;
unsigned long savebarTimeInterval = 1000*60*15; // 15 Минут

unsigned long compasTimeMark     = 0;
unsigned long compasTimeInterval = 500;         // 0.5 Минут

unsigned long gpsTimeMark     = 0;
unsigned long gpsTimeInterval = 2000;           // 2 Секунды

unsigned long mainTimeMark     = 0;
unsigned long mainTimeInterval = 1000;          // 1 Секунда



// Example:  if (isTime(&tripTimeMark,tripTimeInterval))

// ======================= GPS Kalman Filter =================================================================================

#define _HORISONTAL_ACCURACY_RATIO_LIMIT        5.0
#define _PSI_CONSTANT                           (0.03239391 / 3600.0)

static const uint32_t HIFunc[91] PROGMEM = {
  1023702460, 1023703774, 1023707716, 1023714294, 1023723516, 1023735398, 1023749958, 1023767217, 1023787204, 1023809949,
  1023835487, 1023863860, 1023895112, 1023929294, 1023966461, 1024006674, 1024049998, 1024096508, 1024146280, 1024199401,
  1024255963, 1024316064, 1024379813, 1024447325, 1024518724, 1024594145, 1024673731, 1024757637, 1024846031, 1024939090,
  1025037008, 1025139991, 1025248263, 1025362063, 1025481651, 1025607304, 1025739325, 1025878038, 1026023793, 1026176972,
  1026337985, 1026507280, 1026685341, 1026872697, 1027069921, 1027277640, 1027496540, 1027727371, 1027970954, 1028228194,
  1028500086, 1028787729, 1029092337, 1029415259, 1029757994, 1030122211, 1030509781, 1030922798, 1031363624, 1031816853,
  1032069249, 1032340111, 1032631406, 1032945391, 1033284672, 1033652271, 1034051716, 1034487150, 1034963466, 1035486494,
  1036063227, 1036702134, 1037413566, 1038210307, 1039108342, 1040127918, 1040741229, 1041415612, 1042203499, 1043135839,
  1044255964, 1045626460, 1047341212, 1049061876, 1050533983, 1052596225, 1055691202, 1058908126, 1064070175, 1072456375,
  1287568416};

union u32double {
  uint32_t u;
  double d;
};

static const byte  PROGMEM NOT_RUNNING = 0;
static const byte  PROGMEM BAD_SIGNAL = 1;
static const byte  PROGMEM SIGNAL_LOST = 2;
static const byte  PROGMEM HDOP_EXCEEDED = 3;
static const byte  PROGMEM DATA_REJECTED = 4;
static const byte  PROGMEM DATA_GOOD = 5;
static const byte  PROGMEM LAT_ACCURACY_INCREASED = 6;
static const byte  PROGMEM LON_ACCURACY_INCREASED = 7;
static const byte  PROGMEM ACCEPTED_FIXED  = 8;
static const byte  PROGMEM ACCEPTED_UPDATED  = 9;

double _lastLat;
double _lastLon;
double _lastLatHACC;
double _lastLonHACC;

byte  _filterState = NOT_RUNNING;

// ------------------------------------------------------------- for GPS Filter --------------------------

unsigned long BAR_EEPROM_POS = 0;

#define EEPROM_ADDRESS      (0x50) // 24LC512

#define EE24LC512MAXBYTES 524288/8 // 65536 Байт [0-655365]

I2C_eeprom eeprom(EEPROM_ADDRESS,EE24LC512MAXBYTES);

HMC5883L compass;

int xc,yc,pc,zc,north;

MPU6050 mpu;

float heading1;
float heading2;

struct config_t
{
  double last_lat,last_lng;
  unsigned long unix_time;
  double distance;
  double tripToday;
}
configuration;

struct lat_lng_struct {
  double lat,lng;
};

struct lat_lng_struct coor[2];

// EEPROM_readAnything(0, configuration);
// EEPROM_writeAnything(0, configuration);

byte cpos = 0;

double tripToday = 0.0;

// ----------------------- BMP085 ---------------------------------

struct bmp085_t // Данные о давлении,высоте и температуре
{    
  double Press,Alt,Temp;
  unsigned long unix_time; 

} 
bmp085_data;

TinyGPSPlus gps;  // GPS Serial(1)4800

Rtc_Pcf8563 rtc;       // PCF8563

RTC_Millis  rtc_local; 

// rtc.begin(DateTime(F(__DATE__), F(__TIME__))); 
// rtc.adjust(DateTime(2015, 1, 1, 0, 0, 0));
// DateTime now = rtc.now();
// now.hour()

BMP085 bmp = BMP085();  // BMP085

long Temperature = 0, Pressure = 0, Altitude = 0;

boolean First = true;     // После старта
boolean gps_First = true; // После запуска ждем первую точку

long gps_count = 0;

// ====================== Setup ================================================

void setup() {

  Serial.begin(9600);   // Debug
  Serial1.begin(4800);  // GPS

  display0.begin();      // Turn On Display
  display1.begin();      // Turn On Display
  display2.begin();      // Turn On Display
  display3.begin();      // Turn On Display
  display4.begin();      // Turn On Display
  display5.begin();      // Turn On Display

  Wire.begin();
  delay(500);

  bmp.init();  

  mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G);

  delay(500);

  // If you have GY-86 or GY-87 module.
  // To access HMC5883L you need to disable the I2C Master Mode and Sleep Mode, and enable I2C Bypass Mode
  // Turn_On_MPU(); // Включить Компасс и гироскоп

  mpu.setI2CMasterModeEnabled(false);
  mpu.setI2CBypassEnabled(true) ;
  mpu.setSleepEnabled(false);

  compass.begin();

  delay(500);

  compass.setRange(HMC5883L_RANGE_1_3GA);
  compass.setMeasurementMode(HMC5883L_CONTINOUS);
  compass.setDataRate(HMC5883L_DATARATE_30HZ);
  compass.setSamples(HMC5883L_SAMPLES_8);
  // Set calibration offset. See HMC5883L_calibration.ino
  // compass.setOffset(0, 0);

  int minX = 0;
  int maxX = 0;
  int minY = 0;
  int maxY = 0;
  int offX = 0;
  int offY = 0;

  Vector mag = compass.readRaw();

  // Determine Min / Max values
  if (mag.XAxis < minX) minX = mag.XAxis;
  if (mag.XAxis > maxX) maxX = mag.XAxis;
  if (mag.YAxis < minY) minY = mag.YAxis;
  if (mag.YAxis > maxY) maxY = mag.YAxis;

  // Calculate offsets
  offX = (maxX + minX)/2;
  offY = (maxY + minY)/2;

  // compass.setOffset(offX, offY);

  // rtc.initClock();  // Erase Clock

  rtc.setSquareWave(SQW_1HZ);

  // rtc.clearSquareWave();
  //
  // day, weekday, month, century(1=1900, 0=2000), year(0-99)
  //  rtc.setDate(10, 4, 11, 0, 16);
  // hr, min, sec
  //  rtc.setTime(16,14,00);

  display0.clearDisplay();
  display0.display();

  display1.clearDisplay();
  display1.display();

  display2.clearDisplay();
  display2.display();

  display3.clearDisplay();
  display3.display();

  display4.clearDisplay();
  display4.display();

  display5.clearDisplay();
  display5.display();

  /* display5.cp437(true); // Для русских букв  
   display5.setTextSize(2);
   display5.setTextColor(WHITE);
   display5.setTextWrap(0);
   display5.print(char(176));
   display5.display();
   while(1) {} */

  // Для стартовых значений

  bmp.getTemperature(&Temperature);  // Температура
  bmp.getPressure(&Pressure);        // Давление
  bmp.getAltitude(&Altitude);        // Высота 

  bmp085_data.Press = Pressure/133.3;
  bmp085_data.Alt   = Altitude/100.0;
  bmp085_data.Temp  = Temperature/10.0;


  // --- Reset configuration in EEEPROM
  /*
   configuration.last_lat = 0.0;
   configuration.last_lng = 0.0;
   configuration.unix_time = 0;
   configuration.distance = 0.0;
   configuration.tripToday = 0.0;
   */
   
   EEPROM_writeAnything(0, configuration); // Запись
   

  EEPROM_readAnything(0, configuration);  // Чтение
  configuration.tripToday = 0.0;


}

// ================================ Main =====================================


void loop() {

  if (Serial1.available()) {
    gps.encode(Serial1.read());
  }

  if (isTime(&tripTimeMark,tripTimeInterval)) gps_trip(); // 1 Минута
  
  if (isTime(&barshowTimeMark,barshowTimeInterval) || First == true ) {  
    ShowBMP085();
    First =  false;
  }

  if (isTime(&savebarTimeMark,savebarTimeInterval)) {  // 15 Минут Save BAR to EEPROM

    set_GPS_DateTime();

    bmp.getTemperature(&Temperature);  // Температура
    bmp.getPressure(&Pressure);        // Давление
    bmp.getAltitude(&Altitude);        // Высота 

    bmp085_data.Press = Pressure/133.3;
    bmp085_data.Alt   = Altitude/100.0;
    bmp085_data.Temp  = Temperature/10.0;

    Save_Bar_Data();

    configuration.distance = configuration.distance + tripToday;
    configuration.tripToday = tripToday;  

    EEPROM_writeAnything(0, configuration); // Save последнюю координату + Distance(Km)

  }

  if (isTime(&gpsTimeMark,gpsTimeInterval)) Display_GPS(); // Display 5 - GPS


  if (isTime(&compasTimeMark,compasTimeInterval)) Display_OLD_Compass(); // Dislay 4 Compass  
  

  if (isTime(&mainTimeMark,mainTimeInterval)){  // 1 Секунда = 1000

    Display_Time_SunRise();    // Display 0
    Display_Compass();         // Display 2
    Display_Trip();            // Display 3 - Trip

  }

}

// =============================== Functions ================================

void Display_Trip( void ) {

  display3.clearDisplay();

  display3.setCursor(0,0);
  display3.cp437(true); // Для русских букв  
  display3.setTextSize(2);
  display3.setTextWrap(0);
  display3.setTextColor(WHITE);
  display3.print(utf8rus("Одометр.км"));
  display3.setTextSize(3);
  display3.setCursor(0,19);
  display3.println(tripToday/1000.0);
  display3.setTextSize(2);
  display3.print(configuration.distance/1000.0);
  display3.display();

}

// ------------------------------- Set GPS Time ---------------------------------

void set_GPS_DateTime() {

  if (gps.location.isValid() && gps.date.isValid() && gps.time.isValid()) {

    DateTime utc = (DateTime (gps.date.year(), 
    gps.date.month(), 
    gps.date.day(),
    gps.time.hour(),
    gps.time.minute(),
    gps.time.second()) + 60 * 60 * UTC);

    rtc.setSquareWave(SQW_1HZ);

    rtc.clearSquareWave();

    rtc.initClock();
    rtc.setDate(gps.date.day(),1,gps.date.month(),0,gps.date.year()-2000);
    rtc.setTime(gps.time.hour()+UTC,gps.time.minute(), gps.time.second());

    rtc.setSquareWave(SQW_1HZ);

  }

}

// ---------------------------------- Uroven -----------------------------------

void Display_Uroven( void ) {


  display3.clearDisplay();

  Vector normAccel = mpu.readNormalizeAccel();

  int pitch = -(atan2(normAccel.XAxis, sqrt(normAccel.YAxis*normAccel.YAxis + normAccel.ZAxis*normAccel.ZAxis))*180.0)/M_PI;
  int roll = (atan2(normAccel.YAxis, normAccel.ZAxis)*180.0)/M_PI;

  display3.drawCircle(45,14,12,WHITE);
  display3.drawCircle(105,32,15,WHITE); // ok

  display3.drawRect(0,0,80,30,WHITE);   // Roll
  display3.drawRect(85,0,42,63,WHITE);  // Pitch

  display3.setTextColor(WHITE);
  display3.setTextSize(1);  
  display3.setCursor(5,40);
  display3.print("P:");
  display3.print(pitch);
  display3.setCursor(5,50);
  display3.print("R:");  
  display3.print(roll);

  display3.fillCircle(105,32-pitch,10,WHITE); 
  display3.fillCircle(45+roll,14,8,WHITE);

  display3.display();

}

// -------------------------------- GPS TRIP ------------------------------------

void gps_trip( void ) {

  double lat,lng,hdop;
  double dist;

  if (gps_First == false) {

    lat = gps.location.lat();
    lng = gps.location.lng();
    hdop = gps.hdop.value()/100.0;

    if (gps.location.isValid() && gps.date.isValid() && gps.time.isValid() && (gps.hdop.value()/100.0) < 5.0) {
      if (smoothingFilter(lat,lng,hdop) > 4 ) {       
        coor[cpos].lat = lat;
        coor[cpos].lng = lng;
        cpos++;
        if (cpos > 1) {
          dist = gps.distanceBetween(coor[0].lat,coor[0].lng,coor[1].lat,coor[1].lng);
          if ((dist/1000.0) < 2.5) { // Расстояние не может быть больше 2.5 км за минуту
            cpos=1;          
            tripToday = tripToday + dist;
            coor[0].lat = coor[1].lat;
            coor[0].lng = coor[1].lng;
            coor[1].lat = 0.0;
            coor[1].lng = 0.0;
          } 
          else {
            cpos=1;
          }
        }
      }
    }
  }
}

// =================================== GPS ==============================

void Display_GPS( void ) {

  double gps_speed;

  gps_count =  gps.satellites.value();

  display5.clearDisplay();

  display5.cp437(true); // Для русских букв  
  display5.setTextSize(2);
  display5.setTextWrap(0);
  display5.setTextColor(WHITE);
  display5.setCursor(0,0);

  if (gps.location.isValid() && gps.date.isValid() && gps.time.isValid()) {

    gps_speed = gps.speed.kmph();

    if (smoothingFilter(gps.location.lat(),gps.location.lng(),gps.hdop.value()/100.0) < 5 ) { 
      gps_speed = 0.0;
    } 
    else {
      if (gps_First == true) {        
        coor[0].lat = gps.location.lat();
        coor[0].lng = gps.location.lng();
        cpos++;
        gps_First = false;
      }
    }

    display5.print(utf8rus("Скорость"));
    display5.setTextSize(3);
    display5.setCursor(0,19);    
    display5.println(gps_speed);    

    double hdop = gps.hdop.value()/100.0;
    
    display5.setTextSize(2);
    display5.print(hdop);
    display5.print(F("/"));
    display5.print(gps_count);
  } 
  else  {
    display5.println(utf8rus("Найдено:"));
    display5.setTextSize(3);
    display5.setCursor(0,20);
    display5.println(gps_count);
    display5.setTextSize(2);
    display5.setCursor(0,48);
    display5.println(utf8rus("Спутников"));
  } 

  display5.display();

}

// ============ Барометер =============================

String utf8rus(String source)
{
  int i,k;
  String target;
  unsigned char n;
  char m[2] = {
    '0', '\0'                      };

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

  unsigned long OLD = BAR_EEPROM_POS;

  bmp085_data.Press = Pressure/133.3;
  bmp085_data.Alt   = Altitude/100.0;
  bmp085_data.Temp  = Temperature/10.0;

  if (DEBUG) {
    Serial.println( "Data to Write ");
    Serial.println( BAR_EEPROM_POS );
    Serial.println( bmp085_data.Press);
    Serial.println( bmp085_data.Alt );
    Serial.println( bmp085_data.Temp );
    Serial.println( bmp085_data.unix_time );
  }

  const byte* p = (const byte*)(const void*)&bmp085_data;
  for (unsigned int i = 0; i < sizeof(bmp085_data); i++) 
    eeprom.writeByte(BAR_EEPROM_POS++,*p++);

  if (DEBUG) {

    BAR_EEPROM_POS = OLD;

    byte* pp = (byte*)(void*)&bmp085_data; 
    for (unsigned int i = 0; i < sizeof(bmp085_data); i++)
      *pp++ = eeprom.readByte(BAR_EEPROM_POS++); 

    Serial.println( "Data to Read");
    Serial.println( OLD );      
    Serial.println( bmp085_data.Press);
    Serial.println( bmp085_data.Alt );
    Serial.println( bmp085_data.Temp );
    Serial.println( bmp085_data.unix_time );
  }

}


// ---------------- Barometer Graphics ------------------------

void ShowBMP085( void ) {

  int H;

  display1.clearDisplay();

  display1.drawFastVLine(30,0,63, WHITE);
  display1.drawFastHLine(30,63,97, WHITE);

  display1.cp437(true);
  display1.setTextSize(1);
  display1.setTextWrap(0);
  display1.setTextColor(WHITE);
  display1.setCursor(0,0);
  display1.println("temp");  
  display1.println(" ");    
  display1.print(int(bmp085_data.Temp));   
  display1.println(char(176)); // 176  
  display1.println(" ");    
  display1.println(utf8rus("Дав"));
  display1.println(" ");    
  display1.println(int(bmp085_data.Press)); 

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

    byte* pp = (byte*)(void*)&bmp085_data; 

    for (unsigned int i = 0; i < sizeof(bmp085_data); i++)
      *pp++ = eeprom.readByte(BAR_EEPROM_POS++); 

    if ((now.unixtime() - bmp085_data.unix_time) < TWO_DAYS) {

      barArray[j] = bmp085_data.Press; 
      bar_data.push(bmp085_data.Press);

    } 
    else {
      barArray[j] = 0.0;
    }

  }

  BAR_EEPROM_POS = 0;

  int x_pos = 127;

  for(byte j=0;j<96;j++) {

    H = map(barArray[current_position],bar_data.minimum(),bar_data.maximum(),62,0);

    if (barArray[current_position] != 0.0) {  
      display1.drawLine(x_pos,62,x_pos,H,WHITE); // Нарисовать данные    
    }

    if (current_position == 0) current_position = 96;

    current_position--; 

    x_pos--;

  } 

  display1.display();
}

// ================================== Вывести Время и Восход и заход Солнца ===================

void Display_Time_SunRise(void ) {

  // Moscow 55.740404, 37.619706    
  // Sunrise mySunrise(gps.location.lat(),gps.location.lng(),UTC);

  int t;
  byte h_rise=0,m_rise=0,h_set=0,m_set=0;

  if (gps.location.isValid() && gps.date.isValid() && gps.time.isValid()) {

    Sunrise mySunrise(gps.location.lat(),gps.location.lng(),UTC);

    mySunrise.Actual();

    t = mySunrise.Rise(rtc.getMonth(),rtc.getDay()); // Month,Day

      if(t >= 0) {
      h_rise = mySunrise.Hour();
      m_rise = mySunrise.Minute();
    } 
    else { 
      h_rise = 0; 
      m_rise = 0; 
    }    

    t = mySunrise.Set(rtc.getMonth(),rtc.getDay()); // Month,Day

      if(t >= 0) {
      h_set = mySunrise.Hour();
      m_set = mySunrise.Minute();  
    } 
    else { 
      h_set = 0; 
      m_set = 0; 
    }

  }

  display0.clearDisplay();

  display0.cp437(true); // Для русских букв  
  display0.setTextSize(2);
  display0.setTextColor(WHITE);
  display0.setTextWrap(0);
  display0.setTextColor(WHITE);
  display0.setCursor(0,0);

  display0.print(utf8rus("Зах: "));
  display0.print(toZero(h_set));
  display0.print(":");
  display0.println(toZero(m_set));

  display0.setTextSize(4);
  display0.print(toZero(rtc.getHour()));

  if ((rtc.getSecond() % 2) == 0)
    display0.print(":");
  else   display0.print(" ");

  display0.println(toZero(rtc.getMinute()));

  display0.setTextSize(2);
  display0.print(utf8rus("Вос: "));
  display0.print(toZero(h_rise));
  display0.print(":");
  display0.print(toZero(m_rise));

  display0.display();

}
// ==== добавляем ноль впереди для красоты 1:1 = 01:01 ==============

String toZero(int i) {

  String out;

  char tmp[3];

  sprintf(tmp,"%02d",i);

  out = String(tmp);

  return out;

}

void Display_Compass(void) {

  display2.clearDisplay();
  display2.cp437(true); // Для русских букв  
  display2.setTextSize(2);
  display2.setTextColor(WHITE);
  display2.setTextWrap(0);
  display2.setCursor(0,0);
  
  double course = gps.course.deg();

  display2.setTextSize(2);
  display2.println(utf8rus("GPS COG"));
  display2.setTextSize(2);
  display2.println(" ");
  display2.setTextSize(3);
  display2.print(round(course));
  display2.print(char(176));  
  display2.print(gps.cardinal(course));
  display2.display(); 

}

float get_compass( void ) {

  Vector norm = compass.readNormalize();

  float heading = atan2(norm.YAxis, norm.XAxis);

  // Set declination angle on your location and fix heading
  // You can find your declination on: http://magnetic-declination.com/
  // (+) Positive or (-) for negative
  // For Bytom / Poland declination angle is 4'26E (positive)
  // Formula: (deg + (min / 60.0)) / (180 / M_PI);

  // Moscow склонение 10'47

  float declinationAngle = (10.0 + (47.0 / 60.0)) / (180 / M_PI);

  // heading += declinationAngle;

  // Correct for heading < 0 deg and heading > 360 deg

  if (heading < 0) { 
    heading += 2 * PI; 
  }

  if (heading > 2 * PI) { 
    heading -= 2 * PI; 
  }

  // Convert to degrees

  float headingDegrees = heading * 180/M_PI; 

  return(headingDegrees);
}

int getcompasscourse(){

  int ax,ay,az,cx,cz,cy;
  int var_compass;

  Vector norm = compass.readNormalize();

  cx = norm.XAxis;
  cz = norm.ZAxis;
  cy = norm.YAxis;

  Vector normAccel = mpu.readNormalizeAccel();

  ax = normAccel.XAxis;
  ay = normAccel.YAxis;
  az = normAccel.ZAxis;

  float xh,yh,ayf,axf;
  ayf=ay/57.0;//Convert to rad
  axf=ax/57.0;//Convert to rad
  xh=cx*cos(ayf)+cy*sin(ayf)*sin(axf)-cz*cos(axf)*sin(ayf);
  yh=cy*cos(axf)+cz*sin(axf);

  var_compass=atan2((double)yh,(double)xh) * (180 / PI) -90; // angle in degrees
  if (var_compass>0){
    var_compass=var_compass-360;
  }
  var_compass=360+var_compass;

  return (var_compass);
}


// No tilt compensation
float noTiltCompensate(Vector mag)
{
  float heading = atan2(mag.YAxis, mag.XAxis);
  return heading;
}

// Tilt compensation
float tiltCompensate(Vector mag, Vector normAccel)
{
  // Pitch & Roll 

    float roll;
  float pitch;

  roll = asin(normAccel.YAxis);
  pitch = asin(-normAccel.XAxis);

  if (roll > 0.78 || roll < -0.78 || pitch > 0.78 || pitch < -0.78)
  {
    return -1000;
  }

  // Some of these are used twice, so rather than computing them twice in the algorithem we precompute them before hand.
  float cosRoll = cos(roll);
  float sinRoll = sin(roll);  
  float cosPitch = cos(pitch);
  float sinPitch = sin(pitch);

  // Tilt compensation
  float Xh = mag.XAxis * cosPitch + mag.ZAxis * sinPitch;
  float Yh = mag.XAxis * sinRoll * sinPitch + mag.YAxis * cosRoll - mag.ZAxis * sinRoll * cosPitch;

  float heading = atan2(Yh, Xh);

  return heading;
}

// Correct angle
float correctAngle(float heading)
{
  if (heading < 0) { 
    heading += 2 * PI; 
  }
  if (heading > 2 * PI) { 
    heading -= 2 * PI; 
  }

  return heading;
}



float Tcompass (void) {

  // Read vectors

  Vector mag = compass.readNormalize();
  Vector acc = mpu.readScaledAccel();  

  // Calculate headings
  heading1 = noTiltCompensate(mag);
  heading2 = tiltCompensate(mag, acc);

  if (heading2 == -1000)
  {
    heading2 = heading1;
  }

  // Set declination angle on your location and fix heading
  // You can find your declination on: http://magnetic-declination.com/
  // (+) Positive or (-) for negative
  // For Bytom / Poland declination angle is 4'26E (positive)
  // Formula: (deg + (min / 60.0)) / (180 / M_PI);
  float declinationAngle = (10.0 + (26.0 / 60.0)) / (180 / M_PI);
  heading1 += declinationAngle;
  heading2 += declinationAngle;

  // Correct for heading < 0deg and heading > 360deg
  heading1 = correctAngle(heading1);
  heading2 = correctAngle(heading2);

  // Convert to degrees
  heading1 = heading1 * 180/M_PI; 
  heading2 = heading2 * 180/M_PI; 

  // Output
  //  Serial.print(heading1);
  //  Serial.print(":");
  //  Serial.println(heading2);

  return(heading2);

}

// ----------------------------------- OLD Compass --------------------------

void Display_OLD_Compass( void ) {

  double course  = get_compass();
  int north = round(course);

  display4.clearDisplay();

  display4.drawFastVLine(0,0,63, WHITE);  // Для уровня
  display4.drawFastHLine(0,63,60, WHITE); // Для уровня

  display4.drawCircle(96, 32, 28,WHITE);
  
  display4.cp437(true);      // Для русских букв 
  display4.setTextSize(2);
  display4.setTextColor(WHITE);
  display4.setCursor(12,8);  
  display4.print(gps.cardinal(course));
  display4.setCursor(12,33);
  display4.print(north); // Печать азимута   
  display4.print(char(176)); // Печатаем значок градуса

  draw_line();

  pc = north - 180; 
  if (pc > 360) pc = north - 180;

  xc = 96 - (28 * cos(pc*(3.14/180)));
  yc = 32 -(28 * sin(pc*(3.14/180)));

  display4.drawCircle(xc,yc,3,WHITE);

  Vector normAccel = mpu.readNormalizeAccel();

  int pitch = -(atan2(normAccel.XAxis, sqrt(normAccel.YAxis*normAccel.YAxis + normAccel.ZAxis*normAccel.ZAxis))*180.0)/M_PI;
  int roll = (atan2(normAccel.YAxis, normAccel.ZAxis)*180.0)/M_PI;

  display4.drawRect(0,26,7,10, WHITE);  // Для уровня Y
  display4.drawRect(25,56,7,10, WHITE); // Для уровня X

  display3.fillRect(0,26-pitch,7,10,WHITE); 
  
  if (25+roll < 60) {
    display3.fillRect(25+roll,56,7,10,WHITE);
  } 
  else {
    display3.fillRect(60,56,7,10,WHITE);
  }

  display4.display();

}

// ----------------------------- draw_line ---------------------------

void draw_line( void ) {

  int r = 0;

  pc = -180; 

  // ------------------------------- X -------------------------
  
  xc = 96 - (29 * cos(r*(3.14/180)));
  yc = 32 - (29 * sin(r*(3.14/180)));

  display4.drawLine(96,32,xc,yc,WHITE);

  xc = 96 - (29 * cos(pc*(3.14/180)));
  yc = 32 - (29 * sin(pc*(3.14/180)));

  display4.drawLine(96,32,xc,yc,WHITE);
  
  display4.drawCircle(xc,yc,3,WHITE);
  
  // --------------------- Y --------------------------
  pc = -270; 
  
  xc = 96 - (29 * cos(pc*(3.14/180)));
  yc = 32 - (29 * sin(pc*(3.14/180)));

  display4.drawLine(96,32,xc,yc,WHITE);
  
  pc = 270; 
  
  xc = 96 - (29 * cos(pc*(3.14/180)));
  yc = 32 - (29 * sin(pc*(3.14/180)));

  display4.drawLine(96,32,xc,yc,WHITE);

  
}

// ----------------- I2C Scanner

void i2scanner()
{
  byte error, address;
  int nDevices;

  Serial.println(F("Scanning..."));

  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print(F("I2C device found at address 0x"));
      if (address<16) 
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4) 
    {
      Serial.print(F("Unknow error at address 0x"));
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println(F("No I2C devices found\n"));
  else
    Serial.println(F("done\n"));

  delay(5000);           // wait 5 seconds for next scan
}

// GPS Filter

byte smoothingFilter(double lat, double lon, double hdop) {
  u32double conv;
  if (_filterState == NOT_RUNNING) {
    _filterState = ACCEPTED_UPDATED;
    _lastLat = lat;
    _lastLon = lon;
    _lastLatHACC = _PSI_CONSTANT * hdop * _HORISONTAL_ACCURACY_RATIO_LIMIT;
    uint8_t pos;
    if (lat < 0) pos = -lat; 
    else pos = lat;
    // drive the index value to the given bounds 0..90
    while (pos > 90) pos -= 90;
    conv.u = pgm_read_dword_near(HIFunc + pos);
    _lastLonHACC = conv.d * hdop * _HORISONTAL_ACCURACY_RATIO_LIMIT / 3600;
  } 
  else {
    double deltaLat = fabs(lat - _lastLat);
    double LatHACC = _PSI_CONSTANT * hdop * _HORISONTAL_ACCURACY_RATIO_LIMIT;
    if (deltaLat > LatHACC + _lastLatHACC) {
      // GOOD data
      _filterState = ACCEPTED_UPDATED;
      _lastLatHACC = LatHACC;
      _lastLat = lat;
      _lastLon = lon;
    } 
    else {
      if (LatHACC < _lastLatHACC) {
        // GOOD data: accuracy increased
        _filterState = LAT_ACCURACY_INCREASED;
        _lastLatHACC = LatHACC;
        _lastLat = lat;
        _lastLon = lon;
      } 
      else {
        uint8_t pos;
        if (lat < 0) pos = -lat; 
        else pos = lat;
        // drive the index value to the given bounds 0..90
        while (pos > 90) pos -= 90;
        conv.u = pgm_read_dword_near(HIFunc + pos);
        double deltaLon = fabs(lon - _lastLon);
        double LonHACC = conv.d * hdop * _HORISONTAL_ACCURACY_RATIO_LIMIT / 3600;
        if (deltaLon > LonHACC + _lastLonHACC) {
          // GOOD data
          _filterState = ACCEPTED_UPDATED;
          _lastLatHACC = LatHACC;
          _lastLat = lat;
          _lastLon = lon;
        } 
        else {
          if (LatHACC < _lastLatHACC) {
            // GOOD data: accuracy increased
            _filterState = LON_ACCURACY_INCREASED;
            _lastLonHACC = LonHACC;
            _lastLat = lat;
            _lastLon = lon;
          } 
          else {
            // BAD data: poor accuracy
            _filterState = DATA_REJECTED;
          }
        }
      }
    }
  }
  if (_filterState > DATA_GOOD) {
    // GOOD data
  }
  return _filterState;
}

// ---------------------- isTime Functions --------------------

int isTime(unsigned long *timeMark, unsigned long timeInterval) {
  unsigned long timeCurrent;
  unsigned long timeElapsed;
  int result=false;

  timeCurrent = millis();
  if (timeCurrent < *timeMark) {
    timeElapsed=(TIMECTL_MAXTICS-*timeMark) + timeCurrent;
  } 
  else {
    timeElapsed=timeCurrent-*timeMark;
  }

  if (timeElapsed>=timeInterval) {
    *timeMark=timeCurrent;
    result=true;
  }
  return(result);
}








