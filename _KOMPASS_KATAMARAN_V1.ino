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

Adafruit_SSD1306  display(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);  // main display
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

unsigned long BAR_EEPROM_POS = 0;

#define EEPROM_ADDRESS      (0x50) // 24LC512

#define EE24LC512MAXBYTES 524288/8 // 65536 Байт [0-655365]

I2C_eeprom eeprom(EEPROM_ADDRESS,EE24LC512MAXBYTES);

HMC5883L compass;

int xc,yc,pc,zc,north;

MPU6050 mpu;

float heading1;
float heading2;

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



unsigned long currentMillis;
unsigned long PreviousInterval = 0;  
unsigned long onePreviousInterval = 0; 
unsigned long barPreviousInterval = 0;

TinyGPSPlus gps;  // GPS Serial(1)4800
char nmea;        // NMEA input

Rtc_Pcf8563 rtc;  // PCF8563

BMP085 bmp = BMP085();  // BMP085

long Temperature = 0, Pressure = 0, Altitude = 0;

boolean First = true;  // После старта

long gps_count = 0;

// ====================== Setup ================================================

void setup() {

  Serial.begin(9600);   // Debug
  // pinMode(LED, OUTPUT);  // LED
  Serial1.begin(4800);  // GPS
  
  display.begin();       // Turn On Display
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

  compass.setOffset(offX, offY);

  // rtc.initClock();  // Erase Clock

  rtc.setSquareWave(SQW_1HZ);

  // rtc.clearSquareWave();

  // day, weekday, month, century(1=1900, 0=2000), year(0-99)
  //  rtc.setDate(10, 4, 11, 0, 16);
  // hr, min, sec
  //  rtc.setTime(16,14,00);

  display.clearDisplay();
  display.display();
  
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
  

  // Для стартовых значений

  bmp.getTemperature(&Temperature);  // Температура
  bmp.getPressure(&Pressure);        // Давление
  bmp.getAltitude(&Altitude);        // Высота 

  First = true;


}

// ================================ Main =====================================


void loop() {


  currentMillis = millis();

  if(currentMillis - PreviousInterval > (FIVE_MINUT*3) ) {  // 15 Минут Save BAR to EEPROM
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
    // Serial.print(nmea);
  }

  if(currentMillis - onePreviousInterval > 500 ) {  // 1 Секунда = 1000
    onePreviousInterval = currentMillis;  

    Display_GPS();              // Display 5
    Display_Test();            // Display 1 - Барометр
    Display_Time_SunRise();    // Display 0
    Display_Compass();         // Display 2
    Display_OLD_Compass();        // Dislay 4
    Display_Uroven();              // Display 3
    
    // if (digitalRead(LED) == 1) digitalWrite(LED,LOW); 
    // else digitalWrite(LED,HIGH);
  }

}

// =============================== Functions ================================

void Display_Uroven( void ) {

  
  display3.clearDisplay();
  
  Vector normAccel = mpu.readNormalizeAccel();

  int pitch = -(atan2(normAccel.XAxis, sqrt(normAccel.YAxis*normAccel.YAxis + normAccel.ZAxis*normAccel.ZAxis))*180.0)/M_PI;
  int roll = (atan2(normAccel.YAxis, normAccel.ZAxis)*180.0)/M_PI;

  display3.fillRect(0,0,127,63,BLACK);

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

// =================================== GPS ==============================

void Display_GPS( void ) {

  gps_count = gps.satellites.value();

  display5.clearDisplay();

  display5.cp437(true); // Для русских букв  
  display5.setTextSize(2);
  display5.setTextColor(WHITE);
  display5.setTextWrap(0);
  display5.fillRect(0,0,127,63,BLACK);
  display5.setTextColor(WHITE);
  display5.setCursor(0,0);

  if (gps_count < 3) {
    display5.println(utf8rus("Поиск"));
    display5.setTextSize(4);
    display5.println(gps_count);
    display5.setTextSize(2);
    display5.println(utf8rus("Спутников"));
  } 
  else {
    display5.println(gps.location.lat(), 6);  // Широта
    display5.println(gps.location.lng(), 6);  // Долгота      
    display5.print("COG:"); 
    display5.println(gps.course.deg());
    display5.print(utf8rus("км/ч:")); 
    display5.print(gps.speed.kmph());
  }

  display5.display();

}


// ============ Барометер =============================

void Display_Test( void ) {

  
  display1.clearDisplay();
  
  display1.cp437(true); // Для русских букв  
  display1.setTextSize(2);
  display1.setTextColor(WHITE);
  display1.setTextWrap(0);
  display1.fillRect(0,0,127,16,BLACK);
  display1.setTextColor(WHITE);
  display1.setCursor(0,0);
  display1.print(rtc.formatTime());

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

  ShowBMP085(First);
  display1.display();

}

String utf8rus(String source)
{
  int i,k;
  String target;
  unsigned char n;
  char m[2] = {
    '0', '\0'  };

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

void ShowBMP085(boolean fs) {

  int H;

  display1.drawFastVLine(0,20,43, WHITE);
  display1.drawFastHLine(0,63,127, WHITE);

  if ((currentMillis - barPreviousInterval > FIVE_MINUT/2) || fs == true ) {  
    barPreviousInterval = currentMillis;      

    First = false;

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

    if (DEBUG) {
      for(byte j=0;j<96;j++) Serial.println(barArray[j]);
      Serial.println("===============");
    }

    BAR_EEPROM_POS = 0;

    // barArray[0] = Pressure/133.3;  // Текущие значения  
    // bar_data.push(Pressure/133.3);

    int x_pos = 127;

    for(byte j=0;j<96;j++) {

      H = map(barArray[current_position],bar_data.minimum(),bar_data.maximum(),62,20);

      display1.drawLine(x_pos,20,x_pos,62, BLACK); // Стереть линию

      if (barArray[current_position] != 0.0) {     
        display1.drawLine(x_pos,62,x_pos,H,WHITE); // Нарисовать данные    
      }

      if (current_position == 0) current_position = 96;

      current_position--; 

      x_pos--;

    } 

  }  

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

  display.clearDisplay();

  display.cp437(true); // Для русских букв  
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setTextWrap(0);
  display.fillRect(0,0,127,63,BLACK);
  display.setTextColor(WHITE);
  display.setCursor(0,0);

  display.print(utf8rus("Зах: "));
  display.print(toZero(h_set));
  display.print(":");
  display.println(toZero(m_set));

  display.setTextSize(4);
  display.print(toZero(rtc.getHour()));

  if ((rtc.getSecond() % 2) == 0)
    display.print(":");
  else   display.print(" ");

  display.println(toZero(rtc.getMinute()));

  display.setTextSize(2);
  display.print(utf8rus("Вос: "));
  display.print(toZero(h_rise));
  display.print(":");
  display.print(toZero(m_rise));

  display.display();

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
  display2.fillRect(0,0,127,63,BLACK);
  display2.setTextColor(WHITE);
  display2.setCursor(0,0);

  display2.setTextSize(2);
  display2.println(utf8rus("Компас"));

  display2.setTextSize(4);
  north = round(get_compass());
  north = getcompasscourse();  // Не то
  north = round(Tcompass());
  display2.print(north);
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

void Display_OLD_Compass( void ) {

  north  = round(get_compass());

  display4.clearDisplay();

  display4.fillRect(0,0,127,63,BLACK);

  display4.drawCircle(96, 32, 30,WHITE);

  get_dir_print(1,10);      // Печать направления
  
  display2.cp437(true);     // Для русских букв 
  display4.setTextSize(2);
  display4.setCursor(1,35);
  display4.print(round(get_compass())); // Печать азимута    
  display4.print(char(176)); // Печатаем значок градуса

  draw_line();//U8GLIB_SSD1306_128X64 u8g(13, 11, 10, 9);	// SW SPI Com: SCK = 13, MOSI = 11, CS = 10, A0 = 9


  pc = north - 180; 
  if (pc > 360) pc = north - 180;

  xc = 96 - (29 * cos(pc*(3.14/180)));
  yc = 32 -(29 * sin(pc*(3.14/180)));

  display4.drawCircle(xc,yc, 3,WHITE);

  display4.display();

}

void get_dir_print( int x, int y) {

  int z = round(get_compass());

  if (z > 0 & z < 90)       { 
    print_dir('N',x,y);  
    print_dir('E',x+15,y);  
  }
  if (z > 90 & z < 180)   { 
    print_dir('E',x,y);  
    print_dir('S',x+15,y);  
  }
  if (z > 180 & zc < 270) { 
    print_dir('S',x,y);  
    print_dir('W',x+15,y); 
  }
  if (z > 270 & z < 360) { 
    print_dir('W',x,y); 
    print_dir('N',x+15,y);  
  }
 

}

void print_dir(char a, int x, int y) {

  display4.setTextSize(2);
  display4.setTextColor(WHITE);
  display4.setCursor(x,y);
  if (a=='N') display4.print("N");
  if (a=='S') display4.print("S"); 
  if (a=='E') display4.print("E");   
  if (a=='W') display4.print("W");  

}


void draw_line( void ) {

  int r = 0;

  pc = -180; 

  xc = 96 - (29 * cos(r*(3.14/180)));
  yc = 32 - (29 * sin(r*(3.14/180)));

  display4.drawLine(96,32,xc,yc,WHITE);

  xc = 96 - (29 * cos(pc*(3.14/180)));
  yc = 32 -(29 * sin(pc*(3.14/180)));

  display4.drawLine(96,32,xc,yc,WHITE);
  display4.drawCircle(xc,yc, 3,WHITE);

}

