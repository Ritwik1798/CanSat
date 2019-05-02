  /*
<TEAM ID>,<MISSION TIME>,<PACKET COUNT>,<ALTITUDE>, <PRESSURE>,
<TEMP>,<VOLTAGE>,<GPS TIME>,<GPS LATITUDE>,<GPS LONGITUDE>,<GPS
ALTITUDE>,<GPS SATS>,<PITCH>,<ROLL>,<BLADE SPIN RATE>,<SOFTWARE
STATE>,<BONUS DIRECTION>
*/
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU9250.h"
#include "Seeed_BMP280.h"     
#include <Adafruit_GPS.h>
#include <SD.h>
#include <EEPROM.h>
#include <RTClock.h>
#include <Servo.h>

#define GPSSerial Serial1
#define GPSECHO false
#define TeamID 1989

BMP280 bmp280;
Adafruit_GPS GPS(&GPSSerial);
MPU9250 IMU(Wire,0x68);
File mySensorData;
RTClock rtclock (RTCSEL_LSI); // initialise
Servo myservo;

const char * delim = " :";
const char * weekdays[] = {"Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun"};
const char * months[] = {"Dummy", "Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec" };
const int timezone = 8;      // change to your timezone
const int chipSelect = 4;
const int format_pin = PA0;
const int tmp36 = 15;
const int rpm = 12;
const int servo = 8;

const uint16_t Address_pc = 0x10;
const uint16_t Address_mt_hour = 0x11;
const uint16_t Address_mt_minute = 0x12;
const uint16_t Address_mt_second = 0x13;

uint16 Hour = 0;
uint16 Minute = 0;
uint16 Second = 0;
uint16 Hour_start = 0;
uint16 Minute_start = 0;
uint16 Second_start = 0;
uint16 packet_count = 0;
uint16_t Address_global;
uint16 EEPROM_status;
uint16 Data_global;
uint16_t timer = millis();

float temperature;
float pressure;
float altitude;
float countrpm;
float SLpressure_mB;
int ELEVATION = 300; 
int status;
int pos =0;
char s[128]; 
time_t compensated_time, stored_time;
tm_t time_struct;
uint8_t dateread[11];
int offset = 20;
bool dispflag = true;
long rtc_time = 0;



void ext_temp();
void gps_init();
void imu_init();
void sd_init();
void blade_rpm();
void get_time();
void EEPROM_init();
void DisplayConfig();
void imu_data();
void gps_data();
void store_data_imu();

uint16 flash_retrieve(uint16_t Address, uint16_t Data);
void flash_store(uint16_t Address, uint16_t Data);
void flash_var_init();
uint8_t str2month(const char * d);
void ParseBuildTimestamp(tm_t & mt);     
void mission_time();

void setup()
{
  pinMode(format_pin,INPUT_PULLUP);
  pinMode(rpm,INPUT);
  myservo.attach(servo); 
  Serial.begin(9600);
 
  Serial.println("Start");
  rtc_init();
  gps_init();
  imu_init();
  sd_init();
  packet_count = flash_retrieve(Address_pc,packet_count);
  Hour_start = flash_retrieve(Address_mt_hour, Hour_start);
  Minute_start = flash_retrieve(Address_mt_minute, Minute_start);
  Second_start = flash_retrieve(Address_mt_second, Second_start);
  Serial.println(packet_count);
}

void loop() {
  if(!digitalRead(format_pin)) {
    Serial.println("initialising flash variables....");
    flash_var_init();
  }
  if(millis()- rtc_time >= 1000) {
    compensated_time++;
    rtc_time = millis();
    get_time();
    mission_time();
  }
  gps_data();
  ext_temp();
  blade_rpm();
  packet_count++;
  if(packet_count%3 == 0) {
    flash_store(Address_pc, packet_count);
  }
  Serial.println(packet_count);
  delay(500);
}

void ext_temp(){
  float offset = 0.5;
  int val = analogRead(tmp36);
  float voltage = val * 5.0;
  voltage = voltage/1024;
  float etemp = (voltage - offset)*100;
  Serial.print(etemp,2);
  Serial.println("degC");
}

void blade_rpm(){
  int count = 0;
  boolean countFlag = LOW;
  unsigned long currentTime = 0;
  unsigned long startTime = millis();
  while (currentTime <= 1000)
  {
    if (digitalRead(rpm) == HIGH)
    {
      countFlag = HIGH;
    }
    if (digitalRead(rpm) == LOW && countFlag == HIGH)
    {
      count++;
      countFlag=LOW;
    }
    currentTime = millis() - startTime;
  }
  float countRpm = int(60000/float(1000))*count;
  Serial.println(countrpm);
}

void sd_init() {
  Serial.print("Initializing SD card...");
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    return;
  }
  Serial.println("card initialized.");
}

void gps_init() {
  Serial.println("Adafruit GPS library basic test!");
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);
  delay(1000);
  GPSSerial.println(PMTK_Q_RELEASE);
}

void imu_init() {
  Wire.begin();
  Serial.println("Working");
  if(!bmp280.init()){
     Serial.println("Device error!");
  }
  while(!Serial) {} 
  status = IMU.begin();
  if (status < 0) {
     Serial.println("IMU initialization unsuccessful");
     Serial.println("Check IMU wiring or try cycling power"); 
     Serial.print("EEPROM_status: ");
     Serial.println(status);
     while(1) {}
  }
  IMU.setGyroRange(MPU9250::GYRO_RANGE_250DPS);
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
  IMU.setSrd(19);
}

void imu_data() {
  IMU.readSensor();
  Serial.print("X-axis angle:  ");
  Serial.print(IMU.getGyroX_rads(),3);
  Serial.print("\t");
  Serial.print("Y-axis angle:  ");
  Serial.print(IMU.getGyroY_rads(),3);
  Serial.print("\t");
  Serial.print("Z-axis angle:  ");  
  Serial.print(IMU.getGyroZ_rads(),3);
  Serial.println("\t"); 
  temperature = bmp280.getTemperature();
  pressure = bmp280.getPressure();
  SLpressure_mB = (((pressure)*0.001/pow((1-((float)(ELEVATION))/44330), 5.255))/100.0);
  altitude = bmp280.calcAltitude(SLpressure_mB);
  Serial.print("Temperature: ");
  Serial.print(temperature, 2);
  Serial.println("deg C");
  Serial.print("Pressure: ");
  Serial.print(SLpressure_mB, 2);
  Serial.println(" Pa");
  Serial.print("Altitude: ");
  Serial.print(altitude, 2);
  Serial.print(" m");
  Serial.println();
  mySensorData = SD.open("Data.txt", FILE_WRITE);
  if (mySensorData) 
    store_data_imu();
  else
    Serial.println("error opening datalog.txt");   
}

void gps_data() {
  char c = GPS.read();
  if (GPSECHO)
    if (c) 
      Serial.print(c);
  if (GPS.newNMEAreceived()) {
    Serial.println(GPS.lastNMEA());
    if (!GPS.parse(GPS.lastNMEA()))
      return;
  }
  if (timer > millis()) 
    timer = millis();
  if (millis() - timer > 2000) {
    timer = millis(); // reset the timer
    Serial.print("\nTime: ");
    Serial.print(GPS.hour, DEC); Serial.print(':');
    Serial.print(GPS.minute, DEC); Serial.print(':');
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
    if (GPS.fix) {  
      Serial.print("Latitude: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", ");
      Serial.print("Latitude: ");
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
    }
    imu_data();
  } 
}

void store_data_imu() {
  mySensorData.println(TeamID);
  mySensorData.println(",");
  mySensorData.print("Temperature(Degrees):"); 
  mySensorData.println(temperature);                             
  mySensorData.print("Pressure(Pascals):");                       
  mySensorData.println(pressure);                        
  mySensorData.print("Altitude(Meters):");                             
  mySensorData.println(altitude);                        
  mySensorData.print("Yaw :");
  mySensorData.print(IMU.getGyroX_rads());
  mySensorData.print("\t");
  mySensorData.print("Pitch :");
  mySensorData.print(IMU.getGyroY_rads());
  mySensorData.print("\t");
  mySensorData.print("Roll :");
  mySensorData.println(IMU.getGyroZ_rads());
  mySensorData.close(); 
  delay(3000);
}

void EEPROM_init() {
  EEPROM.PageBase0 = 0x801F000;
  EEPROM.PageBase1 = 0x801F800;
  EEPROM.PageSize  = 0x400;
  DisplayConfig();
  
  EEPROM_status = EEPROM.init();
  Serial.print("EEPROM.init() : ");
  Serial.println(EEPROM_status, HEX);
  Serial.println();   
}

void DisplayConfig() {
  Serial.print  ("EEPROM.PageBase0 : 0x");
  Serial.println(EEPROM.PageBase0, HEX);
  Serial.print  ("EEPROM.PageBase1 : 0x");
  Serial.println(EEPROM.PageBase1, HEX);
  Serial.print  ("EEPROM.PageSize  : 0x");
  Serial.print  (EEPROM.PageSize, HEX);
  Serial.print  (" (");
  Serial.print  (EEPROM.PageSize, DEC);
  Serial.println(")");
}

uint16_t flash_retrieve(uint16_t Address, uint16_t Data) {
  Address_global = Address;
  Data_global = Data;
  EEPROM_status = EEPROM.read(Address_global, &Data_global);
  Serial.print("EEPROM.read(0x");
  Serial.print(Address_global, HEX);
  Serial.print(", &..) = 0x");
  Serial.print(Data_global, HEX);
  Serial.print(" : EEPROM_status : ");
  Serial.println(EEPROM_status, HEX);
  return Data_global;
}

void flash_store(uint16_t Address, uint16_t Data) {
  Address_global = Address;
  Data_global = Data;
  EEPROM_status = EEPROM.write(Address_global, Data_global);
  Serial.print("EEPROM.write(0x");
  Serial.print(Address_global, HEX);
  Serial.print(", 0x");
  Serial.print(Data_global, HEX);
  Serial.print(") : EEPROM_status : ");
  Serial.println(EEPROM_status, HEX);
}

void flash_var_init() {
  packet_count = 0;
  Hour_start = (uint16_t)time_struct.hour;
  Minute_start = (uint16_t)time_struct.minute;
  Second_start = (uint16_t)time_struct.second;
  flash_store(Address_pc, packet_count);
  flash_store(Address_mt_hour, Hour_start);
  flash_store(Address_mt_minute, Minute_start);
  flash_store(Address_mt_second, Second_start);
}

void ParseBuildTimestamp(tm_t & mt)
{
    // Timestamp format: "Dec  8 2017, 22:57:54"
    sprintf(s, "Timestamp: %s, %s\n", __DATE__, __TIME__);
    char * token = strtok(s, delim);
    while( token != NULL ) {
        uint8_t m = str2month((const char*)token);
        if ( m>0 ) {
            mt.month = m;
            token = strtok(NULL, delim); 
            mt.day = atoi(token);
            token = strtok(NULL, delim); 
            mt.year = atoi(token) - 1970;
            token = strtok(NULL, delim); 
            mt.hour = atoi(token);
            token = strtok(NULL, delim); 
            mt.minute = atoi(token);
            token = strtok(NULL, delim); 
            mt.second = atoi(token);
        }
        token = strtok(NULL, delim);
    }
}

uint8_t str2month(const char * d)
{
    uint8_t i = 13;
    while ( (--i) && strcmp(months[i], d)!=0 );
    return i;
}

void get_time() {
  if ( Serial.available()>10 ) {
    for (uint8_t i = 0; i<11; i++) {
      dateread[i] = Serial.read();
    }
    Serial.flush();
    compensated_time = atol((char*)dateread);
    rtclock.setTime(rtclock.TimeZone(compensated_time, timezone)); //adjust to your local date
  }
  if (stored_time != compensated_time && dispflag == true )
  {
    stored_time = compensated_time;
    rtclock.breakTime(rtclock.now(), time_struct);
    sprintf(s, "RTC timestamp: %s %u %u, %s, %02u:%02u:%02u\n",
    months[time_struct.month], time_struct.day, time_struct.year+1970, weekdays[time_struct.weekday], time_struct.hour, time_struct.minute, time_struct.second);
    Serial.print(s);
  }
}

void rtc_init() {
  rtc_time = millis();
  ParseBuildTimestamp(time_struct);  
  compensated_time = rtclock.makeTime(time_struct) + 25 - offset; // additional seconds to compensate build and upload delay
  rtclock.setTime(compensated_time);
  stored_time = compensated_time;
}

void mission_time() {
  Hour = (uint16_t)time_struct.hour - Hour_start;
  if( (uint16_t)time_struct.minute - Minute_start < 0 ) {
    Minute = ((uint16_t)time_struct.minute + 60) - Minute_start;
    Hour--;
  }
  else {
    Minute = (uint16_t)time_struct.minute - Minute_start;
  }
  if( (uint16_t)time_struct.second - Second_start < 0 ) {
    Second = (uint16_t)time_struct.second + 60 - Second_start;
    Minute--;
  }
  else {
    Second = (uint16_t)time_struct.second - Second_start;
  }
  Serial.println("Mission_time: ");
  Serial.print(Hour);Serial.print(":");Serial.print(Minute);Serial.print(":");Serial.println(Second);
}

