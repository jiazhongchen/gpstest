// demo: CAN-BUS Shield, receive data with interrupt mode
// when in interrupt mode, the data coming can't be too fast, must >20ms, or else you can use check mode
// loovee, 2014-6-13

#include <SPI.h>
#include "mcp_can.h"
//#include <LiquidCrystal_I2C.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_9DOF.h>
#include <Adafruit_L3GD20_U.h>
#include <TimeLib.h>

static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
TinyGPSCustom gpsFix(gps, "GPGGA", 7); // the parameter after the 7th comma in GPGGA string

//LiquidCrystal_I2C lcd(0x27,20,4); 

// the cs pin of the version after v1.1 is default to D9
// v0.9b and v1.0 is default D10
const int SPI_CS_PIN = 10;

MCP_CAN CAN(SPI_CS_PIN);                                    // Set CS pin

unsigned char flagRecv = 0;
unsigned char len = 0;
unsigned char buf[8];
unsigned long long tick; 
unsigned long long lasttick;
char str[20];
unsigned long recvTick = 0;
unsigned long lastRecvTick = 0;
String readStr;
int currentSeconds = 0;
time_t prevDisplay = 0; // when the digital clock was displayed

struct record {
    int           eventID;
    char          datetime[20];
    float         lat;
    float         lng;
    int           alt;
    int           sats;
    int           gX;
    int           gY;
    int           gZ;
    int           yaw;
    int           pitch;
    int           roll;
    unsigned long totalEngineHours;
    unsigned long totalFuelConsumption;
    unsigned int  currentFuelRate;
    unsigned long totalIdleFuel;
    unsigned long totalIdleHour;
    unsigned int  engineRPM;
    int           oilTemperature;
    int           coolantTemperature;
    int           vehicleSpeed;
    int           gearSelection;
};

unsigned char stmp[8];
typedef struct record Record;
Record CR;

/* Assign a unique ID to the sensors */
Adafruit_9DOF                 dof   = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);

/*String getValue(String data, char separator, int index) {
    int found = 0;
    int strIndex[] = {0, -1 };
    int maxIndex = data.length()-1;
    for (int i=0; i<=maxIndex && found<=index; i++) {
        if (data.charAt(i)==separator || i==maxIndex) {
            found++;
            strIndex[0] = strIndex[1]+1;
            strIndex[1] = (i == maxIndex) ? i+1 : i;
        }
    }
    return found>index ? data.substring(strIndex[0], strIndex[1]) : "";
}*/

void setup() {
    //lcd.init();                      // initialize the lcd 
    //lcd.backlight();
    serialPortsInit();    
    //Serial.println("hello world");
    canInterruptInit();
    initSensors();
    CR.eventID = 0;
    //CR.datetime[20];
    CR.lat = 0;
    CR.lng = 0;
    CR.alt = 0;
    CR.sats = 0;
    CR.gX = 0;
    CR.gY = 0;
    CR.gZ = 0;
    CR.yaw = 0;
    CR.pitch = 0;
    CR.roll = 0;
    CR.totalEngineHours = 0;
    CR.totalFuelConsumption = 0;
    CR.currentFuelRate = 0;
    CR.totalIdleFuel = 0;
    CR.totalIdleHour = 0;
    CR.engineRPM = 0;
    CR.oilTemperature = 0;
    CR.coolantTemperature = 0;
    CR.vehicleSpeed = 0;
    CR.gearSelection = 0;
    //ssd1306Init();  
}

void loop() {
    checkJ1939();
    check9DOF();   
    checkGPS();
    //wifiCommunication(10000);
    //displayInfo();
    if (Serial2.available()) {
        String str = Serial2.readString();        
        Serial.println(str);
    }       
    if (timeStatus()!= timeNotSet) {
        if (now() != prevDisplay) { //update the display only if the time has changed
            prevDisplay = now();
            sprintf(CR.datetime, "%04d%02d%02d%02d%02d%02d", year(), month(), day(), hour(), minute(), second());                                
        }
    }    
}

void serialPortsInit() {
    Serial.begin(115200);
    Serial1.begin(115200);
    Serial2.begin(115200);
    Serial3.begin(GPSBaud);
}

void canInterruptInit(void) {
    while (CAN_OK != CAN.begin(CAN_250KBPS)) {            // init can bus : baudrate = 250k
        Serial.println("CAN BUS Shield init fail");
        Serial.println(" Init CAN BUS Shield again");
        delay(100);
    }
    Serial.println("CAN BUS Shield init ok!");
    //lcd.setCursor(0,0);
    //lcd.print("CAN BUS Init OK");
    attachInterrupt(0, MCP2515_ISR, FALLING); // start interrupt
}

void MCP2515_ISR(void) {
    flagRecv = 1;
}

void checkGPS() {

    unsigned long currentMillis = millis();    
    static unsigned long previousMillis2 = 0;
    if (currentMillis - previousMillis2 > 500 ) {
        while (Serial3.available() > 0) {
            if (gps.encode(Serial3.read())) saveGPSInfo();
        }
        previousMillis2 = currentMillis;
    }
    /*if (millis() > 5000 && gps.charsProcessed() < 10) {
        lcd.setCursor(0,1);
        lcd.print("GPS Invalid, check");
        while(true);
    }*/
}

void saveGPSInfo() {

    if (gps.date.isValid() && gps.time.isValid() && gps.time.isUpdated()) {
        int year = gps.date.year();
        byte month = gps.date.month();
        byte day = gps.date.day();
        byte hour = gps.time.hour();
        byte minute = gps.time.minute();
        byte second = gps.time.second();
        //sprintf(CR.datetime, "%04d%02d%02d%02d%02d%02d", gps.date.year(), gps.date.month(), gps.date.day(), gps.time.hour(), gps.time.minute(), gps.time.second()); 
        sprintf(CR.datetime, "%04d%02d%02d%02d%02d%02d", year, month, day, hour, minute, second);         
        //Serial.println(CR.datetime);
        if (timeStatus()== timeNotSet) {
            setTime(hour, minute, second, day, month, year);
            sprintf(CR.datetime, "%04d%02d%02d%02d%02d%02d", year, month, day, hour, minute, second);                    
        } 
        //currentSeconds = gps.time.second();
        //CR.sats = gps.satellites.value();
        //Serial.println(CR.datetime);
        //lcd.setCursor(0,2);
        //lcd.print(CR.datetime);   
    }
    
    if (gps.location.isValid() && gps.location.isUpdated()) {
        CR.lat = gps.location.lat();
        CR.lng = gps.location.lng();        
        //lcd.setCursor(0,3);
        //lcd.print(CR.lat, 5);
        //lcd.setCursor(10,3);
        //lcd.print(CR.lng, 5);
    }

    if (gps.altitude.isValid() && gps.altitude.isUpdated()) {
        CR.alt = gps.altitude.meters();
    }
    if (gpsFix.isUpdated()) {
        //gps.satellites.value()
        String satelliteNum = gpsFix.value();
        CR.sats = satelliteNum.toInt();      
        Serial.print("Satellite Number:");
        Serial.println(CR.sats);
        //lcd.setCursor(16, 2);
        //lcd.print(CR.sats);
    }

}

/*void displayInfo() {
    if (millis() % 1000 != 0) return;
    char currentTime[9];
    int thisHour = gps.time.hour() + 10;
    sprintf(currentTime, "%02d:%02d:%02d", thisHour, gps.time.minute(), gps.time.second());  
    if (currentSeconds >= 0 && currentSeconds < 12) { //0 to 19 seconds
        if (currentSeconds == 0) lcd.clear();
        lcd.setCursor(0, 0); //display title and time
        lcd.print("GPS Info    ");
        lcd.setCursor(12, 0);
        lcd.print(currentTime);
        lcd.setCursor(0,1); //latitude
        lcd.print("LAT:");
        lcd.setCursor(10,1);       
        lcd.print(CR.lat, 6);
        lcd.setCursor(0,2); //longitude
        lcd.print("LONG:");
        lcd.setCursor(10,2);               
        lcd.print(CR.lng, 6);       
        lcd.setCursor(0,3); //altidue & satellite Fix
        lcd.print("ALT:");
        lcd.setCursor(5,3);  
        lcd.print(CR.alt);
        lcd.setCursor(10,3);
        lcd.print("SATS:");
        lcd.setCursor(16,3);        
        lcd.print(CR.sats);       
         
    } else if (currentSeconds >= 12 && currentSeconds < 24) { //10 to 20 seconds
        if (currentSeconds == 12) lcd.clear();
        lcd.setCursor(0, 0);  //display title and time
        lcd.print("CAN info1   ");      
        lcd.setCursor(12, 0);
        lcd.print(currentTime);       
        
        lcd.setCursor(0,1);
        lcd.print("total eng Hr:");
        lcd.setCursor(14,1);
        lcd.print(CR.totalEngineHours);

        lcd.setCursor(0,2);
        lcd.print("ttl fuel con:");
        lcd.setCursor(14,2);
        lcd.print(CR.totalFuelConsumption);
                
        lcd.setCursor(0,3);
        lcd.print("cur fuelrate:");
        lcd.setCursor(14,3);
        lcd.print(CR.currentFuelRate); 
        
    } else if (currentSeconds >= 24 && currentSeconds < 36) { //20 to 30 seconds
      
        if (currentSeconds == 24) lcd.clear();
        lcd.setCursor(0, 0);  //display title and time
        lcd.print("CAN info2   ");      
        lcd.setCursor(12, 0);
        lcd.print(currentTime);       
        
        lcd.setCursor(0,1);
        lcd.print("ttl Idl Fuel:");
        lcd.setCursor(13,1);
        lcd.print(CR.totalIdleFuel);

        lcd.setCursor(0,2);
        lcd.print("ttl Idl Hour:");
        lcd.setCursor(13,2);
        lcd.print(CR.totalIdleHour);
                
        lcd.setCursor(0,3);
        lcd.print("engine RPM:");
        lcd.setCursor(13,3);
        lcd.print(CR.engineRPM); 
                                       
    } else if (currentSeconds >= 36 && currentSeconds < 48 ) { //30 to 45 seconds

        if (currentSeconds == 36) lcd.clear();
        lcd.setCursor(0, 0);  //display title and time
        lcd.print("CAN info3   ");      
        lcd.setCursor(12, 0);
        lcd.print(currentTime);         
    
        lcd.setCursor(0,1);        
        lcd.print("oil Temp:");
        lcd.setCursor(12,1);
        lcd.print(CR.oilTemperature);

        lcd.setCursor(0,2);
        lcd.print("coolTemp:");
        lcd.setCursor(12,2);
        lcd.print(CR.coolantTemperature);
                
        lcd.setCursor(0,3);
        lcd.print("spd:");
        lcd.setCursor(5,3);
        lcd.print(CR.vehicleSpeed);    
           
        lcd.setCursor(9, 3);  //display title and time
        lcd.print("gearSel:");      
        lcd.setCursor(16, 0);
        lcd.print(CR.gearSelection);       
     
    } else { //40 to 59 seconds

        if (currentSeconds == 48) lcd.clear();
        lcd.setCursor(0, 0); //display title and time
        lcd.print("GForce info ");        
        lcd.setCursor(12, 0);
        lcd.print(currentTime); 

        lcd.setCursor(0,1);        
        lcd.print("gX:");
        lcd.setCursor(5,1);
        lcd.print(CR.gX);
               
        lcd.setCursor(10,1);        
        lcd.print("gY:");
        lcd.setCursor(15,1);
        lcd.print(CR.gY);

        lcd.setCursor(0,2);        
        lcd.print("gZ:");
        lcd.setCursor(5,2);
        lcd.print(CR.gZ);

        lcd.setCursor(10,2);        
        lcd.print("yaw:");
        lcd.setCursor(15,2);
        lcd.print(CR.gZ);

        lcd.setCursor(0,3);        
        lcd.print("pch:");
        lcd.setCursor(5,3);
        lcd.print(CR.gZ);
        
        lcd.setCursor(10,3);        
        lcd.print("rol:");
        lcd.setCursor(15,3);
        lcd.print(CR.gZ);
        
    }
    
}*/

void checkJ1939() {
  
   if(flagRecv) {        // check if get data
        flagRecv = 0;     // clear flag
        while (CAN_MSGAVAIL == CAN.checkReceive()) { //read from J1939
            // read data,  len: data length, buf: data buf
            INT32U can_id = CAN.getCanId();  
            CAN.readMsgBuf(&len, buf);         

            switch (can_id) {
              
                case 0xFEE9: //total Fuel Consumption             
                    memset(&(CR.totalFuelConsumption), 0, sizeof(unsigned long));
                    memcpy(&(CR.totalFuelConsumption),buf+4,4);
                    CR.totalFuelConsumption /= 2;      
                    //Serial.println(CR.totalFuelConsumption);
              
                break;
                
                case 0xFEF2: //Fuel Rate
                    memset(&(CR.currentFuelRate), 0, sizeof(unsigned int));
                    memcpy(&(CR.currentFuelRate), buf, 2);
                    CR.currentFuelRate /= 20;           
                    //Serial.println(CR.currentFuelRate);                      
                break;            
  
                case 0xFEDC: //total Idle Fuel and total Idle hours
                    memset(&(CR.totalIdleFuel), 0, sizeof(unsigned long));
                    memset(&(CR.totalIdleHour), 0, sizeof(unsigned long));                
                    memcpy(&(CR.totalIdleFuel), buf, 4);
                    CR.totalIdleFuel /= 2;    
                    memcpy(&(CR.totalIdleHour), buf+4, 4);
                    CR.totalIdleHour /= 20;                                
                break;
  
                case 0xFEE5: //total engine hour
                    memset(&(CR.totalEngineHours), 0, sizeof(unsigned long));
                    memcpy(&(CR.totalEngineHours), buf, 4);
                    CR.totalEngineHours /= 20;                
                break;
  
                case 0xF004: //total engine RPM
                    memset(&(CR.engineRPM), 0, sizeof(unsigned int));
                    memcpy(&(CR.engineRPM), buf+3, 2);
                    CR.engineRPM /= 8;                
                break;
  
                case 0xFEEE: //coolant Temperatue, oil Temperature
                    /*Serial.println(can_id, HEX);
                    for (int i = 0; i != 8; ++i) {
                        Serial.print(buf[i], HEX);
                        Serial.print("\t");
                    }
                    Serial.println();*/
                    CR.coolantTemperature = buf[0] - 40;
                    //Serial.println(CR.coolantTemperature);                    
                    memset(&(CR.oilTemperature), 0, 2);
                    memcpy(&(CR.oilTemperature), buf+2, 2);
                    CR.oilTemperature /= 32;     
                    CR.oilTemperature -= 273;
                break;
  
                case 0xFF23: //vehicle speed, gear selection
                    CR.vehicleSpeed = buf[2] / 5;
                    CR.gearSelection = buf[0]; 
                break;              
                       
            }                    
        }
    }
}

void check9DOF() {
  
    /*if (Serial1.available()) { // read from gyroscope
        //int inByte = Serial1.read();
        //Serial.write(inByte);
        readStr = Serial1.readStringUntil('\n');
        //Serial.println(readStr);
        CR.gX    = getValue(readStr, ',', 0).toInt();
        CR.gY    = getValue(readStr, ',', 1).toInt();
        CR.gZ    = getValue(readStr, ',', 2).toInt();
        CR.yaw   = getValue(readStr, ',', 3).toFloat();
        CR.pitch = getValue(readStr, ',', 4).toFloat();
        CR.roll  = getValue(readStr, ',', 5).toFloat();      

        // send data:  id = 0x01, standrad frame, data len = 8, stmp: data buf
        memset(stmp, 0, 8);
        memcpy(stmp, &(CR.gX), 4);
        memcpy(stmp+4, &(CR.gY), 4);
        CAN.sendMsgBuf(0x00FFC100, 1, 8, stmp);    
          
        memset(stmp, 0, 8);
        memcpy(stmp, &(CR.gZ), 4);
        memcpy(stmp+4, &(CR.yaw), 4);
        CAN.sendMsgBuf(0x00FFC200, 1, 8, stmp);     
         
        memset(stmp, 0, 8);
        memcpy(stmp, &(CR.pitch), 4);
        memcpy(stmp+4, &(CR.roll), 4);            
        CAN.sendMsgBuf(0x00FFC300, 1, 8, stmp);
    } else { //time out to receive CAN message
        recvTick = millis();
        if (recvTick - lastRecvTick > 10000) {
            Serial1.print("a"); //print any char to start read CAN
            lastRecvTick = recvTick;
        }
    }*/
    sensors_event_t accel_event;
    sensors_event_t mag_event;
    sensors_vec_t   orientation;
    unsigned long currentMillis = millis();
    static unsigned long previousMillis1 = 0;
  
    /* Read the accelerometer and magnetometer */
    accel.getEvent(&accel_event);
    mag.getEvent(&mag_event);

    if (dof.fusionGetOrientation(&accel_event, &mag_event, &orientation) && currentMillis - previousMillis1 > 500 ) {
        /* 'orientation' should have valid .roll and .pitch fields */
        /*Serial.print(F("Accelerometer: "));
        Serial.print(accel_event.acceleration.x);
        Serial.print(F(" "));
        Serial.print(accel_event.acceleration.y);
        Serial.print(F(" "));
        Serial.print(accel_event.acceleration.z -9.80);
        Serial.print(F("---"));
        Serial.print(F("Orientation: "));
        Serial.print(orientation.roll);
        Serial.print(F(" "));
        Serial.print(orientation.pitch);
        Serial.print(F(" "));
        Serial.print(orientation.heading);
        Serial.println(F("")); */     
        CR.gX    = (int)(accel_event.acceleration.x * 100);
        CR.gY    = (int)(accel_event.acceleration.y * 100);
        CR.gZ    = (int)(accel_event.acceleration.z * 100) - 980;
        CR.yaw   = (int)(orientation.heading * 100);
        CR.pitch = (int)(orientation.pitch * 100);
        CR.roll  = (int)(orientation.roll * 100);         
 
        // send data:  id = 0x01, standrad frame, data len = 8, stmp: data buf
        memset(stmp, 0, 8);
        memcpy(stmp, &(CR.gX), 4);
        memcpy(stmp+4, &(CR.gY), 4);
        CAN.sendMsgBuf(0x00FFC100, 1, 8, stmp);    
          
        memset(stmp, 0, 8);
        memcpy(stmp, &(CR.gZ), 4);
        memcpy(stmp+4, &(CR.yaw), 4);
        CAN.sendMsgBuf(0x00FFC200, 1, 8, stmp);     
         
        memset(stmp, 0, 8);
        memcpy(stmp, &(CR.pitch), 4);
        memcpy(stmp+4, &(CR.roll), 4);            
        CAN.sendMsgBuf(0x00FFC300, 1, 8, stmp);
                    
        previousMillis1 = currentMillis;

    }
        
}

void wifiCommunication(int ticksToWait) {

    tick = millis();
    if (tick - lasttick > ticksToWait) {  //send every 30 seconds
        
        char buf[256];        
        char lat_str[10];
        char lng_str[10];
        dtostrf(CR.lat,   9, 5, lat_str);
        dtostrf(CR.lng,   9, 5, lng_str);                                                                                                              
        //Serial.println("start to write to remote server");
        memset(buf, 0, sizeof(char));
        sprintf(buf, "%s,%s,%s,%d,%d,%d,%d,%d,%d,%d,%lu,%lu,%u,%lu,%lu,%u,%d,%d,%d,%d", 
        CR.datetime, lat_str, lng_str, CR.sats, CR.gX, CR.gY, CR.gZ, CR.yaw, CR.pitch, CR.roll,
        CR.totalEngineHours, CR.totalFuelConsumption, CR.currentFuelRate, CR.totalIdleFuel, CR.totalIdleHour,
        CR.engineRPM, CR.oilTemperature, CR.coolantTemperature, CR.vehicleSpeed, CR.gearSelection);
        //Serial.println(buf);
        Serial2.println(buf);  
        //Serial2.flush();
        lasttick = tick;

       
    } else if (abs(CR.gX) > 1000 || abs(CR.gY) > 1000 || abs(CR.gZ) > 1000) {
      
        char buf[256];        
        char lat_str[10];
        char lng_str[10];
        dtostrf(CR.lat,   9, 5, lat_str);
        dtostrf(CR.lng,   9, 5, lng_str);                                                                                                              
        memset(buf, 0, sizeof(char));
        sprintf(buf, "%s,%s,%s,%d,%d,%d,%d,%d,%d,%d,%lu,%lu,%u,%lu,%lu,%u,%d,%d,%u,%u", 
        CR.datetime, lat_str, lng_str, CR.sats, CR.gX, CR.gY, CR.gZ, CR.yaw, CR.pitch, CR.roll,
        CR.totalEngineHours, CR.totalFuelConsumption, CR.currentFuelRate, CR.totalIdleFuel, CR.totalIdleHour,
        CR.engineRPM, CR.oilTemperature, CR.coolantTemperature, CR.vehicleSpeed, CR.gearSelection);
        Serial2.println(buf);  
        //Serial2.flush();

    }
    

  
    /*if (Serial.available()) {
      int inByte = Serial.read();
      Serial2.write(inByte);
      //Serial.write(inByte);
    }*/
}

void initSensors() {
    if(!accel.begin()) {
        /* There was a problem detecting the LSM303 ... check your connections */
        Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
        while(1);
    }
    if(!mag.begin()) {
        /* There was a problem detecting the LSM303 ... check your connections */
        Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
        while(1);
    }
}


/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
