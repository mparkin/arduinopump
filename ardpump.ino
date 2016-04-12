#include <modbus.h>
#include <modbusDevice.h>
#include <modbusRegBank.h>
#include <modbusSlave.h>
#include <Wire.h>


modbusDevice regBank;
modbusSlave slave;

#define RS485TxEnablePin 2
#define RS485Baud 9600
#define RS485Format SERIAL_8E1

/*
modbus registers follow the following format
00001-09999  Digital Outputs, A master device can read and write to these registers
10001-19999  Digital Inputs, A master device can only read the values from these registers
30001-39999  Analog Inputs, A master device can only read the values from these registers
40001-49999  Analog Outputs, A master device can read and write to these registers 

Analog values are 16 bit unsigned words stored with a range of 0-32767
Digital values are stored as bytes, a zero value is OFF and any nonzero value is ON
*/

int rStartStop       = 1;      //start-stop 
int rBrake           = 2;      //brake 
int rDir             = 3;      //dir 
int rMode            = 4;      //mode 
int rAlarmReset      = 5;      //alarmReset 
int rTurnMode        = 6;      //turnMode 
int rDurationMode    = 7;      //durationMode 
int rHomeMode        = 8;      //homeMode 
int rAlarm           = 10001;  //alarm
int rPulseCounts     = 30001;  //pulsecounts
int rHomeCounts      = 30002;  //homeFlagCounts
int rSpeed           = 40001;  //speed
int rS1              = 40002;  //S1
int rS2              = 40003;  //S2
int rS3              = 40004;  //S3
int rS4              = 40005;  //S4
int rS5              = 40006;  //S5
int rS6              = 40007;  //S6
int rT1              = 40008;  //T1
int rT2              = 40009;  //T2
int rT3              = 40010;  //T3
int rT4              = 40011;  //T4
int rT5              = 40012;  //T5
int rTrn             = 40013;  //turns
int rPeriod          = 40014;  //time Period
int startStop        = 3;
int brake            = 4;
int dir              = 5;
int mode             = 6;
int alarmReset       = 7;
int pulseCount       = 8;
int homeSensor       = A1;
int flagSensor       = A0;
// I2C
// pin A4-A5 reserved for I2C. DAC is I2C and serial eeprom will be on I2C
int alarm            = A6;
int bytecount        = 0;
int flagcounter      = 0;
int revcounter       = 0;
int stopcount        = 0;
int SlaveID          = 80;
bool homeFlag        = false;
bool hispeed         = false;
bool Debug           = true;
int speed            = 2047;
unsigned long period = 1000;
int trn              = 0;
bool started         = false;
int DACaddress       = 0x62;
float turnconstant   = 26.42;
float intv           = 0.0;
float T1 = 31, T2 = 31, T3 = 31,T4=31,T5=31;
float S1 = 1000,S2 = 1000, S3 = 1000, S4 = 1000,S5 =1000, S6 = 1000;
bool turnMode = false, durationMode=false,homeMode=false;

void pumpSpeed(uint16_t sval) {
  if(!Debug) {
   if (sval > 4000) sval = 4000;
   Wire.beginTransmission(DACaddress); // transmit to slave device #4
   Wire.write(0x40);
   Wire.write(sval / 16);
   Wire.write((sval %16)<<4);
   Wire.endTransmission();    // stop transmitting
 }
}

void twistedSpot() {
    if (homeFlag){
       if(homeMode){
          digitalWrite(startStop, HIGH);
          return;
       }
       if (!hispeed){
           hispeed = true;
           int startcount = revcounter;
           pumpSpeed(S1);
           stopcount = T1+revcounter;
           while(revcounter < stopcount)
           {
               delay(1);
           };
           pumpSpeed(S2);
           stopcount = T2+revcounter;
           while(revcounter < stopcount)
           {
               delay(1);
           };
           pumpSpeed(S3);
           stopcount = T3+revcounter;
           while(revcounter < stopcount)
           {
               delay(1);
           };
           pumpSpeed(S4);
           stopcount = T4+revcounter;
           while(revcounter < stopcount)
           {
               delay(1);
           };
           pumpSpeed(S5);
           stopcount = T5+revcounter;
           while(revcounter < stopcount)
           {
               delay(1);
           };
           pumpSpeed(speed);
           homeFlag=false;
       }
    }
    if (hispeed & !homeFlag){
          pumpSpeed(speed);
          hispeed = false;
    }
}

void updatePins(){
  speed  = regBank.get(rSpeed);
  S1     = regBank.get(rS1);  //S1
  S2     = regBank.get(rS2);  //S2
  S3     = regBank.get(rS3);  //S3
  S4     = regBank.get(rS4);  //S4
  S5     = regBank.get(rS5);  //S5
  S6     = regBank.get(rS6);  //S6
  T1     = regBank.get(rT1);  //T1
  T2     = regBank.get(rT2);  //T2
  T3     = regBank.get(rT3);  //T3
  T4     = regBank.get(rT4);  //T4
  T5     = regBank.get(rT5);  //T5
  trn    = regBank.get(rTrn);
  period = regBank.get(rPeriod);
  regBank.set(rAlarm,(digitalRead(alarm ? 1:0)));
  turnMode     = (regBank.get(rTurnMode)           == 1 ? true : false); 
  durationMode = (regBank.get(rDurationMode)       == 1 ? true : false);
  homeMode     = (regBank.get(rHomeMode)           == 1 ? true : false);
  digitalWrite(startStop,regBank.get(rStartStop)   == 1 ? HIGH : LOW); //start-stop
  digitalWrite(brake,regBank.get(rBrake)           == 1 ? HIGH : LOW); //brake
  digitalWrite(dir,regBank.get(rDir)               == 1 ? HIGH : LOW); //dir
  digitalWrite(mode,regBank.get(rMode)             == 1 ? HIGH : LOW); //mode
  digitalWrite(alarmReset,regBank.get(rAlarmReset) == 1 ? HIGH : LOW); //alarmReset
}

void pumpRun() {
  updatePins();
  twistedSpot();
  regBank.set(rPulseCounts,pulseCount);
  regBank.set(rHomeCounts,flagcounter);
}

void setup()
{   

//Assign the modbus device ID.  
  regBank.setId(SlaveID);
  pinMode(startStop, OUTPUT);                // sets pin as output
  digitalWrite(startStop,HIGH);              // set pump to stopped
  pinMode(brake, OUTPUT);                    // sets pin as output
  pinMode(dir, OUTPUT);                      // sets pin as output
  pinMode(mode, OUTPUT);                     // sets pin as output
  pinMode(alarmReset, OUTPUT);               // sets pin as output
  pinMode(pulseCount, INPUT_PULLUP);         // sets pin as input
  pinMode(alarm, INPUT_PULLUP);              // sets pin as input
  pinMode(homeSensor, INPUT_PULLUP);         // sets pin as input
//  pinMode(flagSensor, INPUT_PULLUP);       // sets pin as input
  attachInterrupt(homeSensor, homeSet, FALLING);
//  attachInterrupt(homeSensor, homeReset, RISING);
//  attachInterrupt(flagSensor, flagCount, FALLING);
  attachInterrupt(pulseCount, revCount, FALLING);
//  Wire.setSpeed(CLOCK_SPEED_400KHZ);
  Wire.begin();
          
//Add Digital Output registers 00001-00016 to the register bank
  regBank.add(10001);         //alarm
 
//Add Digital Input registers 10001-10008 to the register bank
  regBank.add(rStartStop);    //start-stop 
  regBank.add(rBrake);        //brake 
  regBank.add(rDir);          //dir 
  regBank.add(rMode);         //mode 
  regBank.add(rAlarmReset);   //alarmReset 
  regBank.add(rTurnMode);     //turnMode 
  regBank.add(rDurationMode); //durationMode 
  regBank.add(rHomeMode);     //homeMode 

//Add Analog Input registers 30001-10010 to the register bank
  regBank.add(rPulseCounts);  //pulsecounts
  regBank.add(rHomeCounts);   //homeFlagCounts

//Add Analog Output registers 40001-40020 to the register bank
  regBank.add(rSpeed);        //speed
  regBank.add(rS1);           //S1
  regBank.add(rS2);           //S2
  regBank.add(rS3);           //S3
  regBank.add(rS4);           //S4
  regBank.add(rS5);           //S5
  regBank.add(rS6);           //S6
  regBank.add(rT1);           //T1
  regBank.add(rT2);           //T2
  regBank.add(rT3);           //T3
  regBank.add(rT4);           //T4
  regBank.add(rT5);           //T5
  regBank.add(rTrn);          //trn
  regBank.add(rPeriod);       //period  
/* Initialize modbus registers with defaults */
  regBank.set(rT1,T1);
  regBank.set(rT2,T2);
  regBank.set(rT3,T3);
  regBank.set(rT4,T4);
  regBank.set(rT5,T5);
  regBank.set(rS6,S6);
  regBank.set(rS1,S1);
  regBank.set(rS2,S2);
  regBank.set(rS3,S3);
  regBank.set(rS4,S4);
  regBank.set(rS5,S5);
  regBank.set(rSpeed,speed);
/* 
Assign the modbus device object to the protocol handler
This is where the protocol handler will look to read and write
register data.  Currently, a modbus slave protocol handler may
only have one device assigned to it.
*/
  slave._device = &regBank;
  slave.setBaud(&Serial,RS485Baud,RS485Format,RS485TxEnablePin);
}

void loop()
{
     slave.run();
     pumpSpeed(speed);  
     pumpRun();
}

void homeSet()
{
  homeFlag = true;
  flagcounter++;
}

void homeReset()
{
  homeFlag = false;
}

void revCount()
{
    revcounter++;
}

void flagCount()
{
    flagcounter++;
}

