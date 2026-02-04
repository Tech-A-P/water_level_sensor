/*
authors: Pietro Mascherpa, Fabio Brandalese, Roberto Garza 
contact: roberto.garza@uni-konstanz.de
copyright: CC BY-NC-SA 4.0
date current version: 2026.02.03 (YYYY/MM/DD)
backend: ThingSpeak
Hardwere: 3DM 2025.06 SmartWT v2

The first time that you charge this program make a send with the computer alimentation, and the SmartWT swich-off for make correct inizialization
Check the TechAP instructions book for information about the hardware component of the system.
*/

#include "SIM900.h"
#include <SoftwareSerial.h>
#include "inetGSM.h"
#include "SerialRead.h"
#include <String.h>
#include <EEPROM.h>
#include <math.h>

#define PING_PIN 3                               // ultrasound emitter
#define ECHO_PIN 4                               // ultrasound receiver
#define TIMER_PIN 6                              // stand-by timer
#define GSM_SHIELD_PIN 9                         // switch for GSM shield
#define BATT_PIN A7                              // analogPin for the measure of the battery tension
#define EEPROMinit 0                             // address of the Eeprom for verify if is the first time of initialization of the eeprom. The first upload of the program we inizialize the eeprom value and write here 1. So for the rest of the use we know that the eeprom was inizialize correctly the first time. We use a 4byte value for have a low probability that casually the first time of the eeprom read his value is casually equal to 1. Whit 4 byte we have 0.0000023% of probability
#define EEPROMcount 4                            // address of the Eeprom when allocate the number (int) when EepromRead reach 100000 value
/*
One position of the Eeprom use 1 Byte so you have to consider the typo of data that you are writing in.
The Eeprom position depend form the type of data that you would to write in. If i would to write a folat value I have to mantain 4 byte, so 4 position
Eeprom space 1 byte 1 position:
  - for int value: 2 byte 
  - for long value: 4 byte
  - for float value: 4 byte
  - for double value: 4 byte
But if you change the  board you have to verfy this number
*/

int EEPROM_Interval = 6;                        // address of the Eeprom where allocate the number of accension (ulong)
int EEPROM_LastSend = 10;                       // address of the Eeprom where allocate the number of accension at the last send (ulong)
int EEPROM_Measure = 14;                        // address of the Eeprom where allocate the old distance measurment (float)
const int EEPROMstep = 12;                      // distance in Eeprom position from the first EepromInterval position to the first Eeprom free position, for new allocation of the Eeprom values, when the first allocations are full. Is the number of Eeprom position used starting from the posiition 6 because the first 6 eeprom address are used for the inizialized control

InetGSM inet;

// TODO: try to optimize data structures to save memory

// constants for data communication
const char* SIM_APN = "iot.1nce.net";            // the PN of your SIM
const String THINGSPEAK_API_KEY = "YOUR_API_KEY";           // write API key from thingspeak
const char* THINGSPEAK_ENDPOINT_ROOT = "api.thingspeak.com";
const String THINGSPEAK_FIELD_NUMBER = "YOUR_FIELD_NUMBER";      // ThingSpeak field index for the water level value
const String THINGSPEAK_BATTERY_NUMBER = "YOUR_BATTERY_NUMBER";    // ThingSpeak field index for the battery value 
const int MAX_NUMBER_ATTEMPTS = 3;                // Max number of trials to send, after this if the send is not ok sensor goes off for the interval time

// variables for data communication
char URL[100];                                    //if thare are more of 3 field URL must be larger then 100
char mesg[50];
char serial_text[20];
int numdata;
long h = 30;
boolean started = false;

// variables for measurement
float delay_sensor;                                // time needed for the signal to reach detector
const int number_repetitions = 10;                 // number of measures to median to get a single value to store
float distance_array[number_repetitions];          // number_repetitions elements array containing single measures
const int speed_sound_air = 343;                   // speed of sound in the air at 24C° [m/s]
const int max_distance = 450;                      // maximum distance recorded reliably by the sensor
float single_distance;                             // single measure [cm]
float distance_median;                             // median measure [cm]
float distance;                                    // median distance corrected and ready to send [cm]
const float height_zero = 0;                       // height of the detector from the ground
float measure_offset = 2.29;                       // measuring measure_offset of the sensor | TODO-USER: set as distance between the surface of the sensor and the edge of the box (Fig. 1)
float correction_factor = 1.028;                   // correct for distance of the target from sensor (increase measure by 2,8%)

// variables for battery tension check
float Volt;                                        // variable for save the volt of the battery
int AnalogBatt;                                    // variable where store the analog read of the battery pin
const float PartitorFactor = 1.39;                 // value that multiplicated for AnalogBatt give back the value on the tension partitor: egual to (R1+R2)/R1  where R1 is the resistence connected to GND and R2 is the resistance connected to +12
const float SogliaBatt = 3.70;                     // threshold when stop the send of the data
const float Vdelta = 0 ;                           // the difference between tue battery voltage and the voltage measured on the two pin wher arduino measure

// variables for count the number of start and start after n start
unsigned long EepromInit;                          // variable to check whether the EEPROM has been initialised correctly
int EepromCount;                                   // variabile dove conto il numero di volte in cui ho superato il valore massimo di eeprom limit
unsigned long EepromRead;                          // variabile in cui memorizzo il valore vletto nella eeprom del numero accensioni
unsigned long LastEepromSend;                      // variabile in cui memorizzo il valore della Eeprom all'ultimo invio
int EepromNumber;                                  // variabile dove calcolo la differenza tra il valore Eeprom e il valore della Eeprom all'ultimo invio
float EepromDistance;                              // variabile in cui memorizzo l'ultimo valore letto dal sensore DEVO USARE PUT E GET PER DATI SOPRA I 2 BYTE

// 12 con resistenza da 5 min significa 1 ore 
int EepromStart = 12;                              // here I enter the maximum number of ignitions reached, which I want it to send.
const float Tolerance = 10 ;                       // constant that tells me the Tolerance to give to the measurement so that it is considered different from the previous one
float abs_distance;                                // this is used for conditions that check whether the value read is equal to the previous one. The absolute value is needed because for negative values, the results of the if statement become opposite.

// variables for check the correct send of measure
int number_attempt_send = 0;                       // number of attempts to measure and send the data
String status = "";                                // communication status


void setup() {
// Serial monitor inzialization
 Serial.begin(9600);

// ***************************************************
// CHECK BATTERY TENSION
 if (!checkBatteryTension()){
   Serial.println(F("Battery low! Switch off."));
   standBy();
 }

// ***************************************************
// TEST EEPROM INIZIALIZATION and if is at his limit of use reculate the correct address
 InitEeprom();

// ***************************************************
// CHECK NUMBER OF START & IF MASURE IS CHANGE
  // set pins for measurement
  pinMode(PING_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Get the measure
  distance = getMeasure();

  // take the number of start from the position EEPROM_Interval and save it in EepromRead
  EEPROM.get(EEPROM_Interval, EepromRead);
  // take the last distance value
  EEPROM.get(EEPROM_Measure, EepromDistance);
  // take the last value od EepromRead when send the measure
  EEPROM.get(EEPROM_LastSend, LastEepromSend);
  // increment the number for count the current starting
  EepromRead = EepromRead +1;
  // difference between EpromRead (number of start of the sensor) and LastEepromSend the last number of start when the masure was send)
  EepromNumber = EepromRead - LastEepromSend;
  // absolute value of distance for make the next if condition work correctly. because distance could be positive or negative
  abs_distance = fabs(distance);
  Serial.print(F("Number of attemps:"));
  Serial.println(EepromNumber);

// test if is the moment to send the measure. 
  
  // check if EepromNumber (the number of start from the last send) is minor of EepromStart (the number of accension for send measure)
  bool accensioniOK = (EepromNumber < EepromStart);
  // check if the measure is in the range of tolerance
  bool misuraOK = (abs_distance > EepromDistance - Tolerance) && (abs_distance < EepromDistance + Tolerance);
  
  // debug on serial monitor
  Serial.print(F("Reach the number of accension for start? (0 if true) ")); 
  Serial.println(accensioniOK);
  Serial.print(F("The masure is change significantly for the last measure? (0 if true) ")); 
  Serial.println(misuraOK);

  // Save the new value of EepromRead (the number of start updated) and abs_distance (the last absolute measure taken) in Eeprom 
  EEPROM.put(EEPROM_Interval, EepromRead);
  EEPROM.put(EEPROM_Measure, abs_distance);
  delay(10);

  // Check if both the condition are true (OFF the SmartWT) if only one is false is the  time to send
  if (accensioniOK && misuraOK) {
      //NOT time to send: measure isn't change AND the number of accension isn't reach
        Serial.println(F("EEPROM OFF!"));
        // SmartWT off
        standBy();
      }else{
      //TIME to send: measure is change OR the number of accension is reach
        // write the new value of Eeprom_LastSend (the last number of start when the masure was send)
        EEPROM.put(EEPROM_LastSend, EepromRead);
      }

// if the program arrive here the previus IF is false and is the time to send. Bacause with StandBy() the SmartWT go off
 Sim900on();
}

void loop()
{
  number_attempt_send = number_attempt_send + 1;
  Serial.print(F("# Attempts: "));
  Serial.println(number_attempt_send);

  // communication check and possible second attempt of measuring and sending data
  if (status == "200") {
   // if the status is 200 the data are correctly send 
    Serial.println(F("Data correctly sent"));   
    //SmartWT off, don't delete. Is here for simplyfy the proram read
    standBy();              
  } else {
  // the status is not 200 is necessary to send masure
    if (started == false) {
      setComShield();                          // GSM shield configuration
    }
    if (started == true) {
      sendData();                              // data communication & response from server
    }
  }  
  
  // max number of attempts for measuring and sending data, if all number of attempts failes, just stand by the system
  if(number_attempt_send > MAX_NUMBER_ATTEMPTS){                   
    standBy();
  }
}

//function for swich off SmartWT
void standBy() {
  digitalWrite(TIMER_PIN, HIGH);                  // switch timer off
  Serial.println(F("System in stand by"));
  delay(3000);
}

// function for check the battery tension. Return TRUE if battery is over the tension limit , FALSE if the battery is down. And calculate the battery tension
bool checkBatteryTension(){
  // read the tension on the analogic pin
  AnalogBatt = analogRead(BATT_PIN);
  // convert the value read (0-1023) in tension (0-5V) the value 5330 is measured on the arduino board.
  Volt = map(AnalogBatt, 0, 1023, 0, 5080 );
  // we add Vdelta because when the courrent flow throw the TPL there is a drop of Vdelta
  Volt = (Volt/1000)+ Vdelta;
  // calulation of the tension of the battery over the tension partitor
  Volt = Volt * PartitorFactor;
  // Stampa la Volt sul monitor seriale
  Serial.print(F("Tension read on the battery: "));
  // Serial.println(AnalogBatt);
  Serial.println(Volt);
  // if the tension go down the minimum level switch off the arduino
   if (Volt < SogliaBatt) {
      return false;
    }else{
      return true;
    }
}

// function for initialize the Eeprom value at the first program upload and when the Eeprom reach limit
void InitEeprom() {
EEPROM.get(EEPROMinit, EepromInit);
Serial.println(F("EEPROM is inizialized? if is 1 yes:"));
Serial.println(EepromInit);

if (EepromInit != 1){
//if EepromInit is not 1 is the first time that this arduino run this program. So is necessary to inizialize the Eeprom value
  Serial.println(F("EEPROM Inizialized"));
  // set EEPROMinit = 1, so i know that Eeprom is inizialized at the first upload
  EepromInit = 1;
  EEPROM.put(EEPROMinit, EepromInit);
  
  // set EEPROMcount = 0 because is the first start and start to use the first eeprom position
  EepromCount = 0;
  EEPROM.put(EEPROMcount, EepromCount);
 
  // set EepromRead = 0 because is the first start
  EepromRead = 0;
  EEPROM.put(EEPROM_Interval, EepromRead);
  
  // set LastEepromSend = 0 because no send are done
  LastEepromSend = 0;
  EEPROM.put(EEPROM_LastSend, LastEepromSend);

  // ser EerpomDistance = 0 because is the first measure
  EepromDistance = 0;
  EEPROM.put(EEPROM_Measure, EepromDistance);

}

// check not to overcome the hardware Eeprom limit of 100 000
if (EepromRead > 100000){
//if the 100 000 limit is over swich the Eeprom adress to the next position
  EEPROM.get(EEPROMcount, EepromCount);
  // increase EepromCount of 1. This variable is the one that is use for calculate the EepromAdress at every Start
  EepromCount = EepromCount + 1;
  // Save EepromCount in EEPROM
  EEPROM.put(EEPROMcount, EepromCount);
  Serial.println(F("EepromAdress updated for reaching the 100000 limit for the turn number:"));
  Serial.println(EepromCount);
}

Serial.println(F("Calculating the EEPROM address, EEPROM_Interval address is:"));
// calculate the EEPROM address using EepromCount
 EEPROM.get(EEPROMcount, EepromCount);
 EEPROM_Interval = EEPROM_Interval + (EepromCount * EEPROMstep);     // PROGRAM adress of the Eeprom when allocate the number of accension 
 EEPROM_LastSend = EEPROM_LastSend + (EepromCount * EEPROMstep);     // PROGRAM adress of the Eeprom when allocate the number of accension at the last send
 EEPROM_Measure = EEPROM_Measure + (EepromCount * EEPROMstep);       // PROGRAM adress of the Eeprom when allocate the old distance measurment
 Serial.println(EEPROM_Interval);

}

// function of set the SIM900 connection
void setComShield() {
  // 2400 is absolutly necessary for a correct communnication
  if (gsm.begin(2400)) {
    Serial.println("\nstatus=READY");
    started = true;
    Serial.println(F("Set Shield for Internet Connection"));
  } else {
    Serial.println("\nstatus=IDLE");
  }
}

// function for sand data
void sendData() {
  // sanity check
  if (inet.attachGPRS(SIM_APN, "", "")) {
    Serial.println("status=ATTACHED");
  } else {
    Serial.println("status=ERROR");
  }

  // set communication
  gsm.SimpleWriteln("AT+CIFSR");                                  // Read IP address.
  delay(1000);
  gsm.WhileSimpleRead();                   
  delay(500);                                                     // Read until serial buffer is empty.

  // Prepare the string for send the corrected measure + battery voltage
  String url = "/update?api_key=" + THINGSPEAK_API_KEY + "&field" + THINGSPEAK_FIELD_NUMBER + "=" + String(distance , 1) + "&field" + THINGSPEAK_BATTERY_NUMBER + "=" + String(Volt , 1) ; 

  // convert the string to char
  url.toCharArray(URL, 100);
  Serial.println(F("URL"));
  // if the URL is not show is not corrected generate. Check if the RAM memory is full. 
  Serial.println(URL);    

  numdata = inet.httpGET(THINGSPEAK_ENDPOINT_ROOT, 80, URL, mesg, h);

  //If numdata is >0 the send is done
  if (numdata > 0) {
    Serial.println(F("SEND OK, Status = 200"));
    status = "200";
   } else {
    Serial.println(F("SEND FAILED"));
   }
}

// function for make the measurment, give distance as result
float getMeasure() {
  for (int j = 0; j < number_repetitions; j++) {
    digitalWrite(PING_PIN, LOW);
    //dont touch this delays are important for clean measure
    delayMicroseconds(100);
    digitalWrite(PING_PIN, HIGH);
    //dont touch this delays are important for clean measure
    delayMicroseconds(30);
    digitalWrite(PING_PIN, LOW);
    delay_sensor = pulseIn(ECHO_PIN, HIGH);

    //convert the time of fly of the sensor in distance
    distance_median = (delay_sensor * speed_sound_air / 20000);

    //if the distance measured is over the max_distance that sensor can read set 0
    if (distance_median > max_distance) {
      distance_median = 0;
    }
    
    distance_array[j] = distance_median;
    // MUST be of 50ms for accurate measurments
    delay(50);
  }

  // print array
  Serial.println(F("array"));
  for (int j = 0; j < 10; j++) {
    Serial.print(distance_array[j]);
    Serial.print(F(" "));
  }
  Serial.println();

  // Sort number for calculate median
  bubbleSort(distance_array, 10);
  // Calculate median
  distance_median = median(distance_array, 10);
  // calculate the real distance that is a information for the user
  distance = (height_zero - measure_offset) - (distance_median * correction_factor);

  Serial.print(F("distance: "));
  Serial.println(distance);

  return distance;  
}

// function for swich on Sim900
void Sim900on (){
  // switch on GSM shield procedure. R13 must be welded on the shield
  Serial.println(F("Swich on Shield and start program"));
  pinMode(GSM_SHIELD_PIN, OUTPUT);
  digitalWrite(GSM_SHIELD_PIN, LOW);
  delay(1000); // /
  digitalWrite(GSM_SHIELD_PIN, HIGH);
  delay(2000); // /
  digitalWrite(GSM_SHIELD_PIN, LOW);
}

// function for sort number for calculate median
void bubbleSort(float arr[], int n) {
  for (int i = 0; i < n-1; i++) {
    for (int j = 0; j < n-i-1; j++) {
      if (arr[j] > arr[j+1]) {
        float temp = arr[j];
        arr[j] = arr[j+1];
        arr[j+1] = temp;
      }
    }
  }
}

// function for calculate median 
float median(float arr[], int n) {
  if (n % 2 == 0) {
    return (arr[n/2 - 1] + arr[n/2]) / 2.0;
  } else {
    return arr[n/2];
  }
}
