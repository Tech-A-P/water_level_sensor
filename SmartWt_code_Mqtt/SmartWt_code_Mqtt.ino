/*
MQTT program for SIM900 for server that use a simple MQTT with no TLS secure protocol. 

authors: Pietro Mascherpa
contact: mascherpa.pietro@gmail.com
copyright: CC BY-NC-SA 4.0
date current version: 2026.02.03 (YYYY/MM/DD)
backend:  ResIoT
Hardwere: 3DM 2025.06 SmartWT v2

The first time that you charge this program make a send with the computer alimentation, and the SmartWT swich-off for make correct inizialization.
Check the TechAP instructions book for information about the hardware component of the system.
*/

//Questo è importante anche se devo capire perchè
#define TINY_GSM_MODEM_SIM900

#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <SoftwareSerial.h>
#include <String.h>
#include <EEPROM.h>

#define SerialMon Serial                         // istance for serial monitor 

//*************************************************************
//ULSTRASONIC SENSOR
// define pin
#define PING_PIN 3                               // ultrasound emitter
#define ECHO_PIN 4                               // ultrasound receiver

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

//SEND data mqtt&Sim900
#define GSM_SHIELD_PIN 9                           // switch for GSM shield

// SIM900 pin 7 (RX Arduino) e 8 (TX Arduino) 
// Create istance for SIM900 communication
SoftwareSerial SerialAT(7, 8); // RX, TX

// Initialize the GSM modem using the serial interface
TinyGsm modem(SerialAT);
// Create a TCP client that communicates through the GSM modem
TinyGsmClient client(modem);
// Initialize the MQTT client using the GSM TCP client
PubSubClient mqtt(client);

//VARIABLES SETTINGS
//SIM settings
const char apn[]  = "iot.1nce.net";
const char user[] = "";
const char pass[] = "";

//se sono attivi i protocolli TLS connesione impossibile 
const char mqtt_server[] = "spiga.resiot.net";
const int  mqtt_port = 61110;
const char mqtt_client_id[] = "SPIGA";
const char mqtt_user[] = "resiot";
const char mqtt_pass[] = "8lmAHL90MyRx";

// topic RESIOT
const char topic[] = "SPIGA_000_ARDU";

//for modify the data send go in funtion send (search: "payload")

// variables for check the correct send of measure
int number_attempt_send = 0;                       // number of attempts to measure and send the data
//Max number of attemps to send data if fail
const int MAX_NUMBER_ATTEMPTS = 3;

//*************************************************************
//BATTERY CHECK
// define pin
#define BATT_PIN A7                              // analogPin for the measure of the battery tension

// variables for battery tension check
float Volt;                                        // variabile per monitorare la Volt della batteria
int AnalogBatt;                                    // variabile in cui metto la misura del pin analogico a cui è connessa la batteria
const float PartitorFactor = 1.39;                 // valore che moltièlicato per la tensione misurata dal pin analogico mi da il valore sul partitore di tensione: Uguale al rapporto (R1+R2)/R1  dove R1 è la resistenza collegata a GND e R2 è la resistenza collegata a +12
const float SogliaBatt = 3.60;                     // limite di Volt a cui non faccio più funzionare niente.
const float Vdelta = 0 ;                           // la differenza tra il voltaggio vero della batteria e il voltaggio misurato sui due punti in cui misura l'arduino. 

//*************************************************************
//EEPROM 
/*
One position of the Eeprom use 1 Byte (and Nano Every has 256 byte 0-255) so you have to consider the typo of data that you are writing in.
The Eeprom position depend form the type of data that you would to write in. If i would to write a folat value I have to mantain 4 byte, so 4 position
Eeprom space 1 byte 1 position:
  - for int value: 2 byte 
  - for long value: 4 byte
  - for float value: 4 byte
  - for double value: 4 byte
But if you change the  board you have to verfy this number
*/

// EEprom struct, must be before define
struct EepromRecord {
  uint16_t switchCounter;   // numero accensioni
  uint16_t interval;        // intervallo / stato logico
  uint8_t  valid1;          // primo marker
  uint8_t  valid2;          // secondo marker
};

// define parameteres
#define EEPROM_VALID 0xAA
#define EEPROM_START_ADDR  0
#define EEPROM_END_ADDR    230   // here put the last address of eeprom that you wont be write with data for the start of the system (if you need some space don't write the last adress)
#define EEPROM_RECORD_SIZE sizeof(EepromRecord)
#define EEPROM_MAX_RECORDS ((EEPROM_END_ADDR - EEPROM_START_ADDR) / EEPROM_RECORD_SIZE)

// variable for the funtion restituition
uint16_t SwitchCounter = 0;
uint16_t LastDistanceSend = 0;

// variabile for last eeprom adress for write
int lastValidIndex = -1;

//*************************************************************
//TIMER AND NUMBER OF ACCENSION
#define TIMER_PIN 6                              // stand-by timer
// 12 con resistenza da 5 min significa 1 ore 
int EepromStart = 12;                              // qui metto il numero di accensioni massime raggiunte le quali voglio che faccia l'invio
const float Tolerance = 2 ;                       // costante che mi dice la Tolerance da dare alla misura perchè venga considerata diversa dalla precedente
float abs_distance;                                // questo serve per le condizioni che verificano se il valore letto è uguale a quello precedente. il valore assoluto serve perchè per i valori negativi, i risultati dell'if diventano opposti


void setup() {
  // Serial connection.
  SerialMon.begin(115200);
  SerialAT.begin(2400);
  delay(100);

// ***************************************************
// CHECK BATTERY TENSION
 if (!checkBatteryTension()){
   Serial.println(F("Battery low! Switch off."));
   standBy();
 }

// ***************************************************
// TEST EEPROM INIZIALIZATION and calculate the EEPROM swich position
//after this function you have SwichCounter that count the number of accension and LastDistanceSend that is the last distance measured
 InitEeprom();

// ***************************************************
// Take new measure and take old value for the Eeprom
  // set pins for measurement
  pinMode(PING_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Get the measure
  distance = getMeasure();
  // absolute value of distance for logic comparison
  abs_distance = fabs(distance);
  Serial.print(F("Number of attemps:"));
  Serial.println(SwitchCounter);

// ***************************************************
// Test if is the moment to send measure
  // check if SwitchCounter (the number of start from the last send) is minor of EepromStart (the number of accension for send measure)
  bool accensioniOK = (SwitchCounter < EepromStart);
  // check if the measure is in the range of tolerance
  bool misuraOK = (abs_distance > LastDistanceSend - Tolerance) && (abs_distance < LastDistanceSend + Tolerance);
  
  // debug on serial monitor
  Serial.print(F("Reach the number of accension for start? (0 if true) ")); 
  Serial.println(accensioniOK);
  Serial.print(F("The masure is change significantly for the last measure? (0 if true) ")); 
  Serial.println(misuraOK);

  // Check if both the condition are true (OFF the SmartWT) if only one is false is the  time to send
  if (accensioniOK && misuraOK) {
      //NOT time to send: measure isn't change AND the number of accension isn't reach
        Serial.println(F("EEPROM OFF!"));
        // Save new variable in EEPROM
        SaveState();
        //for be sure that system don't goes off too fast
        delay(15);
        // SmartWT off
        standBy();
      }else{
      //TIME to send: measure is change OR the number of accension is reach
      // at the and of the program, only if the measure is correctly send i update EEPROM_NewLastSend and EEPROM_NewMeasure
      }

// if the program arrive here the previus IF is false and is the time to send measure. Bacause fi IF is true with StandBy() the SmartWT go off
  SerialMon.println("Start modem...");
  modem.restart();

// ***************************************************
// Swich on SIM900
// va acceso dopo perchè senno arduino scalda e falsa la temp. 
 Sim900on();
 
}

void loop() {
  number_attempt_send = number_attempt_send + 1;
  Serial.print(F("# Attempts: "));
  Serial.println(number_attempt_send);
  bool sent = false;

  // communication check and possible second attempt of measuring and sending data
   while (!sent && number_attempt_send < MAX_NUMBER_ATTEMPTS) {
   // try to send data, 
   //if send is false and attempts<max remain in cicle

   // increment attempts
   number_attempt_send++;

    //GSM  connection
    if (!connectGSM()) {
      Serial.println(F("GSM connection failure."));
      continue;
    }

    //send data
    if (sendMQTT(distance)) {
        Serial.println(F("Data correctly sent, update the EEPROM value and go off."));   
        //Change status of sent variable
        sent = true;
        // write zero the number of accension because you send data
        SwitchCounter = 0; 
        // Update the LastDistanceSend with the last measure token
        LastDistanceSend = abs_distance;
        //write this two variable in a new record in EEPROM (the position is know by the program)
        SaveState();
        //for be sure that system don't goes off too fast
        delay(15);
        //SmartWT off, don't delete. Is here for simplyfy the program read
        standBy();
    } else {
      Serial.println(F("Mqtt send Fail"));
    }
    //time dalay for stability
    delay(2000);   
  }

  if (!sent) {
    Serial.println(F("Send fail after the max number of attemps set."));
  }

  standBy();
}

// ***************************************************
// FUNCTION SECTION

// function for swich off SmartWT
void standBy() {
  digitalWrite(TIMER_PIN, HIGH);                  // switch timer off
  Serial.println(F("System in stand by"));
  delay(3000);
}

// function for check the battery tension. Return TRUE if battery is over the tension limit , FALSE if the battery is down. AnIntervalValued calculate the battery tension
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

// function for initialize the Eeprom value at the first program upload and update the Eeprom Swich number (at every turn he write number in a different eeprom adress an trak this addres with a marler (valid) at the next strat you search the marker and find your last data)
void InitEeprom() {

  lastValidIndex = -1;
  EepromRecord tempRecord;
  EepromRecord currentRecord;

//EEPROM scan
  for (int i = 0; i < EEPROM_MAX_RECORDS; i++) {

    int addr = EEPROM_START_ADDR + i * EEPROM_RECORD_SIZE;
    EEPROM.get(addr, tempRecord);

   if (tempRecord.valid1 == EEPROM_VALID &&
     tempRecord.valid2 == EEPROM_VALID &&
     tempRecord.switchCounter != 0xFFFF) {
      lastValidIndex = i;
    } else {
      break;  // primo record non valido → stop
    }
  }

//trak the last data write (trak valid data here is the last that you send)
  if (lastValidIndex >= 0) {
    //where is the vild index here is the last site where i write
    int addr = EEPROM_START_ADDR + lastValidIndex * EEPROM_RECORD_SIZE;
    EEPROM.get(addr, currentRecord);

    SwitchCounter = currentRecord.switchCounter;
    LastDistanceSend = currentRecord.interval;

  } else {
    // Prima accensione
    SwitchCounter = 0;
    LastDistanceSend = 0;
  }
  //increment counter
  SwitchCounter++;   // ogni accensione incrementa
}

void SaveState() {
  EepromRecord newRecord;

  // calucale next index where write
  int nextIndex = lastValidIndex + 1;

  if (nextIndex >= EEPROM_MAX_RECORDS) {

    // erase valid from all area
    for (int i = 0; i < EEPROM_MAX_RECORDS; i++) {
      int addr = EEPROM_START_ADDR + i * EEPROM_RECORD_SIZE;
      EEPROM.update(addr + offsetof(EepromRecord, valid1), 0x00);
      EEPROM.update(addr + offsetof(EepromRecord, valid2), 0x00);
    }

    nextIndex = 0;
  }

  int nextAddr = EEPROM_START_ADDR + nextIndex * EEPROM_RECORD_SIZE;

  // prepare record for being write
  newRecord.switchCounter = SwitchCounter;
  newRecord.interval      = LastDistanceSend;
  newRecord.valid1        = 0x00;
  newRecord.valid2        = 0x00;

  //wirte data 
  EEPROM.put(nextAddr, newRecord);
  //write vaild1 
  EEPROM.update(nextAddr + offsetof(EepromRecord, valid1), EEPROM_VALID);
  //write vaild2
  EEPROM.update(nextAddr + offsetof(EepromRecord, valid2), EEPROM_VALID);

  // aggiorno indice corrente
  lastValidIndex = nextIndex;
}

bool connectGSM() {
  Serial.println(F("Attendo rete GSM..."));
  if (!modem.waitForNetwork(60000L)) {
    Serial.println(F("GSM FAIL"));
    return false;
  }
  Serial.println(F("GSM OK"));

  Serial.print(F("Segnale: "));
  Serial.println(modem.getSignalQuality());

  Serial.println(F("Connessione GPRS..."));
  if (!modem.gprsConnect(apn, user, pass)) {
    Serial.println(F("GPRS FAIL"));
    return false;
  }
  Serial.println(F("GPRS OK"));
  return true;
}

bool sendMQTT(float distance) {
  mqtt.setServer(mqtt_server, mqtt_port);

  Serial.println(F("Connessione MQTT..."));
  if (!mqtt.connect(mqtt_client_id, mqtt_user, mqtt_pass)) {
    Serial.print(F("MQTT FAIL, state="));
    Serial.println(mqtt.state());
    return false;
  }

  // payload JSON (correct the dimension)
  char msg[120];

  // string where put data
  String payload = "{";
  payload += "\"distance\":";
  payload += String(distance, 1);
  payload += ",\"battery\":";
  payload += String(Volt, 2);
  payload += "}";

  //correct de dimension fo char
  payload.toCharArray(msg, 120);
  Serial.print(F("PUB -> "));
  Serial.println(msg);

  bool ok = mqtt.publish(topic, msg);

  mqtt.disconnect();

  if (ok) {
    Serial.println(F("MQTT OK"));
    return true;
  } else {
    Serial.println(F("MQTT publish FAIL"));
    return false;
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

float median(float arr[], int n) {
  if (n % 2 == 0) {
    return (arr[n/2 - 1] + arr[n/2]) / 2.0;
  } else {
    return arr[n/2];
  }
}
