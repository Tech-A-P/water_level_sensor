/*
authors: Pietro Mascherpa, Fabio Brandalese, Roberto Garza 
contact: roberto.garza@uni-konstanz.de
copyright: ??? iDEPOT
date current version: 2024.09.15 (YYYY/MM/DD)
backend: ThingSpeak

This code is the firmware used in Mascherpa et al. 2025 (DOI: ).
Check the paper for instructions about the hardware component of the system.
*/

#include "SIM900.h"
#include <SoftwareSerial.h>
#include "inetGSM.h"
#include "SerialRead.h"
#include <String.h>
#include <EEPROM.h>

#define PING_PIN 4                               // ultrasound emitter
#define ECHO_PIN 3                               // ultrasound receiver
#define TIMER_PIN 6                              // stand-by timer
#define GSM_SHIELD_PIN 9                         // switch for GSM shield
#define BATT_PIN A7                              // analogPin for the measure of the battery tension
#define EEPROMinit 0                             // address of the Eeprom for verify if is the first time of initialization of the eeprom. The first upload of the probram we inizialize the eeprom value and write here 1. So for the rest of the use we know that the eeprom was inizialize correctly the first time. We use a 4byte value for have a low probability that casually the first time of the eeprom read his value is casually equal to 1. Whit 4 byte we have 0.0000023% of probability
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
int EEPROM_Interval = 6;                        // address of the Eeprom when allocate the number of accension (ulong)
int EEPROM_LastSend = 10;                        // address of the Eeprom when allocate the number of accension at the last send (ulong)
int EEPROM_Measure = 14;                         // address of the Eeprom when allocate the old distance measurment (float)
const int EEPROMstep = 12;                      // distance in Eeprom position from the first EepromInterval position to the first Eeprom free position, for new allocation of the Eeprom values, when the first allocations are full. Is the number of Eeprom position used starting from the posiition 6 because the first 6 eeprom address are used for the inizialized control

InetGSM inet;

// TODO: try to optimize data structures to save memory

// constants for data communication
const char* SIM_APN = "iot.1nce.net";
const String THINGSPEAK_API_KEY = "YOUR_USERNAME";
const char* THINGSPEAK_ENDPOINT_ROOT = "api.thingspeak.com";
const String THINGSPEAK_FIELD_NUMBER = "4";      // ThingSpeak field index for the water level value
const String THINGSPEAK_BATTERY_NUMBER = "5";    // ThingSpeak field index for the battery value 
const int MAX_NUMBER_ATTEMPTS = 3;

// variables for data communication
char URL[100];
char mesg[50];
char serial_text[20];
int numdata;
long h = 30;
boolean started = false;

// variables for measurement
float delay_sensor;                        // time needed for the signal to reach detector
const int number_repetitions = 10;                  // number of measures to median to get a single value to store
float distance_array[number_repetitions];          // number_repetitions elements array containing single measures
const int speed_sound_air = 331.45;    // speed of sound in the air [m/s]
const int max_distance = 450;       // maximum distance recorded reliably by the sensor
float single_distance;              // single measure [cm]
float distance_median;              // median measure [cm]
float distance;                     // median distance corrected and ready to send [cm]
const float height_zero = 0;       // height of the detector from the ground
float measure_offset = 2.29;         // measuring measure_offset of the sensor | TODO-USER: set as distance between the surface of the sensor and the edge of the box (Fig. 1)
float correction_factor = 1.028;   // correct for distance of the target from sensor (decrease measure by 3,06%)

//dichiarazione variabili CONTROLLO TENSIONE
float Volt;                     // variabile per monitorare la Volt della batteria
int AnalogBatt;                     // variabile in cui metto la misura del pin analogico a cui è connessa la batteria
const float PartitorFactor = 2.76;       // valore che moltièlicato per la tensione misurata dal pin analogico mi da il valore sul partitore di tensione: Uguale al rapporto (R1+R2)/R1  dove R1 è la resistenza collegata a GND e R2 è la resistenza collegata a +12
const float SogliaBatt = 0;           // limite di Volt a cui non faccio più funzionare niente.
const float Vdelta = 0.31;               // la differenza tra il voltaggio vero della batteria e il voltaggio misurato sui due punti in cui misura l'arduino. 

//dichiarazione variabili per ACCENSIONE
unsigned long Eeprom_Init;                    // variabile per verificare se la eeprom è stata correttamente inizializzata
int Eeprom_Count;                              // variabile dove conto il numero di volte in cui ho superato il valore massimo di eeprom limit
unsigned long EepromRead;                     // variabile in cui memorizzo il valore vletto nella eeprom del numero accensioni
unsigned long LastEepromSend;                 // variabile in cui memorizzo il valore della Eeprom all'ultimo invio
int EepromNumber;                             // variabile dove calcolo la differenza tra il valore Eeprom e il valore della Eeprom all'ultimo invio
float Eeprom_distance;                        // variabile in cui memorizzo l'ultimo valore letto dal sensore DEVO USARE PUT E GET PER DATI SOPRA I 2 BYTE
//18 con resistenza da 5 min significa 1.5 ore 
int EepromStart = 18;              // qui metto il numero di accensioni raggiunte le quali voglio che faccia l'invio
const float Tolerance = 2 ;        //costante che mi dice la Tolerance da dare alla misura perchè venga considerata diversa dalla precedente
float abs_distance;                //questo serve per le condizioni che verificano se il valore letto è uguale a quello precedente. il valore assoluto serve perchè per i valori negativi, i risultati dell'if diventano opposti

//dichiarazione variabili per CONTROLLO FINALE invio
int number_attempt_send = 0;        // number of attempts to measure and send the data
String status = "";                 // communication status

void setup() {
  // at start TIMER_PIN must be LOW    // TODO: check if I can get rid of this block
  pinMode(TIMER_PIN, OUTPUT);
  digitalWrite(TIMER_PIN, LOW);
  delay(100);
  // Serial connection.
  Serial.begin(9600);

//*************************************************** questo va davanti a tutti, qperchè se la batteria è scarica non bisogna fare un cazzo così la preserviamo
//CHECK BATTERY TENSION
  // read the tension on the analogic pin
  AnalogBatt = analogRead(BATT_PIN);
  // convert the value read (0-1023) in tension (0-5V) the value 5330 is measured on the arduino board.
  Volt = map(AnalogBatt, 0, 1023, 0, 5000 );
  // we add 0.65 because when the courrent flow throw the TPL there is a drop of 0.65V
  Volt = (Volt/1000)+ Vdelta;
  // calulation of the tension of the battery over the tension partitor
  Volt = Volt * PartitorFactor;
  // Stampa la Volt sul monitor seriale
  Serial.print(F("Tension read on the battery: "));
  Serial.println(AnalogBatt);
  Serial.println(Volt);
  // if the tension go down the minimum level switch off the arduino

   if (Volt < SogliaBatt) {
            Serial.println(F("Battery low! Switch off."));
            standBy();
        }


//*****************************************************************
//TEST if the Eeprom inizialization and if is at his limit of use
EEPROM.get(EEPROMinit, Eeprom_Init);
Serial.println(Eeprom_Init);
if (Eeprom_Init != 1){
  Serial.println(F("Inizialized EEPROM"));
  //inizializzo a 1 così so che da qui in poi la eeprom sarà inizializzata
  Eeprom_Init = 1;
  EEPROM.put(EEPROMinit, Eeprom_Init);
  //inizializzo a zero così al primo giro utilizzo gli indirizzi iniziali
  Eeprom_Count = 0;
  EEPROM.put(EEPROMcount, Eeprom_Count);
  //inizializzo a zero perchè il pirmo giro è la prima accensione vera
  EepromRead = 0;
  EEPROM.put(EEPROM_Interval, EepromRead);
  //inizializzo a zero perchè il primo giro non ha inviato
  LastEepromSend = 0;
  EEPROM.put(EEPROM_LastSend, LastEepromSend);
  //inizializzo a zero perchè la prima misura
  Eeprom_distance = 0;
  EEPROM.put(EEPROM_Measure, Eeprom_distance);
  //la prima volta che accendi, per verificare un invio devi prendere una misura lontana e riaccendere fino a che non hai superato il numero di accensioni
}

//se tutto è stato inizializzato verifico di non aver superato il limite di 100000
if (EepromRead > 100000){
  EEPROM.get(EEPROMcount, Eeprom_Count);
  //incremento il valore di 1 e lo scrivo nella Eeprom
  Eeprom_Count = Eeprom_Count + 1;
  EEPROM.put(EEPROMcount, Eeprom_Count);
  Serial.println(F("EepromAdress updated for reaching the 100000 limit"));
}

Serial.println(F("Calculating the EEPROM address, EEPROM_Interval address is:"));
//calcolo gli indirizzi di eeprom da usare nel programma, leggendo prima il valore di EepromCount
 EEPROM.get(EEPROMcount, Eeprom_Count);
 EEPROM_Interval = EEPROM_Interval + (Eeprom_Count * EEPROMstep);     // PROGRAM adress of the Eeprom when allocate the number of accension 
 EEPROM_LastSend = EEPROM_LastSend + (Eeprom_Count * EEPROMstep);     // PROGRAM adress of the Eeprom when allocate the number of accension at the last send
 EEPROM_Measure = EEPROM_Measure + (Eeprom_Count * EEPROMstep);       // PROGRAM adress of the Eeprom when allocate the old distance measurment
 Serial.println(EEPROM_Interval);

//***************************************************
//CONTROLLO NUMERO ACCENSIONI & VALORE INVIATO
  
 // set pins for measurement
  pinMode(PING_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  //Get the measure
  getMeasure();
  // take the number of start from the position EEPROM_Interval and save it in EepromRead
  EEPROM.get(EEPROM_Interval, EepromRead);
  delay(10);
  // take the last distance value
  EEPROM.get(EEPROM_Measure, Eeprom_distance);
  delay(10);
  // take the last value od EepromRead when send the measure
  EEPROM.get(EEPROM_LastSend, LastEepromSend);
  delay(10);
  //increment the number for count the current starting
  EepromRead = EepromRead +1;
  //calcolo la differenza tra EepromRead e il suo valore all'ultimo invio se è uguale alla soglia invio
  EepromNumber = EepromRead - LastEepromSend;
  //calcolo la misura in valore assoluto che mi serve per fare che le prossime condizioni lavorino giuste anche se il valore è negativo
  abs_distance = abs(distance);
  Serial.print(F("Number of attemps:"));
  Serial.println(EepromNumber);

//test if is the moment to send the measure. 
  // Controllo se il numero di accensioni è inferiore al limite
  bool accensioniOK = (EepromNumber < EepromStart);
  // Controllo se la misura è dentro il range di Tolerance
  bool misuraOK = (abs_distance > Eeprom_distance - Tolerance) && (abs_distance < Eeprom_distance + Tolerance);
  //debug sulle condizioni
  Serial.print(F("Non ancora raggiunto il limite? ")); 
  Serial.println(accensioniOK);
  Serial.print(F("La misura è rimasta uguale? ")); 
  Serial.println(misuraOK);
  //Save the new Eeprom value
  EEPROM.put(EEPROM_Interval, EepromRead);
  EEPROM.put(EEPROM_Measure, abs_distance);
  delay(10);

  //controllo se entrambe le condizioni sono vere spengo, altrimenti invio
  if (accensioniOK && misuraOK) {
        Serial.println(F("EEPROM OFF!"));
        //spengo perchè non era il momento di accendersi o perchè la misura è identica  
        digitalWrite(TIMER_PIN, HIGH);
        delay(1000);
       }
  //with a false condition is the time to sent. 
   //write the new value of Eeprominterval when i send the data
   EEPROM.put(EEPROM_LastSend, EepromRead);
   delay(10);

  // switch on GSM shield
  Serial.println(F("Swich on Shield and start program"));
  pinMode(GSM_SHIELD_PIN, OUTPUT);
  digitalWrite(GSM_SHIELD_PIN, LOW);
  delay(1000); ///
  digitalWrite(GSM_SHIELD_PIN, HIGH);
  delay(2000); ///
  digitalWrite(GSM_SHIELD_PIN, LOW);
  sprintf(serial_text, "%, high", GSM_SHIELD_PIN);
  Serial.println(serial_text);
//PROVA
  //number_attempt_send = 0;  // TODO: check if I need it
}

void loop()
{
  delay(1000);                                   // TODO: check if I can remove this
  number_attempt_send = number_attempt_send + 1;
  sprintf(serial_text, "# Attempts: %", number_attempt_send);
  Serial.print(serial_text);
  //AGGIUNTO PERCHè GLI SPTINTF NON FUNZIONANO
  Serial.println(number_attempt_send);

  // communication check and possible second attempt of measuring and sending data
  if (status == "200") {
    Serial.println(F("Data correctly sent"));   // then program jumps to the end and switch to stand by
  } else {
    if (started == false) {
      setComShield();                          // GSM shield configuration
    }
    if (started == true) {
      sendData();                              // data communication & response from server
    }
  }  
//PROVA
/* 
  // ??? is this really for debug purposes only? NO
  if (debug) {
    debugLog();
  }
*/
  if(status == "200"){
    standBy();
  }
  if(number_attempt_send > MAX_NUMBER_ATTEMPTS){                   // max number of attempts for measuring and sending data, if all number of attempts failes, just stand by the system
    standBy();
  }
}

void standBy() {
  digitalWrite(TIMER_PIN, HIGH);                  // switch timer off
  Serial.println(F("System in stand by"));
}

//void debugLog() {
 
//}

void setComShield() {
  // connect
  if (gsm.begin(2400)) {
    Serial.println("\nstatus=READY");
    started = true;
    Serial.println(F("Starting Shield"));
  } else {
    Serial.println("\nstatus=IDLE");
  }
}

// ??? why all these delays?
void sendData() {
  // sanity check
  if (inet.attachGPRS(SIM_APN, "", "")) {
    Serial.println("status=ATTACHED");
  } else {
    Serial.println("status=ERROR");
  }
  delay(100);  // TODO: check if I can delete

  // set communication
  gsm.SimpleWriteln("AT+CIFSR");                                  // Read IP address.
  delay(1000);
  gsm.WhileSimpleRead();                   
  delay(500);                                                     // Read until serial buffer is empty.

  String url = "/update?api_key=" + THINGSPEAK_API_KEY + "&field" + THINGSPEAK_FIELD_NUMBER + "=" + String(distance , 1) + "&field" + THINGSPEAK_BATTERY_NUMBER + "=" + String(Volt , 1) ; // send the corrected measure + number of attempts

  url.toCharArray(URL, 100);
  Serial.println(F("URL"));
  Serial.println(URL);    //printo l'url in formato string 

  numdata = inet.httpGET(THINGSPEAK_ENDPOINT_ROOT, 80, URL, mesg, h);
  delay(500);  // TODO: check if I can delete
  
  //check the status
  // read a char at a time from serial to get the whole received string (we have number_repetitions measures, which should stay in number_char_to_read slots)
  int number_char_to_read = number_repetitions * 3;
  for(int c=0; c<number_char_to_read; c++) {
    delay(20);  // ? why do we have to wait?
    //PROVA
    char read_char = gsm.read();
    String characterRead = String(read_char);
    sprintf(serial_text, "Read char: %", characterRead);
    Serial.print(serial_text);
    Serial.println(characterRead);

    // use H char as a stamp for start of message
    if(characterRead == "H") {
      status = "";
        
      // read all the char of interest (45 in our case)
      for(int n=1; n<=45; n++) {
        delay(30);
        //PROVA
        char read_char = gsm.read();
        String characterRead = String(read_char);
        sprintf(serial_text, "Read char: %", characterRead);
        Serial.print(serial_text);
        Serial.println(characterRead);
      
        sprintf(serial_text, "n char: %", n);
        Serial.println(serial_text);
        Serial.println(n);

        // between 8 ans 12 we can extract the status
        if(n > 8 & n < 12){
          status += read_char;
        }
      }
    }
  }
  sprintf(serial_text, "status: %", status);
  Serial.print(serial_text);
  Serial.println(status);
}

void getMeasure() {
  for (int j = 0; j < number_repetitions; j++) {
    digitalWrite(PING_PIN, LOW);
    delayMicroseconds(100); // time needed to dissipate ultrasonic noise in the measurement cone
    digitalWrite(PING_PIN, HIGH);
    delayMicroseconds(30); // time needed to keep emitter active for enough time to consistently get a measurement
    digitalWrite(PING_PIN, LOW);
    delay_sensor = pulseIn(ECHO_PIN, HIGH);

    distance_median = (delay_sensor * speed_sound_air / 20000);          // compute distance in cm (dividing by 2e4 accounts for back and forth of the sound wave and conversion of the flight time into distance)

    if (distance_median > max_distance) {                         // above max_distance the sensor becomes unreliable, so we store a recognizible quantity
      distance_median = -1;
    }
    
    distance_array[j] = distance_median;
    delay(50); // MUST be of 50ms for accurate measurments
  }

//Print of array
 Serial.println(F("array"));
 for (int j = 0; j < 10; j++) {
    Serial.print(distance_array[j]);
    Serial.print(F(" "));
  }
  Serial.println(); // Stampa una riga vuota

  bubbleSort(distance_array, 10);                                 // sort the array
  distance_median = median(distance_array, 10);                   // compute median
  distance = (height_zero - measure_offset) - (distance_median * correction_factor);
  //sprintf(serial_text, "distance: %f", distance);  //non printa il valore della distanza perchè spintf non supporta direttamente i float andrebbero prima convertiti in stringa con dtostrf()
  Serial.print(F("distance: "));
  Serial.println(distance);
  delay(50); // TODO: test if I can get rid of it
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
