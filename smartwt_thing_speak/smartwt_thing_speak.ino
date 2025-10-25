/*
authors: Pietro Mascherpa, Fabio Brandalese, Roberto Garza 
contact: roberto.garza@uni-konstanz.de
copyright: CC BY-NC-SA 4.0
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

#define PING_PIN 4                               // ultrasound emitter
#define ECHO_PIN 3                               // ultrasound receiver
#define TIMER_PIN 6                              // stand-by timer
#define GSM_SHIELD_PIN 9                         // switch for GSM shield
#define POWER_PIN 13                             // power system

InetGSM inet;

// TODO: try to optimize data structures to save memory

// constants for data communication
const bool debug = false;
const char* SIM_APN = "YOUR_APN";
const String THINGSPEAK_API_KEY = "YOUR_API_KEY";
const char* THINGSPEAK_ENDPOINT_ROOT = "api.thingspeak.com";
const String THINGSPEAK_FIELD_NUMBER = "3";      // ThingSpeak field index for the water level value
const String THINGSPEAK_ATTEMPT_NUMBER = "6";    // ThingSpeak field index for the current measurement attempt number   // TODO: include only in debug version
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
const int speed_sound_air = 343;    // speed of sound in the air [m/s]
const int max_distance = 450;       // maximum distance recorded reliably by the sensor
float single_distance;              // single measure [cm]
float distance_median;              // median measure [cm]
float distance;                     // median distance corrected and ready to send [cm]
const float height_zero = 20;       // height of the detector from the ground
float measure_offset = 2.6;         // measuring measure_offset of the sensor | TODO-USER: set as distance between the surface of the sensor and the edge of the box (Fig. 1)
float correction_factor = 1.028;   // correct for distance of the target from sensor

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

  // switch it on without using current from TPL
  char switch_on_command_code[6] = "J13_UP";
  Serial.println(switch_on_command_code);
  pinMode(POWER_PIN, OUTPUT);
  digitalWrite(POWER_PIN, HIGH);    //autoritenuta di accensione così non usiamo la corrente del tpl.
  delay(500);                       // delay needed for stable start

  // ping TPL to switch it off
  digitalWrite(TIMER_PIN, HIGH);
  delay(500);
  digitalWrite(TIMER_PIN, LOW);

  // test GSM shield
  Serial.println(F("GSM Shield testing."));
  pinMode(GSM_SHIELD_PIN, OUTPUT);
  digitalWrite(GSM_SHIELD_PIN, LOW);
  delay(1000); ///
  digitalWrite(GSM_SHIELD_PIN, HIGH);
  delay(2000); ///
  digitalWrite(GSM_SHIELD_PIN, LOW);
  sprintf(serial_text, "%, high", GSM_SHIELD_PIN);
  Serial.println(serial_text);

  // set pins for measurement
  pinMode(PING_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  number_attempt_send = 0;  // TODO: check if I need it
}

void loop()
{
  delay(1000);                                   // TODO: check if I can remove this
  number_attempt_send = number_attempt_send + 1;
  sprintf(serial_text, "# Attempts: %", number_attempt_send);
  Serial.println(serial_text);

  // communication check and possible second attempt of measuring and sending data
  if (status == "200") {
    Serial.println(F("Data correctly sent"));   // then program jumps to the end and switch to stand by
  } else {
    getMeasure();

    if (started == false) {
      setComShield();                          // GSM shield configuration
    }
    if (started == true) {
      sendData();                                // data communication
    }
  }  
  
  // ??? is this really for debug purposes only? 
  if (debug) {
    debugLog();
  }

  if(status == "200"){
    standBy();
  }
  if(number_attempt_send > MAX_NUMBER_ATTEMPTS){                   // max three attempts for measuring and sending data, if all three failes, just stand by the system
    standBy();
  }
}

void standBy() {
  digitalWrite(TIMER_PIN, HIGH);                  // switch timer off
  digitalWrite(POWER_PIN, LOW);                   // safe power off
  Serial.println(F("System in stand by"));
}

void debugLog() {
  // ??? check the following line is a correct interpretation of what comes next
  // read a char at a time from serial to get the whole received string (we have number_repetitions measures, which should stay in number_char_to_read slots)
  int number_char_to_read = number_repetitions * 3;
  for(int c=0; c<number_char_to_read; c++) {
    delay(20);  // ? why do we have to wait?
    String read_char = String(gsm.read());
    sprintf(serial_text, "Read char: %", read_char);
    Serial.println(serial_text);

    // use H char as a stamp for start of message
    if(read_char == "H") {
      status = "";
        
      // read all the char of interest (45 in our case)
      for(int n=1; n<=45; n++) {
        delay(30);
        String read_char = String(gsm.read());

        sprintf(serial_text, "Read char: %", read_char);
        Serial.println(serial_text);
        
        sprintf(serial_text, "n char: %", n);
        Serial.println(serial_text);

        // between 8 ans 12 we can extract the status
        if(n > 8 & n < 12){
          status += read_char;
          
          sprintf(serial_text, "status: %", status);
          Serial.print(serial_text);
        }
      }
    }
  }
}

void setComShield() {
  // connect
  if (gsm.begin(2400)) {
    Serial.println("\nstatus=READY");
    started = true;
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
  delay(500);                      // Read until serial buffer is empty.

  String url = "/update?api_key=" + THINGSPEAK_API_KEY + "&field" + THINGSPEAK_FIELD_NUMBER + "=" + String(distance , 1) + "&field" + THINGSPEAK_ATTEMPT_NUMBER + "=" + String(number_attempt_send); // send the corrected measure + number of attempts

  url.toCharArray(URL, 100);
  Serial.println(F("URL"));
  Serial.println(URL);

  numdata = inet.httpGET(THINGSPEAK_ENDPOINT_ROOT, 80, URL, mesg, h);
  delay(50);  // TODO: check if I can delete
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
    delay(50); // TODO: test if I can get rid of it
  }
  bubbleSort(distance_array, 10);                                 // sort the array
  distance_median = median(distance_array, 10);                   // compute median
  distance = (height_zero - measure_offset) - (distance_median * correction_factor);
  sprintf(serial_text, "distance: %f", distance);
  Serial.println(serial_text);
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
