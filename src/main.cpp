#include "SIM900.h"
#include <SoftwareSerial.h>
#include "inetGSM.h"
#include "SerialRead.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <String.h>
// #include "env.h"

// To change pins for Software Serial, use the two lines in GSM.cpp.

// GSM Shield for Arduino
// www.open-electronics.org
// this code is based on the example of Arduino Labs.

// Simple sketch to start a connection as client.

#define PING_PIN 3
#define ECHO_PIN 4
#define DONE_PIN 6 // 6 CON SHIELD
#define TEMPERATURE_PIN 5
#define RELE_PIN 2

InetGSM inet;
// CallGSM call;
// SMSGSM sms;

// dichiarazione variabili

char mesg[50];
int numdata;
long h = 30;
boolean started = false;
// double measure;

OneWire oneWire(TEMPERATURE_PIN);    // sensore di temperatura
DallasTemperature sensors(&oneWire); // sensore di temperatura

// String THINGSPEAK_API_KEY = "G44TANS8WMVAW8SH";

void setup()
{
  pinMode(DONE_PIN, OUTPUT);
  digitalWrite(DONE_PIN, LOW); // set low the done pin
  delay(5);

  pinMode(RELE_PIN, OUTPUT);
  digitalWrite(RELE_PIN, HIGH); // set HIGT the rele pin and switch on the sim 900.
  delay(1000);                  // wait that rele is closed and sim 900 take power and than turn on the module

  pinMode(9, OUTPUT); // turn on the module form program è necesario saldare il jumper R13 sulla scheda e fa tutto lui.
  digitalWrite(9, LOW);
  delay(1000);
  digitalWrite(9, HIGH);
  delay(2000);
  digitalWrite(9, LOW);
  delay(3000); // turn on the module form program qui finisce la parte necessaria per accentere il modulo appena si alimenta la scheda, può essere usato anche nel programma per accendere e spegnere

  delay(3000); // aspetta che sia stablita la connessione

  // Serial connection.
  Serial.begin(9600);
  Serial.println("GSM Shield testing.");
  // Start configuration of shield with baudrate.
  // For http uses is raccomanded to use 4800 or slower.
  if (gsm.begin(2400))
  {
    Serial.println("\nstatus=READY");
    started = true;
  }
  else
    Serial.println("\nstatus=IDLE");

  if (started)
  {
    // GPRS attach, put in order APN, username and password. (for ThingMobile APN:TM)
    // If no needed auth let them blank.
    if (inet.attachGPRS("ibox.tim.it", "", ""))
      Serial.println("status=ATTACHED");
    else
      Serial.println("status=ERROR");
    delay(1000);

    // Read IP address.
    gsm.SimpleWriteln("AT+CIFSR");
    delay(5000);
    // Read until serial buffer is empty.
    gsm.WhileSimpleRead();

    // measure=getMeasurement();// take the measurement
    // Serial.print(measure);

    // const int NR = 20;                      // numero di ripetizioni di misurazione, una ogni 100 ms (con 20 fa la media su 2 secondi)
    double duration;    // tempo di volo
    double sum = 0;     // somma dei tempi di volo per fare la media
    double average;     // media dei tempi di volo
    double Temperature; // temperatura dell'aria
    double soundSpeed;  // velocità del suono nell'aria
    double cm;
    double cm_notC;

    pinMode(PING_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    pinMode(TEMPERATURE_PIN, OUTPUT);

    // SENSORE TEMPERATURA

    delay(100);
    sensors.requestTemperatures();
    Temperature = sensors.getTempCByIndex(0); // misura la temperatura

    soundSpeed = 20.051401994234 * sqrt(Temperature + 273.15); // calcola la velocità del suono nell'aria in base alla temperatura misurata

    // SENSORE ULTRASUONI
    // misura il tempo di volo NR volte, con un ritardo di 100 ms tra le misurazioni

    for (int j = 0; j < 20; j++)
    {

      // breve segnale LOW per assicurare un segnale HIGH stabile:
      digitalWrite(PING_PIN, LOW);
      delayMicroseconds(2);
      digitalWrite(PING_PIN, HIGH); // manda un impulso di 10 us
      delayMicroseconds(10);
      digitalWrite(PING_PIN, LOW);
      duration = pulseIn(ECHO_PIN, HIGH); // misura il tempo di volo dell'impulso
      delay(300);                         // delay tra le misurazioni per evitare interferenze

      sum = sum + duration; // aggiunge il valore misurato alla somma dei tempi di volo (inizialmente 0)
    }

    average = sum / 20; // calcola la media dei tempi di volo

    // CALCOLO DISTANZA

    cm_notC = average * 346 / 20000;   // calcola la distanza NON corretta con la velocità del suono: velocità standard di 320m/s
    cm = average * soundSpeed / 20000; // calcola la distanza in cm
    /*
      Serial.print(cm_notC);
      Serial.println("cm_notC");
      Serial.print(cm);
      Serial.print("cm");
      Serial.println();
      Serial.print(Temperature);
      Serial.print("° gradi");
      Serial.println();
    */
    // sum = 0;                                      //resetta la somma dei tempi di volo a zero
    // return cm;
    delay(100);

    // TCP Client GET, send a GET request to the server andsave the reply invia la misura corretta & non corretta & la temperatura

    char URL[70]; // add the measure to the url of the client GET
    // String url = "/update?api_key=G44TANS8WMVAW8SH&field1="+ String(cm, 2)+ "&field2=" + String (cm_notC , 2) + "&field3=" + String (Temperature , 2); //convert measure (double) in string , indicate the decimal number.
    String url = "/update?api_key=CF5WF9CLTGQ3V72O&field1=" + String(cm, 2) + "&field2=" + String(cm_notC, 2) + "&field3=" + String(Temperature, 2); // convert measure (double) in string , indicate the decimal number.
    delay(1000);
    url.toCharArray(URL, 70);
    // Serial.println("url");
    Serial.print(URL);

    numdata = inet.httpGET("api.thingspeak.com", 80, URL, mesg, h); // CANCELLARE 15 ALTIRMENTI MANDA SEMPRE QUELLO
    // Print the results.
    /*        Serial.println("\nNumber of data received:");
            Serial.println(numdata);
            Serial.println("\nData received:");
            Serial.println(mesg);
    /*
    delay(3000);
         //invia la misura non corretta
            url = "/update?api_key=G44TANS8WMVAW8SH&field2="+ String(cm_notC, 2); //convert measure (double) in string , indicate the decimal number.
            url.toCharArray(URL,45);
            Serial.print(URL);
            Serial.println("url");

            numdata=inet.httpGET("api.thingspeak.com", 80, URL, mesg, h); // CANCELLARE 15 ALTIRMENTI MANDA SEMPRE QUELLO
            //Print the results.
            Serial.println("\nNumber of data received:");
            //Serial.println(numdata);
            Serial.println("\nData received:");
            //Serial.println(mesg);

    delay(3000);
         //invia la temperatura
            //invia la misura non corretta
            url = "/update?api_key=G44TANS8WMVAW8SH&field3="+ String(Temperature, 2); //convert measure (double) in string , indicate the decimal number.
            url.toCharArray(URL,45);
            Serial.print(URL);
            Serial.println();

            numdata=inet.httpGET("api.thingspeak.com", 80, URL, mesg, h); // CANCELLARE 15 ALTIRMENTI MANDA SEMPRE QUELLO
            //Print the results.
            Serial.println("\nNumber of data received:");
            //Serial.println(numdata);
            Serial.println("\nData received:");
            //Serial.println(mesg);
    */
    digitalWrite(RELE_PIN, LOW);
    delay(200);
    digitalWrite(DONE_PIN, HIGH);
    delay(1000);
    digitalWrite(DONE_PIN, LOW);
    delay(200);
  }
};

void loop()
{
  // Read for new byte on serial hardware,
  // and write them on NewSoftSerial.
  serialhwread();
  // Read for new byte on NewSoftSerial.
  serialswread();
  // Serial.println("loop");
  // delay(500);
};

/*
long getMeasurement()     //libreria che prende la misura per ottenere il risultato dall'ultrasuoni.
                          //scritta da Andrea Galli e Giovanni Ottaiano
{
  const int NR = 20;                      // numero di ripetizioni di misurazione, una ogni 100 ms (con 20 fa la media su 2 secondi)
  double duration;                        //tempo di volo
  double sum = 0;                         //somma dei tempi di volo per fare la media
  double average;                         //media dei tempi di volo
  double Temperature;                     //temperatura dell'aria
  double soundSpeed;                      //velocità del suono nell'aria
  double cm;
  double cm_notC;

  pinMode(PING_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(TEMPERATURE_PIN, OUTPUT);

//SENSORE TEMPERATURA

     delay(100);
     sensors.requestTemperatures();
     Temperature = sensors.getTempCByIndex(0);                 //misura la temperatura


  soundSpeed = 20.051401994234 * sqrt(Temperature + 273.15);    //calcola la velocità del suono nell'aria in base alla temperatura misurata

  //SENSORE ULTRASUONI
  //misura il tempo di volo NR volte, con un ritardo di 100 ms tra le misurazioni

   for (int j=0; j<NR; j++)
   {

  // breve segnale LOW per assicurare un segnale HIGH stabile:
     digitalWrite(PING_PIN, LOW);
     delayMicroseconds(2);
     digitalWrite(PING_PIN, HIGH);             //manda un impulso di 10 us
     delayMicroseconds(10);
     digitalWrite(PING_PIN, LOW);
      duration = pulseIn(ECHO_PIN,HIGH);       //misura il tempo di volo dell'impulso
      delay(100);                              //delay tra le misurazioni per evitare interferenze

   sum = sum + duration;                       //aggiunge il valore misurato alla somma dei tempi di volo (inizialmente 0)
   }

  average = sum / NR;                   // calcola la media dei tempi di volo

//CALCOLO DISTANZA

  cm_notC = average/20000;                //calcola la distanza NON corretta con la velocità del suono
  cm = average*soundSpeed/20000;          //calcola la distanza in cm

  Serial.print(cm_notC);
  Serial.print("cm_notC");
  Serial.print(cm);
  Serial.print("cm");
  Serial.println();
  Serial.print(Temperature);
  Serial.print("° gradi");
  Serial.println();

  sum = 0;                                      //resetta la somma dei tempi di volo a zero
  return cm;

}
*/