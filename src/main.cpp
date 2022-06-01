#include "SIM900.h"
#include <SoftwareSerial.h>
#include "inetGSM.h"
#include "SerialRead.h"
#include <stdlib.h>

// #include <../.env/env.h>

// To change pins for Software Serial, use the two lines in GSM.cpp.

// GSM Shield for Arduino
// www.open-electronics.org
// this code is based on the example of Arduino Labs.

// Simple sketch to start a connection as client.

#define PING_PIN 3
#define ECHO_PIN 4

long microsecondsToCentimeters(long microseconds)
{ // libreria per trasformare da microsecondi a centimetri (da mettere fuori dal loop)
  return microseconds / 29 / 2;
}

long getMeasurement() // libreria che prende la misura per ottenere il risultato dall'ultrasuoni.
{
  Serial.print("Taking measurement...");
  long duration, cm;
  pinMode(PING_PIN, OUTPUT);
  digitalWrite(PING_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(PING_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(PING_PIN, LOW);
  pinMode(ECHO_PIN, INPUT);
  duration = pulseIn(ECHO_PIN, HIGH);
  cm = microsecondsToCentimeters(duration);
  Serial.print(cm);
  Serial.print("cm");
  Serial.println();
  delay(100);
  return cm;
}

InetGSM inet;
// CallGSM call;
// SMSGSM sms;

char mesg[50];
int numdata;
boolean started = false;
long h = 0;

const char *THINGSPEAK_API_KEY = "G44TANS8WMVAW8SH";

void setup()
{
  pinMode(9, OUTPUT); // turn on the module form program è necesario saldare il jumper R13 sulla scheda e fa tutto lui.
  digitalWrite(9, LOW);
  delay(1000);
  digitalWrite(9, HIGH);
  delay(2000);
  digitalWrite(9, LOW);
  delay(3000); // turn on the module form program qui finisce la parte necessaria per accentere il modulo appena si alimenta la scheda, può essere usato anche nel programma per accendere e spegnere

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

  Serial.println("i'm here");

  if (started)
  {
    // GPRS attach, put in order APN, username and password.
    // If no needed auth let them blank.
    if (inet.attachGPRS("TM", "", ""))
      Serial.println("status=ATTACHED");
    else
      Serial.println("status=ERROR");
    delay(1000);

    // Read IP address.
    gsm.SimpleWriteln("AT+CIFSR");
    delay(5000);
    // Read until serial buffer is empty.
    gsm.WhileSimpleRead();

    h = getMeasurement(); // take the measurement

    // TCP Client GET, send a GET request to the server and
    // save the reply.

    numdata = inet.httpGET("api.thingspeak.com", 80, "/update?api_key=G44TANS8WMVAW8SH&field1=15", mesg, h);
    // Print the results.
    Serial.println("\nNumber of data received:");
    Serial.println(numdata);
    Serial.println("\nData received:");
    Serial.println(mesg);

    inet.connectTCP("api.thingspeak.com", 80);

    Serial.println("tcp");
    char const *uri = ("https://api.thingspeak.com/update?api_key=" + THINGSPEAK_API_KEY + "&field1=" + String(h)).c_str();
    Serial.println(uri);
    char response[200];
    // inet.println(str);//begin send data to remote server

    char const *uri_segment = ("/update?api_key=" + THINGSPEAK_API_KEY + "&field1=4").c_str();
    inet.httpGET("http://api.thingspeak.com", 80, uri_segment, response, 200);
  }
};

void loop()
{
  // Read for new byte on serial hardware,
  // and write them on NewSoftSerial.
  serialhwread();
  // Read for new byte on NewSoftSerial.
  serialswread();
};
