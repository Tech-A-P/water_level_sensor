#include "SIM900.h"
#include <SoftwareSerial.h>
#include "inetGSM.h"
#include "SerialRead.h"
// #include "env.h"

// To change pins for Software Serial, use the two lines in GSM.cpp.

// GSM Shield for Arduino
// www.open-electronics.org
// this code is based on the example of Arduino Labs.

// Simple sketch to start a connection as client.

InetGSM inet;
// CallGSM call;
// SMSGSM sms;

char mesg[50];
int numdata;
boolean started = false;
long h = 30;

String THINGSPEAK_API_KEY = "G44TANS8WMVAW8SH";


void setup()
{
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

    // TCP Client GET, send a GET request to the server and
    // save the reply.

        numdata=inet.httpGET("api.thingspeak.com", 80, "/update?api_key=G44TANS8WMVAW8SH&field1=9", mesg, 50);
        //Print the results.
        Serial.println("\nNumber of data received:");
        Serial.println(numdata);
        Serial.println("\nData received:");
        Serial.println(mesg);
    
    

   // inet.connectTCP("api.thingspeak.com", 80);

  /* Serial.println("tcp");
    char const* uri0 = ("https://api.thingspeak.com/update?api_key=" + THINGSPEAK_API_KEY + "&field1=" + String(h)).c_str();
    Serial.println("I am alive");
    Serial.println(uri0);
    char response[200];
    // inet.println(str);//begin send data to remote server
    Serial.println("I am still alive");

    char const* uri1 = ("/update?api_key=" + THINGSPEAK_API_KEY + "&field1=4").c_str();
    Serial.println("Am I immortal?!");
    Serial.println(uri1);
    inet.httpGET("http://api.thingspeak.com", 80, uri1, response, 200);
    Serial.println(response);
    Serial.println("Zio Pippo");
  */  
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