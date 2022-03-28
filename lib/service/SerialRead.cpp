
#include <SoftwareSerial.h>
#include "inetGSM.h"

int i = 0;
char inSerial[50];
char msg[50];

void serialhwread()
{
  i = 0;
  if (Serial.available() > 0)
  {
    while (Serial.available() > 0)
    {
      inSerial[i] = (Serial.read());
      delay(10);
      i++;
    }

    inSerial[i] = '\0';
    if (!strcmp(inSerial, "/END"))
    {
      Serial.println("_");
      inSerial[0] = 0x1a;
      inSerial[1] = '\0';
      gsm.SimpleWriteln(inSerial);
    }
    // Send a saved AT command using serial port.
    if (!strcmp(inSerial, "TEST"))
    {
      Serial.println("SIGNAL QUALITY");
      gsm.SimpleWriteln("AT+CSQ");
    }
    // Read last message saved.
    if (!strcmp(inSerial, "MSG"))
    {
      Serial.println(msg);
    }
    else
    {
      Serial.println(inSerial);
      gsm.SimpleWriteln(inSerial);
    }
    inSerial[0] = '\0';
  }
}

void serialswread()
{
  gsm.SimpleRead();
}