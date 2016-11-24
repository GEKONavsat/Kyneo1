/**************************************************************************************************
 * Kyneo Basic Test Example.
 * 
 * Created on 22 Jan 2015 by GEKO Navsat S.L.
 * 
 * This example is free software, given under a GPL3 license.
 * 
 * KYNEO is an Arduino-compliant board which includes Movement & Location Sensors and a GNSS device. 
 * All these sensors' data can be logged into an micro-SD or, if a XBee compatible RF module is 
 * attached, they can be wirelessly shared among other devices within the network.
 * 
 * KYNEO is a product designed by GEKO Navsat S.L. 
 * http://www.gekonavsat.com
 * 
 * This simple sketch uses Kyneo board definitions and very basic Arduino code to blink a LED and
 * display Kyneo battery level through serial. This may be used as a basic board test program.  
 * 
 *************************************************************************************************/

//#include <LynxBoard.h>
#include <KyneoBoard.h>
//#include <SoftwareSerial.h>  // This include may be omitted in this example to reduce sketch size
//#include <KyneoGNSS.h>       // This include may be omitted in this example to reduce sketch size


void setup()
{
  boardInit();
  Serial.begin(57600);
}

void loop()
{
  Serial.print("Battery level (mV): ");
  Serial.println(analogRead(BATT_ADC_CH)*10);  // If Aref=1.1V, readValue*10 is ~ Vbattery in mV. 
  digitalWrite(GP_LED_PIN, HIGH);   // LED ON.
  delay(500);
  digitalWrite(GP_LED_PIN, LOW);    // LED OFF.
  delay(500);
}

void boardInit()
{
  pinMode(GP_LED_PIN, OUTPUT);     // Digital output
  pinMode(BATT_LEVEL_PIN, INPUT);  // Analog Input
  analogReference(INTERNAL1V1);    // Sets Analog Reference: internally generated 1.1 Volts
}
