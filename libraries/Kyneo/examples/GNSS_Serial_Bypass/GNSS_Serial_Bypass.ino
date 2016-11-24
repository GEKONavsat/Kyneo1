/**************************************************************************************************
 * Kyneo GNSS Data Bypass Example.
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
 * This sketch make use of KyneoGNSS library, which internally uses SoftwareSerial library to 
 * configure, read data or send commands to the GNSS device present in Kyneo board. SoftwareSerial 
 * is part of the Arduino libs.
 * 
 * This code example does the following:
 * 1) Initialise the GNSS device, assumming it has the default settings:
 * - Default Serial Config: 9600 bauds
 * - NMEA output: 1Hz fix rate. GPVTG, GPGGA, GNGSA, GPRMC/GNRMC: (rate/1); GPGSV/GLGSV: (rate/5) 
 * 
 * Note: the GNSS device goes back to default settings removing the/running out of battery.
 * 
 * 2) Illustrate how to configure another baudrate and change NMEA output using the KyneoGNSS lib. 
 * 
 * 3) Bypass all the data from GNSS to Serial (Kyneo USB-Serial interface)
 * 
 * Note: "Serial" can be changed with "Serial1", to forward the frames to any serial device  
 * connected to Kyneo headers.
 *************************************************************************************************/
#include <KyneoBoard.h>
#include <SoftwareSerial.h>
#include <KyneoGNSS.h>

KyneoGNSS gnss;

void setup()
{
  Serial.begin(57600);            // Choose your preferred baudrate here.
  gnss.init();                    // equivalent to init(9600). Use last valid baudrate setting here... 
  gnss.setBaudrate(9600);  

  //set rate divider(GPGLL, GPRMC, GPVTG, GPGGA, GPGSA, GPGSV, PMTKCHN). 
  gnss.setNMEAOutput(    0,     1,     5,     0,     0,     0,       0);  //GPRMC 1Hz, GPVTG 0.2Hz

}

void loop()
{
  while(gnss.available()){
    Serial.print(gnss.read());
  }
}

