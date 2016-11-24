/**************************************************************************************************
* \file BluetoothBee.h
*
* \brief Library for BlueTooth Bee by Seeedstudio (first testing version; revision pending)
*
* \author	GEKO NavSat S.L. <info@gekonavsat.com>
* \since 	10/04/2015
* \date	10/04/2015
* \version	1.0
*************************************************************************************************/

#ifndef BLUETOOTHBEE_H_
#define BLUETOOTHBEE_H_

#include <avr/io.h>
#include <string.h>
#include <stddef.h>
#include <stdio.h>
#include "Arduino.h"
#include "SoftwareSerial.h"
#include "HardwareSerial.h"
#include "KyneoBoard.h"

float latMadrid[4] = {40.413679, 40.415419, 40.419920, 40.405167};
float lonMadrid[4] = {-3.692217, -3.684266, -3.688783, -3.689105};
uint32_t refTemp = 1447240271;   // 11/11/2015 11:11 UTC

//#define BT_SERIAL
#define BT_SERIAL_1
//#define BT_SERIAL_EMULATION
		//#define BTBEE_RX_PIN	8	//BT out MCU in
		//#define BTBEE_TX_PIN	9		//BT in MCU out

#define BT_DEBUG 1

class BluetoothBee
{
	public:
		BluetoothBee();
	
		void init(uint32_t bauds = 57600);
		void connect();
		void data2Peer();
		void data2Peer(char* frame);
		void char2Peer(char myChar);
		int waitForResponse(uint16_t ms, const char* resp);
		char getpeerConnected();
	
		#if defined BT_SERIAL_EMULATION
		// Arduino-like Serial Communication function wrappers.
		void begin(uint32_t bauds) { return BTSerial.begin(bauds); }
		void flush() { return BTSerial.flush(); }
		char available() { return BTSerial.available(); }
		char read() { return BTSerial.read(); }
		char write(uint8_t byte) { return BTSerial.write(byte); }
		char print(const char* buf) { return BTSerial.print(buf); }
	
		#elif defined BT_SERIAL_1
		void begin(uint32_t bauds) { return Serial1.begin(bauds); }
		void flush() { return Serial1.flush(); }
		char available() { return Serial1.available(); }
		char read() { return Serial1.read(); }
		char write(uint8_t byte) { return Serial1.write(byte); }
		char print(const char* buf) { return Serial1.print(buf); }
	
		#else			//BT_SERIAL
		void begin(uint32_t bauds) { return Serial.begin(bauds); }
		void flush() { return Serial.flush(); }
		char available() { return Serial.available(); }
		char read() { return Serial.read(); }
		char write(uint8_t byte) { return Serial.write(byte); }
		char print(const char* buf) { return Serial.print(buf); }
		#endif
	
	private:
		char peerConnected;
		#if defined BT_SERIAL_EMULATION
		SoftwareSerial BTSerial;
		#endif
};

#endif /* BLUETOOTHBEE_H_ */
