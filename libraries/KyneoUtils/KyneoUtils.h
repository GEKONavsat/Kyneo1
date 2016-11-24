/**************************************************************************************************
 * \file 			KyneoUtils.h
 *
 * \brief			Auxiliary miscellaneous functions for Kyneo2.
 *
 * \author			GEKO NavSat S.L. <info@gekonavsat.com>
 * \date			03/11/2016
 * \version			2
 * \modifications: 	None (despite of the version number, this is the first version)
 *************************************************************************************************/ 

#ifndef KYNEOUTILS_H_
#define KYNEOUTILS_H_

#include <stdio.h>
#include "Arduino.h"
#include "KyneoBoard.h"

#define KYNEOUTILS_DEBUG	1		// 0 disabled, 1 errors, 2 errors and info.
#define KYNEOUTILS_VERSION	2		// KyneoGNSS Library version

//DEFAULT MODULE CONFIGURATION
#define TIMEOUT	5000				// Maximum time (millis) waiting for response from GNSS module
#define XBEE_RSSI 3		// ###### ERASE THIS VALUE WHEN EXPORTING TO KYNEO2 #########

class KyneoUtils{
	public:
		KyneoUtils();
		unsigned long rssi();		// XBee RSSI value in us (from a maximum of 64us)
		unsigned long rssi(int k);	// XBee RSSI value refered to "k" (tipically k = 100 or k = 1000)
		void battInit();			// Initialices battery measurement (Ref. Voltage 2.56V)
		float battLevel();	// Returns battery level (mV)
		
	private:
		// uint32_t baudrate;
		// uint16_t fixRate_ms;	// Fix Interval in milliseconds. From 100 (10 Hz) to 10000 (0.1 Hz) or 0 (disabled).
		// uint8_t GPGLLdiv;		// GPGLL rate divider 

		unsigned long auxRssi(uint8_t pin, unsigned long timeout);		// general rssi measure function
};

#endif /* KYNEOGNSS_H_ */