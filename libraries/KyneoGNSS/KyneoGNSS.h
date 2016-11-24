/*******************************************************************************************************
 * \file KyneoGNSS.h
 *
 * \brief	This file is a simple library for using gms-g9, the GNSS device present in Kyneo.
 *
 * \author	GEKO NavSat S.L. <info@gekonavsat.com>
 * \date	12/01/2015
 * \version	1.2
 * \modifications:  includes awaiting loops for response from the GNSS module
 *				    indluces getNMEA family functions
 *					Works with following NMEA frames: GPRMC, GPGGA, GPGLL, GPVTG, GPGSA, GPGSV, PMTKCHN
 *						RMC: time, lat, long, speed, date
 *						GGA: time, lat, long, numsats, altitude
 *						GLL: time, lat, long
 *						VTG: speed
 *						GSA: other...
 *						GSV: numsats
 						
 *******************************************************************************************************/

#ifndef KYNEOGNSS_H_
#define KYNEOGNSS_H_

#include <stdio.h>
#include "Arduino.h"
#include "SoftwareSerial.h"
#include "HardwareSerial.h"
#include "KyneoBoard.h"
#include "NMEAparser.h"

#define KYNEOGNSS_DEBUG		1		// 0 disabled, 1 errors, 2 errors and info.
#define KYNEOGNSS_VERSION   2		// KyneoGNSS Library version

//DEFAULT MODULE CONFIGURATION
#define DFLT_GNSS_BAUDRATE	9600u	// Default is 9600 bauds, 8 bit data, 1 stop bit, no parity.
#define DFLT_FIX_RATE_MS	1000u	// Fix frequency (update rate) is 1 Hz (1/1000 ms)
#define DFLT_GPGLL_DIV		0u
#define DFLT_GPRMC_DIV		1u
#define DFLT_GPVTG_DIV		1u
#define DFLT_GPGGA_DIV		1u
#define DFLT_GPGSA_DIV		1u
#define DFLT_GPGSV_DIV		5u
#define DFLT_PMTKCHN_DIV	0u
#define TIMEOUT				5000	// Maximum time (millis) waiting for response from GNSS module

//NMEA SENTENCES MAXIMUM LENGTH
#define GPGLL_MAX_LENGTH	40u
#define GPRMC_MAX_LENGTH	68u
#define GPVTG_MAX_LENGTH	41u
#define GPGGA_MAX_LENGTH	72u		// Review
#define GPGSA_MAX_LENGTH	68u
#define GPGSV_MAX_LENGTH	68u

// Gms-g9 MTK PROTOCOL COMMANDS
#define MAX_CMD_SIZE		64u
#define MAX_NMEA_DIV		5u
#define MIN_FIX_RATE_MS		100u
#define MAX_FIX_RATE_MS		10000u

// Gms-g9 DEFAULT CORRECT ANSWERS
#define ANS220			   "$PMTK001,220,3*30"			// Answer to setFixRate() (for any valid rate value)
#define ANS250			   "$PMTK001,250,3*37"			// Answer to setDPortBaudrate() (for any parameters introduced)
#define ANS301			   "$PMTK001,301,3*32"			// Answer to SetDGPSMode() (for any mode selected)
#define ANS313             "$PMTK001,313,3*31"			// Answer to SBAS Enable or Disable
#define ANS314             "$PMTK001,314,3*36"			// Answer to setNMEAOutput()
#define ANS353_01		   "$PMTK001,353,3,0,1,0,2*36"	// Answer to setSearchMode(0, 1)
#define ANS353_10		   "$PMTK001,353,3,1,0,0,1*35"	// Answer to setSearchMode(1, 0)
#define ANS353_11		   "$PMTK001,353,3,1,1,0,3*36"	// Answer to setSearchMode(1, 1)
#define AND501fr		   "$PMTK501,"					// First part of the answer to getDGPSMode()
#define	ANS513			   "$PMTK513,1*28"				// Answer to checkSBASEnable()

// Gms-g9 RESTART MODES
#define HOT_START		1
#define WARM_START		2
#define COLD_START		3
#define FULL_COLD_START	4

// Gms-g9 DGPS Correction Modes
#define NoDGPS			0
#define RTCM			1
#define SBAS			2

// Gms-g9 Aux Port DGPS Frame Types
#define DportNONE		0
#define DportRTCM		1
#define DportNMEA		2

class KyneoGNSS{
	public:
		KyneoGNSS();
		
		void init(uint32_t bauds = 9600);
		void setDfltConfig();
		int8_t setRTCMConfig(uint32_t bauds);
		int8_t setSBASConfig();
		void restart(uint8_t start_mode);
		
		void setBaudrate(uint32_t bauds);
		int8_t setFixRate(uint16_t fixRate);
		int8_t setNMEAOutput(uint8_t GPGLL, uint8_t GPRMC, uint8_t GPVTG, uint8_t GPGGA, uint8_t GPGSA, uint8_t GPGSV, uint8_t channel);
		int8_t DisNMEAOutput();
		
		int getSingleNMEA(char *frame, int length);
		int getSingleNMEA(char *frame, int length, unsigned int timeout);
		
		int getNMEAlatlon(char *frame, int length);				// writes NMEA only if it is RMC, GGA or GLL (those that have position info)
		int getNMEAtype(char *frame, int length, char *type);
		int getGSV(char *frame, int length);
		int getGSA(char *frame, int length);
		int getVTG(char *frame, int length);
		int getGLL(char *frame, int length);
		int getRMC(char *frame, int length);
		int getGGA(char *frame, int length);
		int getNMEA(char *frame, int length);
		int getNMEA(char *frame, int length, unsigned int timeout, int numFrames);
		int AuxGetNMEA(char *frame, int length, unsigned int timeout, int numFrames);
		
		int getLatLon(float &lat, float &lon);
		uint32_t getutime();				// gets time in Unix format
	  	void gettime(char *frame);			// gets time in char format						// INTRODUCIR EN LA LIBRERÍA VERSION 2!!!!!!!!!!!
	  	void getdate(char *frame);			// gets date in char format
	  	float getlat();						// gets latitude value
	  	float getlon();						// gest longitude value
	  	int16_t getalt();					// gets altitude value
	  	uint16_t gethdg();					// gets heading value
		
		int8_t setSearchMode(uint8_t GPS_EN, uint8_t GLONASS_EN);
		
		int8_t EnSBAS();
		int8_t DisSBAS();
		int8_t checkSBASEnable();
		int8_t SetDGPSMode(uint8_t mode);
		int8_t getDGPSMode();
		int8_t setDPortBaudrate(uint8_t modeIn, uint8_t modeOut, uint32_t bauds);
		
		uint32_t getBaudrate() { return baudrate; }
		uint16_t getFixRate_ms() { return fixRate_ms; }
		void enable() { digitalWrite(GNSS_EN_PIN, HIGH); }
		void disable() { digitalWrite(GNSS_EN_PIN, LOW); }
		
		void begin(uint32_t bauds) { return GNSSserial.begin(bauds); }
		void flush() { return GNSSserial.flush(); }
		char available() { return GNSSserial.available(); }
		char read() { return GNSSserial.read(); }
		char write(uint8_t byte) { return GNSSserial.write(byte); }
		char print(const char* buf) { return GNSSserial.print(buf); }
		
		// Auxiliary functions
		char getChecksum(char *s, uint8_t length); 
		void sendCommand(char *cmd);
		void sendChar(char c);
		int version();								// returns KyneoGNSS Library version
		
	private:
		SoftwareSerial GNSSserial;
		NMEAparser nmea;
		uint32_t baudrate;
		uint16_t fixRate_ms;	// Fix Interval in milliseconds. From 100 (10 Hz) to 10000 (0.1 Hz) or 0 (disabled).
		uint8_t GPGLLdiv;		// GPGLL rate divider 
		uint8_t GPRMCdiv;		// GPRMC rate divider
		uint8_t GPVTGdiv;		// GPVTG rate divider
		uint8_t GPGGAdiv;		// GPGGA rate divider
		uint8_t GPGSAdiv;		// GPGSA rate divider
		uint8_t GPGSVdiv;		// GPGSV rate divider
		uint8_t PMTKCHNdiv;		// Channel status rate divider.
		int compareStrings(char *a, char *b, int lon);		// Similar to strcmp, just little bit enhanced
		int8_t checkResponse(char *x);
		int AuxGetSingleNMEA(char *frame, int length, unsigned int timeout);
		// int AuxGetNMEA(char *frame, int length);
		 
};

#endif /* KYNEOGNSS_H_ */