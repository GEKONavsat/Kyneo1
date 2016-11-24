/**************************************************************************************************
 * \file KyneoBoard.h
 *
 * \brief	Some hardware definitions for Kyneo_v1 board. 
 *
 * \author	GEKO NavSat S.L. <info@gekonavsat.com>
 * \date	12/01/2015
 * \version	1.0
 *************************************************************************************************/ 

#ifndef KYNEOBOARD_H_
#define KYNEOBOARD_H_

#define KYNEO_VERSION 		1			// Kyneo HW version

#define SD_CS_PIN			22			// Kyneo SD chip select pin => D22 / pin25 in ATmega1284P.

#define GNSS_TX_PIN			0			// MCU TX line (GPS RX line) => D0 / pin40 in ATmega1284P.
#define GNSS_RX_PIN 		1			// MCU RX line (GPS TX line) => D1 / pin41 in ATmega1284P.
#define GNSS_1PPS_PIN		2			// GPS 1 pulse per second output. 
#define GNSS_EN_PIN			23			// GPS Enable, active high with pull-up to 3,3V.

#define GP_LED_PIN			13			// General purpose LED pin => D13 / pin29 in ATmega1284P.

#define BATT_ADC_CH			0			// Kyneo measures Battery Level at pin A0 / pin 37 in ATmega1284P.
#define BATT_LEVEL_PIN		A0			// Kyneo analog channel for Batt level measurement
#define BATT_RTOP			80600.00	// R value (ohms) of the Resistor placed between Vcc and Batt level pin
#define BATT_RBOTTOM		10000.00	// R value (ohms) of the Resistor placed between Batt level pin and GND

#define VCC					5000		// Supply voltage (mV)
	
#endif /* KYNEOBOARD_H_ */