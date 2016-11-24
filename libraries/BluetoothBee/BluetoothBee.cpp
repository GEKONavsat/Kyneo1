/**************************************************************************************************
* \file BluetoothBee.cpp
*
* \brief Library for BlueTooth Bee by Seeedstudio
*
* \author	GEKO NavSat S.L. <info@gekonavsat.com>
* \since 	10/04/2015
* \date		10/04/2015
* \version	1.0
*************************************************************************************************/

#include "BluetoothBee.h"

#if defined BT_SERIAL_EMULATION
BluetoothBee::BluetoothBee() : BTSerial(SoftwareSerial(BTBEE_RX_PIN, BTBEE_TX_PIN))
#else
BluetoothBee::BluetoothBee()
#endif
{
	peerConnected = 0;
}

/**
* \brief { Initialise Bluetooth module in slave mode with ID "Kyneo" }
*/
void BluetoothBee::init(uint32_t bauds)
{
	switch (bauds){
		case 4800:
		case 9600:
		case 14400:
		case 19200:
		case 38400:
		case 57600:
		case 115200:
		break;
		default:
		#if (BT_DEBUG > 0)
		Serial.println("Error: invalid baudrate.");
		Serial.flush();
		#endif
		return;
	}
	begin(bauds);
	peerConnected = 0;
	delay(200);
	print("\r\n+STWMOD=0\r\n");		// Slave mode
	waitForResponse(2500, "OK");
	print("\r\n+STNA=Kyneo\r\n");	// Device name
	waitForResponse(2500, "OK");
	print("\r\n+STAUTO=0\r\n");		// Auto-connect forbidden.
	waitForResponse(2500, "OK");
	print("\r\n+STOAUT=1\r\n");		// Permit paired device to connect me
	waitForResponse(2500, "OK");
	print("\r\n+STPIN=0000\r\n");	// Sets PIN = 0000
	//print("\r\n+DLPIN\r\n");	// Deletes PIN
	waitForResponse(2500, "OK");
	flush();
}

int BluetoothBee::waitForResponse(uint16_t ms, const char* resp)
{
	uint16_t len = sizeof(resp);
	if ((len >= 255) || (len == 0)){
		return 2;
	}
	char i = 0;
	char aux;
	uint32_t expiration = millis() + ms;
	while(millis() < expiration){
		while(available()){
			aux = read();
			#if (BT_DEBUG > 1)
			Serial.print(aux);
			#endif
			if (aux == resp[i]){
				i++;
			}
			else{
				i = 0;
			}
			if (i >= len){
				return 0;
			}
		}
	}
	#if (BT_DEBUG > 0)
	Serial.println("Rsp not rcvd");
	#endif
	return 1;
}

void BluetoothBee::connect()
{
	print("\r\n+INQ=1\r\n");
	if(waitForResponse(10000, "CONNECT:OK") == 0){
		flush();
		peerConnected = 1;
		#if (BT_DEBUG > 0)
		Serial.println("Connected");
		#endif
	}
}

void BluetoothBee::data2Peer()
{
	if(peerConnected == 1){
		char frame[20];
		memset(frame, 0, sizeof(frame));
		char csum = 0;
		uint32_t *ui = (uint32_t*) &frame[4];
		float *lat = (float*) &frame[8];
		float *lon = (float*) &frame[12];
		char j = (unsigned char) refTemp % 4;

		frame[0] = 0x16;
		frame[2] = 0xC0 + j;
		frame[3] = 0x6E;
		*ui = refTemp++;
		*lat = latMadrid[j];
		*lon = lonMadrid[j];
		frame[16] = 0x14 << j;
		frame[18] = j;

		for(char i = 0; i < 20; i++){
			if (i < 19){
				csum ^= frame[i];
			} else{
				frame[i] = csum;
			}
			write(frame[i]);
			#if (BT_DEBUG > 1)
				Serial.print((unsigned char) frame[i], HEX);
				Serial.print(' ');
			#endif
		}
		#if (BT_DEBUG > 1)
			Serial.print("\r\n");
		#endif
	}

	else {
		connect();
	}
}

void BluetoothBee::data2Peer(char* frame)
{
	if(peerConnected == 1){
		print(frame);
	}
	else {
		connect();
	}
}

void BluetoothBee::char2Peer(char myChar)
{
	if(peerConnected == 1){
		write(myChar);
	}
}


char BluetoothBee::getpeerConnected(){
	return this->peerConnected;
}