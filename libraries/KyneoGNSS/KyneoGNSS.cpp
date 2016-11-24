/**************************************************************************************************
 * \file KyneoGNSS.cpp
 *
 * \brief	This file is a simple library for using gms-g9, the GNSS device present in Kyneo.
 *
 * \author	GEKO NavSat S.L. <info@gekonavsat.com>
 * \date	12/01/2015
 * \version	1.1
 *************************************************************************************************/ 

#include "KyneoGNSS.h"

/**
 * \fn <KyneoGNSS() : GNSSserial(SoftwareSerial(GNSS_RX_PIN, GNSS_TX_PIN))>
 * \pre {None.}
 * \brief {Default Constructor. Default configuration is set.}
 */
KyneoGNSS::KyneoGNSS() : GNSSserial(SoftwareSerial(GNSS_RX_PIN, GNSS_TX_PIN))
{
	baudrate = DFLT_GNSS_BAUDRATE;
	fixRate_ms = DFLT_FIX_RATE_MS;
	GPGLLdiv = DFLT_GPGLL_DIV;
	GPRMCdiv = DFLT_GPRMC_DIV;
	GPVTGdiv = DFLT_GPVTG_DIV;
	GPGGAdiv = DFLT_GPGGA_DIV;
	GPGSAdiv = DFLT_GPGSA_DIV;
	GPGSVdiv = DFLT_GPGSV_DIV;
	PMTKCHNdiv = DFLT_PMTKCHN_DIV;
}

/**
 * \fn <init(uint32_t bauds)>
 * \pre {None.}
 * \brief {Initialises the GNSS device.}
 * \param <bauds> {The desired baudrate (Optional) or 9600 by default}
 * \return {void}
 * \note {Use the last valid baudrate setting, as it may be kept in memory due to backup supply}
 */
void KyneoGNSS::init(uint32_t bauds)
{
	enable();	
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
			#if (KYNEOGNSS_DEBUG > 0)
				Serial.println("Error: invalid baudrate.");
				Serial.flush();
			#endif
			return;
	}
	begin(bauds);
}

/**
 * \fn <setBaudrate(uint32_t bauds)>
 * \pre {GNSS software serial must be initialised first.}
 * \brief {Sends command PMTK251 for setting NMEA serial port rate.}
 * \param <bauds> {The desired baudrate. Valid values are 4800,9600,14400,19200,38400,57600,115200}
 * \return {void}
 */
void KyneoGNSS::setBaudrate(uint32_t bauds)
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
			#if (KYNEOGNSS_DEBUG > 0)
				Serial.println("Error: invalid baudrate");
				Serial.flush();
			#endif
			return;
	}
	baudrate = bauds;
	// Command example: $PMTK251,38400*27<CR><LF>
	char *cmd = (char*) malloc(MAX_CMD_SIZE);
	uint8_t length = sprintf(cmd, "$PMTK251,%lu*", bauds);  
	char csum = getChecksum(cmd, length);
	sprintf(cmd + length, "%x\r\n", csum);	// Adds checksum & tail
	
	sendCommand(cmd);
	begin(bauds);
	begin(bauds);
	begin(bauds);
	begin(bauds);
	begin(bauds);
	free(cmd);
}

/**
 * \fn <setDPortBaudrate(uint8_t modeIn, uint8_t modeOut, uint32_t bauds)>
 * \pre {GNSS software serial must be initialised first.}
 * \brief {Sends command PMTK251 for setting NMEA serial port rate.}
 * \param <bauds> {The desired baudrate. Valid values are 4800,9600,14400,19200,38400,57600,115200}
 * \return {void}
 */
int8_t KyneoGNSS::setDPortBaudrate(uint8_t modeIn, uint8_t modeOut, uint32_t bauds){
    char *cmd = (char*) malloc(MAX_CMD_SIZE);
    uint8_t length = 0;
    char csum;
	int8_t ok = 0;
	
    length = sprintf(cmd, "$PMTK250,%u,%u,%lu*", modeIn, modeOut, bauds);      // i.e. $PMTK250,1,0,9600*17
    
    csum = getChecksum(cmd, length);
    sprintf(cmd + length, "%x\r\n", csum);
    sendCommand(cmd);
    free(cmd);
    
    ok = checkResponse(ANS250);		// waits from answer from GNSS (a maximum of TIMEOUT)
	return ok;
}

/**
 * \fn <setFixRate(uint16_t fixRate)>
 * \pre {GNSS software serial must be initialised first.}
 * \brief {Sends command PMTK220 for setting fix frequency (update rate).}
 * \param <updateRate> {The desired rate in milliseconds. Valid values range from 100 to 10000.}
 * \return {void}		
 */
int8_t KyneoGNSS::setFixRate(uint16_t fixRate)
{
	int8_t ok = 0;
	
	if ( (fixRate < MIN_FIX_RATE_MS) || (fixRate > MAX_FIX_RATE_MS) ){
		#if (KYNEOGNSS_DEBUG > 0)
			Serial.println("Error: invalid fix rate.");
			Serial.flush();
		#endif
		ok = 0;
	}
	fixRate_ms = fixRate;
	// Command example: $PMTK220,1000*1F<CR><LF>
	char *cmd = (char*) malloc(MAX_CMD_SIZE);
	uint8_t length = sprintf(cmd, "$PMTK220,%u*", fixRate);  
	char csum = getChecksum(cmd, length);
	sprintf(cmd + length, "%x\r\n", csum);	// Adds checksum & tail
	
	sendCommand(cmd);
	free(cmd);
	
	ok = checkResponse(ANS220);		// waits from answer from GNSS (a maximum of TIMEOUT)
	return ok;
}

/**
 * \fn <setNMEAOutput(uint8_t GPGLL, uint8_t GPRMC, uint8_t GPVTG, uint8_t GPGGA, uint8_t GPGSA, uint8_t GPGSV, uint8_t PMTKCHN)>
 * \pre {GNSS software serial must be initialised first.}
 * \brief {Sends command PMTK314 for setting NMEA output.}
 * \param <GPGLL> {GPGLL sentence: 0 disabled, >1 to set each NMEA sentence rate divider.}
 * \param <GPRMC> {GPRMC/GNRMC (GPS/GPS+GLONASS) sentence: 0 disabled, >1 desired rate divider.}
 * \param <GPVTG> {GPVTG sentence: 0 disabled, >1 desired rate divider.}
 * \param <GPGGA> {GPGGA sentence: 0 disabled, >1 desired rate divider.}
 * \param <GPGSA> {GPGSA/GNGSA (GPS/GPS+GLONASS) sentence: 0 disabled, >1 desired rate divider.}
 * \param <GPGSV> {GPGSV/GLGSV (GPS/GLONASS) sentence: 0 disabled, >1 desired rate divider.}
 * \param <PMTKCHN> {PMTKCHN sentence: 0 disabled, >1 desired rate divider.}
 * \return {0 if no error, -1 otherwise.}
 * \note: {Rate dividers are limited to MAX_NMEA_DIV.}
 */
int8_t KyneoGNSS::setNMEAOutput(uint8_t GPGLL, uint8_t GPRMC, uint8_t GPVTG, uint8_t GPGGA, uint8_t GPGSA, uint8_t GPGSV, uint8_t PMTKCHN)
{
	int8_t ok = 0;
	
	if ( (GPGLL > MAX_NMEA_DIV) || (GPRMC > MAX_NMEA_DIV) || (GPVTG > MAX_NMEA_DIV) ||\
		 (GPGGA > MAX_NMEA_DIV) || (GPGSA > MAX_NMEA_DIV) || (GPGSV > MAX_NMEA_DIV) ) {
		#if (KYNEOGNSS_DEBUG > 0)
			Serial.println("Error: Max NMEA rate divider is 5.");
			Serial.flush();
		#endif
		return -1;		 
	}
	/*						    / NMEA_LENGTH * 1000 (ms/s) \		*
	 *	ESTIMATED MIN_BR =  SUM(  -------------------------  )		*
	 *						    \ FIX_RATE (ms) * NMEA_div  /		*/
	float minBR = 0;
	if (GPGLL > 0) { minBR += (GPGLL_MAX_LENGTH * 1000) / GPGLL; }
	if (GPRMC > 0) { minBR += (GPRMC_MAX_LENGTH * 1000) / GPRMC; }
	if (GPVTG > 0) { minBR += (GPVTG_MAX_LENGTH * 1000) / GPVTG; }
	if (GPGGA > 0) { minBR += (GPGGA_MAX_LENGTH * 1000) / GPGGA; }
	if (GPGSA > 0) { minBR += (GPGSA_MAX_LENGTH * 1000) / GPGSA; }
	if (GPGSV > 0) { minBR += (GPGSV_MAX_LENGTH * 1000) / GPGSV; }
	/**\todo {PMTK length here...} */
	minBR /= fixRate_ms;
	
	if (baudrate < minBR){
		#if (KINEO_GNSSDEBUG > 0)
			Serial.println("Error: Desired NMEA output requires a higher baudrate");
			Serial.flush();
		#endif
		return -1;	
	}
	
	//Current baud rate seems to be enough for the desired output.
	GPGLLdiv = GPGLL;
	GPRMCdiv = GPRMC;
	GPVTGdiv = GPVTG;
	GPGGAdiv = GPGGA;
	GPGSAdiv = GPGSA;
	GPGSVdiv = GPGSV;
	PMTKCHNdiv = PMTKCHN;
	
	// Command example: $PMTK314,1,1,1,1,1,5,0,0,0,0,0,0,0,0,0,0,0,0,0*2C<CR><LF>
	char *cmd = (char*) malloc(MAX_CMD_SIZE);
	uint8_t length = sprintf(cmd, "$PMTK314,%u,%u,%u,%u,%u,%u,0,0,0,0,0,0,0,0,0,0,0,0,%u*", \
							 GPGLL, GPRMC, GPVTG, GPGGA, GPGSA, GPGSV, PMTKCHN);
	char csum = getChecksum(cmd, length);
	sprintf(cmd + length, "%x\r\n", csum);	// Adds checksum & tail
	
	sendCommand(cmd);
	free(cmd);
	
	ok = checkResponse(ANS314);		// waits from answer from GNSS (a maximum of TIMEOUT)
	return ok;						// before it was plain 0 (zero) - now 3 means everything OK
}

/**
 * \fn <DisNMEAOutput()>
 * \pre {GNSS software serial must be initialised first.}
 * \brief {Sends command PMTK314 for setting no NMEA output from GNSS Module.}
 * \return {3 if no error}
 */
int8_t KyneoGNSS::DisNMEAOutput(){
	int8_t ok = 0;
	ok = setNMEAOutput(    0,     0,     0,     0,     0,     0,       0);
	return ok;
}

// -----------------------------------------------------*************************************************--------------------------------------------------------------------------

/**
 * \fn <getSingleNMEA()> (not overload function)
 * \pre {GNSS software serial must be initialised first.}
 * \brief {Gets next NMEA frame transmitted from GNSS module.}
 * \param Char buffer where the NMEA frame is going to be written.
 * \return calls AuxGetSingleNMEA() with default timeout value
 */
int KyneoGNSS::getSingleNMEA(char *frame, int length){
	return AuxGetSingleNMEA(frame, length, TIMEOUT);
}

/**
 * \fn <getSingleNMEA()>  (overload function)
 * \pre {GNSS software serial must be initialised first.}
 * \brief {Gets next NMEA frame transmitted from GNSS module.}
 * \param Char buffer where the NMEA frame is going to be written.
 * \return calls AuxGetSingleNMEA() with user assigned timeout value
 */
int KyneoGNSS::getSingleNMEA(char *frame, int length, unsigned int timeout){
	return AuxGetSingleNMEA(frame, length, timeout);
}

/**
 * \fn <AuxGetSingleNMEA()>  (overload function)
 * \pre {GNSS software serial must be initialised first.}
 * \brief {Gets next NMEA frame transmitted from GNSS module.}
 * \param *frame - Char buffer where the NMEA frame is going to be written.
 *  	  length - sizeof frame
 *        timeout - max time waiting for an NMEA frame to come
 * \return {int} negative if overflow occurred, positive if the NMEA frame was written correctly, zero means timeout without receving full NMEA frame yet
 */
int KyneoGNSS::AuxGetSingleNMEA(char *frame, int length, unsigned int timeout){
	int pos=0, frameON=0, eot=0;
	char c;
	unsigned int now = 0;
	
	now = millis();
			
	while( (frameON == 0) && (eot==0) ){
		if( available() ){
			c = read();
			frame[pos++] = c;
			
			if(c == '\n'){								// end of NMEA frame found
				frame[pos] = (char)0;
				frameON = 1;
			}else if(pos > length){						// overflow occurred
				frameON = -1;
			}
		}
		
		if( ( millis() - now ) > timeout ) eot = 1;		// timeout ended without response
	}
	
	return (pos * frameON);								// positive OK, negative means overflow, zero means no record and timeout
}

/**
 * \fn <getNMEAlatlon()> (limitted by timeout and number of frames)
 * \pre {GNSS software serial must be initialised first.}
 * \brief {Gets next NMEA frame transmitted from GNSS module and writes it if it's RMC, GGA or GLL.}
 * \param Char buffer where the NMEA frames are going to be written.
 * \return 0 if frame it not valid, 3 if it's valid
 */
int KyneoGNSS::getNMEAlatlon(char *frame, int length){
	int ok = 0;
	
	ok = AuxGetNMEA(frame, length, 0, 0);
	
	if(ok != 0){
		if(frame[3]=='G' && frame[4]=='G' && frame[5]=='A'){
		}else if(frame[3]=='G' && frame[4]=='L' && frame[5]=='L'){
		}else if(frame[3]=='R' && frame[4]=='M' && frame[5]=='C'){
		}else{
			frame[0] = (char)0;
			ok = 0;
		}
	}
	return ok;
}

/**
 * \fn <getNMEAtype()> (limitted by timeout and number of frames)
 * \pre {GNSS software serial must be initialised first.}
 * \brief {Gets next NMEA frame transmitted from GNSS module.}
 * \param Char buffer where the NMEA frames are going to be written.
 * \return number of frames written to the frame (only whole NMEA frames)
 */
int KyneoGNSS::getNMEAtype(char *frame, int length, char *type){
	int ok = 0;
	
	ok = AuxGetNMEA(frame, length, 0, 0);
	
	if(ok != 0){
		if(frame[3]==type[0] && frame[4]==type[1] && frame[5]==type[2]){		
		}else{
			frame[0] = (char)0;
			ok = 0;
		}
	}
	return ok;
}

/**
 * \fn <getGSV()> (limitted by timeout and number of frames)
 * \pre {GNSS software serial must be initialised first.}
 * \brief {Gets next NMEA frame transmitted from GNSS module.}
 * \param Char buffer where the NMEA frames are going to be written.
 * \return number of frames written to the frame (only whole NMEA frames)
 */
int KyneoGNSS::getGSV(char *frame, int length){
	int ok = 0;
	
	ok = AuxGetNMEA(frame, length, 0, 0);
	
	if(ok != 0){
		if(frame[3]=='G' && frame[4]=='S' && frame[5]=='V'){		
		}else{
			frame[0] = (char)0;
			ok = 0;
		}
	}
	return ok;
}

/**
 * \fn <getGSA()> (limitted by timeout and number of frames)
 * \pre {GNSS software serial must be initialised first.}
 * \brief {Gets next NMEA frame transmitted from GNSS module.}
 * \param Char buffer where the NMEA frames are going to be written.
 * \return number of frames written to the frame (only whole NMEA frames)
 */
int KyneoGNSS::getGSA(char *frame, int length){
	int ok = 0;
	
	ok = AuxGetNMEA(frame, length, 0, 0);
	
	if(ok != 0){
		if(frame[3]=='G' && frame[4]=='S' && frame[5]=='A'){		
		}else{
			frame[0] = (char)0;
			ok = 0;
		}
	}
	return ok;
}

/**
 * \fn <getVTG()> (limitted by timeout and number of frames)
 * \pre {GNSS software serial must be initialised first.}
 * \brief {Gets next NMEA frame transmitted from GNSS module.}
 * \param Char buffer where the NMEA frames are going to be written.
 * \return number of frames written to the frame (only whole NMEA frames)
 */
int KyneoGNSS::getVTG(char *frame, int length){
	int ok = 0;
	
	ok = AuxGetNMEA(frame, length, 0, 0);
	
	if(ok != 0){
		if(frame[3]=='V' && frame[4]=='T' && frame[5]=='G'){		
		}else{
			frame[0] = (char)0;
			ok = 0;
		}
	}
	return ok;
}

/**
 * \fn <getGLL()> (limitted by timeout and number of frames)
 * \pre {GNSS software serial must be initialised first.}
 * \brief {Gets next NMEA frame transmitted from GNSS module.}
 * \param Char buffer where the NMEA frames are going to be written.
 * \return number of frames written to the frame (only whole NMEA frames)
 */
int KyneoGNSS::getGLL(char *frame, int length){
	int ok = 0;
	
	ok = AuxGetNMEA(frame, length, 0, 0);
	
	if(ok != 0){
		if(frame[3]=='G' && frame[4]=='L' && frame[5]=='L'){		
		}else{
			frame[0] = (char)0;
			ok = 0;
		}
	}
	return ok;
}

/**
 * \fn <getGGA()> (limitted by timeout and number of frames)
 * \pre {GNSS software serial must be initialised first.}
 * \brief {Gets next NMEA frame transmitted from GNSS module.}
 * \param Char buffer where the NMEA frames are going to be written.
 * \return number of frames written to the frame (only whole NMEA frames)
 */
int KyneoGNSS::getGGA(char *frame, int length){
	int ok = 0;
	
	ok = AuxGetNMEA(frame, length, 0, 0);
	
	if(ok != 0){
		if(frame[3]=='G' && frame[4]=='G' && frame[5]=='A'){		
		}else{
			frame[0] = (char)0;
			ok = 0;
		}
	}
	return ok;
}

/**
 * \fn <getRMC()> (limitted by timeout and number of frames)
 * \pre {GNSS software serial must be initialised first.}
 * \brief {Gets next NMEA frame transmitted from GNSS module.}
 * \param Char buffer where the NMEA frames are going to be written.
 * \return number of frames written to the frame (only whole NMEA frames)
 */
int KyneoGNSS::getRMC(char *frame, int length){
	int ok = 0;
	
	ok = AuxGetNMEA(frame, length, 0, 0);
	
	if(ok != 0){
		if(frame[3]=='R' && frame[4]=='M' && frame[5]=='C'){		
		}else{
			frame[0] = (char)0;
			ok = 0;
		}
	}
	return ok;
}

/**
 * \fn <getNMEA()> (limitted by timeout and number of frames)
 * \pre {GNSS software serial must be initialised first.}
 * \brief {Gets next NMEA frame transmitted from GNSS module.}
 * \param Char buffer where the NMEA frames are going to be written.
 * \return number of frames written to the frame (only whole NMEA frames)
 */
int KyneoGNSS::getNMEA(char *frame, int length){
	return AuxGetNMEA(frame, length, 0, 0);
}

/**
 * \fn <getNMEA()> (overload function)
 * \pre {GNSS software serial must be initialised first.}
 * \brief {Gets all NMEA frames transmitted from GNSS module inside the defined timeout window.}
 * \param Char buffer where the NMEA frames are going to be written.
 * \return number of chars written to the frame (only whole NMEA frames)
 */
int KyneoGNSS::getNMEA(char *frame, int length, unsigned int timeout, int numFrames){
	return AuxGetNMEA(frame, length, timeout, numFrames);
}

/**
 * \fn <AuxGetNMEA()> (limitted by timeout and number of frames)
 * \pre {GNSS software serial must be initialised first.}
 * \brief {Gets all NMEA frames transmitted from GNSS module inside the defined timeout window.}
 * \param Char buffer where the NMEA frames are going to be written.
 * \return number of chars written to the frame (only whole NMEA frames)
 */
int KyneoGNSS::AuxGetNMEA(char *frame, int length, unsigned int timeout, int numFrames){
	int pos=0, lastend=0, overf=1, eot=0, num=0, go=0;
	char c;
	unsigned int now = 0;
	
	length--;									// let the last position be free
	if(numFrames == 0) numFrames = 1;			// default value, to avoid being numFrames dependent
	if(timeout == 0) timeout = TIMEOUT;			// default value
	
	now = millis();
			
	while( eot==0 ){
		if( available() ){
			c = read();
			
			if(go==0 && c=='$') go = 1;
			
			if(go==1){
				frame[pos++] = c;
			
				if(c == '\n'){						// end of NMEA frame found
					lastend = pos;						// till now, this is the last NMEA frame written
					num++;								// increase number of frames written in buffer
				}else if(pos > length){				// overflow occurred
					overf = -1;
				}
			}
		}
		
		if( ( ( millis() - now ) > timeout ) || ( num == numFrames ) || (overf == -1) ){	// timeout ended
			if(lastend != 0) frame[lastend] = (char)0;
			eot = 1;
		}
	}
	
	return (num * overf);						// number of frames written to buffer (if negative, some overflow occurred)
}

/**
 * \fn 		<getLatLon()>
 * \pre 	{GNSS software serial must be initialised first.}
 * \brief 	{Gets all NMEA frames transmitted from GNSS module inside the defined timeout window.}
 * \param 	Char buffer where the NMEA frames are going to be written.
 * \return 	number of chars written to the frame (only whole NMEA frames)
 */
int KyneoGNSS::getLatLon(float &lat, float &lon){
	char frame[100];
	int ok = 0;
	
	int frameON = getNMEAlatlon(frame, 300);
	if(frameON != 0){								// frame received
		ok = 1;
		if( nmea.parseNMEA(frame) == 0 ){			// frame parsed to nmea object
			ok = 2;
			lat = nmea.getlat();
			lon = nmea.getlon();
		}
	}
	
	free(frame);
	return ok;
}

// MORE GETTERS..................................................................
uint32_t KyneoGNSS::getutime(){
	return nmea.getutime();
}

void KyneoGNSS::gettime(char *frame){
	nmea.gettime(frame);
}


void KyneoGNSS::getdate(char *frame){
	nmea.getdate(frame);
}

float KyneoGNSS::getlat(){
	return nmea.getlat();
}

float KyneoGNSS::getlon(){
	return nmea.getlon();
}

int16_t KyneoGNSS::getalt(){
	return nmea.getalt();
}

uint16_t KyneoGNSS::gethdg(){
	return nmea.gethdg();
}


// -----------------------------------------------------*************************************************--------------------------------------------------------------------------

/**
 * \fn <setDfltConfig()>
 * \pre {GNSS software serial must be initialised first.}
 * \brief {Restores the default configuration, sending the suitable commands.}
 * \return {void}
 */
void KyneoGNSS::setDfltConfig()
{
	setBaudrate(DFLT_GNSS_BAUDRATE);
	setFixRate(DFLT_FIX_RATE_MS);
	setNMEAOutput(DFLT_GPGLL_DIV, DFLT_GPRMC_DIV, DFLT_GPVTG_DIV, DFLT_GPGGA_DIV, DFLT_GPGSA_DIV, \
				  DFLT_GPGSV_DIV, DFLT_PMTKCHN_DIV);
}

/**
 * \fn <setRTCMConfig()>
 * \pre {GNSS software serial must be initialised first.}
 * \brief {Set RTCM configuration (and disables SBAS congig, since they are incompatible).}
 * \return {3 if everithing OK, 4 if some timeout happened. Any other value means error}
 */
int8_t KyneoGNSS::setRTCMConfig(uint32_t bauds){
	int8_t ok = 0;
	
	ok = DisNMEAOutput();								// no NMEA output
	if(ok == 3){
		ok = SetDGPSMode(1);							// DGPS = RTCM
		if(ok == 3){
			ok = setDPortBaudrate(1,3,bauds);		    // DPort config ON at selected baud rate
			if(ok == 3){
				ok = DisSBAS();							// SBAS Disable (Is this neccesary?)
				if(ok == 3){
					if(getDGPSMode() == 1) ok = 3;		// Check DGPS mode is on RTCM
					else ok = 0;
				}
			}
		}
	}
	return ok;
}

/**
 * \fn <setSBASConfig()>
 * \pre {GNSS software serial must be initialised first.}
 * \brief {Set SBAS configuration (and disables RTCM congig, since they are incompatible.}
 * \return {3 if everithing OK, 4 if some timeout happened. Any other value means error}
 */
int8_t KyneoGNSS::setSBASConfig(){
	int8_t ok = 0;
	
	ok = DisNMEAOutput();								// no NMEA output
	if(ok == 3){
		ok = SetDGPSMode(2);							// DGPS = SBAS
		if(ok == 3){
			ok = setDPortBaudrate(0,0,9600);				// DPort config OFF (at 9600, though it's not going to be used)
			if(ok == 3){
				ok = EnSBAS();							// SBAS Disable
				if(ok == 3){
					if(getDGPSMode() == 2) ok = 3;		// Check DGPS mode is on SBAS
					else ok = 0;
				}
			}
		}
	}
	return ok;
}

/**
 * \fn <restart(uint8_t start_mode)>
 * \pre {GNSS software serial must be initialised first.}
 * \brief {Sends the restart command regarding the requested start mode}
 * \param <start_mode> {Use the defined HOT/WARM/COLD/FULL_COLD start macros. Check Gms-g9 doc.}
 * \return {void}
 */
void KyneoGNSS::restart(uint8_t start_mode)
{
	char *cmd = (char*) malloc(MAX_CMD_SIZE);
	switch(start_mode){
		case HOT_START:			// Use all available data in the NV Store.
			sprintf(cmd, "$PMTK101*32\r\n");
			break;
		case WARM_START:		// Don't use Ephemeris at re-start.
			sprintf(cmd, "$PMTK102*31\r\n");
			break;
		case COLD_START:		// Don't use Time, Position, Almanacs and Ephemeris data.
			sprintf(cmd, "$PMTK103*30\r\n");
			break;
		case FULL_COLD_START:	// Cold start + clear system/user config (reset to factory status).
			sprintf(cmd, "$PMTK104*37\r\n");
			break;
		default:
			#if (KYNEOGNSS_DEBUG > 0)
				Serial.println("Error: invalid restart mode.");
				Serial.flush();
			#endif
			return;
	}
	sendCommand(cmd);
	free(cmd);
	fixRate_ms = DFLT_FIX_RATE_MS;
	GPGLLdiv = DFLT_GPGLL_DIV;
	GPRMCdiv = DFLT_GPRMC_DIV;
	GPVTGdiv = DFLT_GPVTG_DIV;
	GPGGAdiv = DFLT_GPGGA_DIV;
	GPGSAdiv = DFLT_GPGSA_DIV;
	GPGSVdiv = DFLT_GPGSV_DIV;
	begin(DFLT_GNSS_BAUDRATE);
}

/**
 * \fn <setSearchMode(uint8_t GPS_EN, uint8_t GLONASS_EN)>
 * \pre {GNSS software serial must be initialised first.}
 * \brief {Configures the receiver to start searching selected GNSS satellites.}
 * \param <GPS_EN> {GPS enabled (>0) or disabled (0)}
 * \param <GLONASS_EN> {GLONASS enabled (>0) or disabled (0)}
 * \return {3 if everything OK, 4 if Timeout, else if error}
 */
int8_t KyneoGNSS::setSearchMode(uint8_t GPS_EN, uint8_t GLONASS_EN)
{
	int8_t ok = 0;
	
	if ((GPS_EN == 0) && (GLONASS_EN == 0)) {
		#if (KYNEOGNSS_DEBUG > 0)
			Serial.println("Error: ignoring all GNSS staellites is not permitted.");
			Serial.flush();
		#endif
		ok = 0;
		// return;
	}
	else {
		char *cmd = (char*) malloc(MAX_CMD_SIZE);
		// Command example: $PMTK353,1,1*37<CR><LF>
		uint8_t length = sprintf(cmd, "$PMTK353,%u,%u*", (GPS_EN == 0) ? 0: 1, (GLONASS_EN == 0) ? 0: 1);
		char csum = getChecksum(cmd, length);
		sprintf(cmd + length, "%x\r\n", csum);	// Adds checksum & tail
		sendCommand(cmd);
		free(cmd);
		
		// Chech answer from GNSS depending on what was sent to it
		if(!GPS_EN && GLONASS_EN) ok = checkResponse(ANS353_01);
		else if(GPS_EN && !GLONASS_EN) ok = checkResponse(ANS353_10);
		else if(GPS_EN && GLONASS_EN) ok = checkResponse(ANS353_11);
		else ok = 0;
	}
	
	return ok;
}

/**
 * \fn <EnSBAS()>
 * \pre {GNSS software serial must be initialised first.}
 * \brief {Enable SBAS DGPS Correction}
 * \param
 * \return {int8_t: Command response from GNSS Module}
 */
int8_t KyneoGNSS::EnSBAS(){
	int8_t ok = 0;
	
	sendCommand("$PMTK313,1*2E\r\n");
	
	ok = checkResponse(ANS313);
	return ok;
}

/**
 * \fn <DisSBAS()>
 * \pre {GNSS software serial must be initialised first.}
 * \brief {Disable SBAS DGPS Correction}
 * \param
 * \return {int8_t: Command response from GNSS Module}
 */
int8_t KyneoGNSS::DisSBAS(){
	int8_t ok = 0;
	
	sendCommand("$PMTK313,0*2F\r\n");
	
	ok = checkResponse(ANS313);
	return ok;
}

/**
 * \fn <SetDGPSMode(uint8_t mode)>
 * \pre {GNSS software serial must be initialised first.}
 * \brief {Disable SBAS DGPS Correction}
 * \param
 * \return {int8_t: Command response from GNSS Module}
 */
int8_t KyneoGNSS::SetDGPSMode(uint8_t mode){
	int8_t ok = 0;
	
	if(mode == 0)		sendCommand("$PMTK301,0*2C\r\n");		// no DGPS correction
	else if(mode == 1)	sendCommand("$PMTK301,1*2D\r\n");		// RTCM correction
	else				sendCommand("$PMTK301,2*2E\r\n");		// SBAS correction (default)
	
	ok = checkResponse(ANS301);
	return ok;
}

/**
 * \fn <getDGPSMode()>
 * \pre {GNSS software serial must be initialised first.}
 * \brief {Query DGPS mode of the GNSS module}
 * \param {none}
 * \return {int8_t: DGPS code mode, or 4 if Timeout happened}
 */
int8_t KyneoGNSS::getDGPSMode(){
	int8_t mode = 0;
	char gnss_idx = 0;
	char gnss_buf[90];
	unsigned long now = 0;
	int8_t ok = 0;
	char c;
	
	sendCommand("$PMTK401*37\r\n");
	
	now = millis();
	
	while(!ok){
		if(!available()){
			if( ( millis() - now ) > TIMEOUT ) ok = 4;		// timeout ended without response
		}else{
			while(available()){
				c = read();
				if(c == '$'){
	   				gnss_idx = 0;
   					gnss_buf[gnss_idx++] = c;
   				}else if(c == '\n'){
	   				if(compareStrings(gnss_buf,AND501fr,strlen(AND501fr)) == 0){
		   				if(gnss_buf[9] == '1') 		mode = 1;						// RTCM: answer was $PMTK501,1*2B
		   				else if(gnss_buf[9] == '2')	mode = 2;						// SBAS: answer was $PMTK501,2*28
		   				else 						mode = 0;						// No DGPS
	   				}
	   				gnss_idx = 0;
   				}else{
	   				gnss_buf[gnss_idx++] = c;
   				}
   				
			}
		}
	}

	return mode;	
}

/**
 * \fn <checkSBASEnable()>
 * \pre {GNSS software serial must be initialised first.}
 * \brief {Ask to GNSS module if SBAS is enabled}
 * \param
 * \return {int8_t: Command response from GNSS Module}
 */
int8_t KyneoGNSS::checkSBASEnable(){
	uint8_t ok = 0;
	
	sendCommand("$PMTK413*34\r\n");
	
	ok = checkResponse(ANS513);
	return ok;
}

// AUXILIARY FUNCTIONS

/**
 * \fn <getChecksum(char *s, uint8_t length)>
 * \brief {Checksum calculation for NMEA MTK command protocol.} 
 * \param <s> {A string starting with '$' and ending with '*' as input data.}
 * \param <length> {This is the total length of the input data string}
 * \return {The checksum char value or -1 if error}
 * \note {This is a helper function}
 * \note {char '*' forces escaping the function}
 */
char KyneoGNSS::getChecksum(char *s, uint8_t length)
{
	if ( (length == 0) || (*s++ != '$') ) {
		#if (KYNEOGNSS_DEBUG > 0)
		Serial.println("Error: start char not found or length = 0 during checksum calculation");
		Serial.flush();
		#endif
		return -1;
	}
	uint8_t csum = 0;
	while (--length) {
		if (*s == '*') {		// end char detected
			return csum;
		}
		csum ^= *s++;			// bit-wise XOR - checksum calculation
	}
	// This point should not be reached...
	#if (KYNEOGNSS_DEBUG > 0)
		Serial.println("Error: end char not found during checksum calculation");
		Serial.flush();
	#endif
	return -1;
}

/**
 * \fn <sendCommand(char *cmd)>
 * \brief {Transmits a command to the GNSS device } 
 * \param <s> {Command in string format, with the suitable header and checksum tail.}
 * \return {void}
 * \note {This is a helper function}
 */
void KyneoGNSS::sendCommand(char *cmd)
{
	#if (KYNEOGNSS_DEBUG > 1)
		Serial.print("MCU->GNS: ");
		Serial.println(cmd);
		Serial.flush();
		delay(100);
	#endif
	
	GNSSserial.println(cmd);
	delay(200);
}

/**
 * \fn <sendChar(char *c)>
 * \brief {Transmits a command to the GNSS device } 
 * \param <s> {Command in string format, with the suitable header and checksum tail.}
 * \return {void}
 * \note {This is a helper function}
 */
void KyneoGNSS::sendChar(char c){
	GNSSserial.println(c);
}

/**
 * \fn <version()>
 * \brief {Returns Kyneo GNSS Library version number } 
 * \param <> {no parameters needed}
 * \return {int}
 * \note {This is a helper function}
 */
int KyneoGNSS::version(){
	return KYNEOGNSS_VERSION;
}	

/**
 * \fn <compareStrings()>
 * \brief {Compare two strings and returns the number of different chars } 
 * \param <> {string a, strin b, and length of the section to be compared}
 * \return {int: numer of chars that differ. If both are equal, it returns zero}
 * \note {This is a helper function}
 */
int KyneoGNSS::compareStrings(char *a, char *b, int lon){
  int i, j = 0;

  for(i = 0; i<lon; i++){
    if(a[i] == b[i])  j++;
  }

  return (i-j);
}

/**
 * \fn <checkResponse()>
 * \brief {Listens to GNSS Serial Com and checks if a certain message arrives} 
 * \param <> {char *x}
 * \return {int8_t: code number of error/success. Everything OK: 3; Timeout occur: 4; (Future releases will implement more type of codes...)}
 * \note {}
 */
int8_t KyneoGNSS::checkResponse(char *x){
	char gnss_idx = 0;
	char gnss_buf[90];
	unsigned long now = 0;
	int8_t ok = 0;
	char c;
	
	now = millis();
	
	while(!ok){
		if(!available()){
			if( ( millis() - now ) > TIMEOUT ) ok = 4;		// timeout ended without response
		}else{
			while(available()){
				c = read();
				if(c == '$'){
	   				gnss_idx = 0;
   					gnss_buf[gnss_idx++] = c;
   				}else if(c == '\n'){
	   				if(compareStrings(gnss_buf,x,strlen(x)) == 0)  ok = 3;
   						gnss_idx = 0;
   				}else{
	   				gnss_buf[gnss_idx++] = c;
   				}
			}
		}
	}
	
	return ok;
}