/*
 * NMEA.C
 *
 *  Created on: May 21, 2024
 *      Author: sajanduwal
 */

#include "NMEA.h"
#include "math.h"
#include "stdint.h"
#include "stdlib.h"
#include "string.h"
#include "string.h"

int inx = 0;
int hr = 0, min = 0, day = 0, mon = 0, yr = 0;
int daychange = 0;

/* Decodes the GGA Data
 * @GGAbuffer is the buffer which stors the GGA Data
 * @GGASTRUCT is the pointer to the GGA structure (in the GPS Structure)
 * @Returns 0 on success
 * @return 1,2 depending on where the return statement is executed, check function for more details
 */

int decodeGGA(char *GGAbuffer, GGASTRUCT *gga) {
	inx = 0;
	char buffer[12];
	int i = 0;
	while (GGAbuffer[inx] != ',')
		inx++; // 1st ','
	inx++;
	while (GGAbuffer[inx] != ',')
		inx++; //After time ','
	inx++;
	while (GGAbuffer[inx] != ',')
		inx++; // after latitude ','
	inx++;
	while (GGAbuffer[inx] != ',')
		inx++; //After NS ','
	inx++;
	while (GGAbuffer[inx] != ',')
		inx++; // After longitude ','
	inx++;
	while (GGAbuffer[inx] != ',')
		inx++; //After EW ','
	inx++; // reach the character to identify the fix

	if ((GGAbuffer[inx] == '1') || (GGAbuffer[inx] == '2')
			|| (GGAbuffer[inx] == '6')) {
		gga->isFixValid = 1;  // fix available
		inx = 0; // reset the index
	} else {
		gga->isFixValid = 0;
		return 1;
	}
	while (GGAbuffer[inx] != ',')
		inx++; // 1st','

	/********* GET Time *************/
	// (Update the GMT offset at the top of this file)
	inx++;
	memset(buffer, '\0', 12);
	i = 0;
	while (GGAbuffer[inx] != ',') {
		buffer[i] = GGAbuffer[inx];
		i++;
		inx++;
	}

	hr = (atoi(buffer) / 10000); // get the hours from the 6 digit number
	min = ((atoi(buffer) / 100) % 100); // get the minutes from the 6 digit number

	// for NPL Time
	//hr += 5;
	//min += 45;

	// adjust time.. this part still needs to be tested
	if (min > 59) {
		min = min - 60;
		hr++;
	}
	if (hr < 0) {
		hr = 24 + hr;
		daychange--;
	}
	if (hr >= 24) {
		hr = hr - 24;
		daychange++;
	}

	//store the time in the GGA structure
	gga->tim.hour = hr;
	gga->tim.min = min;
	gga->tim.sec = atoi(buffer) % 100;

	/************** GET Latitude *********/
	inx++; // Reach the first number in the latitude
	memset(buffer, '\0', 12);
	i = 0;
	while (GGAbuffer[inx] != ',') {
		buffer[i] = GGAbuffer[inx];
		i++;
		inx++;
	}
	if (strlen(buffer) < 6)
		return 2;
	int16_t num = (atoi(buffer));
	int j = 0;
	while (buffer[j] != '.'){
		j++;
	}
	j++;
	int declen = (strlen(buffer)) - j;
	int dec = atoi((char*) buffer + j);
	float lat = (num / 100.0) + (dec / pow(10, (declen + 2)));
	gga->location.latitude = lat;
	inx++;
	gga->location.NS = GGAbuffer[inx]; // save the N/s into the structure

	/************** GET LONGITUDE ************/
	inx++;
	inx++; // Reach the first number in the latitude
	memset(buffer, '\0', 12);
	i = 0;
	while (GGAbuffer[inx] != ',') {
		buffer[i] = GGAbuffer[inx];
		i++;
		inx++;
	}
	num = (atoi(buffer));
	j = 0;
	while (buffer[j] == '.') {
		j++;
	}
	j++;
	declen = (strlen(buffer)) - j;
	dec = atoi((char*) buffer + j);
	lat = (num / 100.0) + (dec / pow(10, (declen + 2)));
	gga->location.longitude = lat;
	inx++;
	gga->location.EW = GGAbuffer[inx];

//skip position fix

	inx++; // ',' after E/W
	inx++; // position fix
	inx++; // ', after position fix

	/*********** number of satellites ******************/

	inx++;
	memset(buffer, '\0', 12);
	i = 0;
	while (GGAbuffer[inx] != ',') {
		buffer[i] = GGAbuffer[inx];
		i++;
		inx++;
	}
	gga->numofast = atoi(buffer);

//skip HDOP
	inx++;
	while (GGAbuffer[inx] != ',')
		inx++;

//Altitude calculation
	inx++;
	memset(buffer, '\0', 12);
	i = 0;
	while (GGAbuffer[inx] != ',') {
		buffer[i] = GGAbuffer[inx];
		i++;
		inx++;
	}
	num = (atoi(buffer));
	j = 0;
	while (buffer[j] != '.'){
		j++;
	}
	j++;
	declen = (strlen(buffer)) - j;
	dec = atoi((char*) buffer + j);
	lat = (num) + (dec / pow(10, (declen)));
	gga->alt.altitude = lat;
	inx++;
	gga->alt.unit = GGAbuffer[inx];
	return 0;
}

int decodeRMC(char *RMCbuffer, RMCSTRUCT *rmc) {

	inx = 0;
	char buffer[12];
	int i = 0;
	while (RMCbuffer[inx] != ',')
		inx++;  // 1st  ,
	inx++;
	while (RMCbuffer[inx] != ',')
		inx++; // after time ,
	inx++;
	if (RMCbuffer[inx] == 'A') {
		rmc->isValid = 1;
	} else {
		rmc->isValid = 0;
		return 1;
	}
	inx++;
	inx++;
	while (RMCbuffer[inx] != ',')
		inx++;   //after latitude
	inx++;
	while (RMCbuffer[inx] != ',')
		inx++; // afte NS
	inx++;
	while (RMCbuffer[inx] != ',')
		inx++;	//after longitude
	inx++;
	while (RMCbuffer[inx] != ',')
		inx++; // after EW

//get speed
	inx++;
	i = 0;
	memset(buffer, '\0', 12);
	while (RMCbuffer[inx] != ',') {
		buffer[i] = RMCbuffer[inx];
		i++;
		inx++;
	}

	if (strlen(buffer) > 0) {
		int16_t num = (atoi(buffer));
		int j = 0;
		while (buffer[j] != '.'){
			j++;
		}
		j++;
		int declen = (strlen(buffer)) - j;
		int dec = atoi((char*) buffer + j);
		float lat = num + (dec / pow(10, (declen)));
		rmc->speed = lat;
	} else {
		rmc->speed = 0;
	}

	// get course
	inx++;
	i = 0;
	memset(buffer, '\0', 12);
	while (RMCbuffer[inx] != ',') {
		buffer[i] = RMCbuffer[inx];
		i++;
		inx++;
	}

	if (strlen(buffer) > 0) {
		int16_t num = (atoi(buffer));
		int j = 0;
		while (buffer[j] != '.')
			j++;
		j++;
		int declen = (strlen(buffer)) - j;
		int dec = atoi((char*) buffer + j);
		float lat = num + (dec / pow(10, (declen)));
		rmc->course = lat;
	} else {
		rmc->course = 0;
	}

	//get Date
	inx++;
	i = 0;
	memset(buffer, '\0', 12);
	while (RMCbuffer[inx] != ',') {
		buffer[i] = RMCbuffer[inx];
		i++;
		inx++;
	}
	day = atoi(buffer) / 10000;
	mon = (atoi(buffer) / 100) % 100;
	yr = atoi(buffer) % 100;

	day = day + daychange;

	rmc->date.Day = day;
	rmc->date.Mon = mon;
	rmc->date.Yr = yr;

	return 0;
}
