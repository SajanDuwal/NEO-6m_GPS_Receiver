/*
 * NMEA.h
 *
 *  Created on: May 21, 2024
 *      Author: sajanduwal
 */

#ifndef INC_NMEA_H_
#define INC_NMEA_H_

typedef struct {
	int hour;
	int min;
	int sec;
} TIME;

typedef struct {
	float latitude;
	char NS;
	float longitude;
	char EW;
} LOCATION;

typedef struct {
	float altitude;
	char unit;
} ALTITUDE;

typedef struct {
	int Day;
	int Mon;
	int Yr;
} DATE;

typedef struct {
	LOCATION location;
	TIME tim;
	int isFixValid;
	ALTITUDE alt;
	int numofast;
}GGASTRUCT;

typedef struct {
	DATE date;
	float speed;
	float course;
	int isValid;
} RMCSTRUCT;

typedef struct {
	GGASTRUCT ggastruct;
	RMCSTRUCT rmcstruct;
} GPSSTRUCT;

int decodeGGA(char *GGAbuffer, GGASTRUCT *gga);
int decodeRMC(char *RMCbuffer, RMCSTRUCT *rmc);

#endif /* INC_NMEA_H_ */
