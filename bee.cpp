
/*
    Copyright (C) 2013  Andrew Myatt

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

*/

#define HABJOE_MEGAMAIN
//#define DEBUG_ON
#define FULLUNIT

#include <XBee.h>
#include <SPI.h>
#include <RFM22.h>
#include <util/crc16.h>
#include <SdFat.h>
#include "I2Cdev.h"
#include <TinyGPS_HJOE.h>
#include <L3G.h>
#include <ADXL345.h>
#include <Comp6DOF_n0m1.h>
#include <HMC5883L.h> 
#include <BMP085.h>
#define _Digole_Serial_I2C_  
#include <DigoleSerial.h>
#include <Wire.h>
#include <math.h> 


#ifndef cosf
# define cosf cos
#endif

#ifndef sinf
# define sinf sin
#endif

#ifndef tanf
# define tanf tan
#endif

#ifndef fabsf
# define fabsf fabs
#endif

#ifndef fmodf
# define fmodf fmod
#endif

#ifndef sqrtf
# define sqrtf sqrt
#endif

#ifndef cbrtf
# define cbrtf cbrt
#endif

#ifndef hypotf
# define hypotf hypot
#endif

#ifndef squaref
# define squaref square
#endif

#ifndef floorf
# define floorf floor
#endif

#ifndef ceilf
# define ceilf ceil
#endif

#ifndef frexpf
# define frexpf frexp
#endif

#ifndef ldexpf
# define ldexpf ldexp
#endif

#ifndef expf
# define expf exp
#endif

#ifndef coshf
# define coshf cosh
#endif

#ifndef sinhf
# define sinhf sinh
#endif

#ifndef tanhf
# define tanhf tanh
#endif

#ifndef acosf
# define acosf acos
#endif

#ifndef asinf
# define asinf asin
#endif

#ifndef atanf
# define atanf atan
#endif

#ifndef atan2f
# define atan2f atan2
#endif

#ifndef logf
# define logf log
#endif

#ifndef log10f
# define log10f log10
#endif

#ifndef powf
# define powf pow
#endif

#ifndef isnanf
# define isnanf isnan
#endif

#ifndef isinff
# define isinff isinf
#endif

#ifndef isfinitef
# define isfinitef isfinite
#endif

#ifndef copysignf
# define copysignf copysign
#endif

#ifndef signbitf
# define signbitf signbit
#endif

#ifndef fdimf
# define fdimf fdim
#endif

#ifndef fmaf
# define fmaf fma
#endif

#ifndef fminf
# define fminf fmin
#endif

#ifndef truncf
# define truncf trunc
#endif

#ifndef roundf
# define roundf round
#endif

#ifndef lroundf
# define lroundf lround
#endif

#ifndef lrintf
# define lrintf lrint
#endif

// radius of earth in meters
#define RADIUS_OF_EARTH 6378100
// scaling factor from 1e-7 degrees to meters at equator
// == 1.0e-7 * DEG_TO_RAD * RADIUS_OF_EARTH
#define LOCATION_SCALING_FACTOR 0.011131884502145034f
// inverse of LOCATION_SCALING_FACTOR
#define LOCATION_SCALING_FACTOR_INV 89.83204953368922f

/***************************************************
 * Main program
 **************************************************/

// LED Definitions
#define SET_LED_WHITE 4
#define SET_LED_RED 3
#define SET_LED_BLUE 2
#define SET_LED_GREEN 1
#define SET_LED_OFF 0

// Define filenames
#define SD_LOG_FILE "mylog.CSV"


// Arduino Pin assignment
#define PIN_GPS_RX 0        	//Fixed:Note: RX on Board is connected to RX on GPS Board
#define PIN_GPS_TX 1        	//Fixed:Note: TX on Board is connected to TX on GPS Board
#define PIN_GYRO_INT 2       	//Notes: Gyro Interupt
#define PIN_LED_RED 6        	//Fixed: Red LED
#define PIN_LED_BLUE 5       	//Fixed: Blue LED
#define PIN_LED_GREEN 7      	//Fixed: Blue GREEN
#define PIN_IC2_SDA 20       	//Fixed: SDA
#define PIN_IC2_SLC 21       	//Fixed: SLC
#define PIN_SPI_SD_CS 53        //Fixed: Card Select for SD Card
#define PIN_SPI_MOSI 51        	//Fixed: Card Select for DISPLAY 
#define PIN_SPI_MISO 50        	//Fixed: Card Select for DISPLAY 
#define PIN_SPI_SCK 52       	//Fixed: Card Select for DISPLAY

#define ADXL345_ADDRESS 0x53	// Device address as specified in data sheet 
#define HMC5883_ADDRESS 0x1E 	//0011110b, I2C 7bit address of HMC5883
#define BMP085_ADDRESS 0x77  	// I2C address of BMP085
#define DS12864OLED_ADDRESS 0x27	// I2C address of BMP085
#define aref_voltage 3.3 

#define SD_BUFF_SIZE 512
#define MAX_XBUNITS 10
#define MAXscr_count 200
#define MAGDECLINATION 3.0

char SDBuffer[SD_BUFF_SIZE];
unsigned long usl_count;		//4	 
	  
prog_uchar welcomeimage[] PROGMEM = {
255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255
,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255
,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255
,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255
,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255
,255,255,255,255,255,240,0,0,124,0,0,127,255,255,255,255
,255,255,255,255,255,192,0,0,124,0,0,15,255,255,255,255
,255,255,255,255,255,0,0,0,124,0,0,7,255,255,255,255
,255,255,255,255,254,0,0,0,124,0,0,3,255,255,255,255
,255,255,255,255,254,31,255,255,255,255,255,193,255,255,255,255
,255,255,255,255,254,63,255,255,255,255,255,225,255,255,255,255
,255,255,255,255,252,63,255,255,255,255,255,241,255,255,255,255
,255,255,255,255,252,63,255,255,255,255,255,241,255,255,255,255
,255,255,255,255,252,127,255,255,255,255,255,241,255,255,255,255
,255,255,255,255,252,127,255,255,255,255,255,241,255,255,255,255
,255,255,255,255,252,127,255,255,255,255,255,241,255,255,255,255
,255,255,255,255,252,127,255,255,255,255,255,241,255,255,255,255
,255,255,255,255,252,127,255,255,255,255,255,241,255,255,255,255
,255,255,255,255,252,127,255,255,255,255,255,241,255,255,255,255
,255,255,255,255,252,127,255,255,255,255,255,241,255,255,255,255
,255,255,255,255,252,127,255,255,255,255,255,241,255,255,255,255
,255,255,255,255,252,127,255,255,255,255,255,241,255,255,255,255
,255,255,255,255,252,127,255,255,255,255,255,241,255,255,255,255
,255,255,255,255,252,127,255,255,255,255,255,225,255,255,255,255
,255,255,255,255,252,127,255,255,255,255,255,195,255,255,255,255
,255,255,255,255,252,127,255,255,252,0,0,3,255,255,255,255
,255,255,255,255,252,127,255,255,252,0,0,7,255,255,255,255
,255,255,255,255,252,127,255,255,252,0,0,7,255,255,255,255
,255,255,255,255,252,127,255,255,255,255,255,131,255,255,255,255
,255,255,255,255,252,127,255,255,255,255,255,225,255,255,255,255
,255,255,255,255,252,127,255,255,255,255,255,225,255,255,255,255
,255,255,255,255,252,127,255,255,255,255,255,241,255,255,255,255
,255,255,255,255,252,127,255,255,255,255,255,241,255,255,255,255
,255,255,255,255,252,127,255,255,255,255,255,241,255,255,255,255
,255,255,255,255,252,127,255,255,255,255,255,241,255,255,255,255
,255,255,255,255,252,127,255,255,255,255,255,241,255,255,255,255
,255,255,255,255,252,127,255,255,255,255,255,241,255,255,255,255
,255,255,255,255,252,127,255,255,255,255,255,241,255,255,255,255
,255,255,255,255,252,127,255,255,255,255,255,241,255,255,255,255
,255,255,255,255,252,127,255,255,255,255,255,241,255,255,255,255
,255,255,255,255,252,127,255,255,255,255,255,241,255,255,255,255
,255,255,255,255,252,127,255,255,255,255,255,241,255,255,255,255
,255,255,255,255,252,127,255,255,255,255,255,241,255,255,255,255
,255,255,255,255,252,63,255,255,255,255,255,241,255,255,255,255
,255,255,255,255,252,63,255,255,255,255,255,241,255,255,255,255
,255,255,255,255,254,63,255,255,255,255,255,225,255,255,255,255
,255,255,255,255,254,31,255,255,255,255,255,193,255,255,255,255
,255,255,255,255,254,0,0,0,124,0,0,3,255,255,255,255
,255,255,255,255,255,0,0,0,124,0,0,7,255,255,255,255
,255,255,255,255,255,128,0,0,124,0,0,15,255,255,255,255
,255,255,255,255,255,240,0,0,124,0,0,63,255,255,255,255
,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255
,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255
,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255
,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255
,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255
,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255
,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255
,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255
,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255
,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255
,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255
,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255
,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255
};

const unsigned char fonts[] = {6, 10, 18, 51, 120, 123};
const char *fontdir[] = {"0\xb0", "90\xb0", "180\xb0", "270\xb0"};

 /***********************************************************************************************************************************
 * Data definitions
 * 
 *
 */

#if defined(FULLUNIT)
SdFat 				SD;				//SD
SdFile 				SDCard;			//SDFile
L3G 				LGgyro;			//L£G Gyro
ADXL345 			accel;			//accelerometer
HMC5883L 			compass;		//Magnetometer/Compass
Comp6DOF_n0m1 		sixDOF;			//Tilt compensation from Compass
BMP085 				dps;			//Pressure and Temp
DigoleSerialDisp oledDisp(&Wire,DS12864OLED_ADDRESS); 
#endif

TinyGPS_HJOE 		gps;			//GPS
XBee xbee = XBee();
XBeeResponse response = XBeeResponse();
// create reusable response objects for responses we expect to handle 
ZBRxResponse rx = ZBRxResponse();
ModemStatusResponse msr = ModemStatusResponse();
ZBTxStatusResponse txStatus = ZBTxStatusResponse();

XBeeAddress64	remoteAddress64_BRD = XBeeAddress64(0x00000000, 0x0000FFFF);
uint16_t        remoteAddress16_BRD = 0x0000FFFF;


// Declare utility global variables variables
int 		error = 0;
unsigned long elapseXbee = millis();
bool		NEWGPSDATA;
int			scr_no = 0;
int			xb_no = 0;
int			scr_count = 0;


typedef struct{
	  unsigned long usl_count;		//4	  
	  byte 	  year;					//1 		  
	  byte    month;				//1	  
	  byte    day;					//1	  
	  byte    hour;					//1
	  byte    minute;				//1
	  byte    second;				//1
	  long    i_lat;				//4
	  long    i_long;				//4
	  long    i_alt; 				//4	  
	  long    i_compass;			//4  	compass bearing using tilt compensation etc.
	  long 	  as_status;			//1
	  long	  as_command;			//1
	  byte 	  sats;					//1	
	  unsigned long age; 			//4
	  unsigned long ihdop;			//4
	  long    i_angle;				//4	
	  long    i_Hspeed;				//4		Horizontal speed
	  long    i_Vspeed;				//4		Vertical speed
	  int16_t Xax;					//2		accel x
	  int16_t Xay;					//2		accel y
	  int16_t Xaz;					//2		accel z
	  int16_t Xgx;					//2		gyro x
	  int16_t Xgy;					//2		gyro y
	  int16_t Xgz; 					//2		gyro z 
	  int16_t Xmx;					//2		mag x
	  int16_t Xmy;					//2		mag y
	  int16_t Xmz; 					//2		mag z 
	  long	  BMP085_PFULL;			//4 
	  long	  BMP085_TFULL;			//4  
} LOG_DATA_STRUCTURE;	
LOG_DATA_STRUCTURE vals;

 typedef struct {
		  bool		valid;				//1		
		  uint16_t	Address16;			//2
		  XBeeAddress64 Address64;			//8
		  byte 	  	year;				//2		  
		  byte    	month;				//1	  
		  byte      day;				//1		  
    	  byte    	hour;				//1
    	  byte    	minute;				//1
    	  byte    	second;				//1
    	  long    	i_lat;				//4
    	  long    	i_long;				//4
		  long    	i_compass;			//4 
	      long 	  	as_status;			//4
	      long 	  	as_command;			//4		  
		  char		name[10];			//10
    	  long    	c_distance;			//4
    	  long    	c_heading;			//4		  
} TBL_STATUS;
TBL_STATUS xbTable[MAX_XBUNITS];


typedef union {

	uint8_t xbRaw[80];
    typedef struct {
		  byte	  msgtype;				//1    ==1	  
    } MSG_TYPE;
    MSG_TYPE msgtype;
	
    typedef struct {
		  byte	  msgtype;				//1    ==1
		  byte    year;					//1
    	  byte    month;				//1
    	  byte    day;					//1	
    	  byte    hour;					//1	  
    	  byte    minute;				//1
    	  byte    second;				//1
    	  long    i_lat;				//4
    	  long    i_long;				//4
		  long    i_compass;			//4 
	      long 	  as_status;				//4		  
    } MSG_MAIN;
    MSG_MAIN msgmain;
 
    typedef struct {
		  byte	  msgtype;				//1    ==2
		  byte    year;					//1
    	  byte    month;				//1
    	  byte    day;					//1	
    	  byte    hour;					//1
    	  byte    minute;				//1
    	  byte    second;				//1
		  long	  as_command;			//4
    } MSG_CMD;
	MSG_CMD msgcmd;
		
	typedef struct {
		  byte	  msgtype;				//1    ==3
		  char	  name[10];				//4
    } MSG_NAME;
    MSG_NAME msgname;

} XB_PACKET;
XB_PACKET xbPacket;

/*
 * MATHS FUNCTIONS

*/

struct Location {
//uint8_t id;                                                 ///< command id
//    uint8_t options;                                    ///< options bitmask (1<<0 = relative altitude)
//    uint8_t p1;                                                 ///< param 1
    int32_t alt;                                        ///< param 2 - Altitude in centimeters (meters * 100)
    int32_t lat;                                        ///< param 3 - Latitude * 10**7
    int32_t lng;                                        ///< param 4 - Longitude * 10**7
};

// a varient of asin() that checks the input ranges and ensures a
// valid angle as output. If nan is given as input then zero is
// returned.
float safe_asin(float v)
{
    if (isnan(v)) {
        return 0.0;
    }
    if (v >= 1.0f) {
        return PI/2;
    }
    if (v <= -1.0f) {
        return -PI/2;
    }
    return asinf(v);
}

// a varient of sqrt() that checks the input ranges and ensures a
// valid value as output. If a negative number is given then 0 is
// returned. The reasoning is that a negative number for sqrt() in our
// code is usually caused by small numerical rounding errors, so the
// real input should have been zero
float safe_sqrt(float v)
{
    float ret = sqrtf(v);
    if (isnan(ret)) {
        return 0;
    }
    return ret;
}

// a faster varient of atan.  accurate to 6 decimal places for values between -1 ~ 1 but then diverges quickly
float fast_atan(float v)
{
    float v2 = v*v;
    return (v*(1.6867629106f + v2*0.4378497304f)/(1.6867633134f + v2));
}


// constrain a value
float constrain_float(float amt, float low, float high) 
{
	// the check for NaN as a float prevents propogation of
	// floating point errors through any function that uses
	// constrain_float(). The normal float semantics already handle -Inf
	// and +Inf
	if (isnan(amt)) {
		return (low+high)*0.5f;
	}
	return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

// constrain a int16_t value
int16_t constrain_int16(int16_t amt, int16_t low, int16_t high) {
	return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

// constrain a int32_t value
int32_t constrain_int32(int32_t amt, int32_t low, int32_t high) {
	return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}
/*
// degrees -> radians
float radians(float deg) {
	return deg * DEG_TO_RAD;
}

// radians -> degrees
float degrees(float rad) {
	return rad * RAD_TO_DEG;
}

// square
float sq(float v) {
	return v*v;
}
*/
// 2D vector length
float pythagorous2(float a, float b) {
	return sqrtf(sq(a)+sq(b));
}

// 3D vector length
float pythagorous3(float a, float b, float c) {
	return sqrtf(sq(a)+sq(b)+sq(c));
}

float longitude_scale(const struct Location &loc)
{
    static int32_t last_lat;
    static float scale = 1.0;
    if (labs(last_lat - loc.lat) < 100000) {
        // we are within 0.01 degrees (about 1km) of the
        // same latitude. We can avoid the cos() and return
        // the same scale factor.
        return scale;
    }
    scale = cosf(loc.lat * 1.0e-7f * DEG_TO_RAD);
    last_lat = loc.lat;
    return scale;
}

// return distance in meters between two locations
float get_distance(const struct Location &loc1, const struct Location &loc2)
{
    float dlat              = (float)(loc2.lat - loc1.lat);
    float dlong             = ((float)(loc2.lng - loc1.lng)) * longitude_scale(loc2);
    return pythagorous2(dlat, dlong) * LOCATION_SCALING_FACTOR;
}

// return distance in centimetres to between two locations
uint32_t get_distance_cm(const struct Location &loc1, const struct Location &loc2)
{
    return get_distance(loc1, loc2) * 100;
}

// return bearing in centi-degrees between two locations
int32_t get_bearing_cd(const struct Location &loc1, const struct Location &loc2)
{
    int32_t off_x = loc2.lng - loc1.lng;
    int32_t off_y = (loc2.lat - loc1.lat) / longitude_scale(loc2);
    int32_t bearing = 9000 + atan2f(-off_y, off_x) * 5729.57795f;
    if (bearing < 0) bearing += 36000;
    return bearing;
}


/*
 *  extrapolate latitude/longitude given bearing and distance
 * Note that this function is accurate to about 1mm at a distance of 
 * 100m. This function has the advantage that it works in relative
 * positions, so it keeps the accuracy even when dealing with small
 * distances and floating point numbers
 */
void location_update(struct Location &loc, float bearing, float distance)
{
    float ofs_north = cosf(radians(bearing))*distance;
    float ofs_east  = sinf(radians(bearing))*distance;
    location_offset(loc, ofs_north, ofs_east);
}

/*
 *  extrapolate latitude/longitude given distances north and east
 *  This function costs about 80 usec on an AVR2560
 */
void location_offset(struct Location &loc, float ofs_north, float ofs_east)
{
    if (ofs_north != 0 || ofs_east != 0) {
        int32_t dlat = ofs_north * LOCATION_SCALING_FACTOR_INV;
        int32_t dlng = (ofs_east * LOCATION_SCALING_FACTOR_INV) / longitude_scale(loc);
        loc.lat += dlat;
        loc.lng += dlng;
    }
}


/*
  wrap an angle in centi-degrees to 0..36000
 */
int32_t wrap_360_cd(int32_t error)
{
    if (error > 360000 || error < -360000) {
        // for very large numbers use modulus
        error = error % 36000;
    }
    if (error > 36000) error -= 36000;
    if (error < 0) error += 36000;
    return error;
}

/*
  wrap an angle in centi-degrees to -18000..18000
 */
int32_t wrap_180_cd(int32_t error)
{
    if (error > 360000 || error < -360000) {
        // for very large numbers use modulus
        error = error % 36000;
    }
    if (error > 18000) { error -= 36000; }
    if (error < -18000) { error += 36000; }
    return error;
}

/*
  wrap an angle defined in radians to -PI ~ PI (equivalent to +- 180 degrees)
 */
float wrap_PI(float angle_in_radians)
{
    if (angle_in_radians > 10*PI || angle_in_radians < -10*PI) {
        // for very large numbers use modulus
        angle_in_radians = fmodf(angle_in_radians, 2*PI);
    }
    while (angle_in_radians > PI) angle_in_radians -= 2*PI;
    while (angle_in_radians < -PI) angle_in_radians += 2*PI;
    return angle_in_radians;
}

/*
 * LED STATUS DISPLAY FUNTIONS
 * LED is a Common Anode, therefore the pin is the cathode
 * and must be set LOW to be on, and HIGH to be turned off! 
*/
void SET_LED_Status(int stat, int intDelay){
  
  digitalWrite(PIN_LED_RED, HIGH);
  digitalWrite(PIN_LED_GREEN, HIGH);
  digitalWrite(PIN_LED_BLUE, HIGH); 
  if (stat == SET_LED_RED) {
      digitalWrite(PIN_LED_RED, LOW);
  } else if (stat == SET_LED_BLUE) {
      digitalWrite(PIN_LED_BLUE, LOW);
  } else if (stat == SET_LED_GREEN) {
      digitalWrite(PIN_LED_GREEN, LOW);
  } else if (stat == SET_LED_WHITE){
    digitalWrite(PIN_LED_RED, LOW);
    digitalWrite(PIN_LED_GREEN, LOW);
    digitalWrite(PIN_LED_BLUE, LOW); 
  }
  if (intDelay > 0 ) delay(intDelay);
}

void SET_LED_Status(int stat){
  SET_LED_Status(stat, 1000);
}
#if defined(FULLUNIT)
void oleddump() {
	char oledString0[22] = "";
	char oledString1[22] = "";
	char oledString2[22] = "";
	char oledString3[22] = "";
	char oledString4[22] = "";
	int x_origin =90;
	int y_origin =30;
	int x_oncircle = 0;
	int y_oncircle = 0;
	int x_onArrow1 = 0;
	int y_onArrow1 = 0;
	int x_onArrow2 = 0;
	int y_onArrow2 = 0;
	int radius = 6;
	int arrowLen = 15;
	switch (scr_no) {
	case 0: {
		if ( scr_count % 10 == 0 ) {
			int tmp_year = vals.year + 2000;	
			sprintf(oledString0, "%04d-%02d-%02d %02d:%02d:%02d", tmp_year, vals.month, vals.day, vals.hour, vals.minute, vals.second); 
			sprintf(oledString1, "Lt:%ld",vals.i_lat);
			sprintf(oledString2, "Lg:%ld",vals.i_long);  
			sprintf(oledString3, "T:%ld",vals.BMP085_TFULL);
			sprintf(oledString4, "C:%ld",vals.i_compass);
			x_oncircle = x_origin + (radius + arrowLen) * (-cos (vals.i_compass * PI / 180));
			y_oncircle = y_origin + (radius + arrowLen) * (sin (vals.i_compass * PI / 180));
			int iArrow1 = vals.i_compass+315;
			int iArrow2 = vals.i_compass+45;
			if (iArrow1 > 360) iArrow1 = iArrow1 - 360;
			if (iArrow2 > 360) iArrow2 = iArrow2 - 360;
			x_onArrow1 = x_oncircle + radius * (-sin (iArrow1 * PI / 180));
			y_onArrow1 = y_oncircle + radius * (-cos (iArrow1  * PI / 180));
			x_onArrow2 = x_oncircle + radius * (sin (iArrow2 * PI / 180));
			y_onArrow2 = y_oncircle + radius * (cos (iArrow2  * PI / 180));			
			oledDisp.clearScreen();
			oledDisp.setFont(fonts[1]);
			oledDisp.drawStr(0, 0, oledString0);
			oledDisp.drawStr(0, 1, oledString1);
			oledDisp.drawStr(0, 2, oledString2);
			oledDisp.drawStr(0, 3, oledString3);
			oledDisp.drawStr(0, 4, oledString4);	
			oledDisp.drawCircle(x_origin, y_origin, radius); //draw a circle
			oledDisp.drawCircle(x_origin, y_origin, radius+1); //draw a circle
			oledDisp.drawLine(x_origin, y_origin, x_oncircle, y_oncircle);
			oledDisp.drawLine(x_origin+1, y_origin+1, x_oncircle+1, y_oncircle+1);
			oledDisp.drawLine(x_oncircle, y_oncircle, x_onArrow1, y_onArrow1);
			oledDisp.drawLine(x_oncircle, y_oncircle, x_onArrow1+1, y_onArrow1+1);
			oledDisp.drawLine(x_oncircle, y_oncircle, x_onArrow2, y_onArrow2);
			oledDisp.drawLine(x_oncircle, y_oncircle, x_onArrow2+1, y_onArrow2+1);
			}
		scr_count++;
		if (scr_count> MAXscr_count) {
			scr_count = 0;
			scr_no++;
		}
		break; }
	case 1: {
		if (xbTable[xb_no].valid){
			if ( scr_count % 10 == 0 ) {
				sprintf(oledString0, "%d %c%c%c%c%c%c%c%c%c",xb_no,xbTable[xb_no].name[0],
				xbTable[xb_no].name[1],
				xbTable[xb_no].name[2],
				xbTable[xb_no].name[3],
				xbTable[xb_no].name[4],
				xbTable[xb_no].name[5],
				xbTable[xb_no].name[6],
				xbTable[xb_no].name[7],
				xbTable[xb_no].name[8],
				xbTable[xb_no].name[9]); 
				sprintf(oledString1, "Lt:%ld",xbTable[xb_no].i_lat);
				sprintf(oledString2, "Lg:%ld",xbTable[xb_no].i_long);  
				sprintf(oledString3, "D:%ld",xbTable[xb_no].c_distance);  
				sprintf(oledString4, "H:%ld",xbTable[xb_no].c_heading);
				oledDisp.clearScreen();
				oledDisp.setFont(fonts[1]);
				oledDisp.drawStr(0, 0, oledString0);
				oledDisp.drawStr(0, 1, oledString1);
				oledDisp.drawStr(0, 2, oledString2);
				oledDisp.drawStr(0, 3, oledString3);
				oledDisp.drawStr(0, 4, oledString4);
				oledDisp.drawCircle(x_origin, y_origin, radius); //draw a circle
				oledDisp.drawCircle(x_origin, y_origin, radius+1); //draw a circle
				if (xbTable[xb_no].c_heading < 0) {
					oledDisp.drawCircle(x_origin, y_origin, 1); //draw a circle
					oledDisp.drawCircle(x_origin, y_origin, 2); //draw a circle					
				} else {
					int ad_heading = vals.i_compass + xbTable[xb_no].c_heading + MAGDECLINATION;
					if (ad_heading > 0) ad_heading = 360 + ad_heading;
					x_oncircle = x_origin + (radius + arrowLen) * (sin (ad_heading * PI / 180));
					y_oncircle = y_origin + (radius + arrowLen) * (cos (ad_heading * PI / 180));
					oledDisp.drawLine(x_origin, y_origin, x_oncircle, y_oncircle);
					oledDisp.drawLine(x_origin+1, y_origin+1, x_oncircle+1, y_oncircle+1);
					oledDisp.drawCircle(x_oncircle, y_oncircle, 1); //draw a circle
					oledDisp.drawCircle(x_oncircle, y_oncircle, 2); //draw a circle
				} 
			}
			scr_count++;
			if (scr_count> MAXscr_count) {
				scr_count = 0;
				xb_no++;
			}
		} else {
			xb_no++;
		}
		if (xb_no >= MAX_XBUNITS) {
			scr_no=0;
			scr_count = 0;
			xb_no=0;
		}
		break; }
	default: {
		scr_no=0;
		scr_count = 0;
		xb_no=0; }
	}
}
#endif

void datadump() {

    char SDString[200] = "";
   	int tmp_year = vals.year + 2000;	
    
	sprintf(SDString, "%ld,%04d-%02d-%02d,%02d:%02d:%02d,%ld,%ld,%ld,%ld,%ld,%d,%d,%d,%d,%d,%d,%d,%d,%d,%ld,%ld,%ld,%d,%d,%d,\n", 
	vals.usl_count,tmp_year, vals.month, vals.day, vals.hour, vals.minute, vals.second, 
	vals.i_lat,vals.i_long,vals.i_alt,vals.i_angle,vals.i_Hspeed,
	vals.Xax,vals.Xay,vals.Xaz,
	vals.Xgx,vals.Xgy,vals.Xgz,
	vals.Xmx,vals.Xmy,vals.Xmz,
	vals.BMP085_PFULL,vals.BMP085_TFULL,
	vals.i_compass,vals.sats,vals.age,vals.ihdop);  
	
#if defined(FULLUNIT)		
	if (sizeof(SDBuffer)-strlen(SDBuffer) < strlen(SDString)) {
		SDCard.write(SDBuffer,strlen(SDBuffer));
		SDCard.sync();		
		memset(SDBuffer, 0, sizeof(SDBuffer));
	}
	
	strcat(SDBuffer,SDString);
	Serial3.println(SDString);		//Bluetooth
#endif
#if defined(DEBUG_ON)
	Serial.println(SDString);		//Debug
#endif
	for (int i = 0; i<MAX_XBUNITS;i++) {
		if (xbTable[i].valid) {	
			int tmp_year = xbTable[i].year + 2000;	
			sprintf(SDString, "%d,%04d-%02d-%02d,%02d:%02d:%02d,%ld,%ld,%ld,%ld,%ld,%ld,%ld\n",
			i,tmp_year,xbTable[i].month, xbTable[i].day, xbTable[i].hour, xbTable[i].minute,xbTable[i].second,	
			xbTable[i].i_lat,xbTable[i].i_long,xbTable[i].i_compass,xbTable[i].as_status,xbTable[i].as_command,  
			xbTable[i].c_distance,xbTable[i].c_heading);
			
			#if defined(FULLUNIT)
			if (sizeof(SDBuffer)-strlen(SDBuffer) < strlen(SDString)) {
				SDCard.write(SDBuffer,strlen(SDBuffer));
				SDCard.sync();		
				memset(SDBuffer, 0, sizeof(SDBuffer));
			}
			strcat(SDBuffer,SDString);
			Serial3.println(SDString);		//Bluetooth
			#endif			
#if defined(DEBUG_ON)
			Serial.println(SDString);		//Debug
#endif
		}
	}
}

static bool feedgps() {
  while (Serial1.available()) {
    if (gps.encode(Serial1.read()))
      return true;
	}
  return false;
}

void calBrDs(){
	for (int i = 0; i<MAX_XBUNITS;i++) {
		if (xbTable[i].valid) {
			if (((vals.day * 86400) + (vals.hour * 3600) + (vals.minute * 60)) > 
				((xbTable[i].day * 86400) + (xbTable[i].hour * 3600) + (xbTable[i].minute * 60) + 300)) {
				xbTable[i].valid = false;
			} else if (vals.i_lat > 0 || vals.i_long > 0) {
			
			struct Location loc1, loc2;
			loc1.lat = vals.i_lat;
			loc1.lng = vals.i_long;
			loc2.lat = xbTable[i].i_lat;
			loc2.lng = xbTable[i].i_long;
			xbTable[i].c_distance = get_distance_cm(loc1,loc2);
			xbTable[i].c_heading =  wrap_360_cd(get_bearing_cd(loc1,loc2))/100
			Serial.print(xbTable[i].c_distance);
			Serial.print(xbTable[i].c_heading);
			}
		}
	}
}

void XbeeRX(){

    xbee.readPacket();
	
	if (xbee.getResponse().isAvailable()) {

		uint8_t api_id=xbee.getResponse().getApiId();
	        Serial.print("Got Packet: ");
            Serial.print(api_id, HEX);
			Serial.println("");
		if (api_id == ZB_RX_RESPONSE) {

			xbee.getResponse().getZBRxResponse(rx);
			XBeeAddress64 remoteaddress64 = rx.getRemoteAddress64();
			uint16_t remoteaddress = rx.getRemoteAddress16();
			Serial.print("Got remote16 address: ");
            Serial.print(remoteaddress);
			Serial.println("");
			Serial.print("Got remote64 address: ");
            Serial.print(remoteaddress64.getMsb(),HEX);
			Serial.print(" ");
			Serial.print(remoteaddress64.getLsb(),HEX);
			Serial.println("");
			int ifound = -1;
			int i = 0;
			while (ifound == -1 && i <MAX_XBUNITS) {
				if (xbTable[i].valid && xbTable[i].Address64.getMsb() == remoteaddress64.getMsb() && 
					xbTable[i].Address64.getLsb() == remoteaddress64.getLsb()) {
					ifound = i;
					Serial.print("Found Existing Xbee");
					Serial.print(ifound);
					Serial.print(" ");
				} else {
					i++;
				}
			}
			i = 0;
			while (ifound == -1 && i <MAX_XBUNITS) {
				if (!xbTable[i].valid) {
					ifound = i;
					xbTable[ifound].valid = true;
					xbTable[ifound].Address16 = remoteaddress;
					xbTable[ifound].Address64 = remoteaddress64;
					sprintf(xbTable[ifound].name, "%X",remoteaddress64.getLsb());
					Serial.print("Creating New Xbee");
					Serial.print(ifound);
					Serial.print(" ");						
				} else {
					i++;
				}
			}
			if (ifound > -1) {
						Serial.print("Assigning data from Xbee");
						Serial.print(ifound);
						Serial.print(" ");		
				int ilen = rx.getDataLength();
				Serial.print ("Length of data: ");
				Serial.print (ilen);
				Serial.println ("");
				for (int i = 0; i < ilen; i++) {
					xbPacket.xbRaw[i] = rx.getData(i);
					Serial.print(xbPacket.xbRaw[i],HEX);
					Serial.println("");
				}
				
				if (xbPacket.xbRaw[0] == 0) {			
					xbTable[ifound].year 		= xbPacket.msgmain.year;
					xbTable[ifound].month 		= xbPacket.msgmain.month; 
					xbTable[ifound].day 		= xbPacket.msgmain.day; 	  
					xbTable[ifound].hour		= xbPacket.msgmain.hour;
					xbTable[ifound].minute		= xbPacket.msgmain.minute;
					xbTable[ifound].second		= xbPacket.msgmain.second;
					xbTable[ifound].i_lat		= xbPacket.msgmain.i_lat;
					xbTable[ifound].i_long		= xbPacket.msgmain.i_long;
					xbTable[ifound].i_compass	= xbPacket.msgmain.i_compass;
					xbTable[ifound].as_status	= xbPacket.msgmain.as_status;
				} else if (xbPacket.xbRaw[0] == 1) {
					xbTable[ifound].year 		= xbPacket.msgmain.year;
					xbTable[ifound].month 		= xbPacket.msgmain.month; 
					xbTable[ifound].day 		= xbPacket.msgmain.day;  
					xbTable[ifound].hour		= xbPacket.msgcmd.hour;
					xbTable[ifound].minute		= xbPacket.msgcmd.minute;
					xbTable[ifound].second		= xbPacket.msgcmd.second;
					xbTable[ifound].as_command 	= xbPacket.msgcmd.as_command;
				} else if (xbPacket.xbRaw[0] == 2) {
					for (int i = 0; i <10; i++) {
						xbTable[ifound].name[i] = xbPacket.msgname.name[i];
					}
				}
			    char SDString[200] = "";
			int ix = ifound;
            int tmp_year = xbTable[ix].year + 2000;
			sprintf(SDString, "%d,%04d-%02d-%02d,%02d:%02d:%02d,%ld,%ld,%ld,%ld,%ld\n",
			ix,tmp_year,xbTable[ix].month, xbTable[ix].day, xbTable[ix].hour, xbTable[ix].minute,xbTable[ix].second,	
			xbTable[ix].i_lat,xbTable[ix].i_long,xbTable[ix].i_compass,xbTable[ix].as_status,xbTable[ix].as_command);
			Serial.println(SDString);
			}

		} else if (api_id == ZB_TX_STATUS_RESPONSE) {
			//

		} else if (api_id == MODEM_STATUS_RESPONSE) {
			xbee.getResponse().getModemStatusResponse(msr);
			if (msr.getStatus() == ASSOCIATED) {
				Serial.println("Associated");
			} else if (msr.getStatus() == DISASSOCIATED) {
							Serial.println("Disassociated");
			} else {
			}

		} else if (api_id  == REMOTE_AT_COMMAND_RESPONSE){
			//

		} else if (api_id == ZB_IO_SAMPLE_RESPONSE){
			//

		} else if(api_id == AT_RESPONSE){
			//

		} else {
		
		}
	}
}

void setup(){

	Serial.begin(115200);	//Debug
	Serial1.begin(9600); 	//GPS
    Serial2.begin(9600);	//Xbee
	Serial3.begin(115200);	//Bluetooth
    delay(500);
    Serial2.print("\n\r\n\r");
    delay(500);
    Serial2.print("B\n\r");
    Serial2.end();
    Serial2.begin(115200);
    delay (500);
	xbee.begin(Serial2);
	
	pinMode(PIN_LED_GREEN, OUTPUT);
	pinMode(PIN_LED_RED, OUTPUT);
	pinMode(PIN_LED_BLUE, OUTPUT);
	pinMode(PIN_SPI_SD_CS,OUTPUT);  //Chip Select Pin for the SD Card
	pinMode(10,OUTPUT);  //Chip Select Pin for the Display
	
	SET_LED_Status(SET_LED_RED,500);

	//Setup GPS
	gps.init(1);
	gps.configureUbloxSettings();
	vals.usl_count =0;
	SET_LED_Status(SET_LED_GREEN,500);
	
#if defined(FULLUNIT)
	// join I2C bus //start I2C transfer to the Module/Transmitter
	Wire.begin();

	//Start up the LGgyro
    if (!LGgyro.init()) {
		Serial.println("LGgyro not working!!");
	} else {
	    Serial.println("LGgyro OK");
	}
	LGgyro.enableDefault();
	
	//Start up the accelerometer
	accel = ADXL345(); 
	if(accel.EnsureConnected()) {
		Serial.println("Connected to ADXL345.");
	} else{
		Serial.println("Could not connect to ADXL345.");
	}
	accel.SetRange(2, true);		    	// Set the range of the accelerometer to a maximum of 2G.
	accel.EnableMeasurements();				// Tell the accelerometer to start taking measurements.
  
	//Start up the compass
	compass = HMC5883L(); 					// Construct a new HMC5883 compass.
	error = compass.SetScale(1.3); 			// Set the scale of the compass.
	if(error != 0) 							
		Serial.println(compass.GetErrorText(error));	// If there is an error, print it out.
	error = compass.SetMeasurementMode(Measurement_Continuous); // Set the measurement mode to Continuous
	if(error != 0) 							// If there is an error, print it out.
		Serial.println(compass.GetErrorText(error));

	//Start up the Pressure Sensor
	dps = BMP085();
	dps.init(); 
	
	SET_LED_Status(SET_LED_BLUE,500);   

	//Connect to the SD Card
	if(!SD.init(SPI_HALF_SPEED, PIN_SPI_SD_CS)) {
		SET_LED_Status(SET_LED_RED,500);
		SET_LED_Status(SET_LED_OFF);
		Serial.println("SD not working!!");
		delay(1000);
		return;
	} else {
		Serial.println("SD OK");
	}

	SDCard.open(SD_LOG_FILE, O_CREAT | O_WRITE | O_APPEND);	    //Open Logfile
	if (!SDCard.isOpen()) {
		SET_LED_Status(SET_LED_WHITE,500);
		SET_LED_Status(SET_LED_RED,500);
		return;
	}

    oledDisp.begin();
    oledDisp.clearScreen(); 				//Clear screen
    oledDisp.displayConfig(0);    		//set config display ON, 0=off
    oledDisp.disableCursor(); //disable cursor, enable cursor use: enableCursor();
    oledDisp.drawBitmap(1, 1, 128, 64, welcomeimage);
	SET_LED_Status(SET_LED_OFF,0);  
	SET_LED_Status(SET_LED_WHITE,500);
	SET_LED_Status(SET_LED_GREEN,500);
	elapseXbee = millis();
#endif 
	for (int i = 0; i<MAX_XBUNITS;i++) {
		xbTable[i].valid = false;
	}
	NEWGPSDATA = false;
}

void loop() {
	vals.usl_count++;
#if defined(FULLUNIT)
	dps.getTemperature(&vals.BMP085_TFULL); 
	dps.getPressure(&vals.BMP085_PFULL);
	AccelerometerRaw Araw = accel.ReadRawAxis();
	MagnetometerRaw Mraw = compass.ReadRawAxis();
	LGgyro.read();

	// offset compass by hard iron
	Mraw.XAxis += 40;
	Mraw.YAxis += 261;
	Mraw.ZAxis += 54;
	
	vals.Xax = Araw.XAxis;
	vals.Xay = Araw.YAxis;
	vals.Xaz = Araw.ZAxis;
	vals.Xmx = Mraw.XAxis;
	vals.Xmy = Mraw.YAxis;
	vals.Xmz = Mraw.ZAxis;
	vals.Xgx = LGgyro.g.x;
	vals.Xgy = LGgyro.g.y;
	vals.Xgz = LGgyro.g.z;

	//Perform tilt compensation calculation
	sixDOF.compCompass(Mraw.XAxis, -Mraw.YAxis, -Mraw.ZAxis, Araw.XAxis, Araw.YAxis, Araw.ZAxis, true);

	float compHeading = sixDOF.atan2Int(sixDOF.xAxisComp(), sixDOF.yAxisComp());
	compHeading = compHeading /100;
	if (compHeading < 0 ) 
		compHeading = abs(compHeading);
	else 
		compHeading = 180 - compHeading + 180;
		
	vals.i_compass = compHeading;
	

#endif

	byte lcount = 0;
	while (!NEWGPSDATA && lcount < 255) {
		NEWGPSDATA = feedgps();
		lcount++;
	}
		
	if (NEWGPSDATA) {
		int tmp_year = 0;
        byte hundredths;
		gps.crack_datetime(&tmp_year, &vals.month, &vals.day,&vals.hour, &vals.minute, &vals.second, &hundredths, &vals.age);
		vals.year = tmp_year - 2000;
		gps.get_position(&vals.i_lat, &vals.i_long, &vals.age);
		if (gps.altitude() != TinyGPS_HJOE::GPS_INVALID_ALTITUDE && gps.altitude() >= 0) vals.i_alt = gps.altitude(); 
		vals.i_angle = gps.course();
		vals.i_Hspeed = gps.speed(); 
		vals.sats = gps.satellites();
		vals.ihdop = gps.hdop();
		
		SET_LED_Status(SET_LED_BLUE,0);	
	} else {
		SET_LED_Status(SET_LED_GREEN,0);
	}
	
	XbeeRX();
	
	if (millis() - elapseXbee > 2000) {
		xbPacket.msgmain.msgtype = 		0;
		xbPacket.msgmain.year = 		vals.year;		
		xbPacket.msgmain.month = 		vals.month;		
		xbPacket.msgmain.day = 			vals.day;
    	xbPacket.msgmain.hour = 		vals.hour;
    	xbPacket.msgmain.minute = 		vals.minute;
    	xbPacket.msgmain.second = 		vals.second;
    	xbPacket.msgmain.i_lat = 		vals.i_lat;
    	xbPacket.msgmain.i_long = 		vals.i_long;	
		xbPacket.msgmain.i_compass =  	vals.i_compass;
	    xbPacket.msgmain.as_status = 	vals.as_status;

		ZBTxRequest zbTX = ZBTxRequest(remoteAddress64_BRD, xbPacket.xbRaw, sizeof(xbPacket.msgmain));
		xbee.send(zbTX);
		if (xbee.readPacket(500)) {
			// should be a znet tx status            	
			if (xbee.getResponse().getApiId() == ZB_TX_STATUS_RESPONSE) {
				xbee.getResponse().getZBTxStatusResponse(txStatus);

				// get the delivery status, the fifth byte
				if (txStatus.getDeliveryStatus() == SUCCESS) {
					// success.  time to celebrate
					Serial.println("TxSuccess");
				} else {
					// the remote XBee did not receive our packet. is it powered on?
					Serial.println("TxError");
				}
			}
		} else if (xbee.getResponse().isError()) {
			Serial.print("Error reading packet.  Error code: ");  
			Serial.println(xbee.getResponse().getErrorCode());
		}
		elapseXbee = millis();
	}
	calBrDs();	
	datadump();
	#if defined(FULLUNIT)

		oleddump();
    #else
		delay(100);
	#endif
	SET_LED_Status(SET_LED_OFF,0);
	NEWGPSDATA = false;
}
