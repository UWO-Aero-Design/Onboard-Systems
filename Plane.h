////////////////////////////////////////////////////////////////////////
///                       PLANE CLASS HEADER                         ///
////////////////////////////////////////////////////////////////////////
#ifndef PLANE_H
#define PLANE_H

// Arduino libraries
#include <SoftwareSerial.h>
#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <I2Cdev.h>

// Adafruit Libraries
#include <Adafruit_Sensor.h>			// default sensor library
#include <Adafruit_10DOF.h>			// 10 DOF sensor
#include <Adafruit_BMP085_U.h>			// barometer
#include <Adafruit_LSM303_U.h>			// accelerometer & magnometer
#include <Adafruit_L3GD20_U.h>			// gyroscope
#include <Adafruit_GPS.h>			// GPS unit
#include <Adafruit_PWMServoDriver.h>		// servo shield

// Aero Design Libraries
// none yet

#define DEBUG
#define OUTPUT_READABLE_ACCELGYRO

#define NUM_SERVOS 	 7
#define offsetPWM    	400

// GPS set up  
#define GPS_IN		 3  
#define GPS_OUT		 2
#define GPSECHO  	 false
#define GPS_ENABLE   	13			// gps enable/disable
#define GPS_FIX      	12   			// gps fix attained

// analog Sensors
#define PYTO		 A0	 		// air speed sensor

// PWM input pins
#define THRO         4
#define AILE         5
#define ELEV         6
#define RUDD         7
#define GEAR         8
#define AUX          9

// status indicators
#define BUZZER		 10
#define LED 		 11

// set servo range for drop mechansim
#define SERVOMIN     480 // this is the 'minimum' pulse 
#define SERVOMAX     340 // this is the 'maximum' pulse 
#define SERVOMID     375

// BAUD rate
#define BAUD_GPS	9600
#define	BAUD_SERIAL	57600

// I2C Communication
//Barometer address    = 0x77;
//Servo Driver address = 0x40;
//IMU Address          =
//Trinket Address      = 

class Plane
{
 	public:
		Plane();
		~Plane();
		void setup();
		void readCommunicationsFromGroundStation();
		void sendCommunicationsToGroundStation();
	    	void updateOnboardStates();
		void getLocation();
		char getGroundStation();
		bool telemetryRequest();

   private:
	   	void setServoPosition();
		void getAttitude();
		void getAirspeed();
		void getBarometer();
		void initialize_servos();
		void initialize_barometer();
		void initialize_imu();
		void initialize_gps();
		void navigateAutopilot();
		void beep(uint32_t t);
		bool intitalizeGPS();



		void useInterrupt(bool v);  //GPS module

		bool  		autoPilot;
		int   		servoPos[NUM_SERVOS];
		float 		baroAltitude;
		float 		*temperature;
		float 		airspeed;
		double 		pitch;
		double		roll;
		uint32_t 	time;
		uint32_t 	timer;	//GPS module
		float 		lon,lat,alt,gndAlt,heading,spdmps;
		float		x_acc, y_acc, z_acc;
	    	int   		sat;
	   	bool  		gps_fix;
	   	bool  		gps_enable;
	   	bool  		usingInterrupt;

	   	// software serial object
	   	SoftwareSerial gpsSS;


	   	// Adafruit Objects
		Adafruit_GPS 				GPS;	// GPS unit
		Adafruit_PWMServoDriver 		pwm;	// servo shield
		Adafruit_10DOF                		dof;	// 10 dof IMU
		Adafruit_LSM303_Accel_Unified 		accel;	// accelerometer
		Adafruit_LSM303_Mag_Unified   		mag;	// magnometer
		Adafruit_BMP085_Unified       		bmp;	// barometer
 

};

#endif

////////////////////////////////////////////////////////////////////////
///                            END                                   ///
////////////////////////////////////////////////////////////////////////
