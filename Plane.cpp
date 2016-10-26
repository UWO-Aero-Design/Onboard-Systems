////////////////////////////////////////////////////////////////////////
///                       PLANE CLASS HEADER                         ///
////////////////////////////////////////////////////////////////////////
#include "Plane.h"

////////////////////////////////////////////////////////////////////////
///                       PLANE INITIALIZE                           ///
////////////////////////////////////////////////////////////////////////
Plane::Plane(): gpsSS(GPS_IN,GPS_OUT), GPS(&gpsSS), pwm(), dof(), 
					accel(30301), mag(30302), bmp(18001)
{
	// future development for auto stabilization
	autoPilot = false;
	gndAlt = 0;
	usingInterrupt = false; //GPS module

	lon = 0.0;
	lat = 0.0;
	alt = 0.0;
	
	*temperature = 23.0;

	gndAlt = 0.0;
	heading = 0.0;
	spdmps = 0.0;

	sat = 0;

}

////////////////////////////////////////////////////////////////////////
Plane::~Plane() {
	// delete gpsSS;
	// delete bmp;
	// delete pwm;
	// delete dof

}

////////////////////////////////////////////////////////////////////////
///                          PLANE SETUP                             ///
////////////////////////////////////////////////////////////////////////
void Plane::setup()
{
	// communication & i2c begin
	Serial.begin(BAUD_SERIAL);
	Wire.begin();

	// set pin modes
	pinMode(GPS_ENABLE,OUTPUT);
	pinMode(GPS_FIX,INPUT);
	pinMode(THRO,INPUT);
	pinMode(AILE,INPUT);
	pinMode(ELEV,INPUT);
	pinMode(RUDD,INPUT);
	pinMode(GEAR,INPUT);
	pinMode(AUX,INPUT);
	pinMode(BUZZER,OUTPUT);
	pinMode(LED,OUTPUT);

	// start clock
	timer = millis();
	
	// PWM input from 2.4GHz receiver
	pinMode(THRO, INPUT); 
	pinMode(AILE, INPUT); 
	pinMode(ELEV, INPUT); 
	pinMode(RUDD, INPUT); 
	pinMode(GEAR, INPUT);
	pinMode(AUX, INPUT);
		
	// intialize & attach hardware
	initialize_servos();
	initialize_barometer();
	initialize_imu();
	initialize_gps();

	//set default states
	digitalWrite(LED,HIGH);
	getBarometer();
	gndAlt = baroAltitude;


 }

////////////////////////////////////////////////////////////////////////
///                          PLANE FUNCTIONS                         ///
////////////////////////////////////////////////////////////////////////
char Plane::getGroundStation()
{
	//Search for payload toggle
	while (Serial.available())
	{

		char tmp = Serial.read();

		if (tmp == '0'){
			//drop payload
			//beep(1);
			pwm.setPWM(0,0,SERVOMIN);

		}else if (tmp == '1'){
			//load payload
			//beep(1);
			pwm.setPWM(0,0,SERVOMAX);

		}

		return tmp;
	}

	return NULL;
}

bool Plane::telemetryRequest()
{


	
	// test gps
	if (! usingInterrupt) {
	    // read data from the GPS in the 'main loop'
	    char c = GPS.read();
	    // if you want to debug, this is a good time to do it!
	    if (GPSECHO)
	      if (c) Serial.print(c);
	}
  
	// if a sentence is received, we can check the checksum, parse it...
	if (GPS.newNMEAreceived()) {
		// a tricky thing here is if we print the NMEA sentence, or data
		// we end up not listening and catching other sentences! 
		// so be very wary if using OUTPUT_ALLDATA and trytng to print out data
		//Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false

		if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
			return false;  // we can fail to parse a sentence in which case we should just wait for another

	}

	if (getGroundStation() == 'r'){
		// Update telemetry
		updateOnboardStates();

		return true;
	} 

	return false;

}
////////////////////////////////////////////////////////////////////////
void Plane::readCommunicationsFromGroundStation()
{
	/*
		//Read PWM signals from Controller (pilot)
		for (int i = 0; i < NUM_SERVOS; i++) 
		{
			servoPos[i] = pulseIn(THRO+i, HIGH, 20000);
			//delay(15);
		}
*/
		//Serial.println(servoPos[0]);

		//Set Autopilot
		if (servoPos[NUM_SERVOS] >= 1500)
		{
			autoPilot = true;
		}else
		{
			autoPilot = false;
		}

		//Read 915MHz Channel for Waypoint Updates
		if (Serial.available())
		{
			
			//Read in commands
			Serial.println("Received!");
			Serial.flush();
		}


}

////////////////////////////////////////////////////////////////////////
void Plane::sendCommunicationsToGroundStation()
{
	
	// send telemetry to ground station (915MHz channel)
	//Serial.print("$");						// opening character
	//Serial.print(spdmps,0);Serial.print(",");			// GPS speed (ground speed)
    	//Serial.print((int)heading);Serial.print(",");			// GPS heading
    	//Serial.print((int)(baroAltitude*3.28));Serial.print(",");	// barometer altitude
    	//Serial.print(lon,10);Serial.print(",");			// GPS longitude
    	//Serial.print(lat,10);Serial.print(",");			// GPS latitude
    	//Serial.print(alt);Serial.print(",");				// GPS altitude
    	//Serial.print((int)pitch);Serial.print(",");			// IMU x acceleration
    	//Serial.print((int)roll);					// IMU y acceleration
    	//Serial.println("#");						// ending character

	Serial.print("$");						// #0 opening character
	
	Serial.print(spdmps,0);Serial.print(",");			// #1 GPS speed (ground speed)
    	Serial.print((int)heading);Serial.print(",");			// #2 GPS heading
    	Serial.print(lon,10);Serial.print(",");				// #3 GPS longitude
    	Serial.print(lat,10);Serial.print(",");				// #4 GPS latitude
    	Serial.print(alt);Serial.print(",");				// #5 GPS altitude
    	
    	Serial.print((int)(baroAltitude*3.28));Serial.print(",");	// #6 Barometer altitude
    									
    	Serial.print(airspeed);Serial.print(",");			// #7 Pyto tube
    	
    	Serial.print(pitch);Serial.print(",");				// #8 IMU pitch
    	Serial.print(roll);Serial.print(",");				// #9 IMU roll
    	Serial.print(x_acc);Serial.print(",");				// #10 IMU x acceleration
    	Serial.print(y_acc);Serial.print(",");				// #11 IMU y acceleration
    	Serial.print(z_acc);Serial.print(",");				// #12 IMU z acceleration
    	Serial.print(*temperature);						// #13 IMU temperature
    	
    	Serial.println("#");						// ending character
    	// concatenate to be of consistent length

}

////////////////////////////////////////////////////////////////////////
void Plane::setServoPosition()
{
		for (int i = 0; i < NUM_SERVOS; i++) {	
			int tmp = map(servoPos[i],1000,2000,SERVOMIN,SERVOMAX);
			pwm.setPWM(i,0,tmp);
			//delay(15);
		}	

}

////////////////////////////////////////////////////////////////////////
void Plane::navigateAutopilot()
{
	// adjust pitch/yaw/roll targets
	// call AutoPilot Trinket
}

////////////////////////////////////////////////////////////////////////
void Plane::updateOnboardStates(){

	//Update sensors
	getBarometer();
	getAttitude();
	getTemperature(*temperature);
	getAirspeed();
	getLocation();

}

////////////////////////////////////////////////////////////////////////
void Plane::getAttitude()
{
	// nothing yet... read XYZ from trinket running IMU
	// horizon = map(imu.getAccelerationX(),-16000,16000,-90,90);
	//call event
	sensors_event_t accel_event;
	sensors_event_t mag_event;
	sensors_event_t bmp_event;
	sensors_vec_t   orientation;


	/* Calculate pitch and roll from the raw accelerometer data */
	accel.getEvent(&accel_event);
	if (dof.accelGetOrientation(&accel_event, &orientation))
	{
		pitch = orientation.pitch;
		roll  = orientation.roll;
		
		// raw IMU acceleration
		x_acc = accel._accelData.x;
		y_acc = accel._accelData.y;
		z_acc = accel._accelData.z;
		
	}
  
	/* Calculate the heading using the magnetometer */
	mag.getEvent(&mag_event);
	if (dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation))
	{

	}

	/* Calculate the altitude using the barometric pressure sensor */
	bmp.getEvent(&bmp_event);
	if (bmp_event.pressure)
	{

	}

}

////////////////////////////////////////////////////////////////////////
void Plane::getAirspeed()
{
	//Convert airspeed to voltage
	airspeed = map(analogRead(PYTO),515,735,0,60);
	
}

void Plane::getBarometer()
{
	sensors_event_t event;
	bmp.getEvent(&event);
	 
	if (event.pressure)
	{
		bmp.getTemperature(&temperature);
	 	float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
		baroAltitude = (bmp.pressureToAltitude(seaLevelPressure,
					event.pressure)) - gndAlt; 
	  	delay(10);
	}
}

////////////////////////////////////////////////////////////////////////
void Plane::getLocation()
{
 		// if there is a gps fix present, update position
		//if (GPS.fix) {
			
			// lat = GPS.latitude;
			// lon = GPS.longitude;
			// alt = GPS.altitude;

			// for Google maps???
			lat = GPS.latitudeDegrees;
			lon = GPS.longitudeDegrees;
			alt = GPS.altitude;

			spdmps = GPS.speed * 1.852;
			heading = GPS.angle;
			sat = (int)GPS.satellites;
		//}

}

void Plane::beep(uint32_t t)
{
	// default to low state
	digitalWrite(BUZZER,LOW);

	for (int j = 0; j < t; j++)
	{
		for (int i = 0; i < 100; i++)
		{
			digitalWrite(BUZZER,HIGH);
			delay(5);
			digitalWrite(BUZZER,LOW);
			delay(5);
		}
		delay(200);
	}

	//default to low state
	digitalWrite(BUZZER,LOW);
	delay(1000);
}

////////////////////////////////////////////////////////////////////////
bool Plane::intitalizeGPS()
{

	// initialize GPS
	GPS.begin(9600);
	GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
	GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
	GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);
	//GPS.sendCommand(PGCMD_ANTENNA);
	//useInterrupt(true);
	
	//delay(1000);
	gpsSS.println(PMTK_Q_RELEASE); // ask for firmware
	timer = millis();

	return true;
}

////////////////////////////////////////////////////////////////////////
void Plane::useInterrupt(bool v)
{
	//adapter from Adafruit.com
	if (v) {
   		// Timer0 is already used for millis() - we'll just interrupt somewhere
    	// in the middle and call the "Compare A" function above
    	OCR0A = 0xAF;
    	TIMSK0 |= _BV(OCIE0A);
    	usingInterrupt = true;
  	} else {
    	// do not call the interrupt function COMPA anymore
    	TIMSK0 &= ~_BV(OCIE0A);
    	usingInterrupt = false;
  	}

}

/*/ Interrupt is called once a millisecond, looks for any new GPS data, and stores it
void Plane::SIGNAL(TIMER0_COMPA_vect) {
	char c = GPS.read();
	// if you want to debug, this is a good time to do it!
	#ifdef UDR0
	if (GPSECHO)
	   	if (c) UDR0 = c;  
	   	// writing direct to UDR0 is much much faster than Serial.print 
	   	// but only one character can be written at a time. 
	#endif
}*/

////////////////////////////////////////////////////////////////////////
///                             INITIALIZE                           ///
////////////////////////////////////////////////////////////////////////
void Plane::initialize_servos() {
   	
	// initial print to ensure all systems go
	Serial.println("UWO Aero Design - Controls Team");
	Serial.println("Version 1.0: Flight Board Logic");
	Serial.println("________________________________");
	Serial.println(" ");
	Serial.println("Program Initialization:");

   	pwm.begin();	
    	pwm.setPWMFreq(60);

	//attach servos (move to function)
	for (int i = 0; i < NUM_SERVOS; i++) 
	{
		//Set to neutral position
		pwm.setPWM(i,0,SERVOMAX);
		servoPos[i] = 1500;
	}

	Serial.println("Servos:     Successfully Attached");
	//beep(1);

}

////////////////////////////////////////////////////////////////////////
void Plane::initialize_barometer() {

	//initialize barometer
	if(!bmp.begin())
	{
	  /* There was a problem detecting the BMP180 ... check your connections */
	  Serial.println("Ooops, no BMP180 detected ... Check your wiring!");
	  beep(2);
	  while(1);
	}
	Serial.println("Barometer:  Successfully Attached");
	//beep(1);

}

////////////////////////////////////////////////////////////////////////
void Plane::initialize_imu() {

	//initialize imu
	Serial.print("Stabilizer: ");		
	if(!accel.begin())
	{
	  /* There was a problem detecting the LSM303 ... check your connections */
	  Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
	  beep(2);
	  while(1);
	}
	if(!mag.begin())
	{
	  /* There was a problem detecting the LSM303 ... check your connections */
	  Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
	  beep(2);
	  while(1);
	}

	Serial.println("Successfully Attached");
	//beep(1);

}

////////////////////////////////////////////////////////////////////////
void Plane::initialize_gps() {

	gpsSS.begin(BAUD_GPS);

	Serial.print("GPS Module: ");
	if (intitalizeGPS()){
		Serial.println("Successfully Attached");
		//beep(1);
	} else{
		Serial.println("FAILED");
		beep(2);
	}

	Serial.write("\n\n\n");

}

////////////////////////////////////////////////////////////////////////
///                                END                               ///
////////////////////////////////////////////////////////////////////////
