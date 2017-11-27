// The Solar Vector tracking code is from the journal paper
// "Computing the Solar Vector"
// Solar Energy 70(5), 431-441, 2001
// authors M. Blanco-Muriel, D.C. Alarcon-Padilla, T. Lopez-Moratalla, and M. Lara-Coira
// Implementation of the Newton algorithm in C
// http://deadline.3x.ro/newtoncode.html
// instructor demonstration Azimuth / Zenith
#define ASCIIZERO 48 // '0' = 48
#define pi 3.1415926535897932384
#define twoPI (2.0*pi)
#define rad (pi/180.0)
#define dEarthRadius 6371.01 // in km
#define dAstroUnit 149597890 // in km
#define OPTOPINAZ 2
#define OPTOPINZE 3
#define MOTZECWCCW 5
#define MOTZECLK 4
#define MOTAZCWCCW 7
#define MOTAZCLK 6
#define MAXAZHOMESTEP 6250
#define MAXZEHOMESTEP 10000 // ZE home max subject to revision
#define STEPSAZHOME 1800 // AZ home position = East = +90 degrees = 1800 steps ***Update for final version => Home = 22.5 degrees***
#define ANGLEZEHOME 90.0 // ZE home position = Horizon = +90 degrees
#define STEPDLY 5
#define ZENITHHOMENUTPOSITION 24
#define STEPSPERDEGAZ 20.0 // 1.8 deg per step and 36:1 reduction worm gears
#define STEPSPERMMZE 100 // temporary demo
//Latitude and Longitude for McMaster (JHE) = 43.260181 (N), 79.920892 (W). Latitude is considered positive to the North and longitude to the East.
//Use decimal format (Latitude = 43 + 26.0181/60 = 43.434; Longitude = -1 * (79 degrees + 92.0892/60) = -80.535;)
const double MCMASTERLATITUDE = 43.434;
const double MCMASTERLONGITUDE = -80.535;
struct cTime {
int iYear;
int iMonth;
int iDay;
double dHours;
double dMinutes;
double dSeconds;
};
struct cLocation {
double dLongitude;
double dLatitude;
};

struct cSunCoordinates {
double dZenithAngle;
double dAzimuth;
};
struct cTime utcTime;
struct cLocation utcLocation;
struct cSunCoordinates utcSunCoordinates;
int iErrorAZFlag; // error flag homing AZ
int iErrorZEFlag; // error flag homing ZE
int iStepsAZ = STEPSAZHOME;
double dAngleZE = ANGLEZEHOME;
double dZenithNutPosition = ZENITHHOMENUTPOSITION;
int iCurrentZEsteps = 8692;
struct cTime currentTime;
//simulating time cancer code


//*************************************************************************************************************************************************
void setup()
 {
	// setup serial communication
	Serial.begin(9600);
	Serial.println("SolarTracker v4.4");
	Serial.println("Serial Connection initalized");
	Serial.println("");
  currentTime.iYear = 2017;
  currentTime.iMonth = 11;
  currentTime.iDay = 27;
  currentTime.dHours = 15;
  currentTime.dMinutes = 12;
  currentTime.dSeconds = 15;
	pinMode(MOTAZCWCCW,OUTPUT); // AZ motor
	pinMode(MOTAZCLK, OUTPUT);
	pinMode(OPTOPINAZ, INPUT); // opto slot sensor AZ
	digitalWrite(MOTAZCWCCW,HIGH); // always go home CCW = HIGH
	pinMode(MOTZECWCCW,OUTPUT); // EL motor
	pinMode(MOTZECLK, OUTPUT);
	pinMode(OPTOPINZE, INPUT); // opto slot sensor ZE
	digitalWrite(MOTZECWCCW,HIGH); // always go home CCW = HIGH
	utcLocation.dLatitude = MCMASTERLATITUDE;
	utcLocation.dLongitude = MCMASTERLONGITUDE;
	Serial.println("Location: McMaster University, Hamilton, ON");
	Serial.print("Latitude (Decimal Format): "); Serial.println(utcLocation.dLatitude);
	Serial.print("Longitude (Decimal Format): "); Serial.println(utcLocation.dLongitude);
	Serial.println("");
	// home the AZ stepper by looking for blocked opto slot, when home = East = 90 degrees = 1800 steps
	homeAzimuth();
	// home the Zenith stepper
	homeZenith();
} // end setup()
//*************************************************************************************************************************************************
void loop() { 
	getCurrentTime();
	beginTracking();
	Serial.println("");
	delay(2000);
	currentTime.dSeconds= currentTime.dSeconds + 2.5;
	if (currentTime.dSeconds >= 60){
		currentTime.dSeconds = 0;
		currentTime.dMinutes++;
	}
	if (currentTime.dMinutes >= 60){
		currentTime.dMinutes = 0;
		currentTime.dHours++;
	}
	if (currentTime.dHours >= 24){
		currentTime.dHours = 0;
		currentTime.iDay++;
	}
} // end loop()
//*************************************************************************************************************************************************
void getCurrentTime() {
	utcTime.dSeconds = currentTime.dSeconds;
	utcTime.dMinutes = currentTime.dMinutes;
	utcTime.dHours = currentTime.dHours;
	 
	utcTime.iDay = currentTime.iDay;
	utcTime.iMonth = currentTime.iMonth;
	utcTime.iYear = currentTime.iYear;
	Serial.println("");
	Serial.println("Universal Coordinate Time");
	Serial.print("Time (Hh:Mm:Ss): ");
	if ((int)utcTime.dHours < 10) Serial.print("0");
	Serial.print((int)utcTime.dHours, DEC);
	Serial.print(":");
	if ((int)utcTime.dMinutes < 10) Serial.print("0");
	Serial.print((int)utcTime.dMinutes, DEC);
	Serial.print(":");
	if (utcTime.dSeconds < 10) Serial.print("0");
	Serial.println((int)utcTime.dSeconds, DEC);
	Serial.print("Date (Dd/Mm/YYYY) ");
	if (utcTime.iDay < 10) Serial.print("0");
	Serial.print(utcTime.iDay, DEC);
	Serial.print("/");
	if (utcTime.iMonth < 10) Serial.print("0");
	Serial.print(utcTime.iMonth, DEC);
	Serial.print("/");
	if (utcTime.iYear < 10) Serial.print("0");
	Serial.println(utcTime.iYear, DEC);
	Serial.println("");
} // end getCurrentTime()
//*************************************************************************************************************************************************
void homeAzimuth() {
	Serial.println("");
	Serial.println("Homing the Azimuth-tracking stage to 90 degrees East of North");
	// Home the AZ stepper by looking for blocked opto slot. Home = East = 90 degrees = 1800 steps
	iErrorAZFlag = 0;
	int iCount;
	int optoStateAZ;
	for (iCount=0; iCount<3600;iCount++){// should be home in 180 deg worth of steps
		optoStateAZ = digitalRead(OPTOPINAZ);
		if (optoStateAZ == HIGH) { // HIGH is blocked (home)
			break; // now home
		} // end if
		digitalWrite(MOTAZCLK, HIGH); // STEP 1.8 DEG (with 36 reduction = 0.05 deg)
		delay(STEPDLY);
		digitalWrite(MOTAZCLK, LOW);
		delay(STEPDLY);
	} // end for
	if (iCount < MAXAZHOMESTEP) {
		// safely home
		iErrorAZFlag = 0;
		iStepsAZ = STEPSAZHOME;
	}
	else {
		// didn't get home in 270 deg
		iErrorAZFlag = 1;
	} // end if
}
 // end homeAzimuth()
//*************************************************************************************************************************************************
void homeZenith() {
	Serial.println("Homing the Zenith-tracking stage to +90 degrees (Horizon)");
	Serial.println("");
	// home the Zenith stepper
	iErrorZEFlag = 0;
	int iCount;
	int optoStateZE;
	for (iCount=0; iCount<3600; iCount++){// should be home in 180 deg worth of steps
		optoStateZE = digitalRead(OPTOPINZE);
		if (optoStateZE == HIGH) { // HIGH is blocked (home)
			break; // now home
		} // end if
		digitalWrite(MOTZECLK, HIGH); // STEP 1.8 DEG (document amount ratio)
		delay(STEPDLY);
		digitalWrite(MOTZECLK, LOW);
		delay(STEPDLY);
	} // end for
	if (iCount < MAXZEHOMESTEP) {
		// safely home
		iErrorZEFlag = 0;
		dAngleZE = ANGLEZEHOME;
	} 
	else {
		// didn't get home
		iErrorZEFlag = 1;
	} // end if
} // end homeZenith()
//*************************************************************************************************************************************************
void beginTracking() {
	Serial.println("Solar Tracking Initalized.");
	Serial.println("-----------------------------------------------------");
	int iDeltaStepsAZ,iDeltaStepsZE;
	GetSunPos(utcTime,utcLocation,&utcSunCoordinates); // get the current solar vector
	Serial.print("Azimuth = "); Serial.println(utcSunCoordinates.dAzimuth);
	Serial.print("Zenith = "); Serial.println(utcSunCoordinates.dZenithAngle);
	Serial.print("Motor AZ= "); Serial.print((double)iStepsAZ/(double)STEPSPERDEGAZ);
	Serial.print(" Current iStepsAZ= "); Serial.print(iStepsAZ);
	iDeltaStepsAZ = (int)(utcSunCoordinates.dAzimuth*STEPSPERDEGAZ) - iStepsAZ;
	Serial.print(" iDeltaStepsAZ= "); Serial.println(iDeltaStepsAZ);
	MoveMotorAZ(iDeltaStepsAZ);
	iStepsAZ = (int)(utcSunCoordinates.dAzimuth*STEPSPERDEGAZ);
	Serial.print("Motor ZE= "); Serial.print(dAngleZE);
	Serial.print(" Current dAngleZE= "); Serial.println(dAngleZE);
	Serial.print("utcSunCoordinates.dZenithAngle = ");
	Serial.println(utcSunCoordinates.dZenithAngle, DEC);
	if (utcSunCoordinates.dZenithAngle > 00.1 && utcSunCoordinates.dZenithAngle < 89.9) {
		int iFutureZEsteps = getZenithSteps(utcSunCoordinates.dZenithAngle);
		int deltaZenithSteps = iCurrentZEsteps - iFutureZEsteps; //store in getZenith result in variable
		MoveMotorZE(deltaZenithSteps);
		dAngleZE = utcSunCoordinates.dZenithAngle;
		iCurrentZEsteps = iFutureZEsteps;
	}
	else {
		Serial.println(" The sun has set - no update");
		homeAzimuth();
		homeZenith();
	} // end if
	Serial.println("-----------------------------------------------------");
} // end beginTracking()
//*************************************************************************************************************************************************
void MoveMotorAZ(int iDeltaStepsAZ) {
	int iCount;
	Serial.print("Moving Azimuth motor this many steps: ");
	Serial.println(iDeltaStepsAZ); 
	if (iDeltaStepsAZ == 0) {
		return;
	} // end if
	if (iDeltaStepsAZ > 0) {
		digitalWrite(MOTAZCWCCW,LOW); // positive CW = LOW
	}
	else {
		iDeltaStepsAZ = -iDeltaStepsAZ;
		digitalWrite(MOTAZCWCCW,HIGH); // negative CCW = HIGH
	} // end if
	delay(10);
	if (iErrorAZFlag == 0) {
		for (iCount=0; iCount<=iDeltaStepsAZ; iDeltaStepsAZ++){
			digitalWrite(MOTAZCLK, HIGH); // STEP 1.8 DEG (with 36 reduction = 0.05 deg)
			delay(STEPDLY);
			digitalWrite(MOTAZCLK, LOW);
			delay(STEPDLY);
		} // end for
	} // end if
} // end MoveMotorAZ()
//*************************************************************************************************************************************************
void MoveMotorZE(int iDeltaStepsZE) {
	int iCount;
	Serial.print("Moving Zenith motor this many steps: ");
	Serial.println(iDeltaStepsZE);
	if (iDeltaStepsZE == 0) {
		return;
	} // end if
	if (iDeltaStepsZE > 0) {
		digitalWrite(MOTZECWCCW,HIGH); // positive CW = LOW
	}
	else {
		iDeltaStepsZE = -iDeltaStepsZE;
		digitalWrite(MOTZECWCCW,LOW); // negative CCW = HIGH
	} // end if
	delay(10);
	if (iErrorZEFlag == 0) {
		for (iCount=0; iCount<=iDeltaStepsZE; iDeltaStepsZE++){
		digitalWrite(MOTZECLK, HIGH); // STEP 1.8 DEG (with 36 reduction = 0.05 deg)
		delay(STEPDLY);
		digitalWrite(MOTZECLK, LOW);
		delay(STEPDLY);
		} // end for
	} // end if
 } // end MoveMotorZE()
//*************************************************************************************************************************************************
void GetSunPos(struct cTime utcTime, struct cLocation utcLocation, struct cSunCoordinates *utcSunCoordinates)
{
	// Main variables
	double dElapsedJulianDays;
	double dDecimalHours;
	double dEclipticLongitude;
	double dEclipticObliquity;
	double dRightAscension;
	double dDeclination;
	// Auxiliary variables
	double dY;
	double dX;
	// Calculate difference in days between the current Julian Day
	// and JD 2451545.0, which is noon 1 January 2000 Universal Time
	 {
	double dJulianDate;
	long int liAux1;
	long int liAux2;
	// Calculate time of the day in UT decimal hours
	dDecimalHours = utcTime.dHours + (utcTime.dMinutes+ utcTime.dSeconds / 60.0 ) / 60.0;
	// Calculate current Julian Day
	liAux1 =(utcTime.iMonth-14)/12;
	liAux2=(1461*(utcTime.iYear + 4800 + liAux1))/4.0 + (367*(utcTime.iMonth- 2-12*liAux1))/12.0- (3*((utcTime.iYear + 4900+ liAux1)/100.0))/4.0+utcTime.iDay-32075;
	dJulianDate=(double)(liAux2)-0.5+dDecimalHours/24.0;
	// Calculate difference between current Julian Day and JD 2451545.0
	 dElapsedJulianDays = dJulianDate-2451545.0;
	 }
	// Calculate ecliptic coordinates (ecliptic longitude and obliquity of the
	// ecliptic in radians but without limiting the angle to be less than 2*Pi
	// (i.e., the result may be greater than 2*Pi)
	 {
	double dMeanLongitude;
	double dMeanAnomaly;
	double dOmega;
	 dOmega=2.1429-0.0010394594*dElapsedJulianDays;
	 dMeanLongitude = 4.8950630+ 0.017202791698*dElapsedJulianDays; // Radians
	 dMeanAnomaly = 6.2400600+ 0.0172019699*dElapsedJulianDays;
	 dEclipticLongitude = dMeanLongitude + 0.03341607*sin( dMeanAnomaly )
	 + 0.00034894*sin( 2*dMeanAnomaly )-0.0001134
	 -0.0000203*sin(dOmega);
	 dEclipticObliquity = 0.4090928 - 6.2140e-9*dElapsedJulianDays
	 +0.0000396*cos(dOmega);
	 }
	// Calculate celestial coordinates ( right ascension and declination ) in radians
	// but without limiting the angle to be less than 2*Pi (i.e., the result may be
	// greater than 2*Pi)
	 {
	double dSin_EclipticLongitude;
	 dSin_EclipticLongitude= sin( dEclipticLongitude );
	 dY = cos( dEclipticObliquity ) * dSin_EclipticLongitude;
	 dX = cos( dEclipticLongitude );
	 dRightAscension = atan2( dY,dX );
	if( dRightAscension < 0.0 ) dRightAscension = dRightAscension + twoPI;
	 dDeclination = asin( sin( dEclipticObliquity )*dSin_EclipticLongitude );
	 }
	// Calculate local coordinates ( azimuth and zenith angle ) in degrees
	 {
	double dGreenwichMeanSiderealTime;
	double dLocalMeanSiderealTime; 
	double dLatitudeInRadians;
	double dHourAngle;
	double dCos_Latitude;
	double dSin_Latitude;
	double dCos_HourAngle;
	double dParallax;
	dGreenwichMeanSiderealTime = 6.6974243242 +
	0.0657098283*dElapsedJulianDays
	+ dDecimalHours;
	dLocalMeanSiderealTime = (dGreenwichMeanSiderealTime*15
	+ utcLocation.dLongitude)*rad;
	dHourAngle = dLocalMeanSiderealTime - dRightAscension;
	dLatitudeInRadians = utcLocation.dLatitude*rad;
	dCos_Latitude = cos( dLatitudeInRadians );
	dSin_Latitude = sin( dLatitudeInRadians );
	dCos_HourAngle= cos( dHourAngle );
	utcSunCoordinates->dZenithAngle = (acos( dCos_Latitude*dCos_HourAngle
	*cos(dDeclination) + sin( dDeclination )*dSin_Latitude));
	dY = -sin( dHourAngle );
	dX = tan( dDeclination )*dCos_Latitude - dSin_Latitude*dCos_HourAngle;
	utcSunCoordinates->dAzimuth = atan2( dY, dX );
	if ( utcSunCoordinates->dAzimuth < 0.0 )
	utcSunCoordinates->dAzimuth = utcSunCoordinates->dAzimuth + twoPI;
	utcSunCoordinates->dAzimuth = utcSunCoordinates->dAzimuth/rad;
	// Parallax Correction
	dParallax=(dEarthRadius/dAstroUnit)
	*sin(utcSunCoordinates->dZenithAngle);
	utcSunCoordinates->dZenithAngle= (utcSunCoordinates->dZenithAngle
	+ dParallax)/rad;
	}
} // end GetSunPos()

//*************************************************************************************************************************************************
int getZenithSteps(double theta) {
	double beta0; // Initial guess
	double beta; // Approximate solution
	double epsilon; // Maximum error
	int maxIterations; // Maximum number of iterations
	int iterations; // Actual number of iterations
	int converged; // Whether iteration converged
	double thetaRAD = theta * rad;
	 beta0 = 0;
	 epsilon = 0.00001;
	 maxIterations = 10000;
	double nutPosition =-1;
	// Find the first positive solution.
	while (nutPosition <= 0) { 
		beta = newtonsMethod(beta0, epsilon, maxIterations, &iterations, &converged, thetaRAD);
		nutPosition = 25*cos(thetaRAD) + 53*cos(thetaRAD+PI+beta);
		beta0++;
	}
	if (converged) {
		Serial.print("Newton algorithm converged after this many steps: ");
		Serial.println(iterations, DEC);
		Serial.print("f(");
		Serial.print(beta, DEC);
		Serial.print(") = ");
		Serial.println(f(thetaRAD, beta), DEC);
	 }
	else {
		Serial.println("Newton's method algorithm didn't converge");
		Serial.print("The final estimate was: ");
		Serial.println(iterations, DEC);
	}
	// Serial.println("");
	// Serial.print("|AD| length (mm): ");
	// Serial.println(nutPosition, DEC);
	// Serial.println("");
	//calculate and return number of steps
	// 0.004mm per step
	return ((int)(nutPosition / 0.004));
} // end getZenithSteps
//*************************************************************************************************************************************************
double newtonsMethod(double beta0, double epsilon, int maxIterations,
	int* iterations_p, int* converged_p, double thetaRAD ) {
	double beta = beta0;
	double beta_prev;
	int iter = 0;
	do {
	iter++;
	beta_prev = beta;
	beta = beta_prev - f(thetaRAD, beta_prev)/f_prime(thetaRAD, beta_prev);
	} while (fabs(beta - beta_prev) > epsilon && iter < maxIterations);
	if (fabs(beta - beta_prev) <= epsilon)
	*converged_p = 1;
	else
	*converged_p = 0;
	*iterations_p = iter;
	return beta;
} // end newtonsMethod
//*************************************************************************************************************************************************
double f(double thetaRAD, double beta) {
	return 15 + 25*sin(thetaRAD) - 53*sin(thetaRAD + beta);
} // end f
//*************************************************************************************************************************************************
double f_prime(double thetaRAD, double beta) {
	return -53*cos(thetaRAD + beta); //the derivative
} // end f_prime
//*************************************************************************************************************************************************
