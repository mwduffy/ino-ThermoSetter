/*
 * ThermoSetter
 * This is a helper app for calibrating thermistors contained in a Masterbuilt
 * smoker. The intent is to set a target temperature and try to maintain that
 * temperature until the smoker box reaches equilibrium. At that point the power
 * is cut, the thermistor isolated and the resistance is measured. The smoker has 
 * a calibrated Taylor oven thermometer hanging from a rack near the sensor to
 * be measured.
 * This process is repeated at another set points. 
 * 
 * The program has several distinct but related tasks implemented using a
 * superloop pattern. The Arduino loop() tests elapsed time values to determine if
 * a task is active. If not it simply returns. This works because these tasks are
 * ordered by an increasing time interval.
 * The tasks:
 *		Read analog pins for thermistors and setpoint potentiometer
 *		Update LCD display
 *		Modulate heating element based on setpoint and checkpoint adjustment
 *		Checkpoint to evaluate adjustment for rate of temperature change
 *
 * mwd 2021-05-19 revised ThermometerLCD.ino to set and hold temperature to generate more precise data
 * mwd 2021-06-04 revised to use slope and band checkpoints to reduce overruns
 * mwd 2021-06-21 revised OLED pin numbers for Nano Every
 * mwd 2021-06-28 V05 revised Arduino pins for layout of soldered board 
 */ 

#include <LiquidCrystal.h>

#define PROGRAM_NAME "ThermoSetter V05"
#define EXTPOWER		// The smoker will provide external power to the Arduino
#define DEBUG_LVL	1	// Extra output to the Serial interface
//#define DISPLAY_ADC_READING	// select ADC values (temperature is default)
//#define BREADBOARD		// select breadboard layout (soldered board is default)

// TODO implement an ambient temp sensor for the board
//#define BOARD_THERM_PIN A5 	

#ifdef BREADBOARD
#define STOVE_THERM_PIN A7 	// oven temp sensor 
#define MEAT_THERM_PIN  A0 	// meat temp sensor
#define SET_POT_PIN 	A3 	// set target temp
#define STOVE_RELAY_PIN 5 	// relay switch for stove heating element
#define STOVE_LED_PIN 	4   	// LED indicator lamp for stove heating element
#else  
// See Nano Interface Board wire graph
#define STOVE_THERM_PIN A0 	// oven temp sensor 
#define STOVE_RELAY_PIN A1 	// relay switch for stove heating element
#define SET_POT_PIN     A2 	// set target temp
#define MAIN_SWITCH_PIN	A3 	// main switch (used as digital input)
#define LIGHT_SWITCH_PIN A4 	// light switch
#define MEAT_THERM_PIN  A5	// meat temp sensor
#define STOVE_LED_PIN   A6   	// LED indicator lamp (red) for stove heating element
#define MAIN_LED_PIN    A7   	// LED indicator lamp (blue)for main switch
#define OLED_RS  4   		// OLED Register Select
#define OLED_E   6   		// OLED Enable
#define OLED_DB4 7   		// OLED Data Bus 4
#define OLED_DB5 8   		// OLED Data Bus 5
#define OLED_DB6 9   		// OLED Data Bus 6
#define OLED_DB7 10   		// OLED Data Bus 7
#endif

/*
	Define the thermistor sensors and their parameters.
	I don't have definitive information on the devices used in the smoker so to start
	these are heuristics taken from a commonly used thermistor.
	A purpose of this program is to generate data that will be use to refine these
	heuristics. The smoker is taken to a temperature at the top of the expected range.
	The power is turned off and the smoker is allowed to cool. A trusted thermometer and
	a multimeter are used to chart temperature and resistance pairs down to the low range. 
	The SRS Thermistor Calculator is used to refine the Steinhart-Hart coefficients.
	https://www.thinksrs.com/downloads/programs/therm%20calc/ntccalibrator/ntccalculator.html
*/
typedef struct {
	float res;   		// resistance class of the thermistor
	float cA, cB, cC; 	// Steinhart-Hart coefficients for this device
	unsigned int pin;	// pin assignment on the board
} t_thermistor;

const t_thermistor stove_therm = {
	.res = 1.0E5, 
	.cA = 3.345620366E-3, 
	.cB = -2.846696310E-4, 
	.cC =  20.67411171E-7, 
	.pin = STOVE_THERM_PIN
};

const t_thermistor meat_therm = {
	.res = 1.0E5, 
	.cA = 3.345620366E-3, 
	.cB = -2.846696310E-4, 
	.cC =  20.67411171E-7, 
	.pin = MEAT_THERM_PIN
};

#ifdef BOARD_THERM_PIN
const t_thermistor board_therm = {
	.res = 10000, 
	.cA = 1.009249522e-03, 
	.cB = 2.378405444e-04, 
	.cC = 2.019202697e-07, 
	.pin = BOARD_THERM_PIN
};
#endif

/*
	Define conditions for the setpoint temperature. 
	The user turns a linear potentiometer dial to set the value.
	A temperature below a fixed value is defined as standby state where no heating will occur.
	The setpoint temperature is adjusted by the slope of delta temp and a fudge factor. 
*/
const int set_pot_error 	= 12; 	// Pot value actually reads as 10-1021 leaving (1024-10-2) increments
const int max_set_temp  	= 350;	// Set temp to be no more than 350
const int temp_start    	= 150;	// but not set to below this
const int standby_threshold = 170;	// indicates no heating below this setpoint
const int temp_range =  max_set_temp - temp_start;  // allowable range to set temp
const float increment = ((float) temp_range) / ((float) (1024 - set_pot_error));
const float chkpt_rising_fudge = 2.2;	// heuristic "brake" for rising temp changes
const float chkpt_falling_fudge = 1.0;	// heuristic "brake" for falling temp changes

//
// The OLED module uses the LCD library. Initialize it for 4-bit mode.
//
LiquidCrystal lcd(OLED_RS, OLED_E, OLED_DB4, OLED_DB5, OLED_DB6, OLED_DB7); 

#if (DEBUG_LVL >= 1)
// Serial port is only used for debugging
#define SERIAL_BUFFER_SIZE 255
char strbuf[SERIAL_BUFFER_SIZE]; // for formatted console writes
#endif

//
// Task Intervals.
// Current code requires these to be in order of increasing duration as code simply 
// returns when a threshold is not met.
//
const int millis_between_adc_reads = 100;
const int millis_between_display_updates = 1000;
const int millis_between_heater_updates = 3000;
const unsigned long millis_between_checkpoints = 60000L;

//
//  Sensor Context.
//	The analog pins are read at an interval and the readings are stored in arrays for averaging.
//  Each pin has its own context struct.
//	Arrays share a common index based on a loop counter. 
//  The index is clipped to create a circular index. 
//  Each reading updates the average of the array contents. 
//
#define READ_ARRAY_SIZE 8

typedef struct {
	unsigned int pin;
	unsigned int adcval;
	unsigned int read_val[READ_ARRAY_SIZE];
	unsigned int avg_reading;
	float avg_temperature;
	char display_str[9];
} t_reading;

t_reading stove = {
	.pin = STOVE_THERM_PIN
};
t_reading meat = {
	.pin = MEAT_THERM_PIN
};

#ifdef BOARD_THERM_PIN
t_reading board = {
	.pin = BOARD_THERM_PIN
};
#endif

t_reading setpot = {
	.pin = SET_POT_PIN
};

//
// loop context
//
unsigned long time_last_measured = 0;
unsigned long time_last_displayed = 0;
unsigned long time_last_heater_update = 0;
unsigned long time_last_checkpoint = 0;
unsigned long last_minute = 0;
unsigned long read_cnt = 0;
float chkpt_band = 0;
float last_chkpt_temp = 0; 
unsigned int circular_inx = 0;
bool stoveOn = false;
bool displayActive = false;
char hhmm_str[9]; 

//
// function declarations
//
unsigned int getAvgReading(t_reading *);
float getTemp(const t_thermistor *, unsigned int);
#if (DEBUG_LVL >= 1)
void 	dumpReading(const char*, t_reading *);
#endif

// define a few simple functions that are likely to be inlined
/* 
	float2int()
	This takes a float, rounds to nearest integer and returns an int.
	Note an int is OK for temperatures but too small for long running counters.
*/
int float2int(float x) {
	return x >= 0 ? (int)(x + 0.5) : (int)(x - 0.5);
}
/*
	clipIndex()
	This mods the index to make the array circular
*/
unsigned int clipIndex(unsigned long index) {
	return index%READ_ARRAY_SIZE;
}

bool isStoveOn() {
	return stoveOn;
}

void setStoveOn() {
	stoveOn = true;
	digitalWrite(STOVE_RELAY_PIN, HIGH);
	digitalWrite(STOVE_LED_PIN, HIGH);
#if (DEBUG_LVL >= 2)
	Serial.print("==> setStoveOn() \n");
#endif	
}

void setStoveOff() {
	stoveOn = false;
	digitalWrite(STOVE_RELAY_PIN, LOW);
	digitalWrite(STOVE_LED_PIN, LOW);
#if (DEBUG_LVL >= 2)
	Serial.print("==> setStoveOff() \n");
#endif	
}

/*
	quiesceDisplay()
	This gives the appearance of turning the display off. The temperature setting
	potentiometer has a switch which when turned off (passing through the STANDBY
	low temperature setting) will cause the sceen to go blank. For an OLED this
	looks like the power to the module is off. 
*/
void quiesceDisplay() {	
	#if (DEBUG_LVL >= 1)
	Serial.print("==> Quiesce Display\n");
	#endif	
	lcd.clear();
	digitalWrite(MAIN_LED_PIN, LOW);
	displayActive = false;
}

/*
	resetDisplay()
	This gives the appearance of turning the screen on. It drives the OLED module
	intializer. This re-establishes the sync protocol between the OLED module and
	the MCU. By linking this action to the turning on of the temperature setting 
	potentiometer, the user can address screen corruption caused by transients in 
	the circuits by turning the switch off then on without loosing the elapsed time.
*/
void resetDisplay() {
	#if (DEBUG_LVL >= 1)
	Serial.print("==> Reset Display\n");
	#endif	
	digitalWrite(MAIN_LED_PIN, HIGH);
	lcd.begin(16, 2);
	delay(100);
	lcd.setCursor(0, 0);
	lcd.print(PROGRAM_NAME);
	delay(1000);
	lcd.setCursor(0, 1);
	if (time_last_measured == 0) {
		lcd.print("Set temp");
	} else {
		lcd.print("Display reset");
	}
	delay(5000);
	lcd.clear();
	displayActive = true;
}


/********************************
 * setup() 
 ********************************/
void setup()
{
#ifdef EXTPOWER
	analogReference(EXTERNAL);  // tell the board that an external power source is applied to the AREF pin
	for (int i=0; i>3; i++) {
	  analogRead(STOVE_THERM_PIN);    // discard a few reads as per spec.
	  analogRead(MEAT_THERM_PIN);     // discard a few reads as per spec.
	}
#endif

	pinMode(STOVE_RELAY_PIN, OUTPUT);
	pinMode(STOVE_LED_PIN, OUTPUT);
	pinMode(MAIN_LED_PIN, INPUT_PULLUP);
	
#if (DEBUG_LVL >= 1)
	Serial.begin(9600);
	while (!Serial) {}  // wait for serial port to initialize
	Serial.print("Starting ");
	Serial.print(PROGRAM_NAME);
	Serial.print("  ");
	Serial.print(__DATE__);
	Serial.print("  ");
	Serial.print(__TIME__);
	Serial.print("\n");

	Serial.print(" STOVE_THERM_PIN: ");
	Serial.print(STOVE_THERM_PIN);
	Serial.print(" MEAT_THERM_PIN: ");
	Serial.print(MEAT_THERM_PIN);
	Serial.print(" SET_POT_PIN: ");
	Serial.print(SET_POT_PIN);
	Serial.print("\n");

	dumpReading("STOVE INITIAL ", &stove);
	dumpReading("MEAT  INITIAL ", &meat);
	dumpReading("SETPOT INITIAL ", &setpot);

#endif

	resetDisplay();
	
}

/**********************************************************
 * loop()
 * Several tasks are performed at distinct time intervals.
 * The loop simply exits when there is no work to do.
 **********************************************************/
void loop() {

	unsigned int adc_avg_reading;
	unsigned long now = millis(); 

/////////////////////////
// make the measurements
/////////////////////////

	if (now - time_last_measured < millis_between_adc_reads) {
		return;
	}
	time_last_measured = now;

//#if (DEBUG_LVL >= 3)
//  dumpReading("LAST STOVE READ ", &stove);
//#endif

// update the read count and the circular index for the pin data arrays
	circular_inx = clipIndex(++read_cnt);
 
// prepare the time string for display 
	unsigned long seconds = now/1000;
	unsigned long hours = seconds/3600;
	unsigned long minutes = (seconds%3600)/60;
	snprintf(hhmm_str,sizeof(hhmm_str),"%02lu:%02lu",hours,minutes);

//
// read the state of the main switch to control display
// The display is not really turned off, just cleared (it is an OLED so it looks off)
// Resetting the display should address an intermittant sync glitch
//
	int mainsw = digitalRead(MAIN_SWITCH_PIN);
	#if (DEBUG_LVL >= 1)
	Serial.write(strbuf, snprintf(strbuf, sizeof(strbuf),
		"==> mainsw: %d\n", mainsw ));
	#endif	
	if (displayActive) { 
		if (mainsw == LOW) {		// turn it off
			quiesceDisplay();
		}
	} else { 
		if (mainsw == HIGH) {		// turn it on
			resetDisplay();
		}
	}

//	
// read the Set point temperature from a potentiometer controlled by the user
//
	getAvgReading(&setpot);
	setpot.avg_temperature = (increment * setpot.avg_reading) + temp_start;
	// To make the scale easier to dial in, round set temp in 5 degree increments
  	int setpoint_temp_int = (float2int(setpot.avg_temperature) / 5) * 5;  
	snprintf(setpot.display_str, sizeof(setpot.display_str), "S:%03d ", setpoint_temp_int); 
 
//
// read Oven thermistor
//
	adc_avg_reading = getAvgReading(&stove);
	stove.avg_temperature = getTemp(&stove_therm, adc_avg_reading);
	snprintf(stove.display_str,sizeof(stove.display_str),"O:%03d ", float2int(stove.avg_temperature));
#if (DEBUG_LVL >= 3)
  dumpReading("NEW STOVE READ ", &stove);
#endif
//
// read Meat thermistor
//
	adc_avg_reading = getAvgReading(&meat);
	meat.avg_temperature = getTemp(&meat_therm, adc_avg_reading);  
	snprintf(meat.display_str,sizeof(meat.display_str),"M:%03d ", float2int(meat.avg_temperature));

#ifdef BOARD_THERM_PIN
//  
// TODO 3rd thermistor for board temp to be used for warning when board temp > 50 C.
// maybe add buzzer and flash LEDs after stopping heating element
//
	adc_avg_reading = getAvgReading(&board);
	board.avg_temperature = getTemp(&board_therm, adc_avg_reading);  
	snprintf(board.display_str,sizeof(board.display_str),"B:%03d ", float2int(board.avg_temperature));
#endif

#if (DEBUG_LVL >= 1)
	if (minutes > last_minute) { //each minute
		Serial.print("===> Elapsed Time seconds:");
		Serial.print(seconds);
		Serial.print(" minutes:");
		Serial.print(minutes);
		Serial.print(" hours:");
		Serial.print(hours);
		Serial.print(" millis:");
		Serial.print(now);
		Serial.print(" read_cnt:");
		Serial.print(read_cnt);
		Serial.print(" \n");
		last_minute = minutes;

		dumpReading("STOVE READ ", &stove);
//		if (minutes > 50) { // look for anomaly at minute 54
//			dumpReading("STOVE READ ", &stove);
//		}
	}
#endif  

//////////////////////////////
// Display the current status
//////////////////////////////

	if (now - time_last_displayed < millis_between_display_updates) {
		return;
	}
	time_last_displayed = now;
	displayStatus();
	
///////////////////////////////
// Modulate the heating element
// The hardware controlling the heater is a relay, not an SSD, and will experience
// wear if cycled too frequently.
///////////////////////////////

	if (now - time_last_heater_update < millis_between_heater_updates) {
		return;
	}
	#if (DEBUG_LVL >= 2)
	Serial.print("==>Modulating Heater \n");
	#endif  

	time_last_heater_update = now;

/* Some thoughts:
 Set a temperature band for heating and a band for cooling then set current temp against those.
 If heater is already on then temp is likely rising.
 If heater is not on then temp is likely falling (or will be soon).
 To do better would require tracking several minutes of prior readings. 
 For that, another array, a sampling interval, and a slope calculation or PID ...
 When rising, stop the heater before the set temp.
 During initial heating , hysteresis has a greater effect on overshooting the setpoint. 
 When falling start the heater at the set temp.
*/
	if (setpot.avg_temperature < standby_threshold) {
		#if (DEBUG_LVL >= 2)
		Serial.print("==>STANDBY \n");
		#endif
		
	  	setStoveOff();
	  	return;
	}
#if (DEBUG_LVL >= 4)
	Serial.print(" setpot.avg_temperature: ");
	Serial.print(setpot.avg_temperature);
	Serial.print(" stove.avg_temperature: ");
	Serial.print(stove.avg_temperature);
	Serial.print(" Heater State: ");
	Serial.print(stoveOn);
	Serial.print("\n");
#endif  
 
	if (isStoveOn()) {
		if (stove.avg_temperature < (setpot.avg_temperature - chkpt_band)) {
			#if (DEBUG_LVL >= 2)
			Serial.print("==>Leave heater ON \n");
			#endif
		} else {
			#if (DEBUG_LVL >= 2)
			Serial.print("==>Set heater OFF \n");
			#endif
			setStoveOff();
		}
	} else {
		if (stove.avg_temperature < (setpot.avg_temperature - chkpt_band)) {
			#if (DEBUG_LVL >= 2)
			Serial.print("==>Set heater ON \n");
			#endif
			setStoveOn();
		} else {
			#if (DEBUG_LVL >= 2)
			Serial.print("==>Leave heater OFF \n");
			#endif
		}
	}

//////////////////////////////////////////////////
// Checkpoint to adjust setpoint temperature band
//////////////////////////////////////////////////
	#if (DEBUG_LVL >= 2)
		Serial.print("==>Checkpoint.  last:");
		Serial.print(time_last_checkpoint);
		Serial.print(" interval: ");
		Serial.print(millis_between_checkpoints);
		Serial.print(" now: ");
		Serial.print(now);
		Serial.print(" \n");
	#endif  
	if (now - time_last_checkpoint < millis_between_checkpoints) {
		return;
	}
	if (time_last_checkpoint == 0) { // first time only
		last_chkpt_temp = stove.avg_temperature;	
	}

	time_last_checkpoint = now;

/*
 The band is a temperature adjustment above and below the setpoint temperature.
 Look at it as when to cut a boat's motor before you hit the dock.
 Calculate the slope of df/dt, where df is delta Fahrenheit and dt is delta time.
 The slope is a rough guess of where the temperature will be in the next unit interval.
 Since dt is constant (say unit 1 minute) it can be ignored, using just delta temp.

 Multiply slope by a fudge factor to get a temperature band to add or subtract from
 the setpoint given by the user. The band value is signed, positive when rising and 
 negative when falling. So subtract this from the measured temperature to get the
 effective setpoint as a line below (or above) the setpoint.
 This will lower the threshold whan rising and raise it when falling.
 The fudge factor is an approximation of the hysteresis of the smoker box. 
 My smoker heats about 3 times faster than it cools. Cooling is passive.
 The initial heating slope is steep and the box temp continues to rise 20 or more 
 degrees after the heater is shut off. So I use a fudge value between 2 and 3 to 
 increase the band when rising. When the temp is falling the band above the setpoint
 is much smaller. 
*/

	chkpt_band = stove.avg_temperature - last_chkpt_temp; // delta Temp per unit Time
	if (chkpt_band > 0) { 
		chkpt_band *= chkpt_rising_fudge; // estimate temp below setpoint to stop heater
	} else {
		chkpt_band *= chkpt_falling_fudge; // estimate temp above setpoint to restart heater
	}
	
	#if (DEBUG_LVL >= 1)
//	Serial.write(strbuf, snprintf(strbuf, sizeof(strbuf),
//		"==>Checkpoint band: %f cur(F): %f last(F): %f fudge: %f \n", 
//		(double) chkpt_band, (double) stove.avg_temperature, (double) last_chkpt_temp, (double) fudge ));
	Serial.print("==>Checkpoint band=");
	Serial.print(chkpt_band);
	Serial.print(" currentT=");
	Serial.print(stove.avg_temperature);
	Serial.print(" lastT=");
	Serial.print(last_chkpt_temp);
	Serial.print(" risingFF=");
	Serial.print(chkpt_rising_fudge);
	Serial.print(" fallingFF=");
	Serial.print(chkpt_falling_fudge);
	Serial.print(" \n");
	#endif  
	
	last_chkpt_temp = stove.avg_temperature;

} // end loop()

/*
	getAvgReading()
	Given a pointer to a sensor struct, this reads the analog value at the sensor's pin 
	and updates the corresponding array. Then it calculates the current average across 
	that array, saving the unrounded float in the struct and returns the 
	rounded average value as an integer.
*/

unsigned int getAvgReading(t_reading *sensor) {
	float sum = 0; 
	sensor->adcval = analogRead(sensor->pin);
	sensor->read_val[circular_inx] = sensor->adcval;
	for (int i=0; i<READ_ARRAY_SIZE; i++) {
		sum += (float) sensor->read_val[i];
	}
	sensor->avg_reading = (sum/READ_ARRAY_SIZE) + 0.5;
	return sensor->avg_reading;
}

/*
 * getTemp() 
 * Apply Steinhart-Hart to an ADC reading and convert to Fahrenheit.
 * Note thermistor coefficients were empirically determined with help from 
 * the SRS Thermistor Calculator.
 * https://www.thinksrs.com/downloads/programs/Therm%20Calc/NTCCalibrator/NTCcalculator.htm
*/

float getTemp(const t_thermistor *thm, unsigned int reading) {

	double tempK = log(thm->res * ((1024.0 / reading - 1)));
	tempK = 1 / (thm->cA + (thm->cB + (thm->cC * tempK * tempK )) * tempK );
	float tempC = tempK - 273.15;            // Convert Kelvin to Celsius
	float tempF = (tempC * 9.0)/ 5.0 + 32.0; // Convert Celsius to Fahrenheit
	return tempF;
}

/*
	displayStatus()
	This displays formatted status to the LCD and also to the serial port.
	Format for the 16x2 display:
		hh:mm  S:999  *
		O:999  M:999  =
	where hh:mm is elapsed time since reset, S: is set temperature, 
	O: is oven temperature, M: is meat temperature, 
	'*' or '<' indicates the heater is on or off
	'=' or '.' indicates standby state is on or off
*/
void displayStatus() {

	if (! displayActive) {
		return;
	}
	lcd.clear();
	
	lcd.setCursor(0, 0);
	lcd.print(hhmm_str);
#ifdef DISPLAY_ADC_READING
	lcd.setCursor(8, 0);
	lcd.print(setpot.avg_reading);
	lcd.setCursor(0, 1);
	lcd.print(stove.avg_reading);
	lcd.setCursor(8, 1);
	lcd.print(meat.avg_reading);
#else	
	lcd.setCursor(8, 0);
	lcd.print(setpot.display_str);
	lcd.setCursor(0, 1);
	lcd.print(stove.display_str);
	lcd.setCursor(8, 1);
	lcd.print(meat.display_str);
#endif

	char heat_indicator;
	if (isStoveOn()) {
	  heat_indicator = '*';
	  lcd.setCursor(15, 0);
	  lcd.print(heat_indicator);
	} else {
	  heat_indicator = '<';
	  lcd.setCursor(15, 0);
	  lcd.print(heat_indicator);
	}
	char standby_indicator;
	if (setpot.avg_temperature < standby_threshold) {
	  standby_indicator = '=';
	  lcd.setCursor(15, 1);
	  lcd.print(standby_indicator);
	} else {
	  standby_indicator = '.';
	  lcd.setCursor(15, 1);
	  lcd.print(standby_indicator);
	}

#if (DEBUG_LVL >= 1)
	Serial.write(strbuf, snprintf(strbuf, sizeof(strbuf),
	  "HH:MM %s  O:%d  %s  M:%d  %s Set:%d  %s HE:%c SB:%c\n", 
	  hhmm_str, stove.avg_reading, stove.display_str, 
	  meat.avg_reading, meat.display_str, setpot.avg_reading, 
	  setpot.display_str, heat_indicator, standby_indicator) );
#endif

}


#if (DEBUG_LVL >= 1)
void dumpReading(const char* str, t_reading *sensor ) {
/*
	Serial.write(strbuf, snprintf(strbuf, sizeof(strbuf),
	  "==> %s: sensor ptr: %p pin: %d adc: %d array: %d avgT: %d ind: %d  \n", 
	  str, (void *) sensor), sensor->pin, sensor->adcval, 
	  sensor->read_val[circular_inx], sensor->avg_reading,
	  sensor->avg_temperature, circular_inx);
*/
  Serial.print("==>");
  Serial.print(str);
  Serial.print(" sensor ptr: ");
  Serial.print((uintptr_t) sensor, HEX);
  Serial.print(" .pin:");
  Serial.print(sensor->pin);
  Serial.print(" .adcval:");
  Serial.print(sensor->adcval);
  Serial.print(" .read_val[circular_inx]:");
  Serial.print(sensor->read_val[circular_inx]);
  Serial.print(" .avg_reading:");
  Serial.print(sensor->avg_reading);
  Serial.print(" .avg_temperature:");
  Serial.print(sensor->avg_temperature);
  Serial.print(" circular_inx:");
  Serial.print(circular_inx);
  Serial.print(" \n");

}
#endif
