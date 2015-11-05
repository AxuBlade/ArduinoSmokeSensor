#include <LiquidCrystal.h>
#include "Arduino.h"
#include "EEPROM.h"
//#include "prescaler.h"


#define __btnRIGHT   0
#define __btnUP      1
#define __btnDOWN    2
#define __btnLEFT    3
#define __btnSELECT  4
#define __btnNONE    5
#define __lcdBckPIN  10

#define __keyPIN 0

#define __sensorLEDOut 11
#define __sensorBASEIn A1


#define __EEPROM_ADDR A0


#define PS_16  (1 << ADPS2)
#define PS_32  ((1 << ADPS2) | (1 << ADPS0))
#define PS_64  ((1 << ADPS2) | (1 << ADPS1))
#define PS_128 ((1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0))

static LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

unsigned long time;
double measurement;

union sEEPROM {
	struct {
		//uint8_t alredyProgrammed;
		//uint8_t screenBr;
		//uint8_t massBB;
	};
	uint8_t RAW[3];
};



static union sEEPROM data;




uint16_t performMeasurement() {
	/* Generates measurement pulse */
		uint16_t result = 0;
		digitalWrite(__sensorLEDOut,HIGH);
		delayMicroseconds(280);
		result = analogRead(__sensorBASEIn); // with ADC prescaler set to 32, conversion lasts for ~15us
		digitalWrite(__sensorLEDOut, LOW);
		return result;
}


/*
void eeprom_read() {
	for(uint8_t i = 0; i < sizeof(data.RAW)/sizeof(uint8_t); ++i) {
		data.RAW[i] = EEPROM.read(__EEPROM_ADDR + i);
	}
	if(data.alredyProgrammed) {
	//	screenB->val = data.screenBr;
	//	massBB->val = data.massBB;
	}
}

void eeprom_write() {
	data.alredyProgrammed = 1;
	//data.screenBr = screenB->val;
	//data.massBB = massBB->val;
	for(uint8_t i = 0; i < sizeof(data.RAW)/sizeof(uint8_t); ++i) {
		EEPROM.write(__EEPROM_ADDR + i, data.RAW[i]);
	}
}
*/

void setup() {
	// setClockPrescaler(CLOCK_PRESCALER_1); // set clock speed to 1MHz
	pinMode(__lcdBckPIN, OUTPUT); // LCD BACKLIGHT
	pinMode(__sensorLEDOut, OUTPUT); // SENSOR LED BASE
	pinMode(__sensorBASEIn, INPUT);	// SENSOR ANALOG OUTPUT
	
	analogWrite(__lcdBckPIN, 200); // LCD PWM backlight
	lcd.begin(16, 2);              // start the library
	Serial.begin(115200);				// initialize UART Connection
	Serial.println("Arduino Based Smoke Sensor v. 0.1");
	//eeprom_read();

	ADCSRA &= ~(PS_128); // Clear ADC Prescaler
	ADCSRA |= PS_128; // set adc prescaler to 128 (250kHz)

}



uint8_t read_LCD_buttons() {
	static uint16_t adcKey0, adcKeyInd;      // read the value from the sensor

	adcKey0 = adcKeyInd = analogRead(__keyPIN);
	if(adcKey0 < 1000) {
		for(int i = 0; (i < 10)  && ((adcKeyInd = analogRead(__keyPIN)) < 1000 ); ++i);
		if (adcKey0 < 50)   return __btnRIGHT;
		if (adcKey0 < 250)  return __btnUP;
		if (adcKey0 < 450)  return __btnDOWN;
		if (adcKey0 < 650)  return __btnLEFT;
		if (adcKey0 < 850)  return __btnSELECT;
	}
	return __btnNONE;
}


size_t calc_mean(uint8_t it) {
size_t result = 0;
for (uint8_t i = 0; i < it; ++i) {
	result += performMeasurement();
	delay(10);
}
result /= it;
return result;
}

double normalize_to_V(size_t input) {
return (input*5.0)/1024.0;	
}

void write_timestamped(double measurement) {
	Serial.print("TIME: ");
	Serial.print(millis());
	Serial.print(" VOLTAGE: ");
	Serial.print(measurement);
	Serial.println("");
	Serial.flush();
}

void loop() {
	measurement = normalize_to_V(calc_mean(40));
	write_timestamped(measurement);
	lcd.setCursor(0,0);
	lcd.print("V          ");
	lcd.setCursor(4,0);
	lcd.print(measurement);
	lcd.setCursor(0,1);
	lcd.print("BUTTON: ");
	lcd.print(read_LCD_buttons());
			
}

