#include "Arduino.h"
#include <avr/power.h>

#define LCD
#define UART
#define __sensorLEDOut 11
#define __sensorBASEIn A1


#ifdef LCD
	#include <LiquidCrystal.h>
	#define __btnRIGHT   0
	#define __btnUP      1
	#define __btnDOWN    2
	#define __btnLEFT    3
	#define __btnSELECT  4
	#define __btnNONE    5
	#define __lcdBckPIN  10
	#define __keyPIN A0
	
	static LiquidCrystal lcd(8, 9, 4, 5, 6, 7);
#endif

#ifdef KEYS
	uint8_t read_LCD_buttons() {
	static uint16_t adcKey0;      // read the value from the sensor
	tmpResult = 0;
	adcKey0 = adcKeyInd = analogRead(__keyPIN);
	if(adcKey0 < 1000) {
		for(iterator = (1<<count); iterator > 0  && ((tmpResult = analogRead(__keyPIN)) < 1000 ); --iterator);
		if (adcKey0 < 50)   return __btnRIGHT;
		if (adcKey0 < 250)  return __btnUP;
		if (adcKey0 < 450)  return __btnDOWN;
		if (adcKey0 < 650)  return __btnLEFT;
		if (adcKey0 < 850)  return __btnSELECT;
	}
	return __btnNONE;
}
#else 
	uint8_t read_LCD_buttons() { return 0; }
#endif

#ifdef MEMORY
	#include "EEPROM.h"
	#define __EEPROM_ADDR 0
	union sEEPROM {
		struct {
			//uint8_t alredyProgrammed;
			//uint8_t screenBr;
			//uint8_t massBB;
		};
		uint8_t RAW[3];
	};

	static union sEEPROM data;
#endif

#ifdef UART
void write_timestamped(double measurement) {
	Serial.print(millis());
	Serial.print(": Voltage [V]: ");
	Serial.println(measurement,4);
}
#else
write_timestamped(double measurement) {}
#endif



static unsigned long time; // system time
double measurement; // variable to store ADC measurement
static double Vcc; // calibrated Vcc voltage
volatile uint16_t iterator; // global iterator
static uint32_t tmpResult; // temp variable for debouncing

void calibrateVcc(uint8_t count) {
	tmpResult = 0;
	// Read 1.1V reference against AVcc
	ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
	delay(100); // Wait for Vref to settle
	power_timer0_disable();
	for(iterator = (1 << count); iterator > 0; --iterator) {
		ADCSRA |= _BV(ADSC); // Convert
		while (bit_is_set(ADCSRA,ADSC));
		tmpResult += (ADCH<<8) | ADCL;
	}
	power_timer0_enable();
	Vcc = 1100.0 / (tmpResult >> count); // Back-calculate AVcc in mV
}

void performMeasurement(uint8_t count) {
	tmpResult = 0;
	for(iterator = (1 << count); iterator > 0 ; --iterator) {
	/* Generates measurement pulse */
		digitalWrite(__sensorLEDOut,LOW);
		delayMicroseconds(280);
		power_timer0_disable();
		tmpResult += analogRead(__sensorBASEIn); // with ADC prescaler set to 32, conversion lasts for ~15us
		digitalWrite(__sensorLEDOut, HIGH);
		power_timer0_enable();
		delay(10);
	}
	measurement = ((5 * tmpResult) >> count) / 1024.0;
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
	power_spi_disable();
	power_twi_disable();
	power_timer1_disable();
	power_timer2_disable();
	
	calibrateVcc(8);

	pinMode(__sensorLEDOut, OUTPUT); // SENSOR LED BASE
	pinMode(__sensorBASEIn, INPUT);	// SENSOR ANALOG OUTPUT
	digitalWrite(__sensorLEDOut, HIGH);
	
	
	#ifdef LCD	
		pinMode(__lcdBckPIN, OUTPUT); // LCD BACKLIGHT
		digitalWrite(__lcdBckPIN, HIGH); // LCD PWM backlight
		lcd.begin(16, 2);              // start the library
	#endif
	//eeprom_read();
	#ifdef UART
		Serial.begin(9600);				// initialize UART Connection
		Serial.println("Smoke Sensor v.0.1");
	#endif

}





void loop() {
	//measurement = normalize_to_V(calc_mean(100));
	performMeasurement(6);
	write_timestamped(measurement);
	#ifdef LCD
	lcd.setCursor(0,0);
	lcd.print("V          ");
	lcd.setCursor(4,0);
	lcd.print(measurement);
	lcd.setCursor(0,1);
	lcd.print("BUTTON: ");
	lcd.print(read_LCD_buttons());
#endif			
}

