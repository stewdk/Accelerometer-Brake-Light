/*
 * Blinky.c
 *
 * Created: 12/1/2011 1:52:46 AM
 *  Author: Stew
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "USI_TWI_Master.h"

//ADXL345 commands
//I2C address = 0x53 (0xA6 for write, 0xA7 for read)
#define WRITE 0xA6
#define READ 0xA7
#define POWER_CTL_MEASURE_BIT 3
#define DATA_FORMAT_FULL_RES 3
#define DATA_FORMAT_RANGE1 1
#define DATA_FORMAT_RANGE0 0
//ADXL345 registers
#define BW_RATE 0x2C
#define POWER_CTL 0x2D
#define DATA_FORMAT 0x31
//Output data is in two's complement
#define DATAX0 0x32 //Least significant byte
#define DATAX1 0x33 //Most significant byte
#define DATAY0 0x34
#define DATAY1 0x35
#define DATAZ0 0x36
#define DATAZ1 0x37

//Accelerometer pins:
//ADXL345 CS and Vs are wired together and connected to ATtiny84 PA3
//ADXL345 SDA/SDI/SDIO/13 is connected to ATtiny84 PA6/SDA/MOSI/DI with 4.7k pull-up to 3.3VCC
//ADXL345 SDO/ALT ADDRESS is pulled low via 4.7k
//ADXL345 SCL/SCLK is connected to ATtiny84 PA4/USCK/SCL with 4.7k pull-up to 3.3VCC

static void adxl345Init(uint8_t *data) {
	//Tell the accelerometer we want full resolution and +/- 16 g range
	//With these settings, we get a 13-bit two's complement value
	//So in theory:
	//    -4096 is -16g
	//    -256 is -1g
	//    0 is 0g
	//    +256 is +1g
	//    +4095 is +16g
	//In practice, it's pretty much spot on. Awesome.
	data[0] = WRITE;
	data[1] = DATA_FORMAT;
	data[2] = _BV(DATA_FORMAT_FULL_RES) | _BV(DATA_FORMAT_RANGE1) | _BV(DATA_FORMAT_RANGE0);
	USI_TWI_Start_Read_Write(data, 3);

	//Tell the accelerometer how fast to sample data
	data[0] = WRITE;
	data[1] = BW_RATE;
	data[2] = 0x0D; //800 Hz sample rate
	USI_TWI_Start_Read_Write(data, 3);

	//Tell the accelerometer to start measuring
	//Set the POWER_CTL_MEASURE_BIT in the POWER_CTL register
	data[0] = WRITE;
	data[1] = POWER_CTL;
	data[2] = _BV(POWER_CTL_MEASURE_BIT);
	USI_TWI_Start_Read_Write(data, 3);
}

static void getAccelData(uint8_t *data) {
	//Read a value
	data[0] = WRITE;
	data[1] = DATAX0;
	USI_TWI_Start_Read_Write(data, 2);
	data[0] = READ;
	//multiple byte read gets all six data registers
	USI_TWI_Start_Read_Write(data, 7);
}

//integer square root routine (http://www.finesse.demon.co.uk/steven/sqrt.html)
#define iter1(N) \
    try = root + (1 << (N)); \
    if (n >= try << (N))   \
    {   n -= try << (N);   \
        root |= 2 << (N); \
    }

static uint32_t isqrt(uint32_t n)
{
    uint32_t root = 0, try;
    iter1 (15);    iter1 (14);    iter1 (13);    iter1 (12);
    iter1 (11);    iter1 (10);    iter1 ( 9);    iter1 ( 8);
    iter1 ( 7);    iter1 ( 6);    iter1 ( 5);    iter1 ( 4);
    iter1 ( 3);    iter1 ( 2);    iter1 ( 1);    iter1 ( 0);
    return root >> 1;
}

static void setupTimer1(void) {
	TCCR1A = 0;
	// CTC, prescaler = 1024
	TCCR1B = _BV(WGM12) | _BV(CS12) | _BV(CS10);
	// Each tick is approximately 1msec with a main clock of 1mHz
	OCR1A = 100;
	// Interrupt enable
	TIMSK1 = _BV(OCIE1A);
}

static volatile uint8_t gBlinking = 0;
//static volatile uint8_t gBlinkState = 0;
ISR(TIM1_COMPA_vect) {
	if (gBlinking) {
		//Toggle LEDs
		PINA = _BV(PA2) | _BV(PA1) | _BV(PA0);
		gBlinking--;
		/*
		switch (gBlinkState)
		{
		case 0:
			PORTA |= _BV(PA2);
			PORTA &= ~_BV(PA1) | ~_BV(PA0);
			break;
		case 1:
			PORTA |= _BV(PA1);
			PORTA &= ~_BV(PA2) | ~_BV(PA0);
			break;
		case 2:
			PORTA |= _BV(PA0);
			PORTA &= ~_BV(PA2) | ~_BV(PA1);
			break;
		}
		*/
		//gBlinkState = (gBlinkState + 1) % 3;
	}
}

static void setLEDs(int16_t accelValue)
{
	accelValue = -accelValue;
	if (accelValue < 0) {
		PORTA |= _BV(PA2) | _BV(PA1) | _BV(PA0);
		return;
	}
	//LEDs are active low
	//PA2 is the single middle segment
	//PA1 is the two segments between outside and middle
	//PA0 is the outside two segments

	uint16_t hysteresis = 32UL;
	uint16_t seg2threshold = 0UL;//304UL; //256 + 32 + 16
	uint16_t seg1threshold = 60;//448UL; //384 + 32 + 32
	uint16_t seg0threshold = 100;//608UL; //512 + 32 + 64
	uint16_t blinkThreshold = 150;//1024UL;
	uint8_t blinkCount = 12;

	if (accelValue < seg2threshold) {
		accelValue = seg2threshold + 1;
	}

	if (!(PORTA & _BV(PA2))) { //If the segment was on during the previous iteration
		seg2threshold -= hysteresis; //Lower the threshold required to turn the segment off again
	}
	if (!(PORTA & _BV(PA1))) {
		seg1threshold -= hysteresis;
	}
	if (!(PORTA & _BV(PA0))) {
		seg0threshold -= hysteresis;
	}
	if (gBlinking) {
		if (accelValue >= (blinkThreshold - hysteresis)) {
			cli();
			gBlinking = blinkCount; //Set how many times to flash on and off
			sei();
		}
	} else {
		if (accelValue < seg2threshold) {
			//All LEDs off
			PORTA |= _BV(PA2) | _BV(PA1) | _BV(PA0);
		} else if (accelValue < seg1threshold) {
			PORTA &= ~_BV(PA2);
			PORTA |= _BV(PA1) | _BV(PA0);
		} else if (accelValue < seg0threshold) {
			PORTA &= ~_BV(PA2) & ~_BV(PA1);
			PORTA |= _BV(PA0);
		} else if (accelValue < blinkThreshold) {
			//All LEDs on
			PORTA &= ~_BV(PA2) & ~_BV(PA1) & ~_BV(PA0);
		} else {
			PORTA &= ~_BV(PA2) & ~_BV(PA1) & ~_BV(PA0);
			//Blink!
			cli();
			TCNT1 = 0;
			gBlinking = blinkCount; //Set how many times to flash on and off
			sei();
		}
	}
}

int main(void)
{
	uint8_t data[7];
	int16_t x, y, z;
	int16_t rawMagnitude;
	int16_t previousFilteredMagnitude = 0;
	int16_t currentFilteredMagnitude = 0;

	PORTA |= _BV(PA3) | _BV(PA2) | _BV(PA1) | _BV(PA0); //Turn on the accelerometer, make sure LEDs are off (active low)
	DDRA |= _BV(PA3) | _BV(PA2) | _BV(PA1) | _BV(PA0); //LEDs and accelerometer power as output

	USI_TWI_Master_Initialise();

	adxl345Init(data);

	setupTimer1();
	sei();

	//All LEDs on!
	//PORTA &= ~_BV(PA2) & ~_BV(PA1) & ~_BV(PA0);

    while(1)
    {
		getAccelData(data);
		x = (data[2] << 8) | data[1];
		y = (data[4] << 8) | data[3];
		z = (data[6] << 8) | data[5];

		//Absolute value
		//x = x < 0 ? -x : x;
		//y = y < 0 ? -y : y;
		//z = z < 0 ? -z : z;

		rawMagnitude = z;
		//Pythagorean theorem
		//rawMagnitude = isqrt(
		//	(uint32_t)x * (uint32_t)x +
		//	(uint32_t)y * (uint32_t)y +
		//	(uint32_t)z * (uint32_t)z);

		//First order low-pass filter, alpha = 1/20
		currentFilteredMagnitude = previousFilteredMagnitude + (rawMagnitude - previousFilteredMagnitude) / 20;

		setLEDs(currentFilteredMagnitude);

		previousFilteredMagnitude = currentFilteredMagnitude;
    }
}