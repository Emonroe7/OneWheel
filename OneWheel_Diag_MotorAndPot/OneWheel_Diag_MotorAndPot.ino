/*
=  AUTHOR: Eric Monroe
=  
=  Pinout is as follows:
=  Analog
=  0    Gyro
=  1    Accelerometer
=  2    OverallGain Pot
=  3    ThrottleGain Pot    
=
=  Digital
=  5    
=  6    Forward Tilt Switch
=  7    Backward Tilt Switch
= --------------
=  8    OSMC Disable Pin (Send High to Disable OSMC)(OSMC Pin 4 on connector)
=  9    ALI (Reverse PWM)(OSMC Pin 6)
=  10   BLI (Forward PWM)(OSMC Pin 8)
=  11   AHI (OSMC Pin 5) (Can also tie to +5volts if you experience problems with OSMC)
=  12   BHI (OSMC Pin 5) (Can also tie to +5volts if you experience problems with OSMC) 
=  13   Timing Pulse (Turned On and Off once per cycle)
*/
#ifndef __AVR_ATmega328P__
#define __AVR_ATmega328P__
#endif

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif 


//===============================================================
//========================== Libraries ==========================
//===============================================================
#include <SoftwareSerial.h>
#include <avr/pgrmspace.h>
#include <avr/interupt.h>
#include <avr/io.h>
#include <math.h>


#define denom (1023/3)
#define degToRad 0.0174532925
#define radToDeg 57.2957795131
typedef unsigned char u8;


void ports_init(void);
void adc_init(void);
void timer_init(void);
uint16_t adc_read(uint8_t channel);
void sample_inputs(void);
void set_motor(void);

//===============================================================
//===================== Variable Declaration ====================
//===============================================================
boolean tipStart = false;
int accelRead, gyroRead, gyrosum, accsum;
float gainCtrl, throttleGain, overallGain;
float xAcc, xAccRad, xAccDeg;
float gyroRateDeg, gyroRateRad, gyroRateDt;
float angle = 0; 
float angleRad;
float currSpeed = 0;
float balTorque, level;
float softStart = 0.1;
boolean tiltFW, tiltRV;
const int balPointOffset = 20;
const int balPointLevel = 512;
int balPoint;
int i = 0;
uint16_t adc2;

// Sensor Characteristics
// These characteristics are for the Sparkfun ADXL203/ADXRS610 combo board.
// It's old, but extremely easy to interface with. Accelerometer is +/-1.5g
const float gyroStepsPerDeg = 3.072; //steps per degree on gyro, etc. 15mV/degree/sec over 204.8 steps
const int maxGyro = 300;//Gyro is rated for 300 degrees per
const int gyroOffset = 512;//Value to offset for any constant drift to get gyro to read 0 (512 means no drift).
const int accelOffset = 512;//Value to get the accelerometer to read 0 on a level surface.
const float accelDivisor = 204.8;//Value to divide by to get a value between 0 and 1.
const int maxAccelAcceptable = 200;//Max Acceleration reading offset

//===============================================================
//===================== Function Declaration ====================
//===============================================================
void ports_init(void);
void adc_init(void);
void timer_init(void);
uint16_t adc_read(uint8_t channel);
void sample_inputs(void);
void set_motor(void);


//===============================================================
//===================== Program Execution =======================
//===============================================================
void setup() 
{
  ports_init();
  adc_init();
  timer_init();
  sei();
}

void loop() 
{
  adc2 = adc_read(2);
  level = adc2;
  set_motor();
}


//===============================================================
//===================== Function Execution ======================
//===============================================================
void ports_init(void)
{/*
 4 Analog Inputs: Gyro and Accelerometer sensors, and throttle and overall_gain pots
 2 Digital Inputs: TiltForward and TiltBack switches that adjust the location of balance point.
 6 Digital Outputs: OSMCDisable, BHI and AHI, Buzzer or warningLight, and 2 PWM Outputs
 PWM Outputs: ALI and BLI, on pins 9 and 10, PB1 and PB2
 DDRx, where x is the port letter, is used to set whether a pin is an input(0) or output(1)
 Using binary format to make this easier to read
 */
  DDRB  = B00111111;  //PortB (Digital IO 8-13 are all  configured as outputs.
  PORTB = B00000000;  //All Port B Pins are set LOW at the moment
//  DDRC  = B00001111;  //First 4 Analog pins set as input. 
//  PORTC = B00000000;  //PortC pull ups set to LOW to sink current.
  DDRD  = B00000000;  //All PortD pins set as outputs.
  PORTD = B00000000;  //All PortD pins are set to LOW.

  //If you want to modify this code to add a serial device on pins 0 and 1, 
  //this line will give you problems, otherwise, this is ok.
  //Syntax for writing directly to a pin: 1<<PINB2;
    
  //Turn on AHI and BHI
  pinMode(11,OUTPUT);
  pinMode(12,OUTPUT);
  digitalWrite(11, HIGH);
  digitalWrite(12, HIGH);

  
  //Disable OSMC
  1<<PINB0;
}

void adc_init(void)
{
  ACSR = (1 << ACD);    //Turn off Analog Comparator
  ADMUX = 0;            //Make sure ADMUX register is all 0
  ADMUX |= (1<<REFS0);  //VCC is now ref. voltage
  ADCSRA = 0 | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0) | (1<<ADEN) | (1<<ADSC);
  //Set prescaler to 128, Enable ADC with ADEN bit, and Start first conversion.

  //Get bogus first conversion out of the way and initialize the ADC
  while(ADCSRA & (1<<ADSC))
  {
  }
}

void timer_init(void)
{ //We are using Timer1 on the ATMega328. This outputs to ship pins 15 and 16, which map
  //to Arduino Digital pins 9 and 10 and have the names PB1 and PB2. OC1A is PB1, OC1B is PB2  
  //p. 110 on datasheet. Use External Clock on rising edge. This is the 16MHz external oscillator.
  TCCR0B = 0 | (1<<CS02) | (1<<CS01) | (1<<CS00);  

  //PWM Mode is 10 bit, Phase Correct mode. You can find this on p. 136 of ATMega348 datasheet
  //We also Set OC1A/OC1B on Compare Match when upcounting and Clear OC1A/OC1B on Compare Match
  //when downcounting.
  TCCR1A = 0 | (1<<COM1A1) | (1<<COM1A0) | (1<<COM1B1) | (1<<COM1B0) | (1<<WGM11) | (1<<WGM10);

  //Set the clock prescaler to only divde the clock by 1
  TCCR1B = 0 | (1<<CS10);  
}

//adc_read() will read the value on a given analog input pin. Waits untill the 
//ADSC has finished the conversion and then returns the analog value.
uint16_t adc_read(uint8_t channel)
{
  ADMUX = channel;
  ADMUX |= (1<<REFS0);  //Shouldn't be necessary, but just in case.
  ADCSRA |= (1<<ADSC);  //Start Conversion
  while (ADCSRA & (1<<ADSC)) //Wait for conversion to finish
  {  }
  return ADCW;  //You won't find this in the datasheet, but it is in the ATMega328 header file. ADC is add w/ carry
}


void set_motor(void)
{
  int16_t leveli = (int16_t)(level*1023);

  if (leveli < -1020)
  { leveli = -1020;  }
  else if (leveli > 1020)  
  { leveli = 1020;  }

  if ((level <= -0.7)||(level >= 0.7))  
  {/*Buzzer or light*/  }
  else 
  {/*Turn off buzzer*/  }

  if (softStart < 1)  
  { softStart = softStart + 0.001;  }
  else  
  {  softStart = 1;  
  }

  //Enable OSMC
  (PINB & (0<<PINB0));

  if (leveli < 0)  //Going backwards
  {
    OCR1A = -level;
    OCR1B = 0;
  }
  else  //Going forward
  {
    OCR1A = 0;
    OCR1B = leveli;
  } 
}
