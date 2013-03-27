/*
=  AUTHOR: Eric Monroe
=  DATE:  6/7/12
=
=  Monowheel code written for Arduino Duemilanove w/ ATMEGA328.
=  Originally written by John Dingley (XenonJohn) for the ATMega32. 
=  This code has been modified to work with an Arduino Board with an ATMEGA328
=  There are some slight differences, (tilt switches are now digital inputs), but overall 
=  functionality has remained the same. Have also streamlined some of the math.
=  This code makes heavy use of bitwise operations and port manipulation. Sorry. 
=  References have been included . included.
=  Software Serial Library is imported for debugging purposes.
=
=  References:
=  http://www.arcfn.com/2009/07/secrets-of-arduino-pwm.html
=  http://www.arduino.cc/en/Reference/PortManipulation
=  http://arduino.cc/en/Reference/Constants
=  http://arduino.cc/playground/Main/TimerPWMCheatsheet
=  http://www.avrfreaks.net/index.php?name=pnphpbb2&file=viewtopic&t=37871
=  https://sites.google.com/site/qeewiki/books/avr-guide/pwm-on-the-atmega328
=  http://hekilledmywire.wordpress.com/2011/02/23/direct-port-manipulation-using-the-digital-ports-tutorial-part-3/
=  http://web.mit.edu/scolton/www/filter.pdf  
=  http://www.instructables.com/files/orig/FQG/KST3/GZ0MM8D9/FQGKST3GZ0MM8D9.txt
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
=  11   AHI (OSMC Pin 5) (Can also tie to +5volts if you experience problems with OSMC
=  12   BHI (OSMC Pin 5) (Can also tie to +5volts if you experience problems with OSMC 
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
#include <avr/pgrmspace.h>
#include <avr/interupt.h>
#include <avr/io.h>
#include <math.h>


#define denom (1023/3)
#define degToRad 0.0174532925
#define radToDeg 57.2957795131
typedef unsigned char u8;
//Using a char value for channel number because it is only 1 byte.
//The value of channel is always small, so this is the best variable type.


//Program Controls
#define ctrlTiltPresent 1  //Incase a tilt control (switch) isn't connected
#define ctrlGainReading 1  //Gains can be eventually hard coded with #define. Change to 0 then.

//===============================================================
//===================== Variable Declaration ====================
//===============================================================
boolean tipStart = false;
float lastXAcc = 2000;
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
const int balPointOffset = 50;
const int balPointLevel = 512;
int balPoint;
int i = 0;
int pulse = 0;

// Sensor Characteristics
// These characteristics are for the Sparkfun ADXL203/ADXRS610 combo board.
// It's old, but extremely easy to interface with. Accelerometer is +/-1.5g, and runs on 5 volts
const float gyroStepsPerDeg = 3.072; //steps per degree on gyro, etc. 15mV/degree/sec over 204.8 steps
const int maxGyro = 275;//Gyro is rated for 300 degrees per second
const int gyroOffset = 512;//Value to offset for any constant drift to get gyro to read 0 (512 means no drift).
const int accelOffset = 0;//Value to get the accelerometer to read 0 on a level surface.
const float accelDivisor = 204.8;//Value to divide by to get a value between 0 and 1.
const int maxAccelAcceptable = 200;//Max Acceleration reading offset


//===============================================================
//=================== Control System Variables ==================
//===============================================================
float cycleTime = 0.0096;//Cycle time in seconds (currently runs at 9.6 ms or 104 Hz)
float gyroScalingFactor = 1.0;//Shouldn't be necessary but just in case (integral factor for gyro)
float gyroCoefficient = .98;  //Integral gain in balancing control loop
float accelCoefficient = .02; //Its just 1-gyroCoefficient. These 2 coefficients determine the time constant (.49 for us. 

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
  //Serial.begin(9600);
  ports_init();
  adc_init();
  timer_init();
  tipStart = false;
  while(tipStart == false)
  { 
    //Serial.print("\ntipStart = " );                       
    //Serial.print(tipStart);
    sample_inputs();
    if (xAccDeg > 0) //xAccDeg is about +10 when at rest
    { tipStart = false;  }
    else if (xAccDeg < 0)  //Board has been tilted past the point where 
    { 
      tipStart = true;
      softStart = 0.4;
    }
  }
  angle = 0;
  currSpeed = 0;
  sei();
}

void loop() 
{
  while (1)
  {
//    Serial.print("\nLevel = " );                       
//    Serial.print(level,5);
    
    sample_inputs();
    set_motor();
    
//    //Timing pulse
//    if (pulse == 1)
//    { PORTB &= (0<<PINB5) ; 
//      pulse = 0;
//    }
//    else if (pulse ==0)
//    { PORTB |= (1<<PINB5); 
//      pulse = 1;
//    }
  }
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
  pinMode(13,OUTPUT);
  digitalWrite(11, HIGH);
  digitalWrite(12, HIGH);
  1<<PINB5;
  //Disable OSMC
  //1<<PINB0;
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

void sample_inputs(void)
{
  uint16_t adc0, adc1, adc2, adc3;
  accsum = 0;
  gyrosum = 0;

  for (i = 0; i<20; i++)
  {
    adc0 = adc_read(0);  // Gyro
    adc1 = adc_read(1);  // Accelerometer

    gyrosum += adc0;  //Accumulation of 20 gyro readings
    accsum += adc1;   //Accumulation of 20 Accelerometer readings
  } 

  if(ctrlTiltPresent){ //Added for those wanting a simpler build
    tiltFW = (PIND & (1<<PIND6));
    tiltRV = (PIND & (1<<PIND7));
  }
  else{
    tiltFW = false;
    tiltRV = false;
  }

  if (tipStart == false){  //If not yet balancing, read in gain values
    adc2 = adc_read(2);
    adc3 = adc_read(3);
    gainCtrl = (float) adc2/(1023.0/4.0);
    throttleGain = (float)adc3/(1023.0/4.0);

    if (adc2<100){//Not sure if neccessary, but no good reason not to have
      throttleGain = 0.001;
      gainCtrl = 0.001;
    }
    overallGain = gainCtrl * softStart;
  }

  if (tiltRV == true)
  { balPoint = balPointLevel - balPointOffset;  }
  else if (tiltFW == true)
  { balPoint = balPointLevel + balPointOffset;  }
  else
  { balPoint = balPointLevel;  }

  //=============== Accelerometer Processing ================
  xAcc = ((float)accsum/20.0) - balPoint;
  if (xAcc < -maxAccelAcceptable)  
  { xAcc = -maxAccelAcceptable;  }
  else if (xAcc > maxAccelAcceptable)  
  { xAcc = maxAccelAcceptable;  } 
  //xAccRad = ((float)(xAcc - accelOffset))/accelDivisor;//value between -1 and 1
  //xAccRad = asin(xAccRad);//Value between -PI and PI.
  //xAccDeg = xAccRad * radToDeg;  
  xAccDeg = xAcc / 3.45;
  
  //=================== Gyro Processing =====================
  gyroRateDeg = ((float)gyrosum/20.0 - gyroOffset)/gyroStepsPerDeg;
  if (gyroRateDeg < -275)  
  { gyroRateDeg = -maxGyro;  }
  else if (gyroRateDeg > 275)  
  { gyroRateDeg = maxGyro;  }
  gyroRateDt = gyroScalingFactor*cycleTime*gyroRateDeg; //Scaling factor is integral gain basically
  gyroRateRad = gyroRateDeg * degToRad;
  
  //=================== Control System ======================
  angle = ((gyroCoefficient) * (angle + gyroRateDt)) + (accelCoefficient * xAccDeg);
  angleRad = angle * degToRad;
  
  balTorque = 4.5*angleRad + .5*gyroRateRad;//Original: 4.5*angleRad + .5*gyroRateRad;
  currSpeed = (currSpeed + (throttleGain*balTorque*cycleTime)) * 0.98;
  level = (balTorque + currSpeed)*overallGain;
}

void set_motor(void)
{
  int16_t leveli = (int16_t)(level*1023); //Level comes in as a floating point number -1 to +1
  
  //Serial.print("\nLeveli = ");
  //Serial.print(leveli);
  
  if (leveli < -1020)
  { leveli = -1020;  }
  else if (leveli > 1020)  
  { leveli = 1020;  }

  if ((level <= -0.7)||(level >= 0.7))  
  {/*Buzzer or light*/  }
  else 
  {/*Turn off buzzer*/  }

  if (softStart < 1)  
  { softStart = softStart + 0.003;  }
  else  
  {  softStart = 1;  
  }

  //Enable OSMC
  (PINB & (0<<PINB0));

  if (leveli < 0)  //Going backwards
  {
    OCR1A = -leveli;//-level;
    OCR1B = 0;
  }
  else  //Going forward
  {
    OCR1A = 0;
    OCR1B = leveli;//leveli;
  } 
}
