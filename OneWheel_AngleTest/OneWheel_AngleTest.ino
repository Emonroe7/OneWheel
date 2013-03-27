#include <avr/interupt.h>
#include <avr/io.h>

#define denom (1023/3)
#define degToRad 0.0174532925
#define radToDeg 57.2957795131

float angle = 0; 
int accelRead, gyroRead, gyrosum, accsum;
float xAcc, xAccRad, xAccDeg;
float angleRad;
const int accelOffset = 0;//Value to get the accelerometer to read 0 on a level surface.
const float accelDivisor = 204.8;//Value to divide by to get a value between 0 and 1.
const int maxAccelAcceptable = 200;//Max Acceleration reading offset
const int balPointLevel = 512;
int i = 0;
uint16_t adc0, adc1;
int sensorA;

void adc_init(void)
{
  ACSR = (1 << ACD);    //Turn off Analog Comparator
  ADMUX = 0;            //Make sure ADMUX register is all 0
  ADMUX |= (1<<REFS0);  //VCC is now ref. voltage
  ADCSRA = 0 | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0) | (1<<ADEN) | (1<<ADSC);
  //Set prescaler to 128, Enable ADC with ADEN bit, and Start first conversion.
  //Get bogus first conversion out of the way and initialize the ADC
  while(ADCSRA & (1<<ADSC))
  {  }
}

uint16_t adc_read(uint8_t channel)
{
  ADMUX = channel;
  ADMUX |= (1<<REFS0);  //Shouldn't be necessary, but just in case.
  ADCSRA |= (1<<ADSC);  //Start Conversion
  while (ADCSRA & (1<<ADSC)) //Wait for conversion to finish
  {  }
  return ADCW;  //You won't find this in the datasheet, but it is in the ATMega328 header file. ADC is add w/ carry
}

void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600); 
}

void loop() 
{
  accsum = 0;
  
  for (i = 0; i<20; i++)
  { 
      adc1 = adc_read(1);
      
      accsum += adc1;
  }
  
  //=============== Accelerometer Processing ================
  xAcc = (float) (accsum/20.0)-balPointLevel;
/*  if (xAcc < -maxAccelAcceptable)  
  { xAcc = -maxAccelAcceptable;  }
  else if (xAcc > maxAccelAcceptable)  
  { xAcc = maxAccelAcceptable;  } */
  xAccRad = (((float)xAcc - accelOffset))/accelDivisor;//value between -1 and 1
  xAccRad = asin(xAccRad);//Value between -PI and PI.
  xAccDeg = xAccRad * radToDeg;  
  //xAccDeg = xAcc / 3.45;
  Serial.print("\n xAccDeg = " );                       
  Serial.print(xAccDeg);
  //Serial.print("\n Sensor read = " );                       
  //Serial.print(sensorA);   
}
