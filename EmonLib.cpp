/*
  Emon.cpp - Library for openenergymonitor
  Created by Trystan Lea, April 27 2010
  GNU GPL
  modified to use up to 12 bits ADC resolution (ex. Arduino Due)
  by boredman@boredomprojects.net 26.12.2013
  Low Pass filter for offset removal replaces HP filter 1/1/2015 - RW
*/

//#include "WProgram.h" un-comment for use on older versions of Arduino IDE
#include "EmonLib.h"

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define DC_SAMPLES  1024l

//--------------------------------------------------------------------------------------
// Sets the pins to be used for voltage and current sensors
//--------------------------------------------------------------------------------------
void EnergyMonitor::voltage(unsigned int _inPinV, double _VCAL, double _PHASECAL)
{
#if defined(analogPinToChannel)
#if defined(__AVR_ATmega32U4__)
	if (_inPinV >= 18) _inPinV -= 18; // allow for channel or pin numbers
#endif
	_inPinV = analogPinToChannel(_inPinV);
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
	if (_inPinV >= 54) _inPinV -= 54; // allow for channel or pin numbers
#elif defined(__AVR_ATmega32U4__)
	if (_inPinV >= 18) _inPinV -= 18; // allow for channel or pin numbers
#elif defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega644__) || defined(__AVR_ATmega644A__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644PA__)
	if (_inPinV >= 24) _inPinV -= 24; // allow for channel or pin numbers
#else
	if (_inPinV >= 14) _inPinV -= 14; // allow for channel or pin numbers
#endif
	inPinV = _inPinV;
	VCAL = _VCAL;
	PHASECAL = _PHASECAL;

	offsetVlong = 0;

	unsigned long int start = millis();
	for (SampleCount = 0; (millis() - start) < sampling_time_ms; SampleCount++)
	{
		sampleVshort = analogRead(inPinV);
		offsetVlong += sampleVshort;
		if (sampleVshort > startV) startV = sampleVshort;
	}
	offsetVshort = offsetVlong / SampleCount;
	offsetVlong = DC_SAMPLES;
	offsetVlong *= offsetVshort;
}

void EnergyMonitor::current(unsigned int _inPinI, double _ICAL)
{
#if defined(analogPinToChannel)
#if defined(__AVR_ATmega32U4__)
	if (_inPinI >= 18) _inPinI -= 18; // allow for channel or pin numbers
#endif
	_inPinI = analogPinToChannel(_inPinI);
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
	if (_inPinI >= 54) _inPinI -= 54; // allow for channel or pin numbers
#elif defined(__AVR_ATmega32U4__)
	if (_inPinI >= 18) _inPinI -= 18; // allow for channel or pin numbers
#elif defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega644__) || defined(__AVR_ATmega644A__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644PA__)
	if (_inPinI >= 24) _inPinI -= 24; // allow for channel or pin numbers
#else
	if (_inPinI >= 14) _inPinI -= 14; // allow for channel or pin numbers
#endif
	inPinI = _inPinI;
	ICAL = _ICAL;

	unsigned long int start = millis();

	offsetIlong = 0;
	for (SampleCount = 0; (millis() - start)<sampling_time_ms; SampleCount++)
	{
		offsetIlong += analogRead(inPinI);
	}
	offsetIshort = offsetIlong / SampleCount;
	offsetIlong = DC_SAMPLES;
	offsetIlong *= offsetIshort;
}

//--------------------------------------------------------------------------------------
// Sets the pins to be used for voltage and current sensors based on emontx pin map
//--------------------------------------------------------------------------------------
void EnergyMonitor::voltageTX(double _VCAL, double _PHASECAL)
{
   inPinV = 2;
   VCAL = _VCAL;
   PHASECAL = _PHASECAL;

   unsigned long int start = millis();

   offsetVlong = 0;
   for (SampleCount = 0; (millis() - start)<sampling_time_ms; SampleCount++)
   {
	   offsetVlong += analogRead(inPinV);
   }
   offsetVshort = offsetVlong / SampleCount;
   offsetVlong = DC_SAMPLES;
   offsetVlong *= offsetVshort;
}

void EnergyMonitor::currentTX(unsigned int _channel, double _ICAL)
{
   if (_channel == 1) inPinI = 3;
   if (_channel == 2) inPinI = 0;
   if (_channel == 3) inPinI = 1;
   ICAL = _ICAL;

   unsigned long int start = millis();

   offsetIlong = 0;
   for (SampleCount = 0; (millis() - start)<sampling_time_ms; SampleCount++)
   {
	   offsetIlong += analogRead(inPinI);
   }
   offsetIshort = offsetIlong / SampleCount;
   offsetIlong = DC_SAMPLES;
   offsetIlong *= offsetIshort;
}

//--------------------------------------------------------------------------------------
// emon_calc procedure
// Calculates realPower,apparentPower,powerFactor,Vrms,Irms,kWh increment
// From a sample window of the mains AC voltage and current.
// The Sample window length is defined by the number of half wavelengths or crossings we choose to measure.
//--------------------------------------------------------------------------------------
void EnergyMonitor::calcVI(unsigned int crossings, unsigned int timeout)
{
   #if defined emonTxV3
	int SupplyVoltage=3300;
   #else 
	int SupplyVoltage = readVcc();
   #endif

	//Reset accumulators
	sumVlong = 0;
	sumIlong = 0;
	sumPlong = 0;

	unsigned int crossCount = 0;                             //Used to measure number of times threshold is crossed.
	unsigned int numberOfSamples = 0;                        //This is now incremented  

	//-------------------------------------------------------------------------------------------------------------------------
	// 1) Waits for the waveform to be close to 'zero' (mid-scale adc) part in sin curve.
	//-------------------------------------------------------------------------------------------------------------------------
	boolean st=false;                                  //an indicator to exit the while loop

	unsigned long start = millis();    //millis()-start makes sure it doesnt get stuck in the loop if there is an error.

	while(st==false)                                   //the while loop...
	{
		sampleVshort = analogRead(inPinV);                    //using the voltage waveform

		offsetVlong -= offsetVshort;
		offsetVlong += sampleVshort;
		offsetVshort = offsetVlong / DC_SAMPLES;
		filteredV = sampleVshort - offsetVshort;

		startV = (offsetVshort * 7) / 8;

		if (sampleVshort<startV) st = true;
		if ((millis()-start)>sampling_time_ms) st = true;
	}
  
	start = millis();    //millis()-start makes sure it doesnt get stuck in the loop if there is an error.

	while (st==false)                                   //the while loop...
	{
		sampleVshort = analogRead(inPinV);                    //using the voltage waveform

		offsetVlong -= offsetVshort;
		offsetVlong += sampleVshort;
		offsetVshort = offsetVlong / DC_SAMPLES;
		filteredV = sampleVshort - offsetVshort;

		if (sampleVshort>offsetVshort) st = true;
		if ((millis() - start) > sampling_time_ms) st = true;
	}

  //-------------------------------------------------------------------------------------------------------------------------
  // 2) Main measurement loop
  //------------------------------------------------------------------------------------------------------------------------- 
	SampleCount = 0;
	start = millis();

  while ((crossCount < crossings) && ((millis()-start)<timeout)) 
  {
    numberOfSamples++;                       //Count number of times looped.
    lastFilteredV = filteredV;               //Used for delay/phase compensation
    
    //-----------------------------------------------------------------------------
    // A) Read in raw voltage and current samples
    //-----------------------------------------------------------------------------
	sampleIshort = analogRead(inPinI);                 //Read in raw current signal
	sampleVshort = analogRead(inPinV);                 //Read in raw voltage signal
	SampleCount++;

    //-----------------------------------------------------------------------------
    // B) Apply digital low pass filters to extract the 2.5 V or 1.65 V dc offset,
    //     then subtract this - signal is now centred on 0 counts.
    //-----------------------------------------------------------------------------
	offsetIlong -= offsetIshort;
	offsetIlong += sampleIshort;
	offsetIshort = offsetIlong / DC_SAMPLES;
	filteredI = sampleIshort - offsetIshort;

	offsetVlong -= offsetVshort;
	offsetVlong += sampleVshort;
	offsetVshort = offsetVlong / DC_SAMPLES;
	filteredV = sampleVshort - offsetVshort;

	//-----------------------------------------------------------------------------
    // C) Root-mean-square method voltage
    //-----------------------------------------------------------------------------  
	long int sqVlong = filteredV;
	sqVlong *= sqVlong;                //1) square voltage values
    sumVlong += sqVlong;               //2) sum
    
    //-----------------------------------------------------------------------------
    // D) Root-mean-square method current
    //-----------------------------------------------------------------------------   
	long int sqIlong = filteredI;
	sqIlong *= sqIlong;                //1) square current values
	sumIlong += sqIlong;               //2) sum 
    
    //-----------------------------------------------------------------------------
    // E) Phase calibration
    //-----------------------------------------------------------------------------
    int phaseShiftedV = lastFilteredV + PHASECAL * (filteredV - lastFilteredV); 
    
    //-----------------------------------------------------------------------------
    // F) Instantaneous power calc
    //-----------------------------------------------------------------------------   
	long int instPlong = phaseShiftedV;
	instPlong *= filteredI; 	//Instantaneous Power
    sumPlong += instPlong;									//Sum  
    
    //-----------------------------------------------------------------------------
    // G) Find the number of times the voltage has crossed the initial voltage
    //    - every 2 crosses we will have sampled 1 wavelength 
    //    - so this method allows us to sample an integer number of half wavelengths which increases accuracy
    //-----------------------------------------------------------------------------       
    lastVCross = checkVCross;                     
    if (sampleVshort > startV) checkVCross = true; 
                          else checkVCross = false;
    if (numberOfSamples==1) lastVCross = checkVCross;                  
                     
    if (lastVCross != checkVCross) crossCount++;
  }
 
  //-------------------------------------------------------------------------------------------------------------------------
  // 3) Post loop calculations
  //------------------------------------------------------------------------------------------------------------------------- 
  //Calculation of the root of the mean of the voltage and current squared (rms)
  //Calibration coefficients applied. 
  
  double V_RATIO = (VCAL*SupplyVoltage) / (1000.0*ADC_COUNTS);
  Vrms = V_RATIO * sqrt((double)sumVlong / numberOfSamples); 
  
  double I_RATIO = (ICAL*SupplyVoltage) / (1000.0*ADC_COUNTS);
  Irms = I_RATIO * sqrt((double)sumIlong / numberOfSamples); 

  //Calculation power values
  realPower = V_RATIO * I_RATIO * sumPlong / numberOfSamples;
  apparentPower = Vrms * Irms;
  powerFactor=realPower / apparentPower;

//--------------------------------------------------------------------------------------       
}

//--------------------------------------------------------------------------------------
double EnergyMonitor::calcVrms(unsigned int Sampling_Time_ms)
{
#if defined emonTxV3
	int SupplyVoltage = 3300;
#else 
	int SupplyVoltage = readVcc();
#endif

	//Reset accumulators
	sumVlong = 0;

	unsigned long start = millis();

	sampleVshort = analogRead(inPinV);

	for (SampleCount = 0; (millis() - start) < Sampling_Time_ms; SampleCount++)
	{
		while (bit_is_set(ADCSRA, ADSC)); //wait for AD conversion complete.

		//read the ADC conversion result.
		uint8_t low = ADCL;
		uint8_t high = ADCH;

		//start the next conversion
		_SFR_BYTE(ADCSRA) |= _BV(ADSC);

		sampleVshort = (int)(high << 8) + low;

		// Digital low pass filter extracts the 2.5 V or 1.65 V dc offset, 
		//  then subtract this - signal is now centered on 0 counts.
		offsetVlong -= offsetVshort;
		offsetVlong += sampleVshort;
		offsetVshort = offsetVlong / DC_SAMPLES;
		filteredV = sampleVshort - offsetVshort;

		// Root-mean-square method current
		long int sqVlong = filteredV;
		sqVlong *= sqVlong;                //1) square current values
		sumVlong += sqVlong;               //2) sum 
	}

	double V_RATIO = (VCAL*SupplyVoltage) / (1000.0*ADC_COUNTS);
	Vrms = V_RATIO * sqrt((double)sumVlong / SampleCount);

	return Vrms;
}

//--------------------------------------------------------------------------------------
double EnergyMonitor::calcIrms(unsigned int Sampling_Time_ms)
{
  
   #if defined emonTxV3
	int SupplyVoltage=3300;
   #else 
	int SupplyVoltage = readVcc();
   #endif

  //Reset accumulators
  sumIlong = 0;

  unsigned long start = millis();

  sampleIshort = analogRead(inPinI);

  for (SampleCount = 0; (millis()-start)<Sampling_Time_ms; SampleCount++)
  {
	  while (bit_is_set(ADCSRA, ADSC)); //wait for AD conversion complete.

      //read the ADC conversion result.
	  uint8_t low = ADCL;
	  uint8_t high = ADCH;

	  //start the next conversion
	  _SFR_BYTE(ADCSRA) |= _BV(ADSC);

	  sampleIshort = (int)(high << 8) + low;

    // Digital low pass filter extracts the 2.5 V or 1.65 V dc offset, 
	//  then subtract this - signal is now centered on 0 counts.
	offsetIlong -= offsetIshort;
	offsetIlong += sampleIshort;
	offsetIshort = offsetIlong / DC_SAMPLES;
	filteredI = sampleIshort - offsetIshort;

    // Root-mean-square method current
	long int sqIlong = filteredI;
	sqIlong *= sqIlong;                //1) square current values
	sumIlong += sqIlong;               //2) sum 
  }

  double I_RATIO = (ICAL*SupplyVoltage)/(1000.0f*ADC_COUNTS);
  Irms = I_RATIO * sqrt((double)sumIlong / SampleCount); 

  return Irms;
}

void EnergyMonitor::serialprint()
{
    Serial.print(realPower);
    Serial.print(' ');
    Serial.print(apparentPower);
    Serial.print(' ');
    Serial.print(Vrms);
    Serial.print(' ');
    Serial.print(Irms);
    Serial.print(' ');
    Serial.print(powerFactor);
    Serial.println(' ');
    delay(100); 
}

//thanks to http://hacking.majenko.co.uk/making-accurate-adc-readings-on-arduino
//and Jérôme who alerted us to http://provideyourown.com/2012/secret-arduino-voltmeter-measure-battery-voltage/

long EnergyMonitor::readVcc() {
  long result;
  
  //not used on emonTx V3 - as Vcc is always 3.3V - eliminates bandgap error and need for calibration http://harizanov.com/2013/09/thoughts-on-avr-adc-accuracy/

  #if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328__) || defined (__AVR_ATmega328P__)
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);  
  #elif defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_AT90USB1286__)
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  ADCSRB &= ~_BV(MUX5);   // Without this the function always returns -1 on the ATmega2560 http://openenergymonitor.org/emon/node/2253#comment-11432
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
  ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  ADMUX = _BV(MUX3) | _BV(MUX2);
	
  #endif


  #if defined(__AVR__) 
  delay(2);                                        // Wait for Vref to settle
  ADCSRA |= _BV(ADSC);                             // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = READVCC_CALIBRATION_CONST / result;  //1100mV*1024 ADC steps http://openenergymonitor.org/emon/node/1186
  return result;
 #elif defined(__arm__)
  return (3300);                                  //Arduino Due
 #else 
  return (3300);                                  //Guess that other un-supported architectures will be running a 3.3V!
 #endif
}

