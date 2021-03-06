/*
	Emon.h - Library for openenergymonitor
	Created by Trystan Lea, April 27 2010
	GNU GPL
	modified to use up to 12 bits ADC resolution (ex. Arduino Due)
	by boredman@boredomprojects.net 26.12.2013
	Low Pass filter for offset removal replaces HP filter 1/1/2015 - RW
*/

#ifndef EmonLib_h
#define EmonLib_h

#if defined(ARDUINO) && ARDUINO >= 100

#include "Arduino.h"

#else

#include "WProgram.h"

#endif

// define theoretical vref calibration constant for use in readvcc()
// 1100mV*1024 ADC steps http://openenergymonitor.org/emon/node/1186
// override in your code with value for your specific AVR chip
// determined by procedure described under "Calibrating the internal reference voltage" at
// http://openenergymonitor.org/emon/buildingblocks/calibration
#ifndef READVCC_CALIBRATION_CONST
#define READVCC_CALIBRATION_CONST 1126400L
#endif

// to enable 12-bit ADC resolution on Arduino Due, 
// include the following line in main sketch inside setup() function:
//  analogReadResolution(ADC_BITS);
// otherwise will default to 10 bits, as in regular Arduino-based boards.
#if defined(__arm__)
#define ADC_BITS    12
#else
#define ADC_BITS    10
#endif

#define ADC_COUNTS  (1<<ADC_BITS)


class EnergyMonitor
{
public:
	void voltage(unsigned int _inPinV, double _VCAL, double _PHASECAL = 0.0);
	void current(unsigned int _inPinI, double _ICAL);

	void voltageTX(double _VCAL, double _PHASECAL = 0);
	void currentTX(unsigned int _channel, double _ICAL);

	void calcVI(unsigned int Sampling_Time_ms);
	double calcVrms(unsigned int Sampling_Time_ms);
	double calcIrms(unsigned int Sampling_Time_ms);
	void serialprint();

	void startMeter(void);
	void stopMeter(void);
	bool EnergyMeter(void);

	long readVcc();
	//Useful value variables
	double	realPower,
			apparentPower,
			powerFactor,
			Vrms,
			Irms,
			WattHrs;

private:
	const unsigned long int sampling_time_ms = 1000;
	int SupplyVoltage;

	//Set Voltage and current input pins
	int inPinV;
	int inPinI;
	//Calibration coefficients
	//These need to be set in order to obtain accurate results
	double VCAL, V_RATIO;
	double ICAL, I_RATIO;
	double P_RATIO;
	int PHASECAL1, PHASECAL2;

	inline void SelectAnalogPin(unsigned int pin)
	{
		#if defined(ADCSRB) && defined(MUX5)
		ADCSRB = (ADCSRB & ~(1 << MUX5)) | (((pin >> 3) & 0x01) << MUX5);
		#endif
		#if defined(ADMUX)
		ADMUX = (ADMUX & ~0x07) | (pin & 0x07);
		#endif
	};

	char MeterStarted;
	char channel_select;
	char SampleReady;
	unsigned long int sample_start;
	unsigned int _SampleCount;
	long int _sumVlong, _sumIlong, _sumPlong;

public:
	inline void interrupt_handler();

	//--------------------------------------------------------------------------------------
	// Variable declaration for emon_calc procedure
	//--------------------------------------------------------------------------------------
	int sampleVshort;                //sample_ holds the raw analog read value
	int sampleIshort;

	int lastFilteredV, lastFilteredI;
	int	filteredV, filteredI;          //Filtered_ is the raw analog value minus the DC offset
	long int offsetVlong, offsetIlong;   
	int offsetVshort, offsetIshort;                 //Low-pass filter output               
	long int sumVlong, sumIlong, sumPlong;

	unsigned int SampleCount;
	unsigned long int AccumSampleCount;
	unsigned int AccumSampleTime;
};

#endif
