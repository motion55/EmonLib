
#include <EmonLib.h>
EnergyMonitor emon1;             // Create an instance

// constants won't change. Used here to set a pin number :
const int ledPin =  13;      // the number of the LED pin

// Variables will change :
int ledState = LOW;             // ledState used to set the LED

// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store
unsigned long previousMillis = 0;        // will store last time LED was updated

// constants won't change :
const long interval = 1000;           // interval at which to blink (milliseconds)

void setup() 
{
  pinMode(ledPin, OUTPUT);
  
  Serial.begin(9600);

  emon1.voltage(2, 234.26);  // Voltage: input pin, calibration, phase_shift
  emon1.current(1, 60.0);       // Current: input pin, calibration.

  emon1.startMeter();
}

void loop() 
{
  if (emon1.EnergyMeter())
  {
    emon1.serialprint();
  }
}
