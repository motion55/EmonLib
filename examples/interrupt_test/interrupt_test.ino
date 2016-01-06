
#include <EmonLib.h>
EnergyMonitor emon1;             // Create an instance

// constants won't change. Used here to set a pin number :
const int ledPin =  13;      // the number of the LED pin

// Variables will change :
int ledState = LOW;             // ledState used to set the LED

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

    // if the LED is off turn it on and vice-versa:
    if (ledState == LOW) 
    {
      ledState = HIGH;
    }
    else 
    {
      ledState = LOW;
    }
    // set the LED with the ledState of the variable:
    digitalWrite(ledPin, ledState);
  }
}
