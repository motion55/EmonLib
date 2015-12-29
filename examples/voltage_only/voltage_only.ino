// EmonLibrary examples openenergymonitor.org, Licence GNU GPL V3

#include "EmonLib.h"                   // Include Emon Library
EnergyMonitor emon1;                   // Create an instance

void setup()
{  
  Serial.begin(9600);
  
  emon1.voltage(2, 234.26, 1.7);  // Voltage: input pin, calibration, phase_shift
}

void loop()
{
  double Vrms = emon1.calcVrms(1000);  // Calculate Vrms only
  
  Serial.print(" Vrms =");
  Serial.println(Vrms);		       // Vrms
}
