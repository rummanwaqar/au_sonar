#include "Amplifier.h"

/*Function for setting the gain of the AD8336 variable gain amplifier (VGA)
using the digitial to analog converter of a teensy 3.2 to control \
the GPOS pin on the VGA with the pre-amplifier set to 26dB and the GNEG
at a reference voltage of 0.9V.
Datasheet for AD8336 VGA: http://www.analog.com/media/en/technical-documentation/data-sheets/AD8336.pdf*/
void setAmplifierGain(float gain) //gain in dB
{
  float GPOS=0; //Positive terminal on VGA
  float GNEG=0.9; //Negative terminal on VGA
  float GPRA=26;  //Pre-amplifier gain
  float DAC_val;

  if(gain>60)
  {
    Serial.print("Error Value out of range");  //Return error if requested gain is out of range
    return;
  }

  else if(gain<0)
  {
    Serial.print("Error: value out of range");  //Return error if requested gain is out of range
    return;
  }

  else
  {
    GPOS=(gain-GPRA-4.4)/49.9 + GNEG; //Get required GPOS voltage from gain formula according to page 21 of datasheet
    DAC_val=(GPOS/3.3)*4096; //Convert GPOS voltage into DAC input
    analogWrite(DAC_PIN, (int)DAC_val); //Output GPOS voltage on PIN_A14 which connects to AD8336
  }

}
