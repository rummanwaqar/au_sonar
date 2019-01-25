#include "peak_detector.h"

///////////////////////////////////////////////////////////////////////////////////////////
/*Resets the peak detector*/  //
///////////////////////////////////////////////////////////////////////////////////////////
void detectorlow() {
  digitalWrite(PEAK_CLEAR_PIN, LOW);  // Turn on BJT
  resettime.end();                    // turns off the interval call
}

void resetDetector() {
  digitalWrite(PEAK_CLEAR_PIN, HIGH);  // Turn off BJT
  resettime.begin(detectorlow, 25);    // calls detectorlow at 25us
}

///////////////////////////////////////////////////////////////////////////////////////////
/*Initialization of the Peak Detector firmware*/  //
///////////////////////////////////////////////////////////////////////////////////////////
PeakDetector::PeakDetector() {
  pinMode(GPIO_3,
          OUTPUT);  // pin 7 is being used to monitor the ping status result
  pinMode(PEAK_CLEAR_PIN, OUTPUT);  // reset pin for peak detector BJT

  // intial ADC averaging value
  averagingADCValue = 16;

  adc = new ADC();
  adc->setReference(ADC_REFERENCE::REF_EXT,
                    ADC_0);              // sets external reference for ADC
  adc->setResolution(16);                // set the resolution of the ADC in use
  adc->setAveraging(averagingADCValue);  // sets averaging
  adc->setConversionSpeed(
      ADC_CONVERSION_SPEED::HIGH_SPEED_16BITS);          // set conversion speed
  adc->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED);  // set sample speed

  // Setup the initial ping valid duration
  this->PeakDetector::setPingValidStart(PINGVALIDSTART);
  this->PeakDetector::setPingValidEnd(PINGVALIDEND);
  this->PeakDetector::setPeakNoise(PEAK_NOISE);
}
///////////////////////////////////////////////////////////////////////////////////////////
/*Takes the input from the peak detector and checks it against twice average noise level,//
the desired interval, and shows result on pin 11.                                        //
Returns 1 if there is a valid ping                                                       //
Returns 2 if the ping is valid but ramping down                                          //
Returns 3 if the ping is valid but still ramping up                                      //
Returns 0 if the ping is invalid*/  //
///////////////////////////////////////////////////////////////////////////////////////////
int PeakDetector::getPingStatus(float &peaklevel) {
  int pingvalid;
  static int validcounter = 0;
  float goodlevel = 2 * peaknoise;  // twice avgerage noise for safety
  float peakinput =
      adc->analogRead(A8, ADC_0);  // A8 analogueread(8), 0 to 65535
  peaklevel = (Vref * peakinput) / (resolution);  // Adjust result, Vref is max
  if (peaklevel <= goodlevel) {                   // Invalid ping levels
    pingvalid = 0;
    digitalWrite(GPIO_3, LOW);  // Ping flag LOW
    digitalWrite(BUILTIN_LED, LOW);
    return pingvalid;
  }
  if (peaklevel > goodlevel) {  // Valid ping levels
    if (validcounter == 0) {    // Get ping start time and set validcounter
      offsettime = pingoffset;  // set ellaspedMicros back to 0 to avoid
                                // rollover
      validcounter = 1;
    }
    if (validcounter) {  // Offset validping start and end times to remove
                         // echoes
      if ((pingoffset - offsettime) < pingValidStart) {  // Set pingvalid to LOW
        pingvalid = 3;
        digitalWrite(GPIO_3, LOW);  // Ping flag LOW
        digitalWrite(BUILTIN_LED, LOW);
        // Serial.print("Start");
        return pingvalid;
      }
      if (((pingoffset - offsettime) >= pingValidStart) &&
          ((pingoffset - offsettime) < pingValidEnd)) {
        pingvalid = 1;
        digitalWrite(GPIO_3, HIGH);  // Ping flag HIGH
        digitalWrite(BUILTIN_LED, HIGH);
        // Serial.print("Valid");
        return pingvalid;
      }
      if ((pingoffset - offsettime) >=
          pingValidEnd) {  // Set pingvalid to LOW and clear the validcounter
        pingvalid = 2;
        digitalWrite(GPIO_3, LOW);  // Ping flag LOW
        digitalWrite(BUILTIN_LED, LOW);
        validcounter = 0;
        // Serial.print("End");
        return pingvalid;
      }
    }
  }
}

////////////////////////////////////////////////
// HELPER Methods
////////////////////////////////////////////////

void PeakDetector::setPingValidStart(unsigned int time_ms) {
  pingValidStart = time_ms;
}

void PeakDetector::setPingValidEnd(unsigned int time_ms) {
  pingValidEnd = time_ms;
}

void PeakDetector::setADCAveraging(int value) {
  adc->setAveraging(value);
  averagingADCValue = value;
}

int PeakDetector::getADCAveraging(void) { return averagingADCValue; }

void PeakDetector::setPeakNoise(float halfamp) { peaknoise = halfamp; }
