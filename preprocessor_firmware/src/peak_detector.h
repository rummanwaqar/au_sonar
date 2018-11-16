#ifndef PEAK_DETECTOR_H
#define PEAK_DETECTOR_H

#include <ADC.h>
#include <Arduino.h>
#include <IntervalTimer.h>
#include "constants.h"
#include "pinout.h"

// Interrupt timer to control detectorlow
static IntervalTimer resettime;

void detectorlow();    // Turns the peak detector BJT off
void resetDetector();  // calls detectorlow at a set time

class PeakDetector {
 public:
  PeakDetector();  // initializes PeakDetector

  elapsedMicros pingoffset;  // timer for getPingStatus to eliminate echoes

  int getPingStatus(
      float &peaklevel);  // gets the ping level and provides the ping status

  ////////////////////////////////////////////////
  // HELPER Methods
  ////////////////////////////////////////////////

  // Set the time after ping valid, before starting gain controller (ms)
  void setPingValidStart(unsigned int time_ms);

  // Set the time until ping is ended (gain will be floored to 0dB) (ms)
  void setPingValidEnd(unsigned int time_ms);

  // Method for the comm interface to Adjust the averaging
  void setADCAveraging(int value);

  // Method for getting the averager for the ADC
  int getADCAveraging(void);

  // Set the valid threshold
  void setPeakNoise(float halfamp);

 private:
  // used for getPingStatus and initialization
  ADC *adc;

  int offsettime;
  unsigned int pingValidStart;
  unsigned int pingValidEnd;
  unsigned int averagingADCValue;
  float peaknoise;
};

#endif
