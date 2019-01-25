#include <Arduino.h>
#include "pinout.h"

#include "Amplifier.h"
#include "CommInterface.h"
#include "Filter.h"
#include "GainControl.h"
#include "peak_detector.h"

Filter bandPassfilter;
GainControl gainControl;
CommInterface commInterface;

void setup() {
  // setup LED
  pinMode(BUILTIN_LED, OUTPUT);
  digitalWrite(BUILTIN_LED, HIGH);

  // Set the filter frequency in kHz
  bandPassfilter.setCenterFreq(27);

  // Allow the communication interface access to the gain controller
  commInterface.getGainControlPointer(gainControl);

  // Allow the communication interface to access the Filter
  commInterface.getFilterPointer(bandPassfilter);

  Serial1.println("$Teensy Initialized");
  digitalWrite(BUILTIN_LED, LOW);
}

void loop() {
  // If there was a serial interupt with a valid serial string,
  // parse said serial String
  if (commInterface.validIncommingSerialMessage()) {
    commInterface.parseMessage();
  }

  // Get the ping under control for analysis.
  gainControl.runtime();
}

void serialEvent1() { commInterface.checkSerial(); }
