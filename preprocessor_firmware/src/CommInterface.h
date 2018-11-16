#ifndef _COMMINTERFACE_H_
#define _COMMINTERFACE_H_

#include <Arduino.h>
#include <cstdlib>
#include <string>

#include "Filter.h"
#include "GainControl.h"
#include "constants.h"
#include "pinout.h"

#define NUMBER_COMMANDS 2
#define NUMBER_VARIABLES 19

class CommInterface {
 public:
  CommInterface(void);

  // Check the incomming serialEvent here
  void checkSerial(void);

  // If there was a valid serial string, parse the message
  void parseMessage(void);

  // Returns true if the incomming serial string was valid
  bool validIncommingSerialMessage(void);

  // We need to get the pointers to the gain control and filter
  // so we can modify the variables inside those objects
  void getGainControlPointer(GainControl& _gainControl);
  void getFilterPointer(Filter& _filter);

 private:
  String message;

  // Carries the pointer to the gainControl object
  GainControl* gainControl;
  // Carries the point to the filter object
  Filter* filter;

  bool messageReceived;
  bool debugFlag;

  uint8_t commandIndex;
  uint8_t variableIndex;
  uint8_t argument1Length;
  uint8_t argument2Length;

  void getCommand(void);
  void getVariable(void);
  void getArgument(void);
  void getSecondArgument(void);
  String argument1String;
  String argument2String;

  void setValue(void);
  void sendLocalVariable(void);
  void prepareTransmission(void);
  float output_float;
  float output2_float;
  int output_int;

  void cleanup(void);

  const String commands[NUMBER_COMMANDS] = {String(F("set")), String(F("get"))};
  const String variables[NUMBER_VARIABLES] = {
      String(F("desiredPeak")),    String(F("holdGain")),
      String(F("pGain")),          String(F("iGain")),
      String(F("iSaturation")),    String(F("floorGainDur")),
      String(F("nudgeGainDur")),   String(F("invalidPingDur")),
      String(F("nudgeGainValue")), String(F("validPingStart")),
      String(F("validPingEnd")),   String(F("gain")),
      String(F("peakLevel")),      String(F("pingStatus")),
      String(F("centerFreq")),     String(F("debug")),
      String(F("adcAveraging")),   String(F("validMean")),
      String(F("validVariance"))};

  // bool testLED;
  char trashchar;
};

#endif
