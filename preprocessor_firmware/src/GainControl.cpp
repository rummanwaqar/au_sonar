#include "GainControl.h"

// initialize the gain controller
GainControl::GainControl(void) {
  // setup DAC for Amplifier
  analogWriteResolution(12);

  // Starting value for the gain optimizer
  // DEFAULT_GAIN is set in GainControl.h
  optimalGain = DEFAULT_GAIN;

  // Convert the desired peak set in gainControl.h to something
  // the controller can use.
  this->GainControl::setDesiredPeak2Peak(DESIRED_PEAK);

  // Set initial proportional gain
  this->GainControl::setProportionalGain(P_GAIN);

  // Set initial integral gain
  this->GainControl::setIntegralGain(I_GAIN);

  // Setup saturation for the integral
  this->GainControl::setIntegralSaturation(-1 * MAX_I_ERROR, MAX_I_ERROR);

  // Set up the nudge gain duration time
  this->GainControl::setNudgeGainDuration(NUDGE_GAIN_DURATION);

  // Set up the floor gain duration time
  this->GainControl::setFloorGainDuration(FLOOR_GAIN_DURATION);

  // Set up the invalid ping duration time
  this->GainControl::setInvalidPingDuration(INVALID_PING_DURATION);

  // Set the default nudge gain value
  this->GainControl::setNudgeGainValue(NUDGE_VALUE);

  // Set up the validMean and validVariance
  this->GainControl::setValidMean(DEFAULT_VALIDMEAN);
  this->GainControl::setValidVariance(DEFAULT_VALIDVARIANCE);

  // Flag that indicates if we have purposely set the gain to 0dB
  // to kill the signal.
  gainFlooredFlag = false;

  // At initialization, gain will not be held
  holdGainFlag = false;

  // At initialization, assume no ping
  pingStatus = 0;
  previousPing = 0;

  // At initialization, assume debug is disabled
  debugFlag = false;

  serialPingRecievedTimer = millis();
}

// Organizes the controller.
void GainControl::runtime(void) {
  // If the gain is not being floored to 0dB, get the status of the ping
  if (gainFlooredFlag == false) {
    pingStatus = this->GainControl::getPingStatus(peakLevel);
  }

  // If some time has elasped after first identification of the ping
  // and the the gain is not currently being floored....
  // run a iteration of the controller
  if ((pingStatus == 1) && (gainFlooredFlag == false)) {
    // Run an iteration of the gain controller
    // This is the bread an butter of this class
    if (holdGainFlag == false) {
      this->GainControl::gainController();
    }

    // As clearly there is a valid ping, reset the invalid ping Timer
    // invalidPingTimer = millis();
    nudgeTimer = millis();
  }

  // If the ping is complete and the gain is not being floored (to 0dB)
  // Print the peaklevel, and the gain applied. Then floor the gain (to 0dB)
  // for a period of time. See GainControl.h
  /*
  if( (pingStatus==2) && (gainFlooredFlag==false) && (debugFlag==true)){

      //Print debug information

      Serial.print(F("VALID Pk V: "));
      Serial.print( peakLevel );
      Serial.print(F(" Gain: "));
      Serial.println( optimalGain );
      Serial.flush();

  }
  */

  if (pingStatus == 2) {
    if (gainFlooredFlag == false) {
      setAmplifierGain(0);  // floor the gain to 0dB

      // reset the floor gain timer
      floorGainTimer = millis();
      nudgeTimer = millis();
      // reset the invalid ping Timer
      // invalidPingTimer = millis();

      // Gain is floored. Apply the flag.
      gainFlooredFlag = true;
    }
  }

  // If the ping has been invalid for a certain time, and gain is not floored
  // nudge up the gain
  if (gainFlooredFlag == false) {
    if (pingStatus == 0) {
      if (this->GainControl::elapsedTime(nudgeTimer) > nudgeGainDuration) {
        // Nudge the gain up
        if (holdGainFlag == false) {
          optimalGain += nudgeGainValue;
        }

        // Keep gain between 0 and 60dB
        this->GainControl::saturateGain();

        // Apply the optimal gain as computed by the controller
        setAmplifierGain(optimalGain);

        // reset the nudgeTimer
        nudgeTimer = millis();
      }
    }
  }

  // if the ping has been invalid long enough, print out debug info
  // and reset the invalid ping timer
  /*
  if ( this->GainControl::elapsedTime(invalidPingTimer) > invalidPingDuration
                                                     && (gainFlooredFlag ==
  false)
                                                     && (pingStatus == 0)
                                                     && (debugFlag == true)){
      Serial.print(F("INVALID PING"));
      Serial.print(F(" Peak Voltage: "));
      Serial.print( peakLevel );
      Serial.print(F(" Applied Gain: "));
      Serial.println( optimalGain );
      Serial.flush();

      //reset the timer that controls when debug info prints.
      invalidPingTimer = millis();
  }
  */

  // Only floor the gain for a certain duration.
  if (gainFlooredFlag == true) {
    if (this->GainControl::elapsedTime(floorGainTimer) > floorGainDuration) {
      // lower the flag
      gainFlooredFlag = false;

      // apply the gain applied before previously flooring it.
      setAmplifierGain(optimalGain);
    }
  }

  // If there is no ping, or we are currently flooring the gain
  // reset the peak detector
  if ((pingStatus == 0) || (pingStatus == 2)) {
    resetDetector();
  }

  if ((previousPing == 0) && (pingStatus == 1) && (ledSetFlag == false)) {
    digitalWrite(BUILTIN_LED, HIGH);
    ledSetFlag = true;
    // Reset the ledTimer
    ledTimer = millis();
  }

  if (ledSetFlag == true) {
    if (this->GainControl::elapsedTime(ledTimer) > 200) {
      ledSetFlag = false;
      // Turn off LED
      digitalWrite(BUILTIN_LED, LOW);
    }
  }

  if ((pingStatus == 2) && (previousPing == 1)) {
    if (this->GainControl::elapsedTime(serialPingRecievedTimer) > 700) {
      serialPingRecievedTimer = millis();
      if (this->GainControl::checkCalibration()) {
        Serial1.println("$ping cal=1 gain=" + String(optimalGain) +
                       " peakLevel=" + String(peakLevel) +
                       " avgPkLv=" + String(averagePeakLevel) +
                       " variance=" + String(variance, 5) +
                       " samples=" + String(averagePeakLevelCounter));
      } else {
        Serial1.println("$ping cal=0 gain=" + String(optimalGain) +
                       " peakLevel=" + String(peakLevel) +
                       " avgPkLv=" + String(averagePeakLevel) +
                       " variance=" + String(variance, 5) +
                       " samples=" + String(averagePeakLevelCounter));
      }
      averagePeakLevelCounter = 0;
      averagePeakLevel = 0;

      averagePeakLevelSquaredCounter = 0;
      averagePeakLevelSquared = 0;

      Serial1.flush();
    }
  }

  previousPing = pingStatus;
}

void GainControl::gainController(void) {
  // This is the maximum peak value we will accept
  error = desiredPeak - peakLevel;

  // Sum the peakLevel so we can average it
  averagePeakLevel += peakLevel;
  averagePeakLevelCounter++;
  averagePeakLevelSquared += (peakLevel * peakLevel);
  averagePeakLevelSquaredCounter++;

  // get the integral error
  this->GainControl::getIntegralError();

  // Apply the proportional gain.
  optimalGain += pGain * error;
  // Apply the integral gain
  optimalGain += iGain * sumError;

  // Clip the gain between 0 and 60dB
  this->GainControl::saturateGain();

  // Apply Gain
  setAmplifierGain(optimalGain);
}

void GainControl::getIntegralError(void) {
  // The add the current error to the total sum of error
  sumError += error;

  // Staturate the sum of error to keep it from growing too large
  if (sumError > iErrorMax) {
    sumError = iErrorMax;
  }
  if (sumError < iErrorMin) {
    sumError = iErrorMin;
  }
}

void GainControl::saturateGain(void) {
  if (optimalGain > 60) {
    optimalGain = 60;
  }
  if (optimalGain < 0) {
    optimalGain = 0;
  }
}

void GainControl::calculateVariance(void) {
  variance = averagePeakLevelSquared - (averagePeakLevel * averagePeakLevel);
}

// Set desired peak to peak output in mV
void GainControl::setDesiredPeak2Peak(float input) {
  // devide by 2 then add the DC bias
  desiredPeak = input / 2;
  desiredPeak = desiredPeak + DC_BIAS;
}

// Returns the time elapsed since calling millis
unsigned long GainControl::elapsedTime(unsigned long millisTimer) {
  return (millis() - millisTimer);
}

bool GainControl::checkCalibration(void) {
  if (averagePeakLevelCounter != 0) {
    averagePeakLevel = averagePeakLevel / averagePeakLevelCounter;
  } else {
    averagePeakLevel = peakLevel;
  }

  if (averagePeakLevelSquaredCounter != 0) {
    averagePeakLevelSquared =
        averagePeakLevelSquared / averagePeakLevelSquaredCounter;
  } else {
    averagePeakLevelSquared = peakLevel * peakLevel;
  }

  this->GainControl::calculateVariance();

  if ((averagePeakLevel > (desiredPeak - validMean)) &&
      (averagePeakLevel < (desiredPeak + validMean)) &&
      (variance < validVariance) && (variance > (-1 * validVariance))) {
    if ((variance > 0.0001) || (variance < -0.0001)) {
      return true;
    } else {
      return false;
    }

  } else {
    return false;
  }
}

/////////////////////////////////////////////
// HELPER Methods
/////////////////////////////////////////////

void GainControl::setProportionalGain(float gain) { pGain = gain; }

void GainControl::setIntegralGain(float gain) { iGain = gain; }

void GainControl::setIntegralSaturation(float absBound) {
  iErrorMax = absBound;
  iErrorMin = -1 * absBound;
}

void GainControl::setIntegralSaturation(float lowerBound, float upperBound) {
  iErrorMax = upperBound;
  iErrorMin = lowerBound;
}

void GainControl::setFloorGainDuration(unsigned int duration_ms) {
  floorGainDuration = duration_ms;
}

void GainControl::setNudgeGainDuration(unsigned int duration_ms) {
  nudgeGainDuration = duration_ms;
}

void GainControl::setInvalidPingDuration(unsigned int duration_ms) {
  invalidPingDuration = duration_ms;
}

void GainControl::setNudgeGainValue(float gain) { nudgeGainValue = gain; }

void GainControl::setDebugFlag(bool set) { debugFlag = set; }

void GainControl::setGain(float gain) {
  optimalGain = gain;
  setAmplifierGain(gain);
}

void GainControl::setValidMean(float _validMean) { validMean = _validMean; }

void GainControl::setValidVariance(float _variance) { variance = _variance; }

float GainControl::getDesiredPeak(void) { return desiredPeak; }

bool GainControl::getHoldGain(void) { return holdGainFlag; }

float GainControl::getISaturation(void) { return iErrorMax; }

float GainControl::getCurrentOptimalGain(void) { return optimalGain; }

float GainControl::getCurrentPeakLevel(void) { return peakLevel; }

int GainControl::getCurrentPingStatus(void) { return pingStatus; }

void GainControl::setHoldGainFlag(bool flagSetting) {
  holdGainFlag = flagSetting;
}

float GainControl::getProportionalGain(void) { return pGain; }

float GainControl::getIntegralGain(void) { return iGain; }

float GainControl::getValidMean(void) { return validMean; }

float GainControl::getValidVariance(void) { return validVariance; }
