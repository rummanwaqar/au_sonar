#ifndef _GAINCONTROL_H_
#define _GAINCONTROL_H_

#include <Arduino.h>

#include "peak_detector.h"
#include "Filter.h"
#include "Amplifier.h"
#include "pinout.h"
#include "constants.h"

class GainControl : public PeakDetector{

    public:

        GainControl( void );

        //the gain control runtime. Have this Free-running in loop()
        void runtime( void );

        ////////////////////////////////////////////////
        //HELPER Methods
        ////////////////////////////////////////////////

        //Set the desired peak to peak value for the ping.
        void setDesiredPeak2Peak( float input ); //Input in Vpp

        //Set if gain should be held or not
        void setHoldGainFlag( bool flagSetting );

        //Set the proportional gain
        void setProportionalGain( float gain );

        //Set the integral gain
        void setIntegralGain( float gain );

        //Set the limits of the integral
        void setIntegralSaturation( float absBound );
        void setIntegralSaturation( float lowerBound, float upperBound );

        //Set the time the gain should be floored after reading the ping (ms)
        void setFloorGainDuration( unsigned int duration_ms );

        //Set the time the gain should be nudged after invalid pings (ms)
        void setNudgeGainDuration( unsigned int duration_ms );

        //Set the time to print debug info after invalid pings (ms)
        void setInvalidPingDuration( unsigned int duration_ms );

        //Method to set the debug flag Setting
        void setDebugFlag( bool set );

        //Method to manually set gain
        void setGain( float gain );

        //Method to set the valid mean for calibration
        void setValidMean( float _validMean );

        //Method to set the valid variance for calibration
        void setValidVariance( float _validVariance);

        //Set the amount the gain will be nudged after the nudge gain duration (dB)
        void setNudgeGainValue( float gain );

        //Get the current desired peak of the controller
        float getDesiredPeak( void );

        //Get the current status of hold gain
        bool getHoldGain( void );

        //Get the current "optimized" gain in dB as calculated by the Controller
        float getCurrentOptimalGain( void );

        //Get the value of the iSaturation, returns iErrorMax
        float getISaturation( void );

        //Get the current "Peak Level" in Volts as calculated by the peak detector
        float getCurrentPeakLevel( void );

        //Get the current ping status;
        //0=No peak 1=Active Peak 2=End of Active Peak 3=Rise of Peak
        int getCurrentPingStatus( void );

        //Get the current proportional gain value
        float getProportionalGain( void );

        //Get the current integral gain value
        float getIntegralGain( void );

        //Check if the gain is calibrated currently
        bool checkCalibration( void );

        //Get the valid mean
        float getValidMean( void );

        //Get the valid variance
        float getValidVariance( void );


    //Methods and variables that "Black Box" users should not be conserned with.
    private:

        //Controller that incorperates a PI controller.
        void gainController( void );
        //sums the error over time
        void getIntegralError( void );
        //Keeps gain between 0dB and 60dB
        void saturateGain( void );

        //Variables for the gain controller
        float pGain; //proportional gain
        float iGain; //integral gain
        float iErrorMax; //Max value of the integral
        float iErrorMin; //Min value of the integral
        float optimalGain; //The optimal gain found by optimizeGain
        float peakLevel; //the output of the peak detector
        float desiredPeak; //the desired peak-peak value for the controller
        float error;  // error for the proportional gain
        float sumError; // error for the integral (summation) gain

        void calculateVariance( void );
        double averagePeakLevel; //Variable to get the average peakLevel of the signal
        uint16_t averagePeakLevelCounter;
        double averagePeakLevelSquared;
        uint16_t averagePeakLevelSquaredCounter;
        double variance;

        float validMean;
        float validVariance;

        //Returns the time elasped since calling millis on a 'timer'. A simple Helper function
        unsigned long elapsedTime( unsigned long millisTimer );

        //Variables for the runtime
        uint8_t pingStatus;  //Current status of the ping
        uint8_t previousPing;
        //Timers
        unsigned long floorGainTimer; //The timer to track how long gain has been floored
        unsigned long invalidPingTimer; //Timer to track how long gain has been invalid for
        unsigned long nudgeTimer; //Timer to track when to nudge the gain upwards
        unsigned long ledTimer; //Timer to keep the LED long enough for us to see ping
        unsigned long serialPingRecievedTimer; //timer to keep serial from spamming
        unsigned int floorGainDuration; //Time to floor gain after valid ping
        unsigned int nudgeGainDuration; //Time to nudge gain if invalid ping
        unsigned int invalidPingDuration; //Time to print debug information after invalid ping
        float nudgeGainValue; //The amount the gain will be nudged upwards after invalid ping durations

        //Flags
        bool gainFlooredFlag; //Flag for identifing if gain is currently floored
        bool holdGainFlag; //Flag to say the beagle bone wants gain to be held
        bool debugFlag; //Flag to say the device is in debug mode
        bool ledSetFlag; //Flag to say that the led was set
};

#endif
