#ifndef _FILTER_H_
#define _FILTER_H_

#include <Arduino.h>
#include <Adafruit_SI5351.h>

#include "constants.h"

class Filter {
    public:
        Filter( void );

        //Sets the center frequency of the bandpass filter
        void setCenterFreq( int centerFreq );

        //Disable frequency output;
        void disableOutput( void );

        //Retrieve the current filtering frequency in kHz
        int getCenterFreq( void );

    private:
        //Adafruit_SI5351 library object
        Adafruit_SI5351 clockgen;

        //Center frequency in kHz
        int _centerFreq;

        //MultiSynth Config variables
        int div; //Whole number divisor for the MultiSynth (can be 1-900)
        int dem; //When dividing by fractional scaler this is the demoninator
        int num; //When dividing by fractional scaler this is the numerator

        //PLL config variable (can be 24 to 36)
        int PLLMult; //This gets multiplied by 25MHz, which is the crystal frequency

        //This is rDiv, can be 1,2,4,6,...,128
        //rDiv devides the final output
        si5351RDiv_t rDiv; //Must use the struct format. See the library declaration

        ///////////////////////////////////////////
        //EXAMPLE OF CONFIGURATION FOR 25KHz FILTER
        ///////////////////////////////////////////

        /*

        //Crystal Frequency is 25MHz.
        //Ratio of f_Clk / f_filter is 20. So 20 * 25KHz = 500kHz
        //So we need the clock gen to output 500kHz so the filter center freq is 25kHz

        PLLMult = 36;
        //Now the frequency is 25MHz * 36 = 900MHz

        div = 450
        //Now the frequency is 900MHz / 450 = 2MHz

        //Since we're not using a fractional scaler for the MultiSynth, make fraction 0
        num = 0;
        dem = 1;

        rDiv = SI5351_R_DIV_4;
        //Now the frequency is 2MHz / 4 = 500kHz!!! This is what we want!
        //rDiv is using a struct with definitions, thats you use SI5351_R_DIV_X
        //Where X is the divisor.

        */

        //This method gets these values for us with pre made cases
        void getClkScaler( void );
        //For when we do not have a case, a fractional Multisynth divisor will be
        //generated with this method
        void getFractionalScaler( void );

};


#endif
