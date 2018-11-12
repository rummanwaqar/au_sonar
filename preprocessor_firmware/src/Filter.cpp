#include "Filter.h"

//Initializes the Adafruit_SI5351 library
Filter::Filter( void ) {

    clockgen = Adafruit_SI5351();
    clockgen.begin();
}

//Sets the center frequency (in kHz) of the bandpass filter
void Filter::setCenterFreq( int centerFreq ){

    //Store the input into the object.
    _centerFreq = centerFreq;

    //We need to find values to scale the clocks by so that they give us
    //the exact frequency that we need
    this->Filter::getClkScaler();

    //There are two PLLs, PLL_A and PLL_B
    //The PLL goes from 600MHz to 900MHz. The lowest is 600MHz
    //The PLL clock = 25MHz * ScaleFactor => ScaleFactor = 24 to get 600MHz
    clockgen.setupPLL(SI5351_PLL_A, PLLMult, 0, 1);

    //The 600MHz from the PLL will be further devided by a "Multisynth"
    //There are three Multisynths which we only to use one of.
    clockgen.setupMultisynth(CLK2, SI5351_PLL_A, div, num, dem);

    //There can further division using the Rdev. We're deviding by 64.
    //So if you were to take that 600MHz from the PLL and devide it by 128 you
    //get 4687.5kHz
    clockgen.setupRdiv(CLK2, rDiv);

    //Enable the clock outputs
    clockgen.enableOutputs(true);
}

//These well get the scale factors to get the center frequency we need.
//Numbers gathered by simply plugging in random numbers and seeing which
//ones gave even values.
void Filter::getClkScaler( void ){

    //Pre made cases that do not rely on a fractional scaler
    switch(_centerFreq){
        case 20:
            div = 750;
            rDiv = SI5351_R_DIV_2;
            PLLMult = 24;
            break;
        case 25:
            div = 450;
            rDiv = SI5351_R_DIV_4;
            PLLMult = 36;
            break;
        case 27:
            div = 625;
            rDiv = SI5351_R_DIV_2;
            PLLMult = 27;
            break;
        case 30:
            div = 375;
            rDiv = SI5351_R_DIV_4;
            PLLMult = 36;
            break;
        case 35:
            div = 125;
            rDiv = SI5351_R_DIV_8;
            PLLMult = 28;
            break;
        case 40:
            div = 875;
            rDiv = SI5351_R_DIV_1;
            PLLMult = 28;
            break;
        case 45:
            div = 750;
            rDiv = SI5351_R_DIV_1;
            PLLMult = 27;
            break;
        case 50:
            div = 675;
            rDiv = SI5351_R_DIV_1;
            PLLMult = 27;
            break;
        default:
            //No case for this frequency so will generate a fractional scaler
            this->Filter::getFractionalScaler();
            return;

    }

    //No fractional component to the scaler.
    num = 0;
    dem = 1;

}

//This works continuously from 1kHz to 234kHz of input
void Filter::getFractionalScaler( void ){

    //This will give us a freq of 300MHz after PLL and rDiv
    PLLMult = 24;
    rDiv = SI5351_R_DIV_2;

    //The ratio of f_clk/f_center of the filter is 20
    double requiredFreq = 20.0 * _centerFreq;

    //This is the amount we need to devide by. 300000 is the 300MHz
    double scaleFactorDecimal = 300000 / requiredFreq ;

    //chop off the decimal component to get the whole number
    div = (int) scaleFactorDecimal;

    //get the decimalComponent and convert to fraction
    const float precision = 10000.0;  //limited to 20 bits.
    scaleFactorDecimal = scaleFactorDecimal - div;
    int decimalComponent = (int) (scaleFactorDecimal * precision);

    dem = precision;
    num = decimalComponent;

}

int Filter::getCenterFreq( void ){
    return _centerFreq;
}

//Disables the frequency output
void Filter::disableOutput( void ){

    clockgen.enableOutputs(false);
}
