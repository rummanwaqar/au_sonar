#include "CommInterface.h"

CommInterface::CommInterface( void ){

    Serial.begin(115200);
    Serial.setTimeout(1000);

    messageReceived = false;
    debugFlag = false;
    //testLED = false;
}

void CommInterface::parseMessage( void ){

    //Check Serial for messages. Save message and set messageRecievedFlag flag to true
    // ---> Moved to serialInterrupt
    //this->CommInterface::checkSerial();

    //If serial picked up a string that starts with $ and ends with \n we being parse
    if (messageReceived){
        //Try to get the command from the string
        this->CommInterface::getCommand();
        //If failure it'll set messageReceived to zero
    }

    //If we got the command, lets try to get the variable
    if (messageReceived){
        //Try to get the variable from the string
        this->CommInterface::getVariable();
        //If failure it'll set messageReceived to zero
    }

    //If we got the command and variable, lets get the argument if needed
    if (messageReceived){
        //commandIndex 0 is a "set" command, so we need an arguement
        //commandIndex 1 is a "get" command, so we do not need an argument
        if ( commandIndex == 0 ){
            this->CommInterface::getArgument();
        }
    }

    //Message should be parsed. During parsing message could have been invalid
    //So we check messageReceived again to make sure parsing was successful
    if (messageReceived){
        //if debug enabled print information about the command
        if (debugFlag){
            Serial.print(F("Command: "));
            Serial.println(commands[commandIndex]);
            Serial.print(F("Variable: "));
            Serial.println(variables[variableIndex]);

            //CommandIndex = 0 is a "set" command so it would have arguments
            //CommandIndex = 1 is a "get" command so it would NOT have arguments
            if ( commandIndex == 0){
                Serial.print(F("Arguments: "));
                Serial.print(argument1String);
                Serial.print(" ");
                Serial.println(argument2String);
            }

            //Wait untill information is transferred.
            Serial.flush();

        }

        //We should have command, variable, and argument (if "set")
        //So if commandIndex = 1 ("set") then we apply the argument to the variable
        //If commandIndex = 0 ("get") we then grab the local variable and then send it
        if ( commandIndex == 0){

            this->CommInterface::setValue();

        } else if ( commandIndex == 1){

            //sends the value of the selected variable using a get command.
            //message begins with a '$' and ends with a '\n'
            this->CommInterface::sendLocalVariable();
        }

    }

    //Easy cleanup method.
    this->CommInterface::cleanup();
}

//This method checks if the incomming message is valid
void CommInterface::checkSerial( void ){

    messageReceived = false;

    //Message should start with '$' and end with '\n'. Max is 31 characters
    message = Serial.readStringUntil('\n', 32);

    if ((message[0] == '$') && (message.length() >= 4) && (message.length() < 32)){
        //Echo the successful message transfer
        Serial.println(message);
        messageReceived = true;
    } else {
        message = String("");
    }

    //Serial.readStringUntil doesn't clear the buffer completely
    //So this while loop will clear the transmission queue
    while(Serial.available()){
        //Get the trash (ie new line character)
        trashchar = Serial.read();
    }

}

//Get the reference to the gain control object so we can access its variables
void CommInterface::getGainControlPointer(GainControl & _gainControl){
    gainControl = &_gainControl;
}

//Get the reference to the filter object so we can access its variables
void CommInterface::getFilterPointer( Filter & _filter ){
    filter = &_filter;
}

//This method attempts to get the command from the input message string
void CommInterface::getCommand( void ){

    //Loop through all the commands
    for (uint8_t i = 0; i < NUMBER_COMMANDS ; i++ ){
        //if the command length + 1 is larger than the whole message, the obviously
        //that is not the right command, goto the next one
        //This also prevents an indexing outside of the string
        if (commands[i].length()+1 > message.length() ){
            continue;
        }
        //Look for the command inside of the message
        //substring(in, end) returns the string from indexes in to end
        //There is a '$' at the start so we have to offset indexes by 1
        if ( message.substring(1, 1 + commands[i].length() ) == commands[i] ){
            //save the command index. This will be used to identify what the command was
            commandIndex = i;
            messageReceived = true;

            //Remove the command from the message including the '$' (the +1)
            message.remove(0, commands[i].length() + 1);
            return;
        }
    }
    //If no command match then obviously the message was not a valid one
    messageReceived = false;
    message = String("");
}

//Attempt to get the variable from the message
void CommInterface::getVariable( void ){

    //Loop through all the variables
    for (uint8_t i = 0; i < NUMBER_VARIABLES; i++ ){
        //if the variable length + 1 is larger than the whole message, then obviously
        //that is not the right variable, goto the next one
        //Keep in mind that the command portion of message was deleted at the end of the getCommand method
        //Doing this also prevents an indexing outside of the string
        if (variables[i].length()+1 > message.length() ){
            continue;
        }
        //Look for the variable inside of the message
        //substring(in, end) returns the string from indexes in to end
        //There is a ' ' at the start so we have to offset indexes by 1
        if ( message.substring(1, 1 + variables[i].length() ) == variables[i] ){
            variableIndex = i;
            messageReceived = true;

            //Remove the variable from the message including the ' ' (the +1)
            message.remove(0, variables[i].length() + 1);

            return;
        }
    }
    //If no variable match then obviously the message was not a valid one
    messageReceived = false;
    message = String("");

}

//if an argument is required then this method attempts to get it
void CommInterface::getArgument(){

    //So far the command and the variable have been removed from message
    //but there should be still a ' ' before the argument
    if ( message[0] == ' ' ){
        message.remove(0,1);
    } else {
        //If there is no space before the argument then the message was invalid
        message = String("");
        messageReceived = false;
        return;
    }

    //There could be a second argument if variable is "iSaturation"
    //We could just check the variable index but I want to be able to
    //add more variables that need muli arguments in the future
    argument1Length = message.indexOf(' ');

    //if argument1Length is < 0 then there was no ' ' found
    //argument1length is 8 bit unsigned so if it's less than 0 it would be actually be around 255
    //The max size of message is 32 so if we just check if its larger than that then obviously
    //the index for a ' ' was not found
    if (argument1Length > 32){

        //if the variable was "iSaturation", then there should have been a ' '. Invalid if so.
        if( variableIndex == 4){
            messageReceived = false;
            message = String("");
            return;
        }

        argument1Length = 0;
        //Go through the argument and get the length of it
        //This should reject any non numerical characters at the end of the message
        for(uint8_t i = 0; i < message.length() ; i++){
            if ( (isDigit(message[i])) || (message[i] == '.') || (message[i] == '-') ){
                argument1Length = i + 1;
                continue;
            } else {
                argument1Length = i;
                break;
            }
        }

        //Indexing protection
        if ( argument1Length > message.length() ){
            argument1Length = message.length();
        }
        //Remove the crap
        message.remove(argument1Length);
        //Get the string
        argument1String = String(message);

    //This else is for the case of two arguments -> a space was found after argument1
    } else {

        //Currently only "iSaturation" requires two arguments
        if ( variableIndex == 4 ){

            //Do the same as above but we're going to append the argument character by character
            argument1String = String("");
            //Keep in mind that the argument1Length is actually the index of where argument1 ends in message
            for(uint8_t i = 0; i < argument1Length ; i++){
                if ( (isDigit(message[i]) || (message[i] == '.') || (message[i] == '-') )){
                    argument1String.append( message[i] );
                    continue;
                } else {
                    //There is no tolerance here for junk characters as there is still another argument
                    //Thus if there are junk characters this is most definitely invalid
                    messageReceived = false;
                    argument1String = String("");
                    message = String("");
                    return;
                }

            }
            //Once the argument1String is built we will remove it from the message
            message.remove(0, argument1String.length());
            //Get the second argument
            this->CommInterface::getSecondArgument();

        //if there was space found after the argument1 and its not a variable that needs two arguments...
        //message was invalid
        } else {
            messageReceived = false;
            message = String("");
            return;
        }
    }
}

//This method is called when a second argument is to be gathered from message
void CommInterface::getSecondArgument( void ){

    //There should be a space at the start of message left over from getArgument otherwise invalid
    //Remove this space as its not needed.
    if( message[0] == ' ' ){
        message.remove(0, 1);
    } else {
        message = String("");
        argument1String = String("");
        messageReceived = false;
        return;
    }

    //This is the same algorithm as in getArgument, in the case where a space is not found
    //Because this should be the end of the message string
    argument2Length = 0;
    for(uint8_t i = 0; i < message.length() ; i++){
        if ( (isDigit(message[i]) || (message[i] == '.') || (message[i] == '-') )){
            argument2Length = i + 1;
            continue;
        } else {
            argument2Length = i;
            break;
        }
    }

    //Index protection
    if ( argument2Length > message.length() ){
        argument2Length = message.length();
    }

    //Cut the crap
    message.remove(argument2Length);
    argument2String = String("");
    //retrieve message
    argument2String = String(message);

}

//This method applies the arguments to their variables if a set command was used
void CommInterface::setValue( void ){

    output_float = argument1String.toFloat();
    output_int = argument1String.toInt();

    //Variable 4 is "iSaturation", which uses 2nd argument
    if (variableIndex == 4 ){
        output2_float = argument2String.toFloat();
    }

    switch(variableIndex){
        case 0:
            gainControl->setDesiredPeak2Peak(output_float);
            break;
        case 1:
            gainControl->setHoldGainFlag(output_int);
            break;
        case 2:
            gainControl->setProportionalGain(output_float);
            break;
        case 3:
            gainControl->setIntegralGain(output_float );
            break;
        case 4:
            gainControl->setIntegralSaturation(output_float, output2_float);
            break;
        case 5:
            gainControl->setFloorGainDuration(output_int);
            break;
        case 6:
            gainControl->setNudgeGainDuration(output_int);
            break;
        case 7:
            gainControl->setInvalidPingDuration(output_int);
            break;
        case 8:
            gainControl->setNudgeGainValue(output_float);
            break;
        case 9:
            gainControl->setPingValidStart(output_int);
            break;
        case 10:
            gainControl->setPingValidEnd(output_int);
            break;
        case 11:
            gainControl->setGain(output_float);
            break;
        case 14:
            filter->setCenterFreq(output_int);
            break;
        case 15:
            gainControl->setDebugFlag(output_int);
            debugFlag = (bool) output_int;
            break;
        case 16:
            gainControl->setADCAveraging(output_int);
            break;
        default:
            //Do nothing
            break;
    }

}

//This method retrieves the values of the selected variable using a get command.
//Variable is sent over Serial starting with a '$'
void CommInterface::sendLocalVariable( void ){

    switch(variableIndex){
        case 0:
            output_float = gainControl->getDesiredPeak();
            break;
        case 2:
            output_float = gainControl->getProportionalGain();
            break;
        case 3:
            output_float = gainControl->getIntegralGain();
            break;
        case 11:
            output_float = gainControl->getCurrentOptimalGain();
            break;
        case 12:
            output_float = gainControl->getCurrentPeakLevel();
            break;
        case 13:
            output_int = gainControl->getCurrentPingStatus();
            break;
        case 14:
            output_int = filter->getCenterFreq();
            break;
        case 16:
            output_int = gainControl->getADCAveraging();
        default:
            //Do nothing
            break;

    }

    this->CommInterface::prepareTransmission();

}

bool CommInterface::validIncommingSerialMessage( void ){
    return messageReceived;
}

//Prepares value for transmission. ie: float or int then adds a '$' to the start of message
void CommInterface::prepareTransmission( void ){

    //variable index 13 and 14 are pingStatus and centerFreq respectively
    if ( (variableIndex == 13) || (variableIndex == 14 || (variableIndex == 16)) ){
        Serial.print("$");
        Serial.println(output_int);
    } else {
        Serial.print("$");
        Serial.println(output_float);
    }

    Serial.flush();
}

//Simple cleanup method.
void CommInterface::cleanup( void ){

    messageReceived = false;

    message = String("");
    argument1String = String("");
    argument2String = String("");

}
