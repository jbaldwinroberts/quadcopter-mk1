#include "mbed.h"
#include "rtos.h"
#include "hardware.h"

//Variables
int i;

// A thread to get RC commands and convert to correct values
//Channel 1 is roll. min 1000. max 1900
//Channel 2 is pitch. min 1000. max 1900
//Channel 3 is throttle < 900 when not connected. min 1000. max 1900
//Channel 4 is yaw. min 1000. max 1900
//Channel 5 is arm. armed > 1800 else unarmed
//Channel 6 is mode. rate > 1800. stab < 1100
//Channel 7 is spare
//Channel 8 is spare
void RcCommandMonitorThread(void const *args) 
{    
    printf("RC command monitor thread started\r\n");
    
    //Set lost connection iterator to 0
    i = 0;
    
    //Main loop
    while(true)
    {
        //Get channel data - mapped to between 0 and 1
        _ppm->GetChannelData(_rcCommands);
        
        //Check whether transmitter is connected
        if(_rcCommands[2] != -1)
        {
            //Transmitter is connected so reset not connected iterator
            i = 0;
            
            //Map yaw channel
            _rcMappedCommands[0] = - _yawMedianFilter->process(Map(_rcCommands[3], RC_OUT_MIN, RC_OUT_MAX, RC_YAW_RATE_MIN, RC_YAW_RATE_MAX));
        
            //Map thust channel
            _rcMappedCommands[3] = _thrustMedianFilter->process(Map(_rcCommands[2], RC_OUT_MIN, RC_OUT_MAX, RC_THRUST_MIN, RC_THRUST_MAX));
        
            //Map arm channel.
            if(_rcCommands[4] > 0.5 && _armed == false) Arm();
            else if(_rcCommands[4] <= 0.5 && _armed == true)
            {
                Disarm();
            }
            
            //Map mode channel
            if(_rcCommands[5] < 0.5)
            {
                _stab = true;
                _rate = false;
            }
            else
            {
                _stab = false;
                _rate = true;
            }  
        
            //Roll and pitch mapping depends on the mode
            if(_rate == false && _stab == true)//Stability mode
            {
                //Roll
                _rcMappedCommands[2] = _rollMedianFilter->process(Map(_rcCommands[0], RC_OUT_MIN, RC_OUT_MAX, RC_ROLL_ANGLE_MIN, RC_ROLL_ANGLE_MAX));
                //Pitch
                _rcMappedCommands[1] = _pitchMedianFilter->process(-Map(_rcCommands[1], RC_OUT_MIN, RC_OUT_MAX, RC_PITCH_ANGLE_MIN, RC_PITCH_ANGLE_MAX)); //Needs to be reverse
            }
            else if(_rate == true && _stab == false)//Rate mode
            {
                //Roll
                _rcMappedCommands[2] = _rollMedianFilter->process(Map(_rcCommands[0], RC_OUT_MIN, RC_OUT_MAX, RC_ROLL_RATE_MIN, RC_ROLL_RATE_MAX));
                //Pitch
                _rcMappedCommands[1] = _pitchMedianFilter->process(-Map(_rcCommands[1], RC_OUT_MIN, RC_OUT_MAX, RC_PITCH_RATE_MIN, RC_PITCH_RATE_MAX)); //Needs to be reverse
            }
            else
            {
                _rcMappedCommands[1] = 0;
                _rcMappedCommands[2] = 0;
            }
        }
        else
        {
            //Transmitter not connected so increase iterator
            i++;
            
            //If connection has been down for 10 loops then assume the connection really is lost
            if(i > 10 && _armed == true) Disarm();
        }
        Thread::wait(20);
    }
}