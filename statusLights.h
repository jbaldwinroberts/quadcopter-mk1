#include "mbed.h"
#include "rtos.h"
#include "hardware.h"

// The status thread indicates the current system status to the user
void StatusThread(void const *args) 
{
    printf("Status lights thread started\r\n");
    
    int ledState = 0;
    while (true) 
    {
        ledState++;
        if (ledState > 5) { ledState = 0; }
        
        _led1 = (ledState == 0);
        _led2 = (ledState == 1 || ledState == 5);
        _led3 = (ledState == 2 || ledState == 4);
        _led4 = (ledState == 3);
        
        Thread::wait(100);
    }
}
