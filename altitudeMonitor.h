#include "mbed.h"
#include "rtos.h"
#include "hardware.h"

// The altitude monitor thread gets the latest altitude from each sensor and combines into one altitude
void AltitudeMonitorThread(void const *args) 
{
    printf("Altitude monitor thread started\r\n");

    while (true) 
    {
        _maxBotixPingAltitude = _maxBotixSensor.read() / 100; // Convert to meters
        _barometerAltitude = 0;//_freeIMU.getBaroAlt();
        
        
        Thread::wait(100);
    }
}
