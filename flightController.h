#include "mbed.h"
#include "rtos.h"
#include "hardware.h"

//Declarations
void GetAttitude();
void FlightControllerTask(void const *n);
void NotFlying();

//Variables
int             _notFlying = 0; 

//Timers
RtosTimer       *_flightControllerUpdateTimer;

// A thread to control flight
void FlightControllerThread(void const *args) 
{    
    printf("Flight controller thread started\r\n");
     
    //Update Timer
    _flightControllerUpdateTimer = new RtosTimer(FlightControllerTask, osTimerPeriodic, (void *)0);
    int updateTime = (1.0 / FLIGHT_CONTROLLER_FREQUENCY) * 1000;
    _flightControllerUpdateTimer->start(updateTime);
    
    // Wait here forever
    Thread::wait(osWaitForever);
}

void FlightControllerTask(void const *n)
{
    //Get IMU data
    GetAttitude();
    
    //Rate mode
    if(_rate == true && _stab == false)
    {
        //Update rate PID process value with gyro rate
        _yawRatePIDController->setProcessValue(_gyroRate[0]);
        _pitchRatePIDController->setProcessValue(_gyroRate[1]);
        _rollRatePIDController->setProcessValue(_gyroRate[2]);
        
        //Update rate PID set point with desired rate from RC
        _yawRatePIDController->setSetPoint(_rcMappedCommands[0]);
        _pitchRatePIDController->setSetPoint(_rcMappedCommands[1]);
        _rollRatePIDController->setSetPoint(_rcMappedCommands[2]);
        
        //Compute rate PID outputs
        _ratePIDControllerOutputs[0] = _yawRatePIDController->compute();
        _ratePIDControllerOutputs[1] = _pitchRatePIDController->compute();
        _ratePIDControllerOutputs[2] = _rollRatePIDController->compute();
        
        //Set stability PID outputs to 0
        _stabPIDControllerOutputs[0] = 0;
        _stabPIDControllerOutputs[1] = 0;
        _stabPIDControllerOutputs[2] = 0;
    }
    //Stability mode
    else if(_rate == false && _stab == true)
    {
        //Update stab PID process value with ypr
        _yawStabPIDController->setProcessValue(_ypr[0]);
        _pitchStabPIDController->setProcessValue(_ypr[1]);
        _rollStabPIDController->setProcessValue(_ypr[2]);
        
        //Update stab PID set point with desired angle from RC
        _yawStabPIDController->setSetPoint(_yawTarget);
        _pitchStabPIDController->setSetPoint(_rcMappedCommands[1]);
        _rollStabPIDController->setSetPoint(_rcMappedCommands[2]);
        
        //Compute stab PID outputs
        _stabPIDControllerOutputs[0] = _yawStabPIDController->compute();
        _stabPIDControllerOutputs[1] = _pitchStabPIDController->compute();
        _stabPIDControllerOutputs[2] = _rollStabPIDController->compute();
        
        //If pilot commanding yaw
        if(abs(_rcMappedCommands[0]) > 0)
        {  
            _stabPIDControllerOutputs[0] = _rcMappedCommands[0];  //Feed to rate PID (overwriting stab PID output)
            _yawTarget = _ypr[0];
        }
        
        //Update rate PID process value with gyro rate
        _yawRatePIDController->setProcessValue(_gyroRate[0]);
        _pitchRatePIDController->setProcessValue(_gyroRate[1]);
        _rollRatePIDController->setProcessValue(_gyroRate[2]);
        
        //Update rate PID set point with desired rate from stab PID
        _yawRatePIDController->setSetPoint(_stabPIDControllerOutputs[0]);
        _pitchRatePIDController->setSetPoint(_stabPIDControllerOutputs[1]);
        _rollRatePIDController->setSetPoint(_stabPIDControllerOutputs[2]);
        
        //Compute rate PID outputs
        _ratePIDControllerOutputs[0] = _yawRatePIDController->compute();
        _ratePIDControllerOutputs[1] = _pitchRatePIDController->compute();
        _ratePIDControllerOutputs[2] = _rollRatePIDController->compute();
    }
    
    //Testing
    //_ratePIDControllerOutputs[0] = 0; // yaw
    //_ratePIDControllerOutputs[1] = 0; // pitch
    //_ratePIDControllerOutputs[2] = 0; // roll
    //_stabPIDControllerOutputs[0] = 0; // yaw
    //_stabPIDControllerOutputs[1] = 0; // pitch
    //_stabPIDControllerOutputs[2] = 0; // roll

    //Calculate motor power if flying
    //RC Mapped thottle is between 0 and 1
    //Add 0.2 to try to avoid false starts
    if(_rcMappedCommands[3] > (RC_THRUST_MIN + 0.2) && _armed == true)
    {
        //Calculate base power to apply from throttle - returns 1060 at min, 1860 at max
        float basePower = MOTORS_MIN + (_rcMappedCommands[3] * 800);
        
        //Map motor power - each PID returns -100 <-> 100
        _motorPower[0] = basePower + _ratePIDControllerOutputs[1] + _ratePIDControllerOutputs[2] + _ratePIDControllerOutputs[0];
        _motorPower[1] = basePower + _ratePIDControllerOutputs[1] - _ratePIDControllerOutputs[2] - _ratePIDControllerOutputs[0];
        _motorPower[2] = basePower - _ratePIDControllerOutputs[1] - _ratePIDControllerOutputs[2] + _ratePIDControllerOutputs[0];
        _motorPower[3] = basePower - _ratePIDControllerOutputs[1] + _ratePIDControllerOutputs[2] - _ratePIDControllerOutputs[0];
        
        //Check motor power is within limits - if not add/remove constant to all motors to keep motor ratio the same
        float motorFix = 0;
        float motorMin = _motorPower[0];
        float motorMax = _motorPower[0];
        
        for(int i=1; i<4; i++)
        {
            if(_motorPower[i] < motorMin) motorMin = _motorPower[i];
            if(_motorPower[i] > motorMax) motorMax = _motorPower[i];
        }
               
        //Check if min or max is outside of the limits
        if(motorMin < MOTORS_MIN) motorFix = MOTORS_MIN - motorMin;
        else if(motorMax > MOTORS_MAX) motorFix = MOTORS_MAX - motorMax;
        
        //Add/remove constant if neccessary
        for(int i=0; i<4; i++)
        {
            _motorPower[i] = _motorPower[i] + motorFix;
        }
    }

    //Not flying
    else if(_armed == true)
    {
        _yawTarget = _ypr[0];
        
        //Set motors to armed state
        _motorPower[0] = MOTORS_ARMED;
        _motorPower[1] = MOTORS_ARMED;
        _motorPower[2] = MOTORS_ARMED;
        _motorPower[3] = MOTORS_ARMED;
        
        _notFlying ++;
        if(_notFlying > 500) //Not flying for 1 second
        {
            NotFlying();
        }
    } 
    else
    {
        //Disable Motors
        _motorPower[0] = MOTORS_OFF;
        _motorPower[1] = MOTORS_OFF;
        _motorPower[2] = MOTORS_OFF;
        _motorPower[3] = MOTORS_OFF;
        
        _notFlying ++;
        if(_notFlying > 500) //Not flying for 1 second
        {
            NotFlying();
        }
    }
    
    //Set motor power
    _motor1.pulsewidth_us(_motorPower[0]);
    _motor2.pulsewidth_us(_motorPower[1]);
    _motor3.pulsewidth_us(_motorPower[2]);
    _motor4.pulsewidth_us(_motorPower[3]);
}

void GetAttitude()
{
    
    //Get raw data from IMU
    _freeIMU.getYawPitchRoll(_ypr);
    _freeIMU.getRate(_gyroRate);
    
    //Take off zero values to account for any angle inbetween level and ground
    //_ypr[1] = _ypr[1] - _zeroValues[1];
    //_ypr[2] = _ypr[2] - _zeroValues[2];
    
    //Swap pitch and roll angle because IMU is mounted at a right angle to the board
    float pitch = _ypr[2];
    float roll = -_ypr[1];
    _ypr[1] = pitch;
    _ypr[2] = roll;
    
    _ypr[0] = _ypr[0];
    
    //Swap pitch, roll and yaw rate because IMU is mounted at a right angle to the board
    float yaw = _gyroRate[2];
    pitch = _gyroRate[0];
    roll = _gyroRate[1];
    _gyroRate[0] = yaw;
    _gyroRate[1] = pitch;
    _gyroRate[2] = roll;
}

void NotFlying()
{
    //Reset iteratior
    _notFlying = 0;
    
    //Zero gyro
    _freeIMU.zeroGyro();
    
    //Reset I
    _yawRatePIDController->reset();
    _pitchRatePIDController->reset();
    _rollRatePIDController->reset();
    _yawStabPIDController->reset();
    _pitchStabPIDController->reset();
    _rollStabPIDController->reset();
}