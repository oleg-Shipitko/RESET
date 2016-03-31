#include "Manipulators.h"
#include "Dynamixel_control.h"


void soft_delay(long int ticks)
{
    for(; ticks > 0; ticks-- );
}
    uint16_t stVal = 0;
    uint16_t finalVal = 300;
    uint16_t curLoad = 0;

///////////////////////////SEASHELLS///////////////////////////
void closeDoors()
{
    bool flag = false;
    setServoTorque(DOORS_ID, 500);
    setServoCWAngleLimit(DOORS_ID, 0);
    setServoCCWAngleLimit(DOORS_ID, 0);
    setServoMovingSpeed(DOORS_ID, 1023, CCW);
    while(!flag)
    {
        getCurrentLoad(DOORS_ID, &curLoad);
        if (curLoad > 600)
        {
            setServoMovingSpeed(DOORS_ID, 0, CCW);
            flag = true;
        }
    }
}

void liftSeashell()
{
    bool flag = false;
    setServoTorque(LIFT_ID, 500);
    setServoCWAngleLimit(LIFT_ID, 0);
    setServoCCWAngleLimit(LIFT_ID, 0);
    setServoMovingSpeed(LIFT_ID, 1023, CCW);
    setServoMovingSpeed(LIFT_ID, 0, CW);
    while(!flag)
    {
        getCurrentLoad(LIFT_ID, &curLoad);
        if (curLoad > 600)
        {
            curLoad = 0;
            setServoMovingSpeed(LIFT_ID, 1023, CW);
            soft_delay(2000000);
            soft_delay(2000000);
            soft_delay(2000000);
            soft_delay(2000000);
            while(!flag)
            {
                getCurrentLoad(LIFT_ID, &curLoad);
                if (curLoad > 600)
                {
                    setServoMovingSpeed(LIFT_ID, 0, CCW);
                    setServoMovingSpeed(LIFT_ID, 0, CW);
                    flag = true;
                }

            }
        }
    }
}
///////////////////////////////////////////////////////////////
