#include "Manipulators.h"
#include "Dynamixel_control.h"


void soft_delay(unsigned long int ticks)
{
    for(; ticks > 0; ticks--);
}

///////////////////////////TOWER BUILDER///////////////////////

bool openTower(int8_t ID)               // Открыть манипулятор
{
    setServoAngle(ID, OPEN_ANG);
    return 0;
}

bool closeTower(int8_t ID)              // Закрыть манипулятор
{
    setServoAngle(ID, CLOSED_ANG);
    return 0;
}
///////////////////////////////////////////////////////////////

///////////////////////////CUBES CATCHER///////////////////////

bool openCubes()
{
    setServoAngle(ID_RIGHT, OPEN_ANG_RIGHT);
    setServoAngle(ID_LEFT, OPEN_ANG_LEFT);

    return 0;
}

bool closeCubes(int8_t *numberOfCubesCatched)
{
    float right_servo_angle, left_servo_angle, prev_right_servo_angle, prev_left_servo_angle;

    setServoTorque(ID_RIGHT, 500);
    setServoTorque(ID_LEFT, 500);
    setServoAngle(ID_RIGHT, CLOSED_ANG_RIGHT);
    setServoAngle(ID_LEFT, CLOSED_ANG_LEFT);
    soft_delay(1000);
    while((prev_right_servo_angle != right_servo_angle) && (prev_left_servo_angle != left_servo_angle))
    {
        getServoAngle(ID_RIGHT, &right_servo_angle);
        getServoAngle(ID_LEFT, &left_servo_angle);
        prev_right_servo_angle = right_servo_angle;
        prev_left_servo_angle = left_servo_angle;
    }
    if (right_servo_angle - left_servo_angle <= ONE_CUBE_CATCHED_ANGLE) // implement formula for calculating the difference
    {
        *numberOfCubesCatched = 1;
    }
    else if (right_servo_angle - left_servo_angle <= TWO_CUBES_CATCHED_ANGLE) // implement formula for calculating the difference
    {
        *numberOfCubesCatched = 2;
    }
    else
        *numberOfCubesCatched = 0;

    return 0;
}

///////////////////////////////////////////////////////////////

///////////////////////////PNEUMO//////////////////////////////

bool pneumoIn()
{

}

bool pneumoOut()
{

}

bool switchOnPneumo()
{

}

bool switchOffPneumo()
{

}



///////////////////////////////////////////////////////////////
