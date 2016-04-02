#include "Manipulators.h"
#include "Dynamixel_control.h"


void soft_delay(long int ticks)
{
    for(; ticks > 0; ticks--);
}
    uint16_t stVal = 0;
    uint16_t finalVal = 300;
    uint16_t curLoad = 0;

///////////////////////////TOWER BUILDER///////////////////////

bool open_tower(int ID)               // Открыть манипулятор
{
    setServoAngle(ID, OPEN_ANG);
    return true;
}

bool close_tower(int ID)              // Закрыть манипулятор
{
    setServoAngle(ID, CLOSED_ANG);
    return true;
}
///////////////////////////////////////////////////////////////

///////////////////////////CUBES CATCHER///////////////////////

bool open_cubes(int ID_RIGHT, int ID_LEFT)
{
    setServoAngle(ID_RIGHT, OPEN_ANG_RIGHT);
    setServoAngle(ID_LEFT, OPEN_ANG_LEFT);

    return 0;
}

bool close_cubes(int ID_RIGHT, int ID_LEFT)
{
    float right_servo_angle, left_servo_angle, prev_right_servo_angle, prev_left_servo_angle;

    setServoTorque(ID_RIGHT, 500);
    setServoTorque(ID_LEFT, 500);
    setServoAngle(ID_RIGHT, CLOSED_ANG_RIGHT);
    setServoAngle(ID_LEFT, CLOSED_ANG_LEFT);
    soft_delay(20000);
    while((prev_right_servo_angle != right_servo_angle) && (prev_left_servo_angle != left_servo_angle))
    {
        getServoAngle(ID_RIGHT, &right_servo_angle);
        getServoAngle(ID_LEFT, &left_servo_angle);
        prev_right_servo_angle = right_servo_angle;
        prev_left_servo_angle = left_servo_angle;
    }

    return 0;
}

///////////////////////////////////////////////////////////////
