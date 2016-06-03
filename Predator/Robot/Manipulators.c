#include "Manipulators.h"
#include "Dynamixel_control.h"
#include "Board.h"
#include "Interrupts.h"

void softDelay(__IO unsigned long int ticks)
{
    for(; ticks > 0; ticks--);
}

///////////////////////////////////////////////////////////////

///////////////////////////PREDATOR MOUTH///////////////////////
bool openCubesCatcher(void)
{
    setServoAngle((uint8_t)ID_RIGHT, (uint16_t)OPEN_ANG_RIGHT);
    setServoAngle((uint8_t)ID_LEFT, (uint16_t)OPEN_ANG_LEFT);

    return 0;
}

bool closeCubesCatcher(uint8_t* is_catched)
{
    setServoAngle((uint8_t)ID_RIGHT, (uint16_t)CLOSED_ANG_RIGHT);
    setServoAngle((uint8_t)ID_LEFT, (uint16_t)CLOSED_ANG_LEFT);

    return 0;
}
///////////////////////////////////////////////////////////////

///////////////////////////PNEUMO//////////////////////////////
bool switchOnPneumo()
{
    set_pin(PIN1_12V);
    set_pin(PIN2_12V);
    set_pin(PIN2_12V);
    return 0;
}

bool switchOffPneumo()
{
    reset_pin(PIN1_12V);
    reset_pin(PIN2_12V);
    reset_pin(PIN2_12V);
    return 0;
}
///////////////////////////////////////////////////////////////

/////////////////////////////DOORS/////////////////////////////
bool openDoors(void)
{
    setServoAngle((uint8_t)DOOR_RIGHT, (uint16_t)DOOR_OPEN_ANG_RIGHT);
    setServoAngle((uint8_t)DOOR_LEFT, (uint16_t)DOOR_OPEN_ANG_LEFT);

    return 0;
}

bool closeDoors(void)
{
    setServoAngle((uint8_t)DOOR_RIGHT, (uint16_t)DOOR_CLOSED_ANG_RIGHT);
    setServoAngle((uint8_t)DOOR_LEFT, (uint16_t)DOOR_CLOSED_ANG_LEFT);

    return 0;
}
///////////////////////////////////////////////////////////////
