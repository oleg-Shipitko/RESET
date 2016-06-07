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

bool closeCubesCatcher(void)
{
    setServoAngle((uint8_t)ID_RIGHT, (uint16_t)CLOSED_ANG_RIGHT);
    setServoAngle((uint8_t)ID_LEFT, (uint16_t)CLOSED_ANG_LEFT);

    return 0;
}

bool closeCubesCatcherInit(void)
{
    setServoAngle((uint8_t)ID_RIGHT, (uint16_t)CLOSED_ANG_RIGHT_INIT);
    setServoAngle((uint8_t)ID_LEFT, (uint16_t)CLOSED_ANG_LEFT_INIT);

    return 0;
}


bool moveCubesCatcherUp(void)
{
    setServoTorque((uint8_t)ID_UP, 800);
    setServoAngle((uint8_t)ID_UP, (uint16_t)UP_ANG);
    return 0;
}

bool moveCubesCatcherDown(void)
{
    setServoTorque((uint8_t)ID_UP, 400);
    setServoAngle((uint8_t)ID_UP, (uint16_t)DOWN_ANG);

    return 0;
}

bool moveCubesCatcherForward(void)
{
    while(pin_val(EXTI3_PIN))
    {
        setServoMovingSpeed ((uint8_t) ID_FORWARD_BACKWARD, (uint16_t) 1023, (uint16_t) CCW);
    }
    setServoMovingSpeed ((uint8_t) ID_FORWARD_BACKWARD, (uint16_t) 0, (uint16_t) CCW);
    return 0;
}
uint32_t input;
bool moveCubesCatcherBackward(void)
{
    input = pin_val(EXTI4_PIN);
    while(input)
    {
        input = pin_val(EXTI4_PIN);
        setServoMovingSpeed ((uint8_t) ID_FORWARD_BACKWARD, (uint16_t) 1023, (uint16_t) CW);
    }
    setServoMovingSpeed ((uint8_t) ID_FORWARD_BACKWARD, (uint16_t) 0, (uint16_t) CW);
    return 0;
}

//bool moveCubesCatcherForward(void)
//{
//    set_pin(BTN6_PWM_PIN);
//    reset_pin(BTN6_DIR_PIN);
//
//    return 0;
//}

//bool moveCubesCatcherBackward(void)
//{
//    reset_pin(BTN6_PWM_PIN);
//    set_pin(BTN6_DIR_PIN);
//
//    return 0;
//}

///////////////////////////////////////////////////////////////

///////////////////////////PNEUMO//////////////////////////////
bool switchOnPneumo(void)
{
    set_pin(PIN1_12V);
    set_pin(PIN2_12V);
    set_pin(PIN3_12V);
    return 0;
}

bool switchOffPneumo(void)
{
    reset_pin(PIN1_12V);
    reset_pin(PIN2_12V);
    reset_pin(PIN3_12V);
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

bool halfOpenDoors(void)
{
    setServoAngle((uint8_t)DOOR_RIGHT, (uint16_t)DOOR_HALF_OPEN_ANG_RIGHT);
    setServoAngle((uint8_t)DOOR_LEFT, (uint16_t)DOOR_HALF_OPEN_ANG_LEFT);

    return 0;
}

bool closeDoors(void)
{
    setServoAngle((uint8_t)DOOR_RIGHT, (uint16_t)DOOR_CLOSED_ANG_RIGHT);
    setServoAngle((uint8_t)DOOR_LEFT, (uint16_t)DOOR_CLOSED_ANG_LEFT);

    return 0;
}
///////////////////////////////////////////////////////////////

//////////////////////////CUBES HOLDERS////////////////////////
bool openHolders(void)
{
    setServoAngle((uint8_t)LOWER_HOLDER, (uint16_t)LOWER_HOLDER_OPEN_ANG);
    softDelay(9000000);
    setServoAngle((uint8_t)UPPER_HOLDER, (uint16_t)UPPER_HOLDER_OPEN_ANG);
    softDelay(9000000);
    setVoltage((char)CONES_HOLDER_CH - 1, (float)CONES_HOLDER_OPEN_ANG);
    return 0;
}

bool closeHolders(void)
{
    setVoltage((char)CONES_HOLDER_CH - 1, (float)CONES_HOLDER_CLOSED_ANG);
    softDelay(7000000);
    setServoAngle((uint8_t)UPPER_HOLDER, (uint16_t)UPPER_HOLDER_CLOSED_ANG);
    softDelay(7000000);
    setServoAngle((uint8_t)LOWER_HOLDER, (uint16_t)LOWER_HOLDER_CLOSED_ANG);

    return 0;
}
///////////////////////////////////////////////////////////////
