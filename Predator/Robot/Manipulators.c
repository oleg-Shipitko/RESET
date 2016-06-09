#include "Manipulators.h"
#include "Dynamixel_control.h"
#include "Board.h"
#include "Interrupts.h"

typedef void task();

void softDelay(__IO unsigned long int ticks)
{
    for(; ticks > 0; ticks--);
}

///////////////////////////////////////////////////////////////

///////////////////////////PREDATOR MOUTH///////////////////////
bool openCubesCatcher(void)
{
    while(!setServoAngle((uint8_t)ID_RIGHT, (uint16_t)OPEN_ANG_RIGHT)){};
    while(!setServoAngle((uint8_t)ID_LEFT, (uint16_t)OPEN_ANG_LEFT)){};

    return 0;
}

bool closeCubesCatcher(void)
{
    while(!setServoAngle((uint8_t)ID_RIGHT, (uint16_t)CLOSED_ANG_RIGHT)){};
    while(!setServoAngle((uint8_t)ID_LEFT, (uint16_t)CLOSED_ANG_LEFT)){};

    return 0;
}

bool closeCubesCatcherInit(void)
{
    while(!setServoAngle((uint8_t)ID_RIGHT, (uint16_t)CLOSED_ANG_RIGHT_INIT)){};
    while(!setServoAngle((uint8_t)ID_LEFT, (uint16_t)CLOSED_ANG_LEFT_INIT)){};

    return 0;
}


bool moveCubesCatcherUp(void)
{
    while(!setServoTorque((uint8_t)ID_UP, 800)){};
    while(!setServoAngle((uint8_t)ID_UP, (uint16_t)UP_ANG)){};
    return 0;
}

bool moveCubesCatcherDown(void)
{
    while(!setServoTorque((uint8_t)ID_UP, 400)){};
    while(!setServoAngle((uint8_t)ID_UP, (uint16_t)DOWN_ANG)){};

    return 0;
}

bool moveCubesCatcherForward(void)
{
//    char input = pin_val(EXTI3_PIN);
    while(!pin_val(EXTI3_PIN))
    {
        while(!setServoMovingSpeed ((uint8_t) ID_FORWARD_BACKWARD, (uint16_t) 1023, (uint16_t) CCW)){};
    }
    while(!setServoMovingSpeed ((uint8_t) ID_FORWARD_BACKWARD, (uint16_t) 0, (uint16_t) CCW)){};
    return 0;
}

bool moveCubesCatcherBackward(void)
{
    while(!pin_val(EXTI4_PIN))
    {
        while(!setServoMovingSpeed ((uint8_t) ID_FORWARD_BACKWARD, (uint16_t) 1023, (uint16_t) CW)){};
    }
    while(!setServoMovingSpeed ((uint8_t) ID_FORWARD_BACKWARD, (uint16_t) 0, (uint16_t) CW)){};
    return 0;
}

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
    while(!setServoAngle((uint8_t)DOOR_RIGHT, (uint16_t)DOOR_OPEN_ANG_RIGHT)){};
    while(!setServoAngle((uint8_t)DOOR_LEFT, (uint16_t)DOOR_OPEN_ANG_LEFT)){};

    return 0;
}

bool halfOpenDoors(void)
{
    while(!setServoAngle((uint8_t)DOOR_RIGHT, (uint16_t)DOOR_HALF_OPEN_ANG_RIGHT)){};
    while(!setServoAngle((uint8_t)DOOR_LEFT, (uint16_t)DOOR_HALF_OPEN_ANG_LEFT)){};

    return 0;
}

bool closeDoors(void)
{
    while(!setServoAngle((uint8_t)DOOR_RIGHT, (uint16_t)DOOR_CLOSED_ANG_RIGHT)){};
    while(!setServoAngle((uint8_t)DOOR_LEFT, (uint16_t)DOOR_CLOSED_ANG_LEFT)){};

    return 0;
}
///////////////////////////////////////////////////////////////

//////////////////////////CUBES HOLDERS////////////////////////
bool openHolders(void)
{
    float lower_holder_ang, upper_holder_ang;

    getServoAngle((uint8_t)LOWER_HOLDER, &lower_holder_ang);
    while(!setServoAngle((uint8_t)LOWER_HOLDER, (uint16_t)LOWER_HOLDER_OPEN_ANG)){};
    while(lower_holder_ang > 200)
    {
        getServoAngle((uint8_t)LOWER_HOLDER, &lower_holder_ang);
    }



    getServoAngle((uint8_t)UPPER_HOLDER, &upper_holder_ang);
    setServoAngle((uint8_t)UPPER_HOLDER, (uint16_t)UPPER_HOLDER_OPEN_ANG);
    while(upper_holder_ang > 200)
    {
        getServoAngle((uint8_t)UPPER_HOLDER, &upper_holder_ang);
    }



    setVoltage((char)CONES_HOLDER_CH - 1, (float)CONES_HOLDER_OPEN_ANG);
    return 0;
}

bool closeHolders(void)
{
    float lower_holder_ang, upper_holder_ang;

    setVoltage((char)CONES_HOLDER_CH - 1, (float)CONES_HOLDER_CLOSED_ANG);
    softDelay(7000000);


    setServoAngle((uint8_t)UPPER_HOLDER, (uint16_t)UPPER_HOLDER_CLOSED_ANG);
    getServoAngle((uint8_t)UPPER_HOLDER, &upper_holder_ang);
    while(upper_holder_ang < 220)
    {
        getServoAngle((uint8_t)UPPER_HOLDER, &upper_holder_ang);
    }


    while(!setServoAngle((uint8_t)LOWER_HOLDER, (uint16_t)LOWER_HOLDER_CLOSED_ANG)){};

    return 0;
}
///////////////////////////////////////////////////////////////
