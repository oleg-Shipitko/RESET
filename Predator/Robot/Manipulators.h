#ifndef _MANIPULATORS_INCLUDED_
#define _MANIPULATORS_INCLUDED_

#include <stdbool.h>
#include <stdint.h>
#include "Regulator.h"
#include "Pins.h"

extern uint16_t adcData[10];
void softDelay(unsigned long int);

///////////////////////////TOWER BUILDER///////////////////////
#define ID_FRONT 11                     // Front manipulator
#define ID_BACK 11                      // Back manipulator
#define CLOSED_ANG 0
#define OPEN_ANG 300

bool open_tower(int8_t);
bool close_tower(int8_t);
///////////////////////////////////////////////////////////////

///////////////////////////PREDATOR MOUTH///////////////////////
#define ID_RIGHT 5                     // If to look on robot from the back
#define ID_LEFT 2
#define OPEN_ANG_RIGHT 300//300
#define OPEN_ANG_LEFT 0 //0
#define CLOSED_ANG_RIGHT 175
#define CLOSED_ANG_LEFT 125
#define IS_CAUGHT_ANGLE 30      // angle defining difference in manipulator's angles if cubes are caught

bool openCubesCatcher(void);
bool closeCubesCatcher(uint8_t*);
///////////////////////////////////////////////////////////////

///////////////////////////PNEUMO//////////////////////////////
bool pneumoIn();
bool pneumoOut();
bool switchOnPneumo();
bool switchOffPneumo();
///////////////////////////////////////////////////////////////

/////////////////////////////DOORS/////////////////////////////
#define DOOR_RIGHT 10 // If to look on robot from the back
#define DOOR_LEFT 11
#define DOOR_OPEN_ANG_RIGHT 0
#define DOOR_OPEN_ANG_LEFT 300
#define DOOR_CLOSED_ANG_RIGHT 105
#define DOOR_CLOSED_ANG_LEFT 195

bool openDoors(void);
bool closeDoors(void);
///////////////////////////////////////////////////////////////

#endif
