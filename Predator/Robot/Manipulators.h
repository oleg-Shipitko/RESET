#ifndef _MANIPULATORS_INCLUDED_
#define _MANIPULATORS_INCLUDED_

#include <stdbool.h>
#include <stdint.h>
#include "Regulator.h"
#include "Pins.h"

extern uint16_t adcData[10];
void softDelay(unsigned long int);

///////////////////////////PREDATOR MOUTH///////////////////////
#define ID_RIGHT 5                     // If to look on robot from the back
#define ID_LEFT 2
#define ID_UP 8
#define ID_FORWARD_BACKWARD 12
#define DOWN_ANG 0
#define UP_ANG 300
#define OPEN_ANG_RIGHT 170 //220
#define OPEN_ANG_LEFT 50 //0
#define CLOSED_ANG_RIGHT 90
#define CLOSED_ANG_LEFT 130
#define CLOSED_ANG_RIGHT_INIT 120
#define CLOSED_ANG_LEFT_INIT 100
#define CLOSED_ANG_RIGHT_INSIDE 100
#define CLOSED_ANG_LEFT_INSIDE 80
#define IS_CAUGHT_ANGLE 30      // angle defining difference in manipulator's angles if cubes are caught

bool openCubesCatcher(void);
bool closeCubesCatcher(void);
bool closeCubesCatcherInit(void);
bool moveCubesCatcherUp(void);
bool moveCubesCatcherDown(void);
bool moveCubesCatcherForward(void);
bool moveCubesCatcherBackward(void);
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
#define DOOR_HALF_OPEN_ANG_RIGHT 15
#define DOOR_HALF_OPEN_ANG_LEFT 285

bool openDoors(void);
bool closeDoors(void);
bool halfOpenDoors(void);
///////////////////////////////////////////////////////////////

//////////////////////////CUBES HOLDERS////////////////////////
#define UPPER_HOLDER 7
#define LOWER_HOLDER 6
#define CONES_HOLDER_CH 7
#define CONES_HOLDER_OPEN_ANG 0.085
#define UPPER_HOLDER_OPEN_ANG 142
#define LOWER_HOLDER_OPEN_ANG 150
#define CONES_HOLDER_CLOSED_ANG 0.11 //0.12
#define UPPER_HOLDER_CLOSED_ANG 270
#define LOWER_HOLDER_CLOSED_ANG 280

bool openHolders(void);
bool closeHolders(void);
///////////////////////////////////////////////////////////////

#endif
