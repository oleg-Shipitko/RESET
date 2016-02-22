#ifndef _PATHSTRUCTURE_INCLUDED_
#define _PATHSTRUCTURE_INCLUDED_

#define PI             3.1415926535897932384626

typedef float pathPoint[2];
typedef float Center[3];
//{
//  float x;
//  float y;
//  
//} Point;

typedef struct 
{
    Center coordCenter;         // оординаты центра системы координат
    float alphZad;              //угол между вектором глобальный координат и траекторией
    float phiZad;               //угол поворота робота вокруг своей оси
    float lengthTrace;          //длина трассы
    float (*traceVel);      //V_уст, V_нач, V_кон, ј_уск, ј_торм
    float (*omegaVel);      //V_уст, V_нач, V_кон, ј_уск, ј_торм
       
} Path;



void initPaths(void);

#endif

