#include "Regulator.h"
#include "robot.h"
#include "matrix.h"
#include "stdlib.h"
#include "gpio.h"
#include <math.h>
#include "adc.h"
#include "Board.h"
#include "Interrupts.h"
#include "pins.h"

//#define ENCODER_IMITATION // Encoders emulation

////////////////////////////////////////////////////////////////////////////////

float regulatorOut[4] = {0, 0, 0, 0};  //����� �������� ����������, ���������� �������� ����������
float vTargetGlob[3] = {0, 0, 0};// ������ ���������� ��������� ������
float vTargetGlobF[3] = {0, 0, 0};// ������ ���������� ��������� ������ � ������ ����������
float vTargetGlobCA[3] = {0, 0, 0};// ������ ���������� ��������� ������ � ������ Collision Avoidance
PidStruct wheelsPidStruct[4]; //��������� ��� ����������� �����
PidStruct radSpeed; // ��������� ��� ���������� �������� ��������
PidStruct ortoPos;  // ��������� ��� ���������� �� ����������
char collisionIsOn = 0;

uint16_t totalPointComplite = 0;  //����� �������� �����

float MAX_ACCEL = 0.1;
float MAX_ACCEL_INC = 0.03;
float ACCEL_INC = 0.2;
 TVector CurSpeed = {0, 0};
 TVector CurAccel = {0, 0};
 TVector AccelInc1 = {0, 0};
 TVector AccelInc2 = {0, 0};
 TVector TargSpeed = {0, 0};

pathPointStr points[POINT_STACK_SIZE]={ {0.0, 0.0, 0.0, NULL,NULL,0,stopVelFast,stopRotFast,0,1 },  //���� ����� ����������
                                        {0.0, 8.0, 0.0, NULL,NULL,0,stopVelSuperFast,stopRotFast,0,1 },
                                        {-4.0, 4.0, 0.0, NULL,NULL,0,stopVelSuperFast,stopRotFast,0,1 },
                                        {0.0, 4.0, 0., NULL,NULL,0,stopVelSuperFast,stopRotSuperFast,0,1 },
                                        {0.0, 0.0, 0.0, NULL,NULL,0,stopVelSuperFast,stopRotSlow,0,1 },
                                        {0.0, -4.0, 2.0, NULL,NULL,0,stopVelFast,stopRotFast,0,1 },
                                        {0.0, 0.0, 0.0, NULL,NULL,0,stopVelFast,stopRotFast,0,1 },
                                        {0.0, 0.0, 0.0, NULL,NULL,0,stopVelFast,stopRotFast,0,1 },
                                        {0.0, 0.0, 0.0, NULL,NULL,0,stopVelFast,stopRotFast,0,1 },};

//pathPointStr defaultPoint;

char lastPoint = 0;// ��������� �������� ����� � �������
Path curPath; //��������� �������� ������ ��� ������������ ����������

float normalVelSuperFast[5] = {1.5, 0.2, 1.5, 4.0, 3.0};//V_���, V_���, V_���, �_���, �_����  //����������� ��������
float stopVelSuperFast[5] = {1.5, 0.2, -1.2, 4.0, 3.0}; //{0.2,0.1,-0.05,0.2,0.7};            //�������� � ���������� � �����
float standVelSuperFast[5] = {1.5, 0.6, -0.6, 2.0, 2.5};                                       //��������� ��������� ���������

float normalRotSuperFast[5] = {4.0, 2.0, -1.0, 4.0, 3.0};//{4.0, 2.0, 0.2, 4.0, 4.0};//V_���, V_���, V_���, �_���, �_����  //����������� ��������
float stopRotSuperFast[5] = {6.0, 3.0, -1.0, 4.0, 3.0}; //{0.2,0.1,-0.1,0.3,0.6};             //�������� � ���������� � �����
float standRotSuperFast[5] = {6.0, 4.0, -1.0, 2.0, 2.5};                                       //��������� ��������� ���������

float normalVelFast[5] = {0.3, 0.2, 0.2, 4.0, 2.0};//V_���, V_���, V_���, �_���, �_����  //����������� ��������
float stopVelFast[5] = {0.3, 0.2, -0.2, 3.0, 2.0}; //{0.2,0.1,-0.05,0.2,0.7};            //�������� � ���������� � �����
float standVelFast[5] = {0.3, 0.6, -0.6, 2.0, 2.5};                                       //��������� ��������� ���������

float normalRotFast[5] = {3.0, 1.0, -1.0, 4.0, 3.0};//{3.0, 1.0, 0.2, 4.0, 4.0};//V_���, V_���, V_���, �_���, �_����  //����������� ��������
float stopRotFast[5] = {3.0, 1.0, -1.0, 4.0, 3.0}; //{0.2,0.1,-0.1,0.3,0.6};             //�������� � ���������� � �����
float standRotFast[5] = {4.0, 4.0, -1.0, 2.0, 2.5};                                       //��������� ��������� ���������

float normalVelSlow[5] = {0.1, 0.05, 0.05, 4.0, 2.0};//V_���, V_���, V_���, �_���, �_����  //����������� ��������
float stopVelSlow[5] = {0.1, 0.05, -0.05, 3.0, 2.0}; //{0.2,0.1,-0.05,0.2,0.7};            //�������� � ���������� � �����
float standVelSlow[5] = {0.1, 0.05, -0.05, 2.0, 2.5};                                       //��������� ��������� ���������

float normalRotSlow[5] = {1.0, 0.2, -1.0, 4.0, 3.0};//{1.0, 0.2, 0.2, 4.0, 4.0};//V_���, V_���, V_���, �_���, �_����  //����������� ��������
float stopRotSlow[5] = {1.0, 0.2, -1.0, 4.0, 3.0}; //{0.2,0.1,-0.1,0.3,0.6};             //�������� � ���������� � �����
float standRotSlow[5] = {1.0 , 1.0, -1.0, 2.0, 2.5};                                       //��������� ��������� ���������


float * speedType[9] = {normalVelFast,      //0
                        stopVelFast,        //1
                        standVelFast,       //2
                        normalVelSlow,      //3
                        stopVelSlow,        //4
                        standVelSlow,       //5
                        normalVelSuperFast, //6
                        stopVelSuperFast,   //7
                        standVelSuperFast  }; //8/ ����  �������� ���������
float * rotType[9] = {stopRotFast, stopRotFast, standRotFast, stopRotSlow, stopRotSlow,
                        standRotSlow, stopRotSuperFast, stopRotSuperFast, standRotSuperFast};// ���� ������� ���������

////////////////////////////////////////////////////////////////////////////////
//float compareAngles(float ang1, float ang2)
//{
//    float dif = fabs(ang1 - ang2);
//    if (dif > 3.14)
//    {
//        dif =
//    }
//}

void removePoint(pathPointStr * points, char *lastPoint)  // �������� ����� �� �������
{
  char i,j;
  for (i = 0; i < *lastPoint; i++)
  {
    for (j = 0; j < sizeof(pathPointStr); j++)
      *(((char *)(&points[i])) + j) = *(((char *)(&points[i+1])) + j);

  }
   if (*lastPoint > 0)
   {
      for (j=0; j < sizeof(pathPointStr); j++)
      *(((char *)(&points[*lastPoint])) + j) = 0;
     (*lastPoint)--;
   }
}

void addPointInFrontOfQueue(pathPointStr *pointsArray, float *newPoint, char ch, char *lastPoint) // ���������� ����� � ������ �������
{
  int i, j;
  for (i = *lastPoint; i >= 0; i--)
  {
      if (!((i + 1) > POINT_STACK_SIZE))
      {
          for (j = 0; j < sizeof(pathPointStr); j++)
            *(((char *)(&points[i + 1])) + j) = *(((char *)(&points[i])) + j);
      }
  }
  points[0].center[0] = *newPoint;
  points[0].center[1] = *(newPoint + 1);
  points[0].center[2] = *(newPoint + 2);
  points[0].speedVelTipe = speedType[ch];
  points[0].speedRotTipe = rotType[ch];
  points[0].endTask = NULL;
  points[0].movTask = NULL;

  (*lastPoint)++;
}

void initRegulators(void)  // ������������� �����������
{
  //RadSpeed
  {  //��������� ������� ��������
        radSpeed.p_k = 1.0;
        radSpeed.i_k = 0.0;
        radSpeed.d_k = 0.0;
        radSpeed.max_output = 3.0;
        radSpeed.max_sum_error = 10.0;
        radSpeed.pid_on = 1;
  }
    //OrtoPos //��������� ������ ���������� �� ����������
  {
        ortoPos.p_k = 2.0;
        ortoPos.i_k = 0.5;
        ortoPos.d_k = 0.0;
        ortoPos.max_output = 4.0;
        ortoPos.max_sum_error = 0.05;
        ortoPos.pid_on = 1;
  }

}
////////////////////////////////////////////////////////////////////////////////

void Cost(float *inpMatr,char rows,float cost,float *outKoff)
{
  float infnorm;
  MaxValue(inpMatr, rows, &infnorm);
  if (infnorm == 0)
    *outKoff = 0;
  else
    if ((infnorm > 0) && (infnorm < cost))
      *outKoff = 1;
  else
    *outKoff=cost/(infnorm);
}

////////////////////////////////////////////////////////////////////////////////

void MaxValue(float *a,char rows,float *b)
{
  char i;
  *b = fabs(*a);
  for (i = 0; (i <= rows - 1); i++)
    if ((*b) < fabs(*(a+i)))
      *b = fabs(*(a + i));
}

////////////////////////////////////////////////////////////////////////////////

void FunctionalRegulator(float *V_target, float *V_out) // ������ ��������� �������� �������� ����������
{
  //float cosinus = cosf(-robotCoord[2]), sinus = sinf(-robotCoord[2]);
  //float Velocity[3]  = {(*(V_target)), (*(V_target+1)),*(V_target+2)};

  float localVelocity[3];
  //float Radian       = (*(V_target+2));
  float realRad = robotCoord[2];

  //float Ml[4][2]     = {(sinus+cosinus), (cosinus-sinus), (cosinus-sinus), -(sinus+cosinus), (cosinus-sinus), -(sinus+cosinus), (sinus+cosinus), (cosinus-sinus)};
 // float Mfi[4]       = {-(0.14), -(0.14),-(0.14), -(0.14)};  //������� ������� ������� ��������
  float Mrot[3][3]   = {cos(realRad) , sin(realRad), 0,
                        -sin(realRad), cos(realRad), 0,
                                    0,            0, 1};  //������� ��������� ���������� ��������� � ���������

  float VLine[4], VRot[4], VSum[4];
  float MaxMotSpeed;
  float MaxLine,MaxRot;
  // float Kr[2][float InverseKinematics[4][4]2]     = {1.0, 0.0, 0.0, 1.0};
  //float Kfi          = 1.0;
  //float deltaVect[2] = {(*(Coord_target)) - (*(Coord_cur)), (*(Coord_target+1)) - (*(Coord_cur+1))};
  //float deltaPhi     = (*(Coord_target+2)) - (*(Coord_cur+2));
  float Vvect[4], Vrad[4];
  float buf1[3], buf2[4][2], buf3[4];

  //float vectVelOrig[4], vectVelWeighted[4], vectErrOrig[4]/*, vectErrWeighted[3]*/;
  //float phiRadOrig[4], phiRadWeighted[4], phiErrOrig[4]/*, phiErrWeighted[3]*/;
  float c1;
  float c2;
  float Cmax;
  float Kvect, Kphi, KnormVect;


  Cmax = MAX_CAPACITANCE;


  matrixMultiplyM2M(&Mrot[0][0], 3, 3, V_target, 3, 1, &localVelocity[0]);//Ml*Velocity speed in local coordinate system

  matrixMultiplyM2M(&MLineSpeed[0][0], 4, 3, &localVelocity[0], 3, 1, &VLine[0]);//MLineSpeed*localVelocity
  matrixMultiplyM2M(&MRotSpeed[0][0], 4, 3, &localVelocity[0], 3, 1, &VRot[0]);//MRotSpeed*localVelocity

  matrixPlusMinus(&VLine[0], &VRot[0], 4, 1, 1, &VSum[0]);//MLineSpeed+MRotSpeed
  MaxValue(&VSum[0],4, &MaxMotSpeed);  //find maximum speed on wheel

  if (fabs(MaxMotSpeed) > MAX_CAPACITANCE)
  {
     c1 = fabs(MAX_CAPACITANCE/MaxMotSpeed );
  } else c1 = 1;

    matrixMultiplyS2M(&VSum[0], 4, 1, c1, &V_out[0]);

  //matrixMultiplyM2M(&Ml[0][0], 4, 2, &Kr[0][0], 2, 2, &buf2[0][0]);//Ml*Kr
  //matrixMultiplyM2M(&buf2[0][0], 4, 2, &deltaVect[0], 2, 1, &vectErrOrig[0]);//*(delta_vect)


  //matrixMultiplyS2M(&Mfi[0], 4, 1, Radian, &phiRadOrig[0]);//Mfi*Radian;
  //matrixMultiplyS2M(&Mfi[0], 4, 1, Kfi, &buf3[0]);// Mfi*Kfi
 // matrixMultiplyS2M(&buf3[0], 4, 1, deltaPhi, &phiErrOrig[0]);//*(delta_phi)


 // Cost(&vectVelOrig[0], 4, c1, &Kvect);        //cost(vectErr1,c1);
 // matrixMultiplyS2M(&vectVelOrig[0], 4, 1, Kvect, &vectVelWeighted[0]);//vectErr=vectErr1*cost(vectErr1,c1);


  //infnormvect(&vectVelWeighted[0], 3, &KnormVect);
  //c2 =0.2+ fabs(c1 - KnormVect);
 // c2 = Cmax - c1;


  //Cost(&phiRadOrig[0], 4, c2, &Kphi); //cost(phiErr1,c2);
  //matrixMultiplyS2M(&phiRadOrig[0], 4, 1, Kphi, &phiRadWeighted[0]);//phiErr=phiErr1*cost(phiErr1,c2);


  //matrixPlusMinus(&vectVelWeighted[0], &vectErrOrig[0], 4, 1, 1, &buf1[0]);//vectVel1+vectErr
  //matrixMultiplyS2M(&buf1[0], 4, 1, ONE_RO_COS_PHI, &Vvect[0]);//Vvect=((1/ro/cos(phi_rad))*(vectVel1+vectErr));


  //matrixPlusMinus(&phiRadWeighted[0], &phiErrOrig[0], 4, 1, 1, &buf1[0]);//phiRad1+phiErr
  //matrixMultiplyS2M(&buf1[0], 4, 1, ONE_RO_COS_PHI, &Vrad[0]);//((1/ro/cos(phi_rad))*(phiRad1+phiErr))


  //matrixPlusMinus(&Vvect[0], &Vrad[0], 4, 1, 1, &V_out[0]);

}
////////////////////////////////////////////////////////////////////////////////

void Regulate(float *Coord_cur, float *Speed_cur, float *tAlphZad,float *V_etalon,float *alphZad, float *V_local)
{
float t_alph_cur[2][2], t_alph_delta[2][2];
float uS, uE, u[2];
float buf[2][2],buf1[2][2],buf3[2][1];
float cosAlphCur;
float sinAlphCur;

float resultVector = sqrtf((*Coord_cur) * (*Coord_cur) + (*(Coord_cur+1)) * (*(Coord_cur+1)));

if (resultVector==0.0)
{
  cosAlphCur=cosf(*alphZad);
  sinAlphCur=sinf(*alphZad);
}
else
{
cosAlphCur = (*Coord_cur)/resultVector;
sinAlphCur = (*(Coord_cur+1))/resultVector;
}

  t_alph_cur[0][0] =  cosAlphCur;
  t_alph_cur[0][1] =  sinAlphCur;
  t_alph_cur[1][0] = -sinAlphCur;
  t_alph_cur[1][1] =  cosAlphCur;

  matrixTranspose(&t_alph_cur[0][0], 2, 2, &buf[0][0]);   //tDeltaAlph=tAlphCur'
  matrixMultiplyM2M(tAlphZad, 2, 2,&buf[0][0],2,2,&t_alph_delta[0][0]);
  matrixTranspose(&t_alph_delta[0][0], 2, 2, &buf1[0][0]);  // We have current traectory error in buf1 now

////////////////////////////////////////////////////////////////////////////////
//radSpeed.current = (*(Speed_cur+2));
//radSpeed.target = *(V_etalon+1);//controlBoard.x2;
//pidCalc(&radSpeed);
*(V_local+2) =*(V_etalon+1);//// radSpeed.output;
////////////////////////////////////////////////////////////////////////////////
//trackSpeed.current = *(Speed_cur);
//trackSpeed.target = *(V_etalon);//controlBoard.x2;
//pidCalc(&trackSpeed);
//uS = trackSpeed.output;
////////////////////////////////////////////////////////////////////////////////
ortoPos.current =  (*(Coord_cur+1));
ortoPos.target = 0.0;//controlBoard.x2;
pidCalc(&ortoPos);
uE = ortoPos.output;
////////////////////////////////////////////////////////////////////////////////

  uS =(*V_etalon);
  //uE = KOFF_ORTO_TRACE * (*(Coord_cur+1));
  u[0] = uS;
  u[1] = uE;

  matrixMultiplyM2M(&buf1[0][0], 2, 2, &u[0], 2, 1, &buf3[0][0]);
  matrixMultiplyM2M(&buf[0][0],2,2,&buf3[0][0],2,1,V_local);
}

////////////////////////////////////////////////////////////////////////////////

void TrackRegulator(float *Coord_cur, float *SpeedCur, Path *cur, float *V)
{
float velocity[3] = {0.0, 0.0, 0.0};
float cosAlphZad = cosf(cur->alphZad), sinAlphZad = sinf(cur->alphZad);
float t_alph_zad[2][2];
float buf1[3];
float velFromEt[2];
//float tracer;
//float Coord_local_track[3] = {0.0, 0.0, 0.0};

matrixPlusMinus(Coord_cur,cur->coordCenter, 3, 1, -1, &buf1[0]); //(CURCOORD-CUR_CENTER) = buf1

t_alph_zad[0][0] =  cosAlphZad;
t_alph_zad[0][1] =  sinAlphZad;
t_alph_zad[1][0] = -sinAlphZad;
t_alph_zad[1][1] =  cosAlphZad;

matrixMultiplyM2M(&t_alph_zad[0][0], 2, 2, SpeedCur, 2, 1, &(cur->Speed_local_track[0]));
matrixMultiplyM2M(&t_alph_zad[0][0], 2, 2, &buf1[0], 2, 1, &(cur->Coord_local_track[0]));//
cur->Coord_local_track[2] = (robotCoord[2]-cur->coordCenter[2])*cur->phiDir;
if (cur->Coord_local_track[2]<0) cur->Coord_local_track[2]+=2.0*PI;
//if(Coord_local_track[0]<0)
//	tracer=fabs(Coord_local_track[0]);
//else
//	tracer=Coord_local_track[0];


Moving(cur->Coord_local_track[0], (cur->lengthTrace), (cur->traceVel), &velFromEt[0]);
RotMoving((cur->coordCenter[2]), robotCoord[2], (cur->phiZad), (cur->omegaVel), &velFromEt[1]);
//velFromEt[1]= velFromEt[1]*cur->phiDir;
//LOCALCOORD(3,1)=(phiZad-CURCOORD(3,1));


//VELOCITY=regulate(LOCALCOORD,vEt);
Regulate(&(cur->Coord_local_track[0]), &(cur->Speed_local_track[0]), &t_alph_zad[0][0], &velFromEt[0], &(cur->alphZad), &velocity[0]);
matrixCopy(&velocity[0], 3, 1, &V[0]);

}////////////////////////////////////////////////////////////////////////////////

float linars(float *x, float *x0, float *x1)
{
  float out = (((*x - (*x0)) / (*x1 - *(x0))) * (*(x1+1) - (*(x0+1)))) + (*(x0+1));
  return out;
}

void rangeAngle(float * angle)
{
  *angle    = fmod(*angle,2.0*PI);
  if (*angle<-PI) *angle+=2.0*PI;
  if (*angle>PI) *angle-=2.0*PI;
}

void RotMoving(float Start_a, float Coord_a_cur, float Coord_a_targ, float* parameters, float* v_out) //������ �������� � ����������� �� ����������� ����
{
float vStart, vSt, vEnd, acc, decc;
float vAcc, vDecc; // ��������� ������
float out;
//static float vStLast = 0; // ��� ����� ������������ ����������

vSt    = (*parameters); //V_���, V_���, V_���, �_���, �_����
vStart = (*(parameters+1));
vEnd   = (*(parameters+2));
acc    = (*(parameters+3));
decc   = (*(parameters+4));

float distfromstart = Coord_a_cur-Start_a;
rangeAngle(&distfromstart) ;
float angleErr = Coord_a_targ- Coord_a_cur;
rangeAngle(&angleErr) ;
float sign ;

  // Equation of acceleration
vAcc = acc*fabs(distfromstart) + vStart;
  // of decceleration
vDecc = decc*(fabs(angleErr));

out = vSt;
if (vAcc < out)
  out = vAcc;
if (vDecc < out)
{
  if (vDecc < vEnd)
  out = vEnd;
  else
    out = vDecc;
}
if (angleErr<0) out=-out;
*v_out = out;
}
//float stopVel[5]={0.02,0.1,-0.1,0.7,0.9};
////////////////////////////////////////////////////////////////////////////////

void Moving(float Coord_x_cur, float Coord_x_targ, float* parameters, float* v_out) //������ �������� � ����������� �� ����������� ����
{
float vStart, vSt, vEnd, acc, decc;
float vAcc, vDecc; // ��������� ������
float out;
//static float vStLast = 0; // ��� ����� ������������ ����������

vSt    = (*parameters); //V_���, V_���, V_���, �_���, �_����
vStart = (*(parameters+1));
vEnd   = (*(parameters+2));
acc    = (*(parameters+3));
decc   = (*(parameters+4));


  // Equation of acceleration
vAcc = acc*Coord_x_cur + vStart;
if (vAcc< vStart) vAcc = vStart;
  // of decceleration
vDecc = -decc*(Coord_x_cur - Coord_x_targ);

out = vSt;
if (vAcc < out)
  out = vAcc;
if (vDecc < out)
{
  if (vDecc < vEnd)
  out = vEnd;
  else
    out = vDecc;
}
*v_out = out;
}

////////////////////////////////////////////////////////////////////////////////

void pidCalc(PidStruct *pid_control)  //������� ���

{
  float error, dif_error;
  error = pid_control->target - pid_control->current;
  dif_error = error - pid_control->prev_error;
  pid_control->prev_error = error;
  pid_control->sum_error += error;

  if (pid_control->sum_error > pid_control->max_sum_error)
    pid_control->sum_error = pid_control->max_sum_error;
  if (pid_control->sum_error < -pid_control->max_sum_error)
    pid_control->sum_error = -pid_control->max_sum_error;

  if (pid_control->pid_on)
  {
    pid_control->output = ((float)(pid_control->p_k * error)+(pid_control->i_k * pid_control->sum_error)+
            (pid_control->d_k * dif_error));


    if (pid_control->output > pid_control->max_output)
      pid_control->output = pid_control->max_output;
    else if (pid_control->output < -pid_control->max_output)
      pid_control->output = -pid_control->max_output;

    if (pid_control->output < pid_control->min_output && pid_control->output > -pid_control->min_output)
      pid_control->output = 0;

    if ((pid_control->output <= pid_control->pid_output_end) &&(
        pid_control->output >= -pid_control->pid_output_end) &&(
        error <= pid_control->pid_error_end) && (error >= -pid_control->pid_error_end))
      pid_control->pid_finish = 1;
    else
      pid_control->pid_finish = 0;
  }
  else
  {
    pid_control->output = 0;
    pid_control->pid_finish = 0;
  }
}

////////////////////////////////////////////////////////////////////////////////

//Get data from encoders
void GetDataForRegulators(void)
{
int16_t motorSpeedBuf[4];
  int8_t i;
  for(i = 3; i>=0; i--)
  {
    motorSpeedBuf[i] = *encCnt[i];
    *encCnt[i] = 0;
  }
  #ifdef ENCODER_IMITATION
  for(i =3; i>=0; i--)
  {
    motorSpeed[i] =  regulatorOut[i];
    motorCoord[i] += motorSpeed[i]*PID_PERIOD;
  }
  #else
  for(i =3; i>=0; i--)
  {
    motorSpeed[i] =  motorSpeedBuf[i] * DISKR_TO_REAL / PID_PERIOD ;
    motorCoord[i] += motorSpeed[i] * PID_PERIOD;
  }
   #endif
  float realRad = -robotCoord[2];
  float J_inv[4][4];
  float temp[4];
  float temp2[4];

float Mrot[2][2]={ cos(realRad),   sin(realRad),
                    -sin(realRad),  cos(realRad)};
//______________________________________________________________________________
  //matrixInverse((float*)InverseKinematics, 4, (float*)J_inv);
  matrixMultiplyM2M((float*)&InverseKinematics, 4, 4, motorSpeed, 4, 1, &temp);

  matrixMultiplyM2M((float*)&Mrot, 2, 2, &temp, 2, 1, &temp2);



    robotSpeed [0]= temp2[0];
    robotSpeed [1]= temp2[1];
    robotSpeed [2]= temp[2];

//______________________________________________________________________________
  robotCoord[0] += robotSpeed[0]*PID_PERIOD;
  robotCoord[1] += robotSpeed[1]*PID_PERIOD;
  robotCoord[2] += robotSpeed[2]*PID_PERIOD;
  rangeAngle(&robotCoord[2]);

}
///////////////////////////////////////////////////////////////////////////////

void pidLowLevel(void) //���������� ��� ���������� �����
{
//Low level pid target values are set here__________________________________
  char i;
  for(i = 0; i < 4; i++)
  {

    wheelsPidStruct[i].target = regulatorOut[i];//�������� ��������� �������� �� ������������ ����������
    wheelsPidStruct[i].current = motorSpeed[i]; // ������� �������� ��������� �����
    pidCalc(&wheelsPidStruct[i]);
    if (curState.pidEnabled) setVoltage(WHEELS[i], wheelsPidStruct[i].output);
  }
}

void vectorAngle(float x, float y, float* angle)  //������� ���� �������
{
  float arctangensAlpha;

  if (x == 0.0)
  {
    if (y >= 0.0)
      *angle = PI/2.0;
    else
      *angle = 3.0/2.0*PI;
  }
  else
  if (y == 0.0)
  {
      if (x >= 0.0)
      *angle = 0.0;
    else
      *angle = PI;
  }
  else
  {
    arctangensAlpha = atanf(fabs(y/x));

    if ((x < 0.0) && (y > 0.0))
      *angle = PI - arctangensAlpha;
    else
      if ((x < 0.0) && (y < 0.0))
        *angle = PI + arctangensAlpha;
      else
        if ((x > 0.0) && (y < 0.0))
          *angle = 2.0*PI - arctangensAlpha;
        else
          *angle = arctangensAlpha;
  }
}
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////

void CreatePath(pathPointStr * next_point, pathPointStr * cur_point, Path * out) //������� ���������� ������ �������
{
  float delta_x;
  float delta_y;
  float delta_phi;



     delta_x = next_point->center[0]-cur_point->center[0];
     delta_y = next_point->center[1]-cur_point->center[1];
     delta_phi = next_point->center[2]-cur_point->center[2];
    // if (delta_phi<0 ) delta_phi+=2.0*PI;
     rangeAngle( &delta_phi);
     rangeAngle( &(next_point->center[2]));
  out->coordCenter[0] = cur_point->center[0];
  out->coordCenter[1] = cur_point->center[1];
  out->coordCenter[2] = cur_point->center[2];
  if (delta_x<0)
          out->alphZad = atan(delta_y/delta_x)+PI;
  else
      if (delta_x==0)
          {
              if (delta_y<0)
                {
                  out->alphZad = -PI/2;
                }
              else
                if (delta_y==0)
                {
                  out->alphZad=0;
                }
                else
                  {
                    out->alphZad = PI/2;
                  }
          }
       else
        if (delta_x>0)
          out->alphZad = atan(delta_y/delta_x);

 //if (next_point->rotateDir>0) out->phiZad =(delta_phi);
 // if (next_point->rotateDir<0) out->phiZad =fmod(2.0*PI-fabs(delta_phi),2.0*PI);
  out->phiZad =next_point->center[2];
  out->lengthTrace = sqrt((delta_x*delta_x)+(delta_y*delta_y));
  //out->phiDir = next_point->rotateDir;
if (out->lengthTrace == 0)
   out->traceVel=&standVelFast[0];
else
  out->traceVel=next_point->speedVelTipe;
if (delta_phi == 0)
   out->omegaVel=&standRotFast[0];
else
  out->omegaVel=next_point->speedRotTipe;

}

////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////

void SpeedFiltration(float *V,float *vF)
  {
        TargSpeed.x = V[0];
        TargSpeed.y = V[1];

         TVector temp;
        CurSpeed.x = vF[0];
        CurSpeed.y = vF[1];

        temp = subtraction(CurSpeed,TargSpeed);
        ACCEL_INC = pow(mod(temp) * 10.0, 2) / 10.0 + 0.001 + mod(CurAccel) / 2.0;
        if (ACCEL_INC > MAX_ACCEL_INC) { ACCEL_INC = MAX_ACCEL_INC;}
        if (ACCEL_INC < -MAX_ACCEL_INC) { ACCEL_INC = -MAX_ACCEL_INC;}
         TVector accelInc;
        temp = subtraction(TargSpeed, CurSpeed);
        temp = subtraction(temp, CurAccel);
        temp = normalization(temp, ACCEL_INC);
        accelInc = temp;
        temp = subtraction(TargSpeed, CurSpeed);
        temp = subtraction(CurAccel, temp );
        temp = normalization(temp, ACCEL_INC);
        TVector accelDec = temp;

        TVector newAccelInc, newAccelDec;
        newAccelInc = addition(CurAccel, accelInc);
        newAccelDec = addition(CurAccel, accelDec);

         TVector newSpeedInc, newSpeedDec, newSpeedZer;
        newSpeedInc = addition(CurSpeed, newAccelInc);
        newSpeedDec = addition(CurSpeed, newAccelDec);
        newSpeedZer = addition(CurSpeed, CurAccel);
        float timeToStop;
        if (mod(accelInc) > 0)
			 timeToStop = abs(mod(CurAccel)/mod(accelInc))/2.0 + 0.50;
        else timeToStop = 1;
            float incErr = mod(subtraction(TargSpeed,addition(newSpeedInc,
											addition(scale(newAccelInc,timeToStop),
													scale(accelInc,timeToStop*timeToStop/2.0)))));
        float decErr = mod(subtraction(TargSpeed,addition(newSpeedDec,
											addition(scale(newAccelDec,timeToStop),
													scale(accelDec,timeToStop*timeToStop/2.0)))));
        float zerErr = mod(subtraction(TargSpeed,addition(newSpeedDec,
											scale(newAccelDec,timeToStop))));
        if (incErr > decErr)
        {
            if (decErr < zerErr)
            {
                AccelInc1 = accelDec;
                CurSpeed = newSpeedDec;
                CurAccel = newAccelDec;
            }
            else
            {
                AccelInc1.x = 0;
                AccelInc1.y = 0;

                CurSpeed = newSpeedZer;
            }

        }
        else
        {
            if (incErr < zerErr) {
                AccelInc1 = accelInc;
                CurSpeed = newSpeedInc;
                CurAccel = newAccelInc;
            }
            else
            {
                AccelInc1.x = 0;
                AccelInc1.y = 0;
                CurSpeed =newSpeedZer;
            }
        }
        if (mod(CurAccel) > MAX_ACCEL)
   			    CurAccel = normalization(CurAccel, MAX_ACCEL);
        vF[0] = CurSpeed.x;
        vF[1] = CurSpeed.y;
        vF[2] = V[2];
  }

  ////////////////////////////////////////////////////////////////////////////////

  ////////////////////////////////////////////////////////////////////////////////

void collisionAvoidance(float * rV, float *V, float *vCA)
  {
      float realRad = robotCoord[2];
      float Mrot[3][3]   = {cos(realRad) , sin(realRad), 0,
                        -sin(realRad), cos(realRad), 0,
                                    0,            0, 1};
      float localVelocity[3];
      matrixMultiplyM2M(&Mrot[0][0], 3, 3, V, 3, 1, &localVelocity[0]); //Ml*Velocity speed in local coordinate system


      if (((!frontIR1) || (!frontIR2)) && (localVelocity[0] < -0.08))
      {
           vCA[0] = 0.0;
           vCA[1] = 0.0;
           vCA[2] = 0.0;
           collisionIsOn = 1;
      }
      else if ((distanceFromSonar[0] < 40) && (localVelocity[0] < -0.08))
      {
           vCA[0] = V[0]/2.0;
           vCA[1] = V[1]/2.0;
           vCA[2] = V[2]/2.0;
           collisionIsOn = 0;
      }
      else if ((!rightIR) && (localVelocity[1] < -0.08))
      {
          vCA[0] = 0.0;
          vCA[1] = 0.0;
          vCA[2] = 0.0;
          collisionIsOn = 1;
      }
      else if ((!leftIR) && (localVelocity[1] > 0.08))
      {
          vCA[0] = 0.0;
          vCA[1] = 0.0;
          vCA[2] = 0.0;
          collisionIsOn = 1;
      }
      else
      {
          vCA[0] = V[0];
          vCA[1] = V[1];
          vCA[2] = V[2];
          collisionIsOn = 0;
      }
  }
  ////////////////////////////////////////////////////////////////////////////////
