#ifndef _REGULATOR_INCLUDED_
#define _REGULATOR_INCLUDED_

#include "stm32f4xx.h"
#include "Path.h"

#define MAX_WHEEL_SPEED	    0.9// м/с
#define MAX_RAD_SPEED       4.0 // рад/с
#define LINE_SPEED_WEIGHT   0.8
#define ROTATE_SPEED_WEIGHT 1-LINE_SPEED_WEIGHT

#define JOYST_LIN_VEL_KOFF  MAX_WHEEL_SPEED/128.0
#define JOYST_TO_PWM        2.0
#define PHI_DEG             30
#define PHI_RAD             0.5236
#define distA	 	        0.31819
#define distB		        0.2625
#define RO                  (0.01695)           // Радиус колес
#define L                   (0.1)         // Расстояние от центра робота до плоскости колеса энкодера
#define JOYST_RAD_VEL_KOFF  MAX_RAD_SPEED/128.0 //(MAX_WHEEL_SPEED/distA/128.0/5.0)
//#define DISKR_TO_REAL       (2.0*PI*RO/106496.0)
#define DISKR_TO_REAL       (2.0*PI*RO/4096.0/26)
#define ONE_RO_COS_PHI      1.0                   // 23.0947
//#define COFF_TO_RAD         (2.0*PI/(4658.346666667))
#define MAX_CAPACITANCE     0.9 //(120.0*DISKR_TO_REAL/PID_PERIOD)*1.5   // Максимальный вектор задания на ПИДЫ
#define KOFF_ORTO_TRACE     0.9    // Коэффициент траекторной ошибки траекторного регулятора
#define PID_PERIOD          (1.0/100.0)

#define ENC_SET_CUR_POS   1
#define ENC_SET_GEOM   2

#define SEND_STATE   3

#define POINT_STACK_SIZE 30


typedef struct
{
  float p_k; //П коэфициент
  float i_k; //И коэфициент
  float d_k; //Д коэфициент
  float target; //Целевое значение
  float current; //Текущее (необходимо обновлять извне перед каждым расчетом)
  float prev_error; //Предыдущее значение ошибки (для Д регелятора)
  float sum_error; //Суммарная ошибка (для И регулятора)
  float max_sum_error; //Максимальная суммарная ошибка (что бы И регулятор не уходил до максимума если невозможно добиться требуемого значения)
  float max_output; //Максимальный выход, что бы поправка не выходила за рамки
  float min_output;
  float cut_output;
  float output; //Поправка, результат работы ПИД
  char pid_on; //Вкл/Выкл ПИД если выкл то output всегда равен 0, однако все остальное продолжает расчитываться
  char pid_finish;
  float error_dir;
  float pid_error_end;
  float pid_output_end;
} PidStruct;


typedef struct
{
  float center [3];
  char (*movTask)(void);
  char (*endTask)(void);
  float endTaskP1;
  float (*speedVelTipe);
  float (*speedRotTipe);
  char step;
  float lengthTrace;
}pathPointStr;

extern Path curPath;
//extern float Coord_local_track[3];
extern pathPointStr points[POINT_STACK_SIZE];
extern char lastPoint;
extern float normalVelFast[5];//V_уст, V_нач, V_кон, А_уск, А_торм
extern float stopVelFast[5]; //{0.2,0.1,-0.05,0.2,0.7};
extern float standVelFast[5];

extern float normalRotFast[5];//V_уст, V_нач, V_кон, А_уск, А_торм
extern float stopRotFast[5]; //{0.2,0.1,-0.1,0.3,0.6};
extern float standRotFast[5];
extern float * speedType[6];
extern float * rotType[6];
extern PidStruct wheelsPidStruct[4];
extern float vTargetGlob[3];
extern float vTargetGlobF[3];
extern float regulatorOut[4];
extern uint16_t totalPointComplite;
extern float MLineSpeed[4][3];          //
extern float MRotSpeed[4][3];           // Матрицы должны быть определены из вне
extern float InverseKinematics[4][4];   //


void pidCalc(PidStruct *pid_control); //Расчитать ПИД, в качестве параметра - указатель на структуру
void FunctionalRegulator(float *V_target, float *Coord_target, float *Coord_cur, float *V_out);
void pidWheelsFinishWait(void); // Ожидание окончания регулирования пидов колес
void pidLowLevel(void); // Пид нижнего уровня - колеса
void GetDataForRegulators(void);
void infnormvect(float *a,char rows,float *b);
void Cost(float *inpMatr,char rows,float cost,float *outKoff);
void Regulate(float *Coord_err, float *Speed_cur, float *tAlphZad,float *V_etalon,float *alphZad, float *V_local);
void TrackRegulator(float *Coord_cur, float* speedCur, Path *cur, float *V);//void TrackRegulator(float *Coord_cur, float *Coord_center, float *Alph_zad, float *Phi_zad, float *V_etalon, float *V);
//void TrackRegulator(float *Coord_cur,Path *cur, float *V);
float linars(float *x, float *x0, float *x1);
void Moving(float Coord_x_cur, float Coord_x_targ, float* parameters, float* v_out);
void RotMoving(float Start_a, float Coord_a_cur, float Coord_a_targ, float* parameters, float* v_out) ;//ðàñ÷åò ñêîðîñòè â çàâèñèìîñòè îò ïðîéäåííîãî ïóòè

void initRegulators(void);
void vectorAngle(float x, float y, float* angle);
void moving2(float Coord_x_cur, float Coord_x_targ, float* parameters, float* v_out);
signed char digitalize(char data, char lowerLevel, char upperLevel);
void CreatePath(pathPointStr * next_point, pathPointStr * cur_point, Path * out);
void removePoint(pathPointStr * points,char *lastPoint);
void SpeedFiltration(float *V,float *vF);



#endif
