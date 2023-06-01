#ifndef _CONTROL_H_
#define _CONTROL_H_
#include <stdint.h>
#include <stdbool.h>
#include "Config.h"

#define OK 1
#define NOT_OK 2

#define DEADBAND (float)0.21

#define CORRIDOR_LENGTH_1 (float)60	//单位cm
#define CORRIDOR_LENGTH_2 (float)40
#define CORRIDOR_WIDTH	(float)30

#define Circumference 21	//轮子周长，21cm
#define d_distance	(float)0.053846	//四分频，一圈产生1560个脉冲

#define NUM_DET 'n'
#define LINE_DET 'f'
#define DET_OFF 's'

#define LINE_DET_POS 64
	
#define LEFT_QEI_NUM 1
#define RIGHT_QEI_NUM 2
#define LEFT_QEI QEI0_BASE
#define RIGHT_QEI QEI1_BASE
#define CONTROL_PERIOD (float)20	//控制周期20ms
	
#define RED_ON GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_3,8)
#define RED_OFF GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_3,0)
#define YELLOW_ON GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_3,8)
#define YELLOW_OFF GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_3,0)
#define GREEN_ON GPIOPinWrite(GPIO_PORTC_BASE,GPIO_PIN_4,16)
#define GREEN_OFF GPIOPinWrite(GPIO_PORTC_BASE,GPIO_PIN_4,0)
#define KEY_READ GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_2)

/* PID控制相关参数 */
typedef float PIDOut_Type;	//PID的输出值的数据类型
typedef float PIDIn_Type;	//PID的目标、误差的数据类型

typedef struct MOTOR{
	float Speed;	//速度，单位 m/s
	float Tar_Speed;	//电机目标速度
	float Dis;	//电机转过的线性距离，单位cm
	float Dis_Last;	//
	uint32_t QEIPostion;	//本次QEI的计数值
	uint32_t QEIPostion_Last;
	int32_t Dir;	//轮子转动方向，1为正转，-1为反转，0为停止
}motor;

typedef struct CAR{
//	motor *Left_Wheel;
//	motor *Right_Wheel;
	float Speed;	//小车速度
	float Angle;	//小车航向角
	float route;	//小车路程
}car;

typedef struct PID{
	float kp,ki,kd;
	PIDIn_Type target_val,cur_val;
	PIDIn_Type err,err_k1;
	PIDIn_Type i,i_max;
	PIDOut_Type max,min;
	PIDOut_Type output,output_last;
}pid;

extern motor left_motor, right_motor;
extern pid left_pid,right_pid;
extern pid pid_p;	//位置环PID
extern pid pid_a;	//角度环PID
extern pid pid_turn;	//原地转动PID
extern car Car;

void motor_Init(motor *);
void motor_speed_set(float vl,float vr);
void pid_init(pid *P,float p,float i,float d,PIDOut_Type max,PIDOut_Type min);
void pid_realize(pid *p, PIDIn_Type err);
void speed_cal(void);
float first_order_filter(float new_value,float last_value,float a);
void motor_pwm_set(float pwml,float pwmr);
void pid_reset(pid *);
void mission1(void);
void motor_reset(motor* M);
void car_Init(car *c);
float slow_start(float tar_v);
int Turn_90(int dir);
void Turn_180(void);
int point_to_point(float len);
float uint32_sub(uint32_t cur,uint32_t last);
void Encoder_Config(void);
void car_reset(void);
int point_to_point_backfoward(float len);
#endif
