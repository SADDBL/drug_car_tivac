//#include <stdbool.h>
//#include <stdint.h>
//#include "inc/hw_memmap.h"
//#include "inc/hw_types.h"
//#include "inc/hw_ints.h"
//#include "inc/hw_nvic.h"
//#include "inc/hw_uart.h"
//#include "inc/hw_gpio.h"
//#include "driverlib/gpio.h"
//#include "driverlib/uart.h"
//#include "driverlib/sysctl.h"
//#include "driverlib/interrupt.h"
//#include "driverlib/pin_map.h"
//#include "timer.h"
//#include "driverlib/pwm.h"
//#include "driverlib/qei.h"
//#include "utils/uartstdio.h"
#include "Config.h"
#include "control.h"
#include <stdio.h>
int fputc(int ch, FILE *f)
{
	UARTCharPut(UART0_BASE,(uint8_t)ch);
	return ch;
}

car Car;
motor left_motor, right_motor;
pid left_pid,right_pid;	//速度环PID
pid pid_p;	//位置环PID
pid pid_a;	//角度环PID
pid pid_b;	//倒车角度环PID
pid pid_turn;	//原地转动PID
float V_onrun=0.5;
void TIMER0_IERHandler(void);
int count=0;
int mission_select=0;
int main(void)
{
//	float vpl = 16,vil = 0,vdl = 0.04;	//左轮PID
//	float vpr = 16,vir = 0,vdr = 0.04;	//右轮PID
	float vpl = 6,vil = 3,vdl = 0.8;	//左轮PID
	float vpr = 6,vir = 3,vdr = 0.8;	//左轮PID
//	float vpl = 4,vil = 3.5,vdl = 0.8;	//左轮PID
//	float vpr = 4,vir = 3.5,vdr = 0.8;	//右轮PID
//	float vpl = 4,vil = 3.5,vdl = 0.8;	//左轮PID
//	float vpr = 4,vir = 3.5,vdr = 0.8;	//右轮PID
	float pp = 0.07,pi = 0,pd = 0.0;			//位置环PID
	float ap = 0.01,ai = 0,ad =0.00;			//角度环PID
	float bp = 0.004,bi = 0,bd =-0.0001;			//角度环PID
	float tp = 0.008,ti = 0,td =0;			//原地转动PID
	int i,j;
	int32_t pin_read=0;
	//使用PLL输出频率200MHz，第一个系数2.5分频，得到80MHz
  SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);
  //外设初始化
	Config_Master();
	FPUEnable();
	FPULazyStackingEnable();
	
	motor_Init(&left_motor);
	motor_Init(&right_motor);
	pid_init(&left_pid,vpl,vdl,vil,(float)0.99-DEADBAND,(float)DEADBAND-0.99);
	pid_init(&right_pid,vpr,vdr,vir,(float)0.99-DEADBAND,(float)DEADBAND-0.99);
	pid_init(&pid_a,ap,ad,ai,(float)0.5,(float)-0.5);
	pid_init(&pid_b,bp,bd,bi,(float)0.5,(float)-0.5);
	pid_init(&pid_p,pp,pd,pi,(float)0.5,(float)-0.5);
	pid_init(&pid_turn,tp,td,ti,(float)0.5,(float)-0.5);
	
	pin_read = GPIOPinRead(GPIO_PORTA_BASE,GPIO_PIN_2);
	if(!pin_read) mission_select+=1;
	pin_read = GPIOPinRead(GPIO_PORTA_BASE,GPIO_PIN_3);
	if(!pin_read) mission_select+=2;
	pin_read = GPIOPinRead(GPIO_PORTA_BASE,GPIO_PIN_4);
	if(!pin_read) mission_select+=4;

  while(1){
	//	printf("pwm: %f, %f\r\n",left_pid.output,right_pid.output);
	//	printf("dv: %f\r\n",pid_a.output);
	//	printf("vl: %f, %f, %f\r\n",left_motor.Speed,right_motor.Speed,V_onrun);
	//	printf("i=%f\r\n",left_pid.i);
	//	printf("qei: %u, %u, %u\r\n",left_encoder_count,right_encoder_count,right_encoder_count-left_encoder_count);
	//	printf("a: %f, %f, %f\r\n",Car.Angle,-90.0,pid_turn.output);
	//	printf("cam: %d\r\n",line_det_data);
	//	printf("%d\r\n",0);
		//printf("p: %f\r\n",pid_p.output);
	//	printf("route: %f, %f\r\n",Car.route,90.0);
	//	UARTCharPutNonBlocking(UART0_BASE,'a');
		
	}
}

//TIMER0中断服务函数
void TIMER0_IERHandler(void)
{
	static int flag = NOT_OK;
	//static int v;
	uint32_t status=TimerIntStatus( TIMER0_BASE,  true);
	TimerIntClear( TIMER0_BASE,  status);
	
	/***** PID控制环节 *****/
	if(mission_select==1)
		mission1();
	else if(mission_select==2)
		mission2();
	else if(mission_select==4)
		mission3();
	
	//point_to_point_backfoward(3*CORRIDOR_LENGTH_1+CORRIDOR_WIDTH*2+10);
	//point_to_point(15);
}
