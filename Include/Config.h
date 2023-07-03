#include <stdbool.h>
#include <stdint.h>
#include "hw_memmap.h"
#include "hw_types.h"
#include "hw_ints.h"
#include "hw_nvic.h"
#include "hw_uart.h"
#include "hw_gpio.h"
#include "gpio.h"
#include "uart.h"
#include "sysctl.h"
#include "interrupt.h"
#include "pin_map.h"
#include "timer.h"
#include "pwm.h"
#include "qei.h"
#include "uartstdio.h"
#include "fpu.h"

extern void TIMER0_IERHandler(void);

extern uint32_t left_encoder_count,right_encoder_count;	//±àÂëÆ÷¼ÆÊýÖµ
extern int line_det_data;
extern int num_det_data[4];
extern int cross_dis_ang[2];

extern int det_F;
extern int load_F;

extern float V_onrun;
void rigt_encoder_handler(void);
void left_encoder_handler(void);
void Config_Master(void);
void USART7_IRAHandler(void);
void USART5_IRAHandler(void);
void key_handler(void);
void UART_Config(void);
void QEI_Config(void);
void PWM_Config(void);
void PWM_duty(uint32_t ui32Base,uint32_t ui32PWMOut,uint32_t ui32PWMOutBits,uint32_t ui32Gen,float duty);
void TIM_Config(void);
void GPIO_Config(void);
