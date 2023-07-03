#include "Config.h"
#include "control.h"
int line_det_data = LINE_DET_POS;
int num_det_data[4];
int cross_dis_ang[2];
uint32_t left_encoder_count,right_encoder_count;	//编码器计数值

//接收树莓派数据
void USART5_IRAHandler(void){
	int32_t rxbuf;
	static int flag = 0,len = 0;
	static int32_t data_str[7];
	uint32_t status = UARTIntStatus(UART5_BASE,true);
	UARTIntClear(UART5_BASE,status);
	
	/*****  *****/
	while(UARTCharsAvail(UART5_BASE)){
		rxbuf = UARTCharGetNonBlocking(UART5_BASE);
		if(flag==1){//接收巡线数据
			data_str[len] = rxbuf;
			len++;
//			UARTCharPutNonBlocking(UART0_BASE,rxbuf);
//			UARTCharPutNonBlocking(UART0_BASE,len);
			if(len==3) {
				flag = 0;
				line_det_data=0;
				//转换为整形
				line_det_data+=(data_str[0]-'0')*100;
				line_det_data+=(data_str[1]-'0')*10;
				line_det_data+=data_str[2]-'0';
			}
		}
		else if(flag == 2){//接收数字识别数据
			num_det_data[len] = rxbuf - '0';
			len ++;
			if(len==4) {
				flag = 0;
				//关闭视觉
				if(num_det_data[0]!=0){
					UARTCharPutNonBlocking(UART5_BASE,DET_OFF);
					det_F = 0;
				}
			}
		}
		else if(flag == 3){
			data_str[len] = rxbuf;
			len ++;
			if(len==6){
				flag = 0;
				cross_dis_ang[0]+=(data_str[0]-'0')*100;
				cross_dis_ang[0]+=(data_str[1]-'0')*10;
				cross_dis_ang[0]+=data_str[2]-'0';
				cross_dis_ang[1]+=(data_str[3]-'0')*100;
				cross_dis_ang[1]+=(data_str[4]-'0')*10;
				cross_dis_ang[1]+=data_str[5]-'0';
			}
		}
		else if(flag==0){
			//接收巡线数据
			if(rxbuf == 'f'){
				flag = 1;
				len = 0;
			}
			//接收数字识别数据
			else if(rxbuf == 'n'){
				flag = 2;
				len = 0;
			}
			else if(rxbuf == 'a'){
				flag = 3;
				len = 0;
				cross_dis_ang[0] = 0;
				cross_dis_ang[1] = 0;
			}
		}
		
	}
	/*****  *****/
}

void USART7_IRAHandler(void){
	uint32_t status = UARTIntStatus(UART7_BASE,true);
	UARTIntClear(UART7_BASE,status);
	
}

//左轮编码器
void left_encoder_handler(void){
	uint32_t status;
//	int i,j;
	status = GPIOIntStatus(GPIO_PORTF_BASE,true);
	GPIOIntClear(GPIO_PORTF_BASE,status);
	if((status&GPIO_PIN_0)==GPIO_PIN_0){
		if(GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_0)){
			if(GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_1)){
				left_encoder_count++;
				//UARTCharPutNonBlocking(UART0_BASE,'a');
			}
			else 
				//UARTCharPutNonBlocking(UART0_BASE,'b');
				left_encoder_count--;
		}
	}
}

//右轮编码器
void right_encoder_handler(void){
	uint32_t status;
	status = GPIOIntStatus(GPIO_PORTC_BASE,true);
	GPIOIntClear(GPIO_PORTC_BASE,status);
	if(status==GPIO_PIN_6){
		if(GPIOPinRead(GPIO_PORTC_BASE,GPIO_PIN_6)){
		//正转计数值增加
			if(GPIOPinRead(GPIO_PORTC_BASE,GPIO_PIN_5)){
				right_encoder_count++;
			}
			else right_encoder_count--;
		}
	}
}

void key_handler(void){
	uint32_t status;
	//获取中断标志位
	status=GPIOIntStatus(GPIO_PORTF_BASE, true);
	GPIOIntClear(GPIO_PORTF_BASE, status);
//	if(status==GPIO_PIN_2){
//		load_F = 1;
//		if(GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_2)==0){
//			load_F = 0;
//		}
//	}
	//左轮测速
	if(status==GPIO_PIN_0){
		if(GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_0))
		{
			left_encoder_count++;
		}
		else left_encoder_count--;
	}
}

/**
  * @brief  外设初始化总函数
  * @param  None
  * @retval None
  */
void Config_Master(void){
	TIM_Config();
	PWM_Config();
	//QEI_Config();
	UART_Config();
	GPIO_Config();
	Encoder_Config();
}

/**
  * @brief  TIM0初始化
						TIM0周期：50Hz
  * @param  None
  * @retval None
  */
void TIM_Config(void){
	//使能外设TIMER0
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
	//配置定时器，不拆分，向上计数，此时B作为A的拓展，32位计数器，不能预分配
	TimerConfigure(TIMER0_BASE,TIMER_CFG_PERIODIC_UP);
	//设置装载值
	TimerLoadSet(TIMER0_BASE,TIMER_A,SysCtlClockGet()/50-1);
	//注册中断服务函数
	TimerIntRegister(TIMER0_BASE,TIMER_A,TIMER0_IERHandler);
	//开启定时器A超时中断
	TimerIntEnable(TIMER0_BASE,TIMER_TIMA_TIMEOUT);
	//设置中断优先级
	IntPrioritySet(INT_TIMER0A, 0);
	//使能中断
	IntEnable(INT_TIMER0A);
	IntMasterEnable();
	//使能定时器
	TimerEnable( TIMER0_BASE, TIMER_A);
}

/**
  * @brief  PWM发生器初始化
						电机M1：B6,B7；电机M2：D0，D1
  * @param  None
  * @retval None
  */
void PWM_Config(void)
{
  //因为设置了时钟总线是80MHz，所以在这里分一下频设置为4分频，那么PWM时钟就是20MHz
	SysCtlPWMClockSet(SYSCTL_PWMDIV_4);
  //使能GPIO与PWM
  SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOB);
	SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOD);
	SysCtlPeripheralEnable( SYSCTL_PERIPH_PWM0);
  //为GPIO口分配PWM信号
	GPIOPinTypePWM( GPIO_PORTB_BASE,  GPIO_PIN_6);
	GPIOPinTypePWM( GPIO_PORTB_BASE,  GPIO_PIN_7);
	GPIOPinTypePWM( GPIO_PORTD_BASE,  GPIO_PIN_0);
	GPIOPinTypePWM( GPIO_PORTD_BASE,  GPIO_PIN_1);
  //对GPIO口使能引脚复用
	GPIOPinConfigure(GPIO_PB6_M0PWM0);
	GPIOPinConfigure(GPIO_PB7_M0PWM1);
	GPIOPinConfigure(GPIO_PD0_M0PWM6);
	GPIOPinConfigure(GPIO_PD1_M0PWM7);
  //设置PWM模块的发生器为向下计数与不同步计数
	PWMGenConfigure(PWM0_BASE,  PWM_GEN_0,PWM_GEN_MODE_DOWN|PWM_GEN_MODE_NO_SYNC);
	PWMGenConfigure(PWM0_BASE,  PWM_GEN_3,PWM_GEN_MODE_DOWN|PWM_GEN_MODE_NO_SYNC);
  //设置PWM模块的发生器每个计数周期为20000个数，而PWM时钟为20MHz，分频得到1KHz
	PWMGenPeriodSet( PWM0_BASE,  PWM_GEN_0,20000-1);
	PWMGenPeriodSet( PWM0_BASE,  PWM_GEN_3,20000-1);
	
	PWMPulseWidthSet( PWM0_BASE,  PWM_OUT_0,(uint32_t)((PWMGenPeriodGet(PWM0_BASE, PWM_GEN_0)+1)*0.5-1.0));
	PWMPulseWidthSet( PWM0_BASE,  PWM_OUT_1,(uint32_t)((PWMGenPeriodGet(PWM0_BASE, PWM_GEN_0)+1)*0.5-1.0));
	PWMPulseWidthSet( PWM0_BASE,  PWM_OUT_6,(uint32_t)((PWMGenPeriodGet(PWM0_BASE, PWM_GEN_3)+1)*0.5-1.0));
	PWMPulseWidthSet( PWM0_BASE,  PWM_OUT_7,(uint32_t)((PWMGenPeriodGet(PWM0_BASE, PWM_GEN_3)+1)*0.5-1.0));
  //使能第六个通道
	PWMOutputState( PWM0_BASE,  PWM_OUT_0_BIT,false);
	PWMOutputState( PWM0_BASE,  PWM_OUT_1_BIT,false);
	PWMOutputState( PWM0_BASE,  PWM_OUT_6_BIT,false);
	PWMOutputState( PWM0_BASE,  PWM_OUT_7_BIT,false);
  //使能第三个发生器
	PWMGenEnable( PWM0_BASE,  PWM_GEN_0);
	PWMGenEnable( PWM0_BASE,  PWM_GEN_3);
}

/**
  * @brief  设置PWM占空比
  * @param  ui32Base PWM发生器编号
	* @param	ui32PWMOut PWM输出口编号
	* @param	ui32PWMOutBits PWM输出口掩码
	* @param  duty 占空比，范围0-1（不能取0）；duty = 0时关闭PWM输出
  * @retval None
  */
void PWM_duty(uint32_t ui32Base,uint32_t ui32PWMOut,uint32_t ui32PWMOutBits,uint32_t ui32Gen,float duty)
{
	
	if(duty>0){
		PWMPulseWidthSet( ui32Base,  ui32PWMOut,(uint32_t)((PWMGenPeriodGet(ui32Base, ui32Gen)+1)*duty - (float)1.0));
		PWMOutputState( ui32Base,  ui32PWMOutBits,true);
		PWMGenEnable( ui32Base,  ui32Gen);
	}
	else{
		PWMOutputState( ui32Base,  ui32PWMOutBits,false);
		PWMGenEnable( ui32Base,  ui32Gen);
	}
}

/**
  * @brief  编码器初始化
						左轮：F0：A相 F1：B相
						右轮：C6：A相 C5：B相
  * @param  None
  * @retval None
  */
void Encoder_Config(void){
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF)){}
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOC)){}
	
	//解锁PF0引脚
	HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
	HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;//0x80：PD7
	HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;
	
	GPIODirModeSet(GPIO_PORTF_BASE,GPIO_PIN_0|GPIO_PIN_1,GPIO_DIR_MODE_IN);
	GPIODirModeSet(GPIO_PORTC_BASE,GPIO_PIN_5|GPIO_PIN_6,GPIO_DIR_MODE_IN);
	
	GPIOPadConfigSet(GPIO_PORTF_BASE,GPIO_PIN_0,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPD);
	GPIOPadConfigSet(GPIO_PORTF_BASE,GPIO_PIN_1,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPD);
	GPIOPadConfigSet(GPIO_PORTC_BASE,GPIO_PIN_5,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPD);
	GPIOPadConfigSet(GPIO_PORTC_BASE,GPIO_PIN_6,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPD);
	
	GPIOIntTypeSet(GPIO_PORTF_BASE,GPIO_PIN_0,GPIO_RISING_EDGE);
	GPIOIntTypeSet(GPIO_PORTC_BASE,GPIO_PIN_6,GPIO_RISING_EDGE);
	
	GPIOIntRegister(GPIO_PORTF_BASE,left_encoder_handler);
	GPIOIntRegister(GPIO_PORTC_BASE,right_encoder_handler);
	
	GPIOIntEnable(GPIO_PORTF_BASE,GPIO_PIN_0);
	GPIOIntEnable(GPIO_PORTC_BASE,GPIO_PIN_6);
	
	IntEnable(INT_GPIOF);
	IntEnable(INT_GPIOC);
	IntMasterEnable();
	GPIOIntClear(GPIO_PORTC_BASE,GPIO_PIN_6);
	GPIOIntClear(GPIO_PORTF_BASE,GPIO_PIN_0);
	IntPrioritySet(INT_GPIOF,0x07);
	IntPrioritySet(INT_GPIOC,0x07);
	
	left_encoder_count=0x7fffffff;
	right_encoder_count=0x7fffffff;
}

/**
  * @brief  正交编码器QEI初始化，初始化QEI0和QEI1
						QEI0：F0：A相 F1：B相
						QEI1：C6：A相 C5：B相
  * @param  None
  * @retval None
  */
void QEI_Config(void){
	//使能QEI和GPIO
	SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI1);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	
	//解锁PF0引脚
	HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
	HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;//0x80：PD7
	HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;
	
	//设置GPIO复用
	GPIOPinTypeQEI(GPIO_PORTF_BASE,GPIO_PIN_0);
	GPIOPinTypeQEI(GPIO_PORTF_BASE,GPIO_PIN_1);
	GPIOPinConfigure(GPIO_PF0_PHA0);//PF0->编码器A相输入
	GPIOPinConfigure(GPIO_PF1_PHB0);//PF1->编码器B相输入
	GPIOPinTypeQEI(GPIO_PORTC_BASE,GPIO_PIN_6);
	GPIOPinTypeQEI(GPIO_PORTC_BASE,GPIO_PIN_5);
	GPIOPinConfigure(GPIO_PC5_PHA1);//PC5->编码器A相输入
	GPIOPinConfigure(GPIO_PC6_PHB1);//PF1->编码器B相输入

	GPIOPadConfigSet(GPIO_PORTF_BASE,GPIO_PIN_0,GPIO_STRENGTH_4MA,GPIO_PIN_TYPE_STD_WPD);
	GPIOPadConfigSet(GPIO_PORTF_BASE,GPIO_PIN_1,GPIO_STRENGTH_4MA,GPIO_PIN_TYPE_STD_WPD);
	GPIOPadConfigSet(GPIO_PORTC_BASE,GPIO_PIN_5,GPIO_STRENGTH_4MA,GPIO_PIN_TYPE_STD_WPD);
	GPIOPadConfigSet(GPIO_PORTC_BASE,GPIO_PIN_6,GPIO_STRENGTH_4MA,GPIO_PIN_TYPE_STD_WPD);
	
	//配置QEI模块，A、B两相信号边沿均产生计数，不开启索引脉冲复位，正交相位模式，A、B两相信号不交换，最大计数值为0xffffffff
	//正交相位模式下，默认四分频
	QEIConfigure(QEI0_BASE,QEI_CONFIG_CAPTURE_A_B|QEI_CONFIG_NO_RESET|QEI_CONFIG_QUADRATURE|QEI_CONFIG_SWAP,0xffffffff);
	QEIConfigure(QEI1_BASE,QEI_CONFIG_CAPTURE_A_B|QEI_CONFIG_NO_RESET|QEI_CONFIG_QUADRATURE|QEI_CONFIG_NO_SWAP,0xffffffff);
	//禁止QEI中断
	QEIIntDisable(QEI0_BASE,QEI_INTERROR|QEI_INTDIR|QEI_INTINDEX|QEI_INTTIMER);
	QEIIntDisable(QEI1_BASE,QEI_INTERROR|QEI_INTDIR|QEI_INTINDEX|QEI_INTTIMER);
	//使能QEI
	QEIEnable(QEI0_BASE);
	QEIEnable(QEI1_BASE);
	//设置QEI计数器值在中间
	QEIPositionSet(QEI0_BASE,0x7fffffff);
	QEIPositionSet(QEI1_BASE,0x7fffffff);
}

/**
  * @brief  GPIO口初始化
						绿灯：C4，红灯：B3，黄灯：F3
						压感开关：F2
						任务选择开关：S0：A2，S1：A3，S2：A4
  * @param  None
  * @retval None
  */
void GPIO_Config(void){
	//C4,B3,F3输出
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE,GPIO_PIN_4);
	GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE,GPIO_PIN_3);
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE,GPIO_PIN_3);
	GPIOPinWrite(GPIO_PORTC_BASE,GPIO_PIN_4,0);
	GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_3,0);
	GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_3,0);
	//任务选择开关：S0：A2，S1：A3，S2：A4，输入，内部上拉
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	GPIODirModeSet(GPIO_PORTA_BASE,GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4,GPIO_DIR_MODE_IN);
	GPIOPadConfigSet(GPIO_PORTA_BASE,GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
	//压感开关：F2
	GPIODirModeSet(GPIO_PORTF_BASE,GPIO_PIN_2,GPIO_DIR_MODE_IN);
	GPIOPadConfigSet(GPIO_PORTF_BASE,GPIO_PIN_2,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
//	//配置引脚中断
//	GPIOIntTypeSet(GPIO_PORTF_BASE,GPIO_PIN_2,GPIO_BOTH_EDGES);	//上升沿和下降沿都触发
//	GPIOIntRegister(GPIO_PORTF_BASE,key_handler);
//	GPIOIntEnable(GPIO_PORTF_BASE,GPIO_PIN_2);
//	IntEnable(INT_GPIOF);
//	IntMasterEnable();
//	GPIOIntClear(GPIO_PORTF_BASE,GPIO_PIN_2);
//	IntPrioritySet(INT_GPIOF,0xE0);
}

/**
  * @brief  串口初始化
  * @param  None
  * @retval None
  */
void UART_Config(void){
	//调试串口U0
	//使能GPIO
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	//使能UART
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	// 为UART配置GPIO引脚。
	GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);
	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	//配置串口波特时钟源及参数
//	UARTClockSourceSet(UART0_BASE,UART_CLOCK_SYSTEM);
	UARTStdioConfig(0,115200,SysCtlClockGet());

	
	//树莓派串口U5
	//使能GPIO
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	//使能UART
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART5);
	// 为UART配置GPIO引脚。
	GPIOPinConfigure(GPIO_PE4_U5RX);
	GPIOPinConfigure(GPIO_PE5_U5TX);
	GPIOPinTypeUART(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5);
	//配置串口波特时钟源及参数
	UARTClockSourceSet(UART5_BASE,UART_CLOCK_SYSTEM);
	UARTConfigSetExpClk(UART5_BASE,SysCtlClockGet(),115200,UART_CONFIG_WLEN_8|UART_CONFIG_STOP_ONE|UART_CONFIG_PAR_NONE);
	//使能FIFO并设置FIFO深度
	UARTFIFOEnable(UART5_BASE);
	UARTFIFOLevelSet(UART5_BASE,UART_FIFO_RX1_8,UART_FIFO_TX1_8);
	//配置串口中断
	UARTIntRegister(UART5_BASE,USART5_IRAHandler);
	//开启串口接收中断和接收超时中断
	UARTIntEnable(UART5_BASE, UART_INT_RX|UART_INT_RT);
	UARTIntClear(UART5_BASE, UART5_BASE);
	//启用串口0的中断并开启处理器中断控制器
	IntEnable(INT_UART5);
//	
//	//双车通讯串口U7
//	//使能GPIO
//	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
//	//使能UART
//	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART7);
//	// 为UART配置GPIO引脚。
//	GPIOPinConfigure(GPIO_PE0_U7RX);
//	GPIOPinConfigure(GPIO_PE1_U7TX);
//	GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1);
//	//配置串口波特时钟源及参数
//	UARTClockSourceSet(UART7_BASE,UART_CLOCK_SYSTEM);
//	UARTConfigSetExpClk(UART7_BASE,SysCtlClockGet(),9600,UART_CONFIG_WLEN_8|UART_CONFIG_STOP_ONE|UART_CONFIG_PAR_NONE);
//	//使能FIFO并设置FIFO深度
//	UARTFIFOEnable(UART7_BASE);
//	UARTFIFOLevelSet(UART7_BASE,UART_FIFO_RX1_8,UART_FIFO_TX1_8);
//	//配置串口中断
//	UARTIntRegister(UART7_BASE,USART7_IRAHandler);
//	//开启串口接收中断和接收超时中断
//	UARTIntEnable(UART7_BASE, UART_INT_RX|UART_INT_RT);
//	UARTIntClear(UART7_BASE, UART2_BASE);
//	//启用串口0的中断并开启处理器中断控制器
//	IntEnable(INT_UART7);
//	IntMasterEnable();
	
	UARTEnable(UART0_BASE);
	UARTEnable(UART5_BASE);
//	UARTEnable(UART7_BASE);
	
}
