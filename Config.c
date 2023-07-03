#include "Config.h"
#include "control.h"
int line_det_data = LINE_DET_POS;
int num_det_data[4];
int cross_dis_ang[2];
uint32_t left_encoder_count,right_encoder_count;	//����������ֵ

//������ݮ������
void USART5_IRAHandler(void){
	int32_t rxbuf;
	static int flag = 0,len = 0;
	static int32_t data_str[7];
	uint32_t status = UARTIntStatus(UART5_BASE,true);
	UARTIntClear(UART5_BASE,status);
	
	/*****  *****/
	while(UARTCharsAvail(UART5_BASE)){
		rxbuf = UARTCharGetNonBlocking(UART5_BASE);
		if(flag==1){//����Ѳ������
			data_str[len] = rxbuf;
			len++;
//			UARTCharPutNonBlocking(UART0_BASE,rxbuf);
//			UARTCharPutNonBlocking(UART0_BASE,len);
			if(len==3) {
				flag = 0;
				line_det_data=0;
				//ת��Ϊ����
				line_det_data+=(data_str[0]-'0')*100;
				line_det_data+=(data_str[1]-'0')*10;
				line_det_data+=data_str[2]-'0';
			}
		}
		else if(flag == 2){//��������ʶ������
			num_det_data[len] = rxbuf - '0';
			len ++;
			if(len==4) {
				flag = 0;
				//�ر��Ӿ�
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
			//����Ѳ������
			if(rxbuf == 'f'){
				flag = 1;
				len = 0;
			}
			//��������ʶ������
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

//���ֱ�����
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

//���ֱ�����
void right_encoder_handler(void){
	uint32_t status;
	status = GPIOIntStatus(GPIO_PORTC_BASE,true);
	GPIOIntClear(GPIO_PORTC_BASE,status);
	if(status==GPIO_PIN_6){
		if(GPIOPinRead(GPIO_PORTC_BASE,GPIO_PIN_6)){
		//��ת����ֵ����
			if(GPIOPinRead(GPIO_PORTC_BASE,GPIO_PIN_5)){
				right_encoder_count++;
			}
			else right_encoder_count--;
		}
	}
}

void key_handler(void){
	uint32_t status;
	//��ȡ�жϱ�־λ
	status=GPIOIntStatus(GPIO_PORTF_BASE, true);
	GPIOIntClear(GPIO_PORTF_BASE, status);
//	if(status==GPIO_PIN_2){
//		load_F = 1;
//		if(GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_2)==0){
//			load_F = 0;
//		}
//	}
	//���ֲ���
	if(status==GPIO_PIN_0){
		if(GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_0))
		{
			left_encoder_count++;
		}
		else left_encoder_count--;
	}
}

/**
  * @brief  �����ʼ���ܺ���
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
  * @brief  TIM0��ʼ��
						TIM0���ڣ�50Hz
  * @param  None
  * @retval None
  */
void TIM_Config(void){
	//ʹ������TIMER0
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
	//���ö�ʱ��������֣����ϼ�������ʱB��ΪA����չ��32λ������������Ԥ����
	TimerConfigure(TIMER0_BASE,TIMER_CFG_PERIODIC_UP);
	//����װ��ֵ
	TimerLoadSet(TIMER0_BASE,TIMER_A,SysCtlClockGet()/50-1);
	//ע���жϷ�����
	TimerIntRegister(TIMER0_BASE,TIMER_A,TIMER0_IERHandler);
	//������ʱ��A��ʱ�ж�
	TimerIntEnable(TIMER0_BASE,TIMER_TIMA_TIMEOUT);
	//�����ж����ȼ�
	IntPrioritySet(INT_TIMER0A, 0);
	//ʹ���ж�
	IntEnable(INT_TIMER0A);
	IntMasterEnable();
	//ʹ�ܶ�ʱ��
	TimerEnable( TIMER0_BASE, TIMER_A);
}

/**
  * @brief  PWM��������ʼ��
						���M1��B6,B7�����M2��D0��D1
  * @param  None
  * @retval None
  */
void PWM_Config(void)
{
  //��Ϊ������ʱ��������80MHz�������������һ��Ƶ����Ϊ4��Ƶ����ôPWMʱ�Ӿ���20MHz
	SysCtlPWMClockSet(SYSCTL_PWMDIV_4);
  //ʹ��GPIO��PWM
  SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOB);
	SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOD);
	SysCtlPeripheralEnable( SYSCTL_PERIPH_PWM0);
  //ΪGPIO�ڷ���PWM�ź�
	GPIOPinTypePWM( GPIO_PORTB_BASE,  GPIO_PIN_6);
	GPIOPinTypePWM( GPIO_PORTB_BASE,  GPIO_PIN_7);
	GPIOPinTypePWM( GPIO_PORTD_BASE,  GPIO_PIN_0);
	GPIOPinTypePWM( GPIO_PORTD_BASE,  GPIO_PIN_1);
  //��GPIO��ʹ�����Ÿ���
	GPIOPinConfigure(GPIO_PB6_M0PWM0);
	GPIOPinConfigure(GPIO_PB7_M0PWM1);
	GPIOPinConfigure(GPIO_PD0_M0PWM6);
	GPIOPinConfigure(GPIO_PD1_M0PWM7);
  //����PWMģ��ķ�����Ϊ���¼����벻ͬ������
	PWMGenConfigure(PWM0_BASE,  PWM_GEN_0,PWM_GEN_MODE_DOWN|PWM_GEN_MODE_NO_SYNC);
	PWMGenConfigure(PWM0_BASE,  PWM_GEN_3,PWM_GEN_MODE_DOWN|PWM_GEN_MODE_NO_SYNC);
  //����PWMģ��ķ�����ÿ����������Ϊ20000��������PWMʱ��Ϊ20MHz����Ƶ�õ�1KHz
	PWMGenPeriodSet( PWM0_BASE,  PWM_GEN_0,20000-1);
	PWMGenPeriodSet( PWM0_BASE,  PWM_GEN_3,20000-1);
	
	PWMPulseWidthSet( PWM0_BASE,  PWM_OUT_0,(uint32_t)((PWMGenPeriodGet(PWM0_BASE, PWM_GEN_0)+1)*0.5-1.0));
	PWMPulseWidthSet( PWM0_BASE,  PWM_OUT_1,(uint32_t)((PWMGenPeriodGet(PWM0_BASE, PWM_GEN_0)+1)*0.5-1.0));
	PWMPulseWidthSet( PWM0_BASE,  PWM_OUT_6,(uint32_t)((PWMGenPeriodGet(PWM0_BASE, PWM_GEN_3)+1)*0.5-1.0));
	PWMPulseWidthSet( PWM0_BASE,  PWM_OUT_7,(uint32_t)((PWMGenPeriodGet(PWM0_BASE, PWM_GEN_3)+1)*0.5-1.0));
  //ʹ�ܵ�����ͨ��
	PWMOutputState( PWM0_BASE,  PWM_OUT_0_BIT,false);
	PWMOutputState( PWM0_BASE,  PWM_OUT_1_BIT,false);
	PWMOutputState( PWM0_BASE,  PWM_OUT_6_BIT,false);
	PWMOutputState( PWM0_BASE,  PWM_OUT_7_BIT,false);
  //ʹ�ܵ�����������
	PWMGenEnable( PWM0_BASE,  PWM_GEN_0);
	PWMGenEnable( PWM0_BASE,  PWM_GEN_3);
}

/**
  * @brief  ����PWMռ�ձ�
  * @param  ui32Base PWM���������
	* @param	ui32PWMOut PWM����ڱ��
	* @param	ui32PWMOutBits PWM���������
	* @param  duty ռ�ձȣ���Χ0-1������ȡ0����duty = 0ʱ�ر�PWM���
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
  * @brief  ��������ʼ��
						���֣�F0��A�� F1��B��
						���֣�C6��A�� C5��B��
  * @param  None
  * @retval None
  */
void Encoder_Config(void){
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF)){}
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOC)){}
	
	//����PF0����
	HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
	HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;//0x80��PD7
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
  * @brief  ����������QEI��ʼ������ʼ��QEI0��QEI1
						QEI0��F0��A�� F1��B��
						QEI1��C6��A�� C5��B��
  * @param  None
  * @retval None
  */
void QEI_Config(void){
	//ʹ��QEI��GPIO
	SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI1);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	
	//����PF0����
	HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
	HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;//0x80��PD7
	HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;
	
	//����GPIO����
	GPIOPinTypeQEI(GPIO_PORTF_BASE,GPIO_PIN_0);
	GPIOPinTypeQEI(GPIO_PORTF_BASE,GPIO_PIN_1);
	GPIOPinConfigure(GPIO_PF0_PHA0);//PF0->������A������
	GPIOPinConfigure(GPIO_PF1_PHB0);//PF1->������B������
	GPIOPinTypeQEI(GPIO_PORTC_BASE,GPIO_PIN_6);
	GPIOPinTypeQEI(GPIO_PORTC_BASE,GPIO_PIN_5);
	GPIOPinConfigure(GPIO_PC5_PHA1);//PC5->������A������
	GPIOPinConfigure(GPIO_PC6_PHB1);//PF1->������B������

	GPIOPadConfigSet(GPIO_PORTF_BASE,GPIO_PIN_0,GPIO_STRENGTH_4MA,GPIO_PIN_TYPE_STD_WPD);
	GPIOPadConfigSet(GPIO_PORTF_BASE,GPIO_PIN_1,GPIO_STRENGTH_4MA,GPIO_PIN_TYPE_STD_WPD);
	GPIOPadConfigSet(GPIO_PORTC_BASE,GPIO_PIN_5,GPIO_STRENGTH_4MA,GPIO_PIN_TYPE_STD_WPD);
	GPIOPadConfigSet(GPIO_PORTC_BASE,GPIO_PIN_6,GPIO_STRENGTH_4MA,GPIO_PIN_TYPE_STD_WPD);
	
	//����QEIģ�飬A��B�����źű��ؾ������������������������帴λ��������λģʽ��A��B�����źŲ�������������ֵΪ0xffffffff
	//������λģʽ�£�Ĭ���ķ�Ƶ
	QEIConfigure(QEI0_BASE,QEI_CONFIG_CAPTURE_A_B|QEI_CONFIG_NO_RESET|QEI_CONFIG_QUADRATURE|QEI_CONFIG_SWAP,0xffffffff);
	QEIConfigure(QEI1_BASE,QEI_CONFIG_CAPTURE_A_B|QEI_CONFIG_NO_RESET|QEI_CONFIG_QUADRATURE|QEI_CONFIG_NO_SWAP,0xffffffff);
	//��ֹQEI�ж�
	QEIIntDisable(QEI0_BASE,QEI_INTERROR|QEI_INTDIR|QEI_INTINDEX|QEI_INTTIMER);
	QEIIntDisable(QEI1_BASE,QEI_INTERROR|QEI_INTDIR|QEI_INTINDEX|QEI_INTTIMER);
	//ʹ��QEI
	QEIEnable(QEI0_BASE);
	QEIEnable(QEI1_BASE);
	//����QEI������ֵ���м�
	QEIPositionSet(QEI0_BASE,0x7fffffff);
	QEIPositionSet(QEI1_BASE,0x7fffffff);
}

/**
  * @brief  GPIO�ڳ�ʼ��
						�̵ƣ�C4����ƣ�B3���Ƶƣ�F3
						ѹ�п��أ�F2
						����ѡ�񿪹أ�S0��A2��S1��A3��S2��A4
  * @param  None
  * @retval None
  */
void GPIO_Config(void){
	//C4,B3,F3���
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE,GPIO_PIN_4);
	GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE,GPIO_PIN_3);
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE,GPIO_PIN_3);
	GPIOPinWrite(GPIO_PORTC_BASE,GPIO_PIN_4,0);
	GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_3,0);
	GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_3,0);
	//����ѡ�񿪹أ�S0��A2��S1��A3��S2��A4�����룬�ڲ�����
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	GPIODirModeSet(GPIO_PORTA_BASE,GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4,GPIO_DIR_MODE_IN);
	GPIOPadConfigSet(GPIO_PORTA_BASE,GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
	//ѹ�п��أ�F2
	GPIODirModeSet(GPIO_PORTF_BASE,GPIO_PIN_2,GPIO_DIR_MODE_IN);
	GPIOPadConfigSet(GPIO_PORTF_BASE,GPIO_PIN_2,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
//	//���������ж�
//	GPIOIntTypeSet(GPIO_PORTF_BASE,GPIO_PIN_2,GPIO_BOTH_EDGES);	//�����غ��½��ض�����
//	GPIOIntRegister(GPIO_PORTF_BASE,key_handler);
//	GPIOIntEnable(GPIO_PORTF_BASE,GPIO_PIN_2);
//	IntEnable(INT_GPIOF);
//	IntMasterEnable();
//	GPIOIntClear(GPIO_PORTF_BASE,GPIO_PIN_2);
//	IntPrioritySet(INT_GPIOF,0xE0);
}

/**
  * @brief  ���ڳ�ʼ��
  * @param  None
  * @retval None
  */
void UART_Config(void){
	//���Դ���U0
	//ʹ��GPIO
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	//ʹ��UART
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	// ΪUART����GPIO���š�
	GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);
	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	//���ô��ڲ���ʱ��Դ������
//	UARTClockSourceSet(UART0_BASE,UART_CLOCK_SYSTEM);
	UARTStdioConfig(0,115200,SysCtlClockGet());

	
	//��ݮ�ɴ���U5
	//ʹ��GPIO
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	//ʹ��UART
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART5);
	// ΪUART����GPIO���š�
	GPIOPinConfigure(GPIO_PE4_U5RX);
	GPIOPinConfigure(GPIO_PE5_U5TX);
	GPIOPinTypeUART(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5);
	//���ô��ڲ���ʱ��Դ������
	UARTClockSourceSet(UART5_BASE,UART_CLOCK_SYSTEM);
	UARTConfigSetExpClk(UART5_BASE,SysCtlClockGet(),115200,UART_CONFIG_WLEN_8|UART_CONFIG_STOP_ONE|UART_CONFIG_PAR_NONE);
	//ʹ��FIFO������FIFO���
	UARTFIFOEnable(UART5_BASE);
	UARTFIFOLevelSet(UART5_BASE,UART_FIFO_RX1_8,UART_FIFO_TX1_8);
	//���ô����ж�
	UARTIntRegister(UART5_BASE,USART5_IRAHandler);
	//�������ڽ����жϺͽ��ճ�ʱ�ж�
	UARTIntEnable(UART5_BASE, UART_INT_RX|UART_INT_RT);
	UARTIntClear(UART5_BASE, UART5_BASE);
	//���ô���0���жϲ������������жϿ�����
	IntEnable(INT_UART5);
//	
//	//˫��ͨѶ����U7
//	//ʹ��GPIO
//	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
//	//ʹ��UART
//	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART7);
//	// ΪUART����GPIO���š�
//	GPIOPinConfigure(GPIO_PE0_U7RX);
//	GPIOPinConfigure(GPIO_PE1_U7TX);
//	GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1);
//	//���ô��ڲ���ʱ��Դ������
//	UARTClockSourceSet(UART7_BASE,UART_CLOCK_SYSTEM);
//	UARTConfigSetExpClk(UART7_BASE,SysCtlClockGet(),9600,UART_CONFIG_WLEN_8|UART_CONFIG_STOP_ONE|UART_CONFIG_PAR_NONE);
//	//ʹ��FIFO������FIFO���
//	UARTFIFOEnable(UART7_BASE);
//	UARTFIFOLevelSet(UART7_BASE,UART_FIFO_RX1_8,UART_FIFO_TX1_8);
//	//���ô����ж�
//	UARTIntRegister(UART7_BASE,USART7_IRAHandler);
//	//�������ڽ����жϺͽ��ճ�ʱ�ж�
//	UARTIntEnable(UART7_BASE, UART_INT_RX|UART_INT_RT);
//	UARTIntClear(UART7_BASE, UART2_BASE);
//	//���ô���0���жϲ������������жϿ�����
//	IntEnable(INT_UART7);
//	IntMasterEnable();
	
	UARTEnable(UART0_BASE);
	UARTEnable(UART5_BASE);
//	UARTEnable(UART7_BASE);
	
}
