#include "control.h"
#include "math.h"

/***** 标志位 *****/
int det_F;	//识别任务标志位 0：无任务，1：开始阶段识别数字卡片，2：岔路口识别地面数字，3：巡线
int load_F;	//药物装载标志位 0：未装载，1：已装载
static int destination_F;	//目的地标志位 1：近端药房，2：中部药房，3：远端药房
//static int flag=0;	//通用标志位
int state_val = 0;

/***** 任务函数 *****/
void mission1(void){
	static int room_num = 0;	//目标病房号
	float v_temp = 0;
//	int pin_state;
	static int pin_flag=0;
	int i,j,flag;
	/***** 状态选择 *****/
	switch(state_val){
		//状态0，初始化并等待
		case 0:{
			//标志位复位
			det_F = 0;
			load_F = 0;
			destination_F = 1;
			//flag = 0;
			//结构体复位
			car_reset();
			
			//准备识别数字
			det_F = 1;
			UARTCharPutNonBlocking(UART5_BASE,NUM_DET);
			
			//进入下一个状态
			state_val++;
			break;
		}
		//识别数字
		case 1:{
			if(det_F==1) UARTCharPutNonBlocking(UART5_BASE,NUM_DET);
			//获取摄像头识别的数字
			if(det_F==0){
				room_num = num_det_data[0];
				//未读到或读错数字
				if(room_num == 0||room_num>2){
					det_F = 1;
					UARTCharPutNonBlocking(UART5_BASE,NUM_DET);
				}
				//正常，转换到下一个状态
				else{
					state_val++;
				}
			}
			break;
		}
		//等待药品放下
		case 2:{
			//按键去抖
			if(pin_flag==0&&!KEY_READ){
				//延时约20ms
				for(i=0;i<15;i++)
					for(j=0;j<2000;j++);
				
				if(!KEY_READ){
					pin_flag=1;
					//pin_state=1;
					load_F=1;
				}
			}
			if(load_F == 1){
				//延时500ms再启动
				UARTCharPutNonBlocking(UART5_BASE,LINE_DET);
				det_F=3;
				state_val++;
				for(i=0;i<200;i++)
					for(j=0;j<2000;j++);
			}
			break;
		}
		//运动到路口，不需要识别数字
		case 3:{
			flag = point_to_point(CORRIDOR_LENGTH_1+CORRIDOR_WIDTH/2+10);
			//进入十字路口
			if(flag==OK){
				car_reset();
				state_val=5;//进入下一个状态
			}
			break;
		}
		//转90°
		case 5:{
			flag = Turn_90(room_num);
			if(flag == OK){
				det_F=3;
				car_reset();
				if(room_num==1)
					Car.Angle+=90;
				else if(room_num==2)
					Car.Angle-=90;
				UARTCharPutNonBlocking(UART5_BASE,LINE_DET);
				state_val++;
			}
			break;
		}
		//走到药房
		case 6:{
			flag = point_to_point(CORRIDOR_LENGTH_2+CORRIDOR_WIDTH/2);
			if(flag==OK){
				RED_ON;
				
				det_F=0;
				UARTCharPutNonBlocking(UART5_BASE,DET_OFF);
				car_reset();
				flag=NOT_OK;
				state_val++;
			}
			break;
		}
		//等待药物取走
		case 7:{
			//按键去抖
			if(pin_flag==1&&KEY_READ){
				//延时约20ms
				for(i=0;i<15;i++)
					for(j=0;j<2000;j++);
				
				if(KEY_READ){
					pin_flag=0;
					//pin_state=1;
					load_F=0;
				}
			}
			if(load_F==0){
				RED_OFF;
				det_F=3;
				state_val++;
				flag=NOT_OK;
			}
			break;
		}
		//倒车到路口中间
		case 8:{
			flag = point_to_point_backfoward(CORRIDOR_LENGTH_2+CORRIDOR_WIDTH/2);
			if(flag==OK){
				det_F=0;
				UARTCharPutNonBlocking(UART5_BASE,DET_OFF);
				car_reset();
				flag=NOT_OK;
				state_val++;
			}
			break;
		}
		//原地转
		case 9:{
			flag = Turn_90(room_num);
			if(flag == OK){
				det_F=3;
				car_reset();
				if(room_num==1)
					Car.Angle+=90;
				else if(room_num==2)
					Car.Angle-=90;
				
				UARTCharPutNonBlocking(UART5_BASE,LINE_DET);
				flag=NOT_OK;
				state_val++;
			}
			break;
		}
		//返回起点
		case 10:{
			flag = point_to_point(CORRIDOR_LENGTH_1+CORRIDOR_WIDTH/2);
			
			if(flag==OK){
				GREEN_ON;
				det_F = 0;
				car_reset();
				UARTCharPutNonBlocking(UART5_BASE,DET_OFF);
				//motor_pwm_set(0,0);
				state_val++;
			}
			break;
		}
		case 11:{
			motor_pwm_set(0,0);
		}
	}
	/***** 状态选择结束 *****/
}

void mission2(void){
	static int room_num = 0;	//目标病房号
	static int turn_dir = 0;
	float v_temp = 0;
//	int pin_state;
	static int pin_flag=0;
	int i,j,flag = NOT_OK;
	/***** 状态选择 *****/
	switch(state_val){
		//状态0，初始化并等待
		case 0:{
			//标志位复位
			det_F = 0;
			load_F = 0;
			destination_F = 1;
			//flag = 0;
			//结构体复位
			car_reset();
			
			//准备识别数字
			det_F = 1;
			UARTCharPutNonBlocking(UART5_BASE,NUM_DET);
			
			//进入下一个状态
			state_val++;
			break;
		}
		//识别数字
		case 1:{
			if(det_F==1) UARTCharPutNonBlocking(UART5_BASE,NUM_DET);
			//获取摄像头识别的数字
			if(det_F==0){
				room_num = num_det_data[0];
				//未读到数字，再次识别
				if(room_num == 0){
					det_F = 1;
					UARTCharPutNonBlocking(UART5_BASE,NUM_DET);
				}
				//正常，转换到下一个状态
				else{
					det_F=1;
					for(i=0;i<0;i++)
						num_det_data[i]=0;
					state_val++;
				}
			}
			break;
		}
		//等待药品放下
		case 2:{
			//按键去抖
			if(pin_flag==0&&!KEY_READ){
				//延时约20ms
				for(i=0;i<15;i++)
					for(j=0;j<2000;j++);
				if(!KEY_READ){
					pin_flag=1;
					//pin_state=1;
					load_F=1;
				}
			}
			if(load_F == 1){
				//延时500ms再启动
				UARTCharPutNonBlocking(UART5_BASE,LINE_DET);
				det_F=1;
				state_val++;
				for(i=0;i<20;i++)
					for(j=0;j<2000;j++);
			}
			break;
		}
		//运动到中段路口，识别数字
		case 3:{
			flag = point_to_point(2*CORRIDOR_LENGTH_1+CORRIDOR_WIDTH+10);
			//进入十字路口，开始识别数字
			if(flag==OK&&det_F==1){
				UARTCharPutNonBlocking(UART5_BASE,NUM_DET);
			}
			//识别数字完成
			if(det_F==0){
				if(room_num==num_det_data[0]) turn_dir=1;	//左转
				else if(room_num==num_det_data[1]) turn_dir=2;	//右转
				else {
					det_F=1;
					turn_dir=0;
				}
			}
			if(turn_dir!=0){
				UARTCharPutNonBlocking(UART5_BASE,LINE_DET);
				flag=NOT_OK;
				car_reset();
				state_val++;//进入下一个状态
			}
			break;
		}
		//走到路口中心
		case 4:{
			UARTCharPutNonBlocking(UART5_BASE,'4');
			flag = point_to_point(25);
			if(flag == OK){
				flag=NOT_OK;
				car_reset();
				state_val++;//进入下一个状态	
			}
			break;
		}
		//转90°
		case 5:{
			UARTCharPutNonBlocking(UART5_BASE,'5');
			flag = Turn_90(turn_dir);
			if(flag == OK){
				det_F=3;
				flag = NOT_OK;
				car_reset();
				UARTCharPutNonBlocking(UART5_BASE,LINE_DET);
				state_val++;
			}
			break;
		}
		//走到药房
		case 6:{
			flag = point_to_point(CORRIDOR_LENGTH_2+CORRIDOR_WIDTH/2);
			if(flag==OK){
				RED_ON;
				det_F=0;
				UARTCharPutNonBlocking(UART5_BASE,DET_OFF);
				car_reset();
				flag=NOT_OK;
				state_val++;
			}
			break;
		}
		//等待药物取走
		case 7:{
			//按键去抖
			if(pin_flag==1&&KEY_READ){
				//延时约20ms
				for(i=0;i<15;i++)
					for(j=0;j<2000;j++);		
				if(KEY_READ){
					pin_flag=0;
					//pin_state=1;
					load_F=0;
				}
			}
			if(load_F==0){
				RED_OFF;
				det_F=3;
				state_val++;
				flag=NOT_OK;
			}
			break;
		}
		//倒车到路口中间
		case 8:{
			flag = point_to_point_backfoward(CORRIDOR_LENGTH_2+CORRIDOR_WIDTH/2-7);
			if(flag==OK){
				det_F=0;
				UARTCharPutNonBlocking(UART5_BASE,DET_OFF);
				car_reset();
				flag=NOT_OK;
				state_val++;
			}
			break;
		}
		//原地转
		case 9:{
			flag = Turn_90(turn_dir);
			if(flag == OK){
				det_F=3;
				car_reset();
				if(turn_dir==1)
					Car.Angle+=90;
				else if(turn_dir==2)
					Car.Angle-=90;
				UARTCharPutNonBlocking(UART5_BASE,LINE_DET);
				flag=NOT_OK;
				state_val++;
			}
			break;
		}
		//返回起点
		case 10:{
			flag = point_to_point(2*CORRIDOR_LENGTH_1+CORRIDOR_WIDTH+CORRIDOR_WIDTH/2);
			if(flag==OK){
				GREEN_ON;
				det_F = 0;
				car_reset();
				UARTCharPutNonBlocking(UART5_BASE,DET_OFF);
				state_val++;
			}
			break;
		}
		//停止
		case 11:{
			UARTCharPutNonBlocking(UART5_BASE,'3');
			motor_pwm_set(0,0);
		}
	}
	/***** 状态选择结束 *****/
}

void mission3(void){
	static int room_num = 0;	//目标病房号
	static int turn_dir[2]= {0,0};
	float v_temp = 0;
//	int pin_state;
	static int pin_flag=0;
	int i,j,flag = NOT_OK;
	/***** 状态选择 *****/
	switch(state_val){
		//状态0，初始化并等待
		case 0:{
			//标志位复位
			det_F = 0;
			load_F = 0;
			destination_F = 1;
			//flag = 0;
			//结构体复位
			car_reset();
			
			//准备识别数字
			det_F = 1;
			UARTCharPutNonBlocking(UART5_BASE,NUM_DET);
			
			//进入下一个状态
			state_val++;
			break;
		}
		//识别数字
		case 1:{
			if(det_F==1) UARTCharPutNonBlocking(UART5_BASE,NUM_DET);
			//获取摄像头识别的数字
			if(det_F==0){
				room_num = num_det_data[0];
				//未读到数字，再次识别
				if(room_num <4){
					det_F = 1;
					UARTCharPutNonBlocking(UART5_BASE,NUM_DET);
				}
				//正常，转换到下一个状态
				else{
					for(i=0;i<0;i++)
						num_det_data[i]=0;
					state_val++;
				}
			}
			break;
		}
		//等待药品放下
		case 2:{
			//按键去抖
			if(pin_flag==0&&!KEY_READ){
				//延时约20ms
				for(i=0;i<15;i++)
					for(j=0;j<2000;j++);
				if(!KEY_READ){
					pin_flag=1;
					//pin_state=1;
					load_F=1;
				}
			}
			if(load_F == 1){
				//延时500ms再启动
				UARTCharPutNonBlocking(UART5_BASE,LINE_DET);
				det_F=1;
				state_val++;
				for(i=0;i<20;i++)
					for(j=0;j<2000;j++);
			}
			break;
		}
		//运动到远段路口，识别数字
		case 3:{
			flag = point_to_point(3*CORRIDOR_LENGTH_1+2*CORRIDOR_WIDTH+10);
			//进入十字路口，开始识别数字
			if(flag==OK&&det_F==1){
				UARTCharPutNonBlocking(UART5_BASE,NUM_DET);
			}
			//识别数字完成
			if(det_F==0){
				if(room_num==num_det_data[0]||room_num==num_det_data[1]) turn_dir[0]=1;	//左转
				else if(room_num==num_det_data[2]||room_num==num_det_data[3]) turn_dir[0]=2;	//右转
				else {
					det_F=1;
					turn_dir[0]=0;
				}
			}
			if(turn_dir[0]!=0){
				UARTCharPutNonBlocking(UART5_BASE,LINE_DET);
				flag=NOT_OK;
				car_reset();
				state_val++;//进入下一个状态
			}
			break;
		}
		//走到路口中心
		case 4:{
			flag = point_to_point(25);
			if(flag == OK){
				flag=NOT_OK;
				car_reset();
				state_val++;//进入下一个状态	
			}
			break;
		}
		//转90°
		case 5:{
			flag = Turn_90(turn_dir[0]);
			if(flag == OK){
				det_F=3;
				car_reset();
				if(turn_dir[0]==1)
					Car.Angle+=90;
				else if(turn_dir[0]==2)
					Car.Angle-=90;
					UARTCharPutNonBlocking(UART5_BASE,LINE_DET);
					det_F=1;
					state_val++;
			}
			break;
		}
		//走到下一个路口，并识别数字
		case 6:{
			flag = point_to_point(CORRIDOR_LENGTH_1+CORRIDOR_WIDTH/2);
			//进入十字路口，开始识别数字
			if(flag==OK&&det_F==1){
				UARTCharPutNonBlocking(UART5_BASE,NUM_DET);
			}
			//识别数字完成
			if(det_F==0){
				if(room_num==num_det_data[0]) turn_dir[1]=1;	//左转
				else if(room_num==num_det_data[1]) turn_dir[1]=2;	//右转
				else {
					det_F=1;
					turn_dir[1]=0;
				}
			}
			if(turn_dir[1]!=0){
				UARTCharPutNonBlocking(UART5_BASE,LINE_DET);
				flag=NOT_OK;
				car_reset();
				state_val++;//进入下一个状态
			}
			break;
		}
		//走到路口中心
		case 7:{
			flag = point_to_point(15+14);
			if(flag == OK){
				flag=NOT_OK;
				car_reset();
				state_val++;//进入下一个状态	
			}
			break;
		}	
		//转90°
		case 8:{
			flag = Turn_90(turn_dir[1]);
			if(flag == OK){
				det_F=3;
				car_reset();
				if(turn_dir[1]==1)
					Car.Angle+=90;
				else if(turn_dir[1]==2)
					Car.Angle-=90;
				UARTCharPutNonBlocking(UART5_BASE,LINE_DET);
				state_val++;
			}
			break;
		}
		//走到药房
		case 9:{
			flag = point_to_point(CORRIDOR_LENGTH_2+CORRIDOR_WIDTH/2);
			if(flag==OK){
				RED_ON;
				det_F=0;
				UARTCharPutNonBlocking(UART5_BASE,DET_OFF);
				car_reset();
				flag=NOT_OK;
				state_val++;
			}
			break;
		}
		//等待药物取走
		case 10:{
			//按键去抖
			if(pin_flag==1&&KEY_READ){
				//延时约20ms
				for(i=0;i<15;i++)
					for(j=0;j<2000;j++);		
				if(KEY_READ){
					pin_flag=0;
					//pin_state=1;
					load_F=0;
				}
			}
			if(load_F==0){
				RED_OFF;
				det_F=3;
				state_val++;
				flag=NOT_OK;
			}
			break;
		}
		//倒车到路口中间
		case 11:{
			flag = point_to_point_backfoward(CORRIDOR_LENGTH_2+CORRIDOR_WIDTH/2);
			if(flag==OK){
				det_F=0;
			//	UARTCharPutNonBlocking(UART5_BASE,DET_OFF);
				car_reset();
				flag=NOT_OK;
				state_val++;
			}
			break;
		}
		//原地转
		case 12:{
			flag = Turn_90(turn_dir[1]);
			if(flag == OK){
				det_F=3;
				car_reset();
				if(turn_dir[1]==1)
					Car.Angle+=90;
				else if(turn_dir[1]==2)
					Car.Angle-=90;
		//		UARTCharPutNonBlocking(UART5_BASE,LINE_DET);
				flag=NOT_OK;
				state_val++;
			}
			break;
		}
		//走到下一个路口
		case 13:{
				flag = point_to_point(CORRIDOR_LENGTH_1+CORRIDOR_WIDTH+5);
			if(flag == OK){
				flag=NOT_OK;
				car_reset();
				state_val++;//进入下一个状态	
			}
			break;
		}
		//转90°
		case 14:{
			flag = Turn_90(turn_dir[0]%2+1);
			if(flag == OK){
				det_F=3;
				car_reset();
				if(turn_dir[0]==2)
					Car.Angle+=90;
				else if(turn_dir[0]==1)
					Car.Angle-=90;
				UARTCharPutNonBlocking(UART5_BASE,LINE_DET);
				state_val++;
			}
			break;
		}
		//返回起点
		case 15:{
			flag = point_to_point(3*CORRIDOR_LENGTH_1+2.5*CORRIDOR_WIDTH);
			if(flag==OK){
				GREEN_ON;
				det_F = 0;
				car_reset();
				UARTCharPutNonBlocking(UART5_BASE,DET_OFF);
				state_val++;
			}
			break;
		}
		//停止
		case 16:{
			motor_pwm_set(0,0);
			break;
		}
	}
	/***** 状态选择结束 *****/
}
/***** 小车控制函数 *****/
void car_Init(car *c){
	c->Angle = 0;
	c->route = 0;
	c->Speed = 0;
}

/**
  * @brief  小车缓启动
  * @param  tar_v 目标速度
  * @retval None
  */
float slow_start(float tar_v){
	if(fabs(Car.route)>=20)
	{
		return tar_v;
	}
	else if(fabs(Car.route)<=3)
	{
		return tar_v*(float)0.4;
	}
	else if(fabs(Car.route)<=6&&fabs(Car.route)>3)
	{
		return tar_v*(float)0.6;
	}
	else if(fabs(Car.route)<=12&&fabs(Car.route)>6)
	{
		return tar_v*(float)0.75;
	}
	return tar_v;
}


/**
  * @brief  小车从一点直行到另一点
  * @param  len 距离
  * @retval OK 已到达 NOT_OK 未到达
  */
int point_to_point(float len){
	float v_temp;
	speed_cal();
	if(len>30){
		if(Car.route<20){
			v_temp=slow_start(V_onrun);
		}
		else{
			pid_realize(&pid_p,len-Car.route);
			v_temp=pid_p.output;
		}
	}
	else{
		pid_realize(&pid_p,len-Car.route);
		v_temp=pid_p.output;
	}
	
	if(fabs(Car.Speed)<0.03&&fabs(Car.route-len)<0.5){
		motor_pwm_set(0,0);
		return OK;
	}
	else {
		pid_realize(&pid_a,(float)(LINE_DET_POS-line_det_data));
		motor_speed_set(v_temp-pid_a.output,v_temp+pid_a.output);
	}
	return NOT_OK;
}	

/**
  * @brief  小车从一点直行到另一点
  * @param  len 距离
  * @retval OK 已到达 NOT_OK 未到达
  */
int point_to_point_backfoward(float len){
	float v_temp;
	//int flag=1;
	speed_cal();
	len=-len;
	if(len<-30){
		if(Car.route>-20){
			v_temp=slow_start(-V_onrun);
			//flag=0;
		}
		else{
			pid_realize(&pid_p,len-Car.route);
			v_temp=pid_p.output;
		}
	}
	else{
	//	flag=0;
		pid_realize(&pid_p,len-Car.route);
		v_temp=pid_p.output;
	}
	
	if(fabs(Car.Speed)<0.03&&fabs(Car.route-len)<0.5){
		motor_pwm_set(0,0);
		return OK;
	}
	else {
		pid_realize(&pid_b,(float)(LINE_DET_POS-line_det_data));
		motor_speed_set(v_temp+pid_b.output,v_temp-pid_b.output);
	}
	return NOT_OK;
}	

/**
  * @brief  小车原地转90°
  * @param  dir 旋转方向 1：左转 2：右转
  * @retval None
  */
int Turn_90(int dir){
	speed_cal();
	//左转，目标角度-90
	if(dir==1){
		if(fabs(Car.Angle+90)<0.5){
			motor_pwm_set(0,0);
			return OK;
		}
		else{
			pid_realize(&pid_turn,(float)-90-Car.Angle);
			motor_speed_set(pid_turn.output,-pid_turn.output);
		}
	}
	//右转
	else if(dir==2){
		if(fabs(Car.Angle-90)<0.5){
			motor_pwm_set(0,0);
			return OK;
		}
		else{
			pid_realize(&pid_turn,(float)90-Car.Angle);
			motor_speed_set(pid_turn.output,-pid_turn.output);
		}
	}
	return NOT_OK;
}


void car_reset(void){
	Car.route=0;
	Car.Speed=0;
	Car.Angle=0;
	motor_reset(&left_motor);
	motor_reset(&right_motor);
	pid_reset(&left_pid);
	pid_reset(&right_pid);
}

/**
  * @brief  小车原地转180°
  * @param  None
  * @retval None
  */
void Turn_180(void){
	speed_cal();
	pid_realize(&pid_a,180-Car.Angle);
	motor_speed_set(V_onrun+pid_a.output,V_onrun-pid_a.output);
}

/***** 电机控制底层 *****/
/**
  * @brief  电机结构体初始化
  * @param  *M 待初始化的结构体
  * @retval None
  */
void motor_Init(motor *M){
	M->Speed = 0;
	M->Tar_Speed = V_onrun;
	M->Dis = 0;
	M->Dis_Last = 0;
	M->Dir = 0;
	M->QEIPostion = 0x7fffffff;
	M->QEIPostion_Last = 0x7fffffff;
}

/**
  * @brief  电机PWM设置，死区0.25
  * @param  pwml 左边电机PWM值，范围-1-1，取0时电机停止
	* @param  pwmr 右边电机PWM值，范围-1-1，取0时电机停止
  * @retval None
  */
void motor_pwm_set(float pwml,float pwmr){
	//左轮 D0 D1
	if(pwml>0){
		//D0
		PWM_duty(PWM0_BASE,  PWM_OUT_6,PWM_OUT_6_BIT,PWM_GEN_3,pwml);
		//D1
		PWM_duty(PWM0_BASE,  PWM_OUT_7,PWM_OUT_7_BIT,PWM_GEN_3,0);
	}
	else if(pwml<0){
		//D0
		PWM_duty(PWM0_BASE,  PWM_OUT_6,PWM_OUT_6_BIT,PWM_GEN_3,0);
		//D1
		PWM_duty(PWM0_BASE,  PWM_OUT_7,PWM_OUT_7_BIT,PWM_GEN_3,-pwml);
	}
	else {
		//D0
		PWM_duty(PWM0_BASE,  PWM_OUT_6,PWM_OUT_6_BIT,PWM_GEN_3,0);
		//D1
		PWM_duty(PWM0_BASE,  PWM_OUT_7,PWM_OUT_7_BIT,PWM_GEN_3,0);
	}
	
	//右轮 B6 B7
	if(pwmr>0){
		//B6
		PWM_duty(PWM0_BASE,  PWM_OUT_0,PWM_OUT_0_BIT,PWM_GEN_0,pwmr);
		//B7
		PWM_duty(PWM0_BASE,  PWM_OUT_1,PWM_OUT_1_BIT,PWM_GEN_0,0);
	}
	else if(pwmr<0){
		//B6
		PWM_duty(PWM0_BASE,  PWM_OUT_0,PWM_OUT_0_BIT,PWM_GEN_0,0);
		//B7
		PWM_duty(PWM0_BASE,  PWM_OUT_1,PWM_OUT_1_BIT,PWM_GEN_0,-pwmr);
	}
	else {
		//B6
		PWM_duty(PWM0_BASE,  PWM_OUT_0,PWM_OUT_0_BIT,PWM_GEN_0,0);
		//B7
		PWM_duty(PWM0_BASE,  PWM_OUT_1,PWM_OUT_1_BIT,PWM_GEN_0,0);
	}
}

/**
  * @brief  左边电机速度设置
  * @param  vl 左轮目标速度值，+为正转，-为反转，范围-1.2-1.2
	* @param  vr 右轮目标速度值，+为正转，-为反转，范围-1.2-1.2
  * @retval None
  */
void motor_speed_set(float vl,float vr){
	float pwm_l,pwm_r;
	//左轮
	left_pid.cur_val = left_motor.Speed;
	left_pid.target_val = vl;
	pid_realize(&left_pid,left_pid.target_val - left_pid.cur_val);
	pwm_l=left_pid.output==0? 0:left_pid.output+DEADBAND;
	if(left_pid.output>(float)0.1){
		pwm_l = left_pid.output+DEADBAND;
	}
	else if(left_pid.output<(float)-0.1){
		pwm_l = left_pid.output-DEADBAND;
	}
	else pwm_l=0;
	
	//右轮
	right_pid.cur_val = right_motor.Speed;
	right_pid.target_val = vr;
	pid_realize(&right_pid,right_pid.target_val - right_pid.cur_val);
	pwm_r=right_pid.output==0? 0:right_pid.output+DEADBAND;
	if(right_pid.output>(float)0.1){
		pwm_r = right_pid.output+DEADBAND;
	}
	else if(right_pid.output<(float)-0.1){
		pwm_r = right_pid.output-DEADBAND;
	}
	else pwm_r=0;
	
	motor_pwm_set(pwm_l,pwm_r);
}

/**
  * @brief  测速，并更新车轮路程，计算航向角
  * @param  None
  * @retval None
  */
void speed_cal(void){
	uint32_t cur = right_encoder_count;// QEIPositionGet(RIGHT_QEI);
	static float v_l_last, v_r_last,ang_last;
	float err=0;
	//右轮
	right_motor.QEIPostion = cur;
	err = uint32_sub(cur,right_motor.QEIPostion_Last);
	//测速和滤波
	right_motor.Speed = err*10*d_distance/CONTROL_PERIOD;
	right_motor.Speed = first_order_filter(right_motor.Speed, v_r_last,(float)0.6);
	v_r_last = right_motor.Speed;
	//更新路程，单位为cm
	right_motor.Dis += d_distance * err;
	right_motor.QEIPostion_Last = cur;
	//左轮
	cur = left_encoder_count;// QEIPositionGet(LEFT_QEI);
	left_motor.QEIPostion = cur;
	err = uint32_sub(cur,left_motor.QEIPostion_Last);
	//测速和滤波
	left_motor.Speed = err*10*d_distance/CONTROL_PERIOD;
	left_motor.Speed = first_order_filter(left_motor.Speed, v_l_last,(float)0.6);
	v_l_last = left_motor.Speed;
	//更新路程，单位为cm
	left_motor.Dis += d_distance * err;
	left_motor.QEIPostion_Last = cur;
	
	Car.Speed = (right_motor.Speed+left_motor.Speed)/2;
	Car.route = (right_motor.Dis+left_motor.Dis)/2;
	Car.Angle += (left_motor.Speed - right_motor.Speed)*(float)7.1;
	Car.Angle = first_order_filter(Car.Angle,ang_last,(float)0.8);
	ang_last = Car.Angle;
}

float uint32_sub(uint32_t cur,uint32_t last){
	if(cur>last){
		return (float)(cur-last);
	}
	return -(float)(last-cur);
}

/**
  * @brief  motor结构体复位，在切换任务时调用
  * @param  *M motor结构体
  * @retval None
  */
void motor_reset(motor* M){
	M->Speed = 0;
	M->Tar_Speed = 0;
	M->Dis = 0;
	M->Dis_Last = 0;
	M->Dir = 0;
//	M->QEIPostion = 0x7fffffff;
//	M->QEIPostion_Last = 0x7fffffff;
}

/***** PID底层 *****/
void pid_init(pid *pid_controller,float p,float i,float d,PIDOut_Type max,PIDOut_Type min){
	pid_controller->kp = p;
	pid_controller->ki = i;
	pid_controller->kd = d;
	pid_controller->cur_val = 0;
	pid_controller->target_val = 0;
	pid_controller->err = 0;
	pid_controller->err_k1 = 0;
	pid_controller->output = 0;
	pid_controller->max = max;
	pid_controller->min = min;
	pid_controller->output_last = 0;
	pid_controller->i_max = 400;
	pid_controller->i = 0;
}

/**
  * @brief  位置式PID实现函数
  * @param  *p PID结构体
  * @param  err 误差值：err = current - target
  * @retval None
  */
void pid_realize(pid *p, PIDIn_Type err){
	p->err = err;
	//抗积分饱和
	if(p->output_last>p->max||p->output_last<p->min){
		if(p->output_last*p->err<0)//err使积分项绝对值减小
			p->i += p->err;
		else p->i=p->i;
	}
	else p->i += p->err;
	//积分限幅
	if(p->i>p->i_max) p->i = p->i_max;
	else if(p->i<-p->i_max) p->i = -p->i_max;
	
	p->output = p->kp*p->err + p->ki*p->i + p->kd*(p->err - p->err_k1);
	p->err_k1 = p->err;
	p->output_last = p->output;
	
	//输出限幅
	if(p->output>p->max) p->output=p->max;
	if(p->output<p->min) p->output=p->min;
}

/**
  * @brief  PID结构体复位，在切换任务时调用
  * @param  *p PID结构体
  * @param  err 误差值：err = current - target
  * @retval None
  */
void pid_reset(pid* pid_controller){
	pid_controller->cur_val = 0;
	pid_controller->target_val = 0;
	pid_controller->err = 0;
	pid_controller->err_k1 = 0;
	pid_controller->output = 0;
	pid_controller->output_last = 0;
	pid_controller->i = 0;
}

/**
  * @brief  一阶滤波
  * @param  new_value 新值
	* @param  last_value 上一次的值
  * @param  a 滤波系数
  * @retval 滤波后的值
  */
float first_order_filter(float new_value,float last_value,float a){
	//a的取值决定了算法的灵敏度，a越大，新采集的值占的权重越大，算法越灵敏，但平顺性差
	//相反，a越小，新采集的值占的权重越小，灵敏度差，但平顺性好。
	float flitered = new_value*a + last_value*(1-a);
	return flitered;
}
