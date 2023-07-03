#include "control.h"
#include "math.h"

/***** ��־λ *****/
int det_F;	//ʶ�������־λ 0��������1����ʼ�׶�ʶ�����ֿ�Ƭ��2����·��ʶ��������֣�3��Ѳ��
int load_F;	//ҩ��װ�ر�־λ 0��δװ�أ�1����װ��
//static int destination_F;	//Ŀ�ĵر�־λ 1������ҩ����2���в�ҩ����3��Զ��ҩ��
//static int flag=0;	//ͨ�ñ�־λ
int state_val = 0;

/***** ������ *****/
void mission1(void){
	static int room_num = 0;	//Ŀ�겡����
//	float v_temp = 0;
//	int pin_state;
	static int pin_flag=0;
	int i,j,flag = NOT_OK;
	int flag1 = NOT_OK;
	float d_route = 0;
	/***** ״̬ѡ�� *****/
	switch(state_val){
		//״̬0����ʼ�����ȴ�
		case 0:{
			//��־λ��λ
			det_F = 0;
			load_F = 0;
//			destination_F = 1;
			//flag = 0;
			//�ṹ�帴λ
			car_reset();
			
			//׼��ʶ������
			det_F = 1;
			UARTCharPutNonBlocking(UART5_BASE,NUM_DET);
			
			//������һ��״̬
			state_val++;
			break;
		}
		//ʶ������
		case 1:{
			if(cross_dis_ang[0]<0.5) GREEN_ON;
			if(det_F==1) UARTCharPutNonBlocking(UART5_BASE,NUM_DET);
			//��ȡ����ͷʶ�������
			if(det_F==0){
				room_num = num_det_data[0];
				//δ�������������
				if(room_num == 0||room_num>2){
					det_F = 1;
					UARTCharPutNonBlocking(UART5_BASE,NUM_DET);
				}
				//������ת������һ��״̬
				else{
					state_val++;
				}
			}
			break;
		}
		//�ȴ�ҩƷ����
		case 2:{
			//����ȥ��
			if(pin_flag==0&&!KEY_READ){
				//��ʱԼ20ms
				for(i=0;i<15;i++)
					for(j=0;j<2000;j++);
				
				if(!KEY_READ){
					pin_flag=1;
					//pin_state=1;
					load_F=1;
				}
			}
			if(load_F == 1){
				//��ʱ500ms������
				UARTCharPutNonBlocking(UART5_BASE,LINE_DET);
				det_F=3;
				state_val++;
				flag1 = NOT_OK;
				for(i=0;i<200;i++)
					for(j=0;j<2000;j++);
			}
			break;
		}
		//�˶���·�ڣ�����Ҫʶ������
		case 3:{
			flag = point_to_point(CORRIDOR_LENGTH_1+CORRIDOR_WIDTH/2+10+d_route,1);
			if((CORRIDOR_LENGTH_1+CORRIDOR_WIDTH/2-Car.route)<10&&(CORRIDOR_LENGTH_1+CORRIDOR_WIDTH/2-Car.route)>8){
				flag1 = OK;
				UARTCharPutNonBlocking(UART5_BASE,CROSS_DET);
				d_route = (cross_dis_ang[0]-77)/15.8+Car.route-(CORRIDOR_LENGTH_1+CORRIDOR_WIDTH/2);
				if(d_route>0) YELLOW_ON;
				if(d_route<0) RED_ON;
			}
			//����ʮ��·��
			if(flag==OK){
				car_reset();
				Car.Angle=0;
				flag1=NOT_OK;
				d_route=0;
				state_val=5;//������һ��״̬
			}
			break;
		}
		//ת90��
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
		//�ߵ�ҩ��
		case 6:{
			flag = point_to_point(CORRIDOR_LENGTH_2+CORRIDOR_WIDTH/2,0);
//			if(flag==OK){
//				flag1 = Turn_to_0();
//			}
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
		//�ȴ�ҩ��ȡ��
		case 7:{
			//����ȥ��
			if(pin_flag==1&&KEY_READ){
				//��ʱԼ20ms
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
		//������·���м�
		case 8:{
			flag = point_to_point_backfoward(CORRIDOR_LENGTH_2+CORRIDOR_WIDTH/2-3);
			if(flag==OK){
				det_F=0;
				UARTCharPutNonBlocking(UART5_BASE,DET_OFF);
				car_reset();
				flag=NOT_OK;
				state_val++;
			}
			break;
		}
		//ԭ��ת
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
		//�������
		case 10:{
			flag = point_to_point(CORRIDOR_LENGTH_1+CORRIDOR_WIDTH/2,1);
			
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
	/***** ״̬ѡ����� *****/
}

void mission2(void){
	static int room_num = 0;	//Ŀ�겡����
	static int turn_dir = 0;
//	float v_temp = 0;
//	int pin_state;
	static int pin_flag=0;
	int i,j,flag = NOT_OK,flag1 = NOT_OK;
	float d_route=0;
	/***** ״̬ѡ�� *****/
	switch(state_val){
		//״̬0����ʼ�����ȴ�
		case 0:{
			//��־λ��λ
			det_F = 0;
			load_F = 0;
//			destination_F = 1;
			//flag = 0;
			//�ṹ�帴λ
			car_reset();
			
			//׼��ʶ������
			det_F = 1;
			UARTCharPutNonBlocking(UART5_BASE,NUM_DET);
			
			//������һ��״̬
			state_val++;
			break;
		}
		//ʶ������
		case 1:{
			if(det_F==1) UARTCharPutNonBlocking(UART5_BASE,NUM_DET);
			//��ȡ����ͷʶ�������
			if(det_F==0){
				room_num = num_det_data[0];
				//δ�������֣��ٴ�ʶ��
				if(room_num == 0){
					det_F = 1;
					UARTCharPutNonBlocking(UART5_BASE,NUM_DET);
				}
				//������ת������һ��״̬
				else{
					det_F=1;
					for(i=0;i<0;i++)
						num_det_data[i]=0;
					state_val++;
				}
			}
			break;
		}
		//�ȴ�ҩƷ����
		case 2:{
			//����ȥ��
			if(pin_flag==0&&!KEY_READ){
				//��ʱԼ20ms
				for(i=0;i<15;i++)
					for(j=0;j<2000;j++);
				if(!KEY_READ){
					pin_flag=1;
					//pin_state=1;
					load_F=1;
				}
			}
			if(load_F == 1){
				//��ʱ500ms������
				UARTCharPutNonBlocking(UART5_BASE,LINE_DET);
				det_F=1;
				state_val++;
				for(i=0;i<20;i++)
					for(j=0;j<2000;j++);
			}
			break;
		}
		//�˶����ж�·�ڣ�ʶ������
		case 3:{
			flag = point_to_point(2*CORRIDOR_LENGTH_1+CORRIDOR_WIDTH+10,1);
			//����ʮ��·�ڣ���ʼʶ������
			if(flag==OK&&det_F==1){
				UARTCharPutNonBlocking(UART5_BASE,NUM_DET);
			}
			//ʶ���������
			if(det_F==0){
				if(room_num==num_det_data[0]) turn_dir=1;	//��ת
				else if(room_num==num_det_data[1]) turn_dir=2;	//��ת
				else {
					det_F=1;
					turn_dir=0;
				}
			}
			if(turn_dir!=0){
				UARTCharPutNonBlocking(UART5_BASE,LINE_DET);
				flag=NOT_OK;
				car_reset();
				state_val++;//������һ��״̬
			}
			break;
		}
		//�ߵ�·������
		case 4:{
			if(flag1==NOT_OK){
				UARTCharPutNonBlocking(UART5_BASE,CROSS_DET);
				d_route = cross_dis_ang[0]/18;
				if(d_route!=0) flag1 = OK;
			}
			if(flag1 == OK)
				flag = point_to_point(d_route,1);
			if(flag == OK){
				flag=NOT_OK;
				car_reset();
				state_val++;//������һ��״̬	
			}
			break;
		}
		//ת90��
		case 5:{
			flag = Turn_90(turn_dir);
			if(flag == OK){
				det_F=3;
				flag = NOT_OK;
				car_reset();
				flag1=NOT_OK;
				UARTCharPutNonBlocking(UART5_BASE,LINE_DET);
				state_val++;
			}
			break;
		}
		//�ߵ�ҩ��
		case 6:{			
			flag = point_to_point(CORRIDOR_LENGTH_2+CORRIDOR_WIDTH/2,1);
			if(flag==OK){
				RED_ON;
				det_F=0;
				UARTCharPutNonBlocking(UART5_BASE,DET_OFF);
				car_reset();
				flag1=NOT_OK;
				flag=NOT_OK;
				state_val++;
			}
			break;
		}
		//�ȴ�ҩ��ȡ��
		case 7:{
			//����ȥ��
			if(pin_flag==1&&KEY_READ){
				//��ʱԼ20ms
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
		//������·���м�
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
		//ԭ��ת
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
		//�������

		case 10:{
			flag = point_to_point(2*CORRIDOR_LENGTH_1+CORRIDOR_WIDTH+CORRIDOR_WIDTH/2,1);
			if(flag==OK){
				GREEN_ON;
				det_F = 0;
				car_reset();
				UARTCharPutNonBlocking(UART5_BASE,DET_OFF);
				state_val++;
			}
			break;
		}
		//ֹͣ
		case 11:{
			motor_pwm_set(0,0);
		}
	}
	/***** ״̬ѡ����� *****/
}

void mission3(void){
	static int room_num = 0;	//Ŀ�겡����
	static int turn_dir[2]= {0,0};
//	float v_temp = 0;
//	int pin_state;
	static int pin_flag=0;
	int i,j,flag = NOT_OK,flag1 = NOT_OK;
	/***** ״̬ѡ�� *****/
	switch(state_val){
		//״̬0����ʼ�����ȴ�
		case 0:{
			//��־λ��λ
			det_F = 0;
			load_F = 0;
//			destination_F = 1;
			//flag = 0;
			//�ṹ�帴λ
			car_reset();
			
			//׼��ʶ������
			det_F = 1;
			UARTCharPutNonBlocking(UART5_BASE,NUM_DET);
			
			//������һ��״̬
			state_val++;
			break;
		}
		//ʶ������
		case 1:{
			if(det_F==1) UARTCharPutNonBlocking(UART5_BASE,NUM_DET);
			//��ȡ����ͷʶ�������
			if(det_F==0){
				room_num = num_det_data[0];
				//δ�������֣��ٴ�ʶ��
				if(room_num <4){
					det_F = 1;
					UARTCharPutNonBlocking(UART5_BASE,NUM_DET);
				}
				//������ת������һ��״̬
				else{
					for(i=0;i<0;i++)
						num_det_data[i]=0;
					state_val++;
				}
			}
			break;
		}
		//�ȴ�ҩƷ����
		case 2:{
			//����ȥ��
			if(pin_flag==0&&!KEY_READ){
				//��ʱԼ20ms
				for(i=0;i<15;i++)
					for(j=0;j<2000;j++);
				if(!KEY_READ){
					pin_flag=1;
					//pin_state=1;
					load_F=1;
				}
			}
			if(load_F == 1){
				//��ʱ500ms������
				UARTCharPutNonBlocking(UART5_BASE,LINE_DET);
				det_F=1;
				state_val++;
				for(i=0;i<20;i++)
					for(j=0;j<2000;j++);
			}
			break;
		}
		//�˶���Զ��·�ڣ�ʶ������
		case 3:{
			flag = point_to_point(3*CORRIDOR_LENGTH_1+2*CORRIDOR_WIDTH+10,1);
			//����ʮ��·�ڣ���ʼʶ������
			if(flag==OK&&det_F==1){
				UARTCharPutNonBlocking(UART5_BASE,NUM_DET);
			}
			//ʶ���������
			if(det_F==0){
				if(room_num==num_det_data[0]||room_num==num_det_data[1]) turn_dir[0]=1;	//��ת
				else if(room_num==num_det_data[2]||room_num==num_det_data[3]) turn_dir[0]=2;	//��ת
				else {
					det_F=1;
					turn_dir[0]=0;
				}
			}
			if(turn_dir[0]!=0){
				UARTCharPutNonBlocking(UART5_BASE,LINE_DET);
				flag=NOT_OK;
				car_reset();
				state_val++;//������һ��״̬
			}
			break;
		}
		//�ߵ�·������
		case 4:{
			flag = point_to_point(25,1);
			if(flag == OK){
				flag=NOT_OK;
				car_reset();
				state_val++;//������һ��״̬	
			}
			break;
		}
		//ת90��
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
		//�ߵ���һ��·�ڣ���ʶ������
		case 6:{
			flag = point_to_point(CORRIDOR_LENGTH_1+CORRIDOR_WIDTH/2,1);
			//����ʮ��·�ڣ���ʼʶ������
			if(flag==OK&&det_F==1){
				UARTCharPutNonBlocking(UART5_BASE,NUM_DET);
			}
			//ʶ���������
			if(det_F==0){
				if(room_num==num_det_data[0]) turn_dir[1]=1;	//��ת
				else if(room_num==num_det_data[1]) turn_dir[1]=2;	//��ת
				else {
					det_F=1;
					turn_dir[1]=0;
				}
			}
			if(turn_dir[1]!=0){
				UARTCharPutNonBlocking(UART5_BASE,LINE_DET);
				flag=NOT_OK;
				car_reset();
				state_val++;//������һ��״̬
			}
			break;
		}
		//�ߵ�·������
		case 7:{
			flag = point_to_point(15+14,1);
			if(flag == OK){
				flag=NOT_OK;
				car_reset();
				state_val++;//������һ��״̬	
			}
			break;
		}	
		//ת90��
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
		//�ߵ�ҩ��
		case 9:{
			flag = point_to_point(CORRIDOR_LENGTH_2+CORRIDOR_WIDTH/2,0);
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
		//�ȴ�ҩ��ȡ��
		case 10:{
			//����ȥ��
			if(pin_flag==1&&KEY_READ){
				//��ʱԼ20ms
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
		//������·���м�
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
		//ԭ��ת
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
		//�ߵ���һ��·��
		case 13:{
				flag = point_to_point(CORRIDOR_LENGTH_1+CORRIDOR_WIDTH+5,1);
			if(flag == OK){
				flag1 = Turn_to_0();
			}
			if(flag1 == OK){
				flag=NOT_OK;
				car_reset();
				state_val++;//������һ��״̬	
			}
			break;
		}
		//ת90��
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
				flag = NOT_OK;
				state_val++;
			}
			break;
		}
		//�������
		case 15:{
			flag = point_to_point(3*CORRIDOR_LENGTH_1+(float)2.5*CORRIDOR_WIDTH,1);
			if(flag==OK){
				GREEN_ON;
				det_F = 0;
				car_reset();
				UARTCharPutNonBlocking(UART5_BASE,DET_OFF);
				state_val++;
			}
			break;
		}
		//ֹͣ
		case 16:{
			motor_pwm_set(0,0);
			break;
		}
	}
	/***** ״̬ѡ����� *****/
}
/***** С�����ƺ��� *****/
void car_Init(car *c){
	c->Angle = 0;
	c->route = 0;
	c->Speed = 0;
}

/**
  * @brief  С��������
  * @param  tar_v Ŀ���ٶ�
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
  * @brief  С����һ��ֱ�е���һ��
  * @param  len ����
* @param  flag 1:ѭ�� 0����ѭ��
  * @retval OK �ѵ��� NOT_OK δ����
  */
int point_to_point(float len,int flag){
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
		if(flag==1){
			pid_realize(&pid_a,(float)(LINE_DET_POS-line_det_data));
			motor_speed_set(v_temp-pid_a.output,v_temp+pid_a.output);
		}
		else motor_speed_set(v_temp,v_temp);
	}
	return NOT_OK;
}	

/**
  * @brief  С����һ��ֱ�е���һ��
  * @param  len ����
  * @retval OK �ѵ��� NOT_OK δ����
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
  * @brief  С��ԭ��ת90��
  * @param  dir ��ת���� 1����ת 2����ת
  * @retval None
  */
int Turn_90(int dir){
	speed_cal();
	//��ת��Ŀ��Ƕ�-90
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
	//��ת
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
  * @brief  С��ԭ��ת180��
  * @param  None
  * @retval None
  */
void Turn_180(void){
	speed_cal();
	pid_realize(&pid_a,180-Car.Angle);
	motor_speed_set(V_onrun+pid_a.output,V_onrun-pid_a.output);
}

/**
  * @brief  С��ԭ��ת��0��
  * @param  None
  * @retval ״̬
  */
int Turn_to_0(void){
	speed_cal();
	pid_realize(&pid_turn,(float)-Car.Angle/2);
	motor_speed_set(pid_turn.output,-pid_turn.output);
	if(fabs(Car.Angle)<0.2){
		motor_pwm_set(0,0);
		return OK;
	}
	return NOT_OK;
}

/***** ������Ƶײ� *****/
/**
  * @brief  ����ṹ���ʼ��
  * @param  *M ����ʼ���Ľṹ��
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
  * @brief  ���PWM���ã�����0.25
  * @param  pwml ��ߵ��PWMֵ����Χ-1-1��ȡ0ʱ���ֹͣ
	* @param  pwmr �ұߵ��PWMֵ����Χ-1-1��ȡ0ʱ���ֹͣ
  * @retval None
  */
void motor_pwm_set(float pwml,float pwmr){
	//���� D0 D1
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
	
	//���� B6 B7
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
  * @brief  ��ߵ���ٶ�����
  * @param  vl ����Ŀ���ٶ�ֵ��+Ϊ��ת��-Ϊ��ת����Χ-1.2-1.2
	* @param  vr ����Ŀ���ٶ�ֵ��+Ϊ��ת��-Ϊ��ת����Χ-1.2-1.2
  * @retval None
  */
void motor_speed_set(float vl,float vr){
	float pwm_l,pwm_r;
	//����
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
	
	//����
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
  * @brief  ���٣������³���·�̣����㺽���
  * @param  None
  * @retval None
  */
void speed_cal(void){
	uint32_t cur = right_encoder_count;// QEIPositionGet(RIGHT_QEI);
	static float v_l_last, v_r_last,ang_last;
	float err=0;
	//����
	right_motor.QEIPostion = cur;
	err = uint32_sub(cur,right_motor.QEIPostion_Last);
	//���ٺ��˲�
	right_motor.Speed = err*10*d_distance/CONTROL_PERIOD;
	right_motor.Speed = first_order_filter(right_motor.Speed, v_r_last,(float)0.6);
	v_r_last = right_motor.Speed;
	//����·�̣���λΪcm
	right_motor.Dis += d_distance * err;
	right_motor.QEIPostion_Last = cur;
	//����
	cur = left_encoder_count;// QEIPositionGet(LEFT_QEI);
	left_motor.QEIPostion = cur;
	err = uint32_sub(cur,left_motor.QEIPostion_Last);
	//���ٺ��˲�
	left_motor.Speed = err*10*d_distance/CONTROL_PERIOD;
	left_motor.Speed = first_order_filter(left_motor.Speed, v_l_last,(float)0.6);
	v_l_last = left_motor.Speed;
	//����·�̣���λΪcm
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
  * @brief  motor�ṹ�帴λ�����л�����ʱ����
  * @param  *M motor�ṹ��
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

/***** PID�ײ� *****/
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
  * @brief  λ��ʽPIDʵ�ֺ���
  * @param  *p PID�ṹ��
  * @param  err ���ֵ��err = current - target
  * @retval None
  */
void pid_realize(pid *p, PIDIn_Type err){
	p->err = err;
	//�����ֱ���
	if(p->output_last>p->max||p->output_last<p->min){
		if(p->output_last*p->err<0)//errʹ���������ֵ��С
			p->i += p->err;
		else p->i=p->i;
	}
	else p->i += p->err;
	//�����޷�
	if(p->i>p->i_max) p->i = p->i_max;
	else if(p->i<-p->i_max) p->i = -p->i_max;
	
	p->output = p->kp*p->err + p->ki*p->i + p->kd*(p->err - p->err_k1);
	p->err_k1 = p->err;
	p->output_last = p->output;
	
	//����޷�
	if(p->output>p->max) p->output=p->max;
	if(p->output<p->min) p->output=p->min;
}

/**
  * @brief  PID�ṹ�帴λ�����л�����ʱ����
  * @param  *p PID�ṹ��
  * @param  err ���ֵ��err = current - target
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
  * @brief  һ���˲�
  * @param  new_value ��ֵ
	* @param  last_value ��һ�ε�ֵ
  * @param  a �˲�ϵ��
  * @retval �˲����ֵ
  */
float first_order_filter(float new_value,float last_value,float a){
	//a��ȡֵ�������㷨�������ȣ�aԽ���²ɼ���ֵռ��Ȩ��Խ���㷨Խ��������ƽ˳�Բ�
	//�෴��aԽС���²ɼ���ֵռ��Ȩ��ԽС�������Ȳ��ƽ˳�Ժá�
	float flitered = new_value*a + last_value*(1-a);
	return flitered;
}
