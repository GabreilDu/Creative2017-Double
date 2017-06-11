#define __CONTROL_C_
#include "includes.h"


int g_f_pit = 0;
int g_f_enable_mag_steer_control = 0;
int g_f_enable_speed_control = 0;	/* 启用速度控制标志位 */
int g_f_enable_supersonic=0;	/* 启用超声探测标志位 */
int speed = 0;
int update_steer_helm_basement_to_steer_helm(void);
int g_f_big_U=0;
int g_f_big_U_2=0;
int counter=0;
int delay_count=0;

extern WORD Steer_PWM[4];//舵机输出值记录
extern int stuck;
extern int first_received;

DWORD tmp_a, tmp_b;
/*-----------------------------------------------------------------------*/
/* 2017赛季双电机参数 	                                                                      */
/*-----------------------------------------------------------------------*/
int csl=0,csr=0;//currentspeedleft=0,currentspeedright=0;
int tsl=0,tsr=0;//targetspeedleft=0,targetspeedright=0;
int targetspeed=0; 
int	Motor_PWM_MAX=1000; 
int	Motor_PWM_MIN=-1000;
int cyclespeed=115,turnspeed=115,straightspeed=130,cyclespeedleft=135,cyclespeedright=110;
unsigned int speedcounter1=0,speedcounter2=0,speedcounter3=0,speedcounter4=0;
//**********************差速参数***************************/
signed int Speed_kc=15000;
signed int wheel_distance=9;//半车距8
signed int RPID=0;
double r=0;
//**********************电机PID参数**********************************************;	
double Speed_kp_Left=10,Speed_ki_Left=4,Speed_kd_Left=1;//12,0.6
double Speed_kp_Right=10,Speed_ki_Right=4,Speed_kd_Right=1;	//12,0.85

int ErrorLeft=0,PreErrorLeft=0,Pre2ErrorLeft=0,SumErrorLeft=0,ErrorRight=0,PreErrorRight=0,Pre2ErrorRight=0,SumErrorRight=0;
int intErrorLeft=0,intErrorRight=0;
float Pwm_Delta_Left=0,Pwm_Delta_Right=0; 
int tsl_PWM=0,tsr_PWM=0,tsr_Delta=0,error_Delta=0;

/*-----------------------------------------------------------------------*/
/* 舵机初始化 	                                                                      */
/*-----------------------------------------------------------------------*/

/*-----------------------------------------------------------------------*/
/* PIT中断处理函数                                                                 */
/*-----------------------------------------------------------------------*/
void PitISR(void)
{
	g_f_pit = 1;
	//D6 =~ D6;
	g_time_basis_PIT++;	/* 计时 */
	//counter++;	

//	/* start:encoder */
	SpeedCount(); //速度计数
//	data_encoder.is_forward = SIU.GPDI[46].B.PDI;//PC14
//	data_encoder.cnt_old = data_encoder.cnt_new;
//	data_encoder.cnt_new = (WORD)EMIOS_0.CH[24].CCNTR.R;//PD12
//	if (data_encoder.cnt_new >= data_encoder.cnt_old)
//	{
//		data_encoder.speed_now = data_encoder.cnt_new - data_encoder.cnt_old;
//	}
//	else
//	{
//		data_encoder.speed_now = 0xffff - (data_encoder.cnt_old - data_encoder.cnt_new);
//	}
	/* end:encoder */

	/* 开始执行速度控制算法 */
	if (g_f_enable_speed_control)
	{
		//SpeedControl();//不同路段PID,尚未调,不可用
		//control_speed_encoder_pid();
	DoubleSpeedControl2();//双电机驱动
	}
//	
//	//光编记步
//	if (data_encoder.is_forward)
//	{
//		data_encoder.speed_real =(SWORD) data_encoder.speed_now;
//	}
//	else
//	{
//		data_encoder.speed_real = 0 - (SWORD) data_encoder.speed_now;
//	}
//	delay_count+=data_encoder.speed_real;
//	
//	
	//sending_service_package(0x55,0x0000,data_encoder.speed_real);
#if 0
	/* 发送位置 */
	{
		BYTE data[7];
		
		generate_remote_net_frame_to_send_site(WIFI_NET_CMD_CAR_REPORT_CURRENT_SITE, RFID_site_data.site, data);
		generate_remote_frame(WIFI_CMD_NET, data, sizeof(data));
	}
#endif
	EMIOS_0.CH[3].CSR.B.FLAG = 1;//清场中断标志位
	PIT.CH[1].TFLG.B.TIF = 1;	// MPC56xxB/P/S: Clear PIT 1 flag by writing 1
}


/*-----------------------------------------------------------------------*/
/* 设置速度PWM                                                                    */
/*-----------------------------------------------------------------------*/
void set_speed_pwm(int16_t speed_pwm)	//speed_pwm正为向前，负为向后
{
	if (speed_pwm>0)	//forward
	{
		StopL = 0;
		RunL = 1;
		if (speed_pwm>SPEED_PWM_MAX)
		{
			speed_pwm = SPEED_PWM_MAX;
		}
		EMIOS_0.CH[21].CBDR.R = speed_pwm;
		EMIOS_0.CH[22].CBDR.R = 1;
		
	}
	else if (speed_pwm<0)	//backward
	{
		StopL = 1;
		RunL = 0;
		speed_pwm = 0-speed_pwm;
		if (speed_pwm>SPEED_PWM_MAX)
		{
			speed_pwm = SPEED_PWM_MAX;
		}

		EMIOS_0.CH[21].CBDR.R = 1;
		EMIOS_0.CH[22].CBDR.R = speed_pwm;	
	}
	else
	{
		StopL = 1;
		RunL = 0;
		EMIOS_0.CH[21].CBDR.R = 1;
		EMIOS_0.CH[22].CBDR.R = 1;	
	}
}


/*-----------------------------------------------------------------------*/
/* BangBang速度控制                                                             */
/*-----------------------------------------------------------------------*/
void contorl_speed_encoder_bb(void)
{
	int32_t tmp_speed_now;
	
	
	if (data_encoder.is_forward)
	{
		tmp_speed_now = data_encoder.speed_now;
	}
	else
	{
		tmp_speed_now = 0 - data_encoder.speed_now;
	}
	
	if (tmp_speed_now > data_speed_settings.speed_target)
	{
		set_speed_pwm(0 - SPEED_PWM_MAX);
	}
	else if (tmp_speed_now < data_speed_settings.speed_target)
	{
		set_speed_pwm(SPEED_PWM_MAX);
	}
}


/*-----------------------------------------------------------------------*/
/* 获得速度偏差                                                                      */
/* 有问题找叶川                                                                      */
/*-----------------------------------------------------------------------*/
static SWORD get_e0()
{
	SWORD tmp_speed_now;
	SWORD e0;
	if (data_encoder.is_forward)
	{
		tmp_speed_now =(SWORD) data_encoder.speed_now;
	}
	else
	{
		tmp_speed_now = 0 - (SWORD) data_encoder.speed_now;
	}
	e0=data_speed_settings.speed_target-tmp_speed_now;
	return e0;
	
}


/*-----------------------------------------------------------------------*/
/* PID速度控制                                                                       */
/* 有问题找叶川                                                                      */                                                          
/*-----------------------------------------------------------------------*/
void control_speed_encoder_pid(void)
{
	SWORD d_speed_pwm=0;
	SWORD e0;
	static SWORD e1=0;
	static SWORD e2=0;
	static SWORD speed_pwm=SPEED_PWM_MIN;
	e0=get_e0();
	d_speed_pwm=(SWORD)(data_speed_pid.p*(e0-e1));       //P控制
	d_speed_pwm+=(SWORD)(data_speed_pid.d*(e0+e2-2*e1));
	d_speed_pwm+=(SWORD)(data_speed_pid.i*(e0));		
	if(d_speed_pwm>200)
	      d_speed_pwm=200;
	if(d_speed_pwm<-200)
	      d_speed_pwm=-200;   //限制pwm变化量
	speed_pwm+=d_speed_pwm;
	if(speed_pwm>SPEED_PWM_MAX)
			speed_pwm = SPEED_PWM_MAX;
	else if (speed_pwm<0-SPEED_PWM_MAX)
			speed_pwm =0- SPEED_PWM_MAX;    //防止溢出（造成负数）
	set_speed_pwm(speed_pwm);
	e2=e1;
	e1=e0;	
}
/*************************光编计数函数***********************/
void SpeedCount(void)
{
	speedcounter1=EMIOS_0.CH[23].CCNTR.R;              //左E7
	if(speedcounter1<speedcounter2)
		csl=speedcounter1+65535-speedcounter2;         //current speed left
	else 
		csl=speedcounter1-speedcounter2;
	if(backwardleft)
		csl=-csl;
	else 
		csl=csl;
	speedcounter2=speedcounter1;
	speedcounter3=EMIOS_0.CH[24].CCNTR.R;               //右D12
	if(speedcounter3<speedcounter4)
	{
		csr=speedcounter3+65535-speedcounter4;         //current speed right
	}
	else 
		csr=speedcounter3-speedcounter4;	
	if(backwardright) 
		csr=-csr;
	else 
		csr=csr;
	speedcounter4=speedcounter3;
//	if(csl==0||csr==0)
//		backflag=1;
}
/*-----------------------------------------------------------------------*/
/* 双电机控制                                                                       */
/*                                                                       */                                                          
/*-----------------------------------------------------------------------*/
void SET_motor(int leftSpeed,int rightSpeed)
{
	if(leftSpeed>=0) {EMIOS_0.CH[21].CBDR.R = 0;EMIOS_0.CH[22].CBDR.R =leftSpeed;}
		else {EMIOS_0.CH[21].CBDR.R = -leftSpeed;EMIOS_0.CH[22].CBDR.R = 0;}//左轮  E5左退   E6左进
	if(rightSpeed>=0) {EMIOS_0.CH[18].CBDR.R = rightSpeed;EMIOS_0.CH[20].CBDR.R = 0;}
		else {EMIOS_0.CH[18].CBDR.R = 0;EMIOS_0.CH[20].CBDR.R = -rightSpeed;}//右轮  E2右进   E4右退
}
#if 0
void DoubleSpeedControl()//闭环,加差速
{
//	RPID=CENTER-Steer_PWM[3];
//	r=Speed_kc/RPID;
//	tsr=((r-wheel_distance)/r)*targetspeed;//右轮减速
//	tsl=((r+wheel_distance+2)/r)*targetspeed;//左轮加速
//	SET_motor(tsl,tsr);
	tsl=targetspeed;
	tsr=targetspeed;
	
	ErrorLeft=(signed int)(tsl)-(signed int)(csl);
	ErrorRight=(signed int)(tsr)-(signed int)(csr);
	
	SumErrorLeft+=ErrorLeft;
	if(SumErrorLeft>350) SumErrorLeft=350;
	if(SumErrorLeft<-350) SumErrorLeft=-350;	    
	SumErrorRight+=ErrorRight;
	if(SumErrorRight>350) SumErrorRight=350;
	if(SumErrorRight<-350) SumErrorRight=-350;
	
	tsl=Speed_kp_Left*ErrorLeft+Speed_ki_Left*SumErrorLeft+Speed_kd_Left*(ErrorLeft-PreErrorLeft);
	tsr=Speed_kp_Right*ErrorRight+Speed_ki_Left*SumErrorRight+Speed_kd_Left*(ErrorRight-PreErrorRight);
	
	if(tsl>Motor_PWM_MAX)  tsl=Motor_PWM_MAX;
	else if(tsl<Motor_PWM_MIN)  tsl=Motor_PWM_MIN;	    
	if(tsr>Motor_PWM_MAX)  tsr=Motor_PWM_MAX;
	else if(tsr<Motor_PWM_MIN)  tsr=Motor_PWM_MIN;

	SET_motor(tsl,tsr);
	
	PreErrorLeft=ErrorLeft;
	PreErrorRight=ErrorRight;
}
#endif
#if 1
void DoubleSpeedControl2(void)//速度控制增量式
{
//	SET_motor(80,80);
//	return;
//	if(start_flag==0)
//		return;
//	if(cycle_flag)
//	{
		if(TargetSteer==data_steer_helm_basement.right_limit)
		{
			tsl=cyclespeedleft;
			tsr=cyclespeedright;
			D5=0;
		}
		else if(TargetSteer==data_steer_helm_basement.left_limit)
		{
			tsr=cyclespeedleft;
			tsl=cyclespeedright;
			D6=0;
		}
	    else
	    {
		    tsl=targetspeed;
		    tsr=targetspeed;
		    D7=0;
	    }
//	if(targetspeed==straightspeed)
//	{
//		Speed_kp_Left=0.05;//6月4日0.1
//		Speed_ki_Left=0.2;
//		Speed_kp_Right=0.05;//6月4日0.1
//		Speed_ki_Right=0.2;
//	}
//	else
//	{
//		Speed_kp_Left=10;
//		Speed_ki_Left=0.1;
//		Speed_kp_Right=10;
//		Speed_ki_Right=0.1;
//	}
	ErrorLeft=tsl-csl;
	ErrorRight=tsr-csr;
    D8=0;
    tsl_PWM+=(int)(Speed_kp_Left*(ErrorLeft-PreErrorLeft)+Speed_ki_Left*ErrorLeft+Speed_kd_Left*(ErrorLeft+Pre2ErrorLeft-2*PreErrorLeft));
	
	tsr_Delta=(int)(Speed_kp_Right*(ErrorRight-PreErrorRight)+Speed_ki_Right*ErrorRight+Speed_kd_Right*(ErrorRight+Pre2ErrorRight-2*PreErrorRight));
	tsr_PWM+=tsr_Delta;

	if(tsl_PWM>Motor_PWM_MAX)  tsl_PWM=Motor_PWM_MAX;
	else if(tsl_PWM<Motor_PWM_MIN)  tsl_PWM=Motor_PWM_MIN;	    
	if(tsr_PWM>Motor_PWM_MAX)  tsr_PWM=Motor_PWM_MAX;
	else if(tsr_PWM<Motor_PWM_MIN)  tsr_PWM=Motor_PWM_MIN;
	
	SET_motor(tsl_PWM,tsr_PWM);
	Pre2ErrorLeft=PreErrorLeft;
	Pre2ErrorRight=PreErrorRight;
	PreErrorLeft=ErrorLeft;
	PreErrorRight=ErrorRight;
	D5=1;
	D6=1;
	D7=1;
	D8=1;
}
#endif
/*-----------------------------------------------------------------------*/
/* 设置目标速度                                                                      */
/*-----------------------------------------------------------------------*/
void set_speed_target(SWORD speed_target)
{
	data_speed_settings.speed_target = speed_target;
}


/*-----------------------------------------------------------------------*/
/* 设置速度PID控制P值                                                            */
/*-----------------------------------------------------------------------*/
void set_speed_KP(float kp)
{
	data_speed_pid.p = kp;
}


/*-----------------------------------------------------------------------*/
/* 设置速度PID控制I值                                                             */
/*-----------------------------------------------------------------------*/
void set_speed_KI(float ki)
{
	data_speed_pid.i = ki;
}


/*-----------------------------------------------------------------------*/
/* 设置速度PID控制D值                                                            */
/*-----------------------------------------------------------------------*/
void set_speed_KD(float kd)
{
	data_speed_pid.d = kd;
}


/*-----------------------------------------------------------------------*/
/* 设置方向舵机位置                                                                */
/* 统一舵机访问接口                                                                */
/* 负数左舵，正数右舵，零中值                                                 */
/*-----------------------------------------------------------------------*/
void set_steer_helm(SWORD helmData)
{
	if(helmData <= data_steer_helm.left_limit)
	{
		helmData = data_steer_helm.left_limit;
	}
	else if(helmData >= data_steer_helm.right_limit)
	{
		helmData = data_steer_helm.right_limit;
	}
	helm_data_record = helmData;
//	helmData = (WORD)(helmData*data_steer_helm_basement.direction + data_steer_helm_basement.center);
//	LCD_Write_Num(96,6,(ABS((WORD)(helmData))),5);
//	delay_ms(500);
	set_steer_helm_basement(helmData);
}

/*-----------------------------------------------------------------------*/
/* 设置方向舵机位置                                                                */
/* 对于白色信号线的舵机：                                                       */
/* 面对舵机轴，占空比增大，舵机逆时针旋转，对我们的车是左舵    */
/* 对于橙色信号线的舵机：                                                       */
/* 相反                                                                                  */
/* 直接方向舵机寄存器                                                             */
/* 有限幅                                                                               */
/*-----------------------------------------------------------------------*/
void set_steer_helm_basement(WORD helmData)
{
#if 1
//	if(helmData <= 1500)
//	{
//		helmData = 1500;
//	}
//	else if(helmData >= 5000)
//	{
//		helmData = 5000;
//	}
//	if(g_device_NO!=3)
//	{
		if(helmData >= data_steer_helm_basement.left_limit)
		{
			helmData = data_steer_helm_basement.left_limit;
		}
		else if(helmData <= data_steer_helm_basement.right_limit)
		{
			helmData = data_steer_helm_basement.right_limit;
		}
//	}
//	else
//	{
//		if(helmData <= data_steer_helm_basement.left_limit+100)
//		{
//			helmData = data_steer_helm_basement.left_limit+100;
//		}
//		else if(helmData >= data_steer_helm_basement.right_limit-100)
//		{
//			helmData = data_steer_helm_basement.right_limit-100;
//		}
//	}
	if(helmData <=3700)
	{
//		LeftL = 1;
//		RightL = 0;
	}
	else if(helmData >=3800)
	{
//		RightL = 1;
//		LeftL = 0;
	}
	else
	{
//		LeftL = 0;
//		RightL = 0;
	}
#endif
	EMIOS_0.CH[9].CBDR.R = helmData;
}


/*-----------------------------------------------------------------------*/
/* 设置方向舵机底层数据 中值                                                   */
/* 更改方向舵机寄存器                                                             */
/*-----------------------------------------------------------------------*/
void set_steer_helm_basement_center(WORD helmData)
{
	data_steer_helm_basement.center = helmData;
	set_steer_helm_basement(helmData);
}


/*-----------------------------------------------------------------------*/
/* 设置方向舵机底层数据 左极限                                                */
/* 更改方向舵机寄存器                                                             */
/*-----------------------------------------------------------------------*/
void set_steer_helm_basement_left_limit(WORD helmData)
{
	data_steer_helm_basement.left_limit = helmData;
	set_steer_helm_basement(helmData);
}


/*-----------------------------------------------------------------------*/
/* 设置方向舵机底层数据 右极限                                                */
/* 更改方向舵机寄存器                                                             */
/*-----------------------------------------------------------------------*/
void set_steer_helm_basement_right_limit(WORD helmData)
{
	data_steer_helm_basement.right_limit = helmData;
	set_steer_helm_basement(helmData);
}


/*-----------------------------------------------------------------------*/
/* 将方向舵机底层数据更新到方向舵机上层数据                            */
/* 校验数据是否合理                                                                */
/* 合理则修改 返回0                                                                */
/* 不合理拒绝修改 返回1                                                          */
/*-----------------------------------------------------------------------*/
int update_steer_helm_basement_to_steer_helm(void)
{
	data_steer_helm.left_limit=3875;
	data_steer_helm.right_limit=3125 ;
	data_steer_helm.center=3511;
//	data_steer_helm.left_limit=3555;
//	data_steer_helm.right_limit=2775 ;
//	data_steer_helm.center=3185;
	if(data_steer_helm_basement.left_limit < data_steer_helm_basement.center && data_steer_helm_basement.center < data_steer_helm_basement.right_limit)
	{
		data_steer_helm_basement.direction = 1;
		data_steer_helm.left_limit = (SWORD)(data_steer_helm_basement.left_limit - data_steer_helm_basement.center);
		data_steer_helm.right_limit = (SWORD)(data_steer_helm_basement.right_limit - data_steer_helm_basement.center);
	}
	else if (data_steer_helm_basement.left_limit > data_steer_helm_basement.center && data_steer_helm_basement.center > data_steer_helm_basement.right_limit)
	{
		data_steer_helm_basement.direction = -1;
		data_steer_helm.left_limit = (SWORD)(data_steer_helm_basement.center - data_steer_helm_basement.left_limit);
		data_steer_helm.right_limit = (SWORD)(data_steer_helm_basement.center - data_steer_helm_basement.right_limit);
	}
	else
	{
		return 1;
	}
	return 0;
}


/*-----------------------------------------------------------------------*/
/* 获取两个周期计数的差值，常用故写成函数                               */
/*-----------------------------------------------------------------------*/
DWORD diff_time_basis_PIT(const DWORD new_time, const DWORD old_time)
{
	DWORD diff;
	
	if (new_time >= old_time)
	{
		diff = new_time - old_time;
	}
	else
	{
		diff = new_time + (0xFFFFFFFF- old_time);
	}
	
	return diff;
}
#if 0
int abs(int data)
{
	if (data<0)
		data=0-data;
	return data;
}
#endif
/*-----------------------------------------------------------------------*/
/* 设置BMW车门开启关闭                                 */     
/*-----------------------------------------------------------------------*/
void set_door_pwm(int16_t speed_pwm)	//speed_pwm正为向前，负为向后
{
	return;
}
void Road_Stop(void)
{
	set_speed_pwm(0);
}
