#include "includes.h"
#include "math.h"

//*****************************************************************************************************************
//************************************************控制参数************************************************    	  *
//*****************************************************************************************************************

//**********************舵机参数**********************************************
signed long target_offset=0,last_offset=0;	//舵机偏差值记录
double Steer_kp=0,Steer_kd=0;//舵机P、D值
WORD Steer_PWM[4]={0,0,0,0};//舵机输出值记录
WORD steer=0;
extern long angle;
extern long Distanz;


//
////**********************电机参数**********************************************
//word speedcounter1=0,speedcounter2=0;

//signed int Error=0,PreError=0,SumError=0;
//
//double Speed_kp=0,Speed_ki=0,Speed_kd=0;	//电机PID值
//signed int Motor_PWM=0;				//电机底层控制量
//
////********************辅助调试参数******************************************
//unsigned int Counter_Error=0;		//光编接触不牢靠错误计数量
//////********************起始线停车参数******************************************
//byte stop_flag=0;
//byte stop_delay=0;

//********************2016赛季参数******************************************
extern int velocity;
extern int right;
//*****************************************************************************************************************
//************************************************角度控制************************************************    	  *
//*****************************************************************************************************************
void SteerControl()
{
	double adj_rate=asin(1.5/Distanz)*256/360;
	target_offset=angle;
	Steer_kd=2;
	if(ABS(target_offset)>40)
	{
		target_offset=1.5*target_offset;
	}
	else if(target_offset>0&&target_offset<=40)
	{
		target_offset=target_offset+adj_rate;
	}
	else if(target_offset<0&&target_offset>=-40)
	{
		target_offset=target_offset-adj_rate;
	}


                  //1号车PID参数       dby

	if(ABS(target_offset)<16)        {Steer_kp=2;}
	else if(ABS(target_offset)<32)  {Steer_kp=(ABS(target_offset)-16)*0.1875+2;}
	else if(ABS(target_offset)<48)  {Steer_kp=(ABS(target_offset)-32)*0.09375+5;}
	else if(ABS(target_offset)<64)  {Steer_kp=(ABS(target_offset)-48)*0.1875+6.5;}
	else                            {Steer_kp=9;}
//	if(ABS(target_offset)>40)
//	{
//		target_offset=target_offset*1.5;
//		
//	}


	steer=data_steer_helm_basement.center+Steer_kp*target_offset+Steer_kd*(target_offset-last_offset);//位置式PD
	
	//舵机限值+舵机输出
	//set_speed_pwm(velocity);
	set_steer_helm_basement(steer);
	//LCD_Write_Num(105,5,(int)Steer_PWM[3],4);
	//存舵机值和offset值
	//Steer_PWM[0]=Steer_PWM[1];Steer_PWM[1]=Steer_PWM[2];Steer_PWM[2]=Steer_PWM[3];
	last_offset=target_offset;
}

