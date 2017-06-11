//本车现在是1号车程序
#include "includes.h"
BYTE rev_ch;
WORD helm_use=0;
int16_t motor_use=0;
int direction;
BYTE haha;
short high2;
short low2;
short high3;
short low3;
short device_number2;
short data_number;
int supersonic_on_off=1;
int biaoji=0;
int jishu=0;
int zhangai=1;
int i;
int j;
extern int bz;
extern int right;
int velocity=0;
int sum=0;
int first_received=0;
BYTE video_sender;
extern int Hold_a;
extern int Hold_b;
extern int right_turn;
extern int target_near;
extern int get_ss;
extern int targetspeed;
extern int tsl,tsr;
extern int csl,csr;

void Mode0_DebugCamera(void);
void Mode1_SendVideo(void);
void Mode2_GO(void);
void Mode3_Andriod(void);

void main(void)
	{
	init_all_and_POST();
//	targetspeed=50;
	
//	for(;;)
//	{
//	SET_motor(500,500);
//	targetspeed=150;	
//	LCD_Write_Num(105,1,csl,4);
//	LCD_Write_Num(105,2,csr,4);
//	LCD_Write_Num(105,3,tsl,4);
//	LCD_Write_Num(105,4,tsr,4);
//	LCD_Write_Num(105,5,tsl_PWM,4);
//	LCD_Write_Num(105,6,tsr_PWM,4);
//		if(collision_switch1)
//		{
//			D5=1;
//			D6=0;
//		}
//		else
//		{
//			D5=0;D6=1;
//		}
//	}
//	set_steer_helm_basement(3248);
//	set_speed_pwm(500);
//	delay_ms(5000);
//	set_steer_helm_basement(data_steer_helm_basement.right_limit);
//	delay_ms(5000);
//	set_steer_helm_basement(data_steer_helm_basement.center);
//	sending_service_package(0x55,0x0066,0x01F4);
//	delay_ms(2000);
//	sending_service_package(0x55,0x0067,0x01F4);
//	delay_ms(2000);
//	sending_service_package(0x55,0x0068,0x01F4);
//	delay_ms(2000);
//	sending_service_package(0x55,0x0069,0x01F4);
	if(mode==0)		
		Mode0_DebugCamera();//图像显示屏显示，车速20，显示offset RoadType，舵机打角，wifi_car_action不激活
	else if(mode==1)
		Mode1_SendVideo();
	else
		Mode3_Andriod();//远程模式，上位机遥控车
}
void Mode0_DebugCamera(void)
{
	g_f_enable_speed_control=1;
	for (;;)
	{
		if (REMOTE_FRAME_STATE_OK == g_remote_frame_state)
		{
			g_remote_frame_state = REMOTE_FRAME_STATE_NOK;
			Wifi_Ctrl();
			first_received=1;			
		}	
			control_car_action();			// 全场动作控制
	}

}
void Mode1_SendVideo(void)
{
	g_f_enable_speed_control=1;
	for (;;)
	{
		if (REMOTE_FRAME_STATE_OK == g_remote_frame_state)
		{
			g_remote_frame_state = REMOTE_FRAME_STATE_NOK;
			Wifi_Ctrl();
			first_received=1;			
		}	
			control_car_action_stable();			// 全场动作控制
	}
}
void Mode3_Andriod(void)
{
	g_f_enable_speed_control=0;
	for (;;)
	{
		if (REMOTE_FRAME_STATE_OK == g_remote_frame_state)
		{
			g_remote_frame_state = REMOTE_FRAME_STATE_NOK;
			Wifi_Test();			
		}
		control_car_action();	
	}		// 全场动作控制
}
 


