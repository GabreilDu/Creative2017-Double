#ifndef __CONTROL_H__
#define __CONTROL_H__


/* 方向舵机PWM */
#define STEER_HELM_CENTER (3511)
#define STEER_HELM_TEST (1500)
#define STEER_HELM_LEFT (3875)
#define STEER_HELM_RIGHT (3125)
#define STEER_HELM_MIX (1038)
#define STEER_HELM_MAX (6146)

/* 信号舵机PWM */
#define SINGLE_HELM_CENTER (50)

/* 速度PWM */
#define SPEED_PWM_MIN (1)
#define SPEED_PWM_MAX (1900)
#define SPEED_PWM_TEST (20)

/* 速度光编 */
#define SPEED_ENCODER_STOP (0x0000)
#define SPEED_ENCODER_MIN (0x0002)
#define SPEED_ENCODER_MAX (0x01F4)
#define SPEED_ENCODER_TEST (11)

extern int g_f_pit;
extern int g_f_enable_mag_steer_control;
extern int g_f_enable_speed_control;
extern int g_f_enable_rad_control_1;
extern int g_f_enable_rad_control_2;
extern int g_f_enable_speed_control_2;
extern int g_f_enable_single_bridge_control;
extern int g_f_enable_fly_bridge;
extern int g_f_enable_steer_bridge;
extern int g_f_enable_supersonic;	/* 启用超声探测标志位 */

extern int read_rad_xyz;
extern int find_mag_back_box;
extern int find_mag_back_box_2;
extern int find_mag_back_car1;

extern int g_f_big_U;
extern int g_f_big_U_2;
extern int counter;


/* 时间基准 */
#ifdef __CONTROL_C_
DWORD g_time_basis_PIT;
#else
extern const DWORD g_time_basis_PIT;
#endif

/* 方向舵机上层数据 */
#ifdef __CONTROL_C_
struct
{
	SWORD center;
	SWORD left_limit;
	SWORD right_limit;
} data_steer_helm = {(SWORD) 0, STEER_HELM_LEFT-STEER_HELM_CENTER, STEER_HELM_RIGHT-STEER_HELM_CENTER };
#else
extern const struct
{
	SWORD center;
	SWORD left_limit;
	SWORD right_limit;
} data_steer_helm;
#endif

/* 方向舵机底层数据 */
#ifdef __CONTROL_C_
struct
{
	WORD center;
	WORD left_limit;
	WORD right_limit;
	SWORD direction;
} data_steer_helm_basement = { STEER_HELM_CENTER, STEER_HELM_LEFT, STEER_HELM_RIGHT, 1 };
#else
extern const struct
{
	WORD center;
	WORD left_limit;
	WORD right_limit;
	SWORD direction;
} data_steer_helm_basement;
#endif

/* 光编数据 */
#ifdef __CONTROL_C_
struct
{
	WORD cnt_old;
	WORD cnt_new;
	WORD speed_now;
	SWORD speed_real;
	SWORD is_forward;
} data_encoder = { 0x0000, 0x0000, 0x0000, 0,0};
#else
extern struct
{
	WORD cnt_old;
	WORD cnt_new;
	WORD speed_now;
	SWORD speed_real;
	SWORD is_forward;
} data_encoder;
#endif

/* 速度PID数据 */
#ifdef __CONTROL_C_
struct
{
	float p;
	float i;
	float d;
} data_speed_pid = { 22 , 1.5 , 5 };
#else
extern struct
{
	float p;
	float i;
	float d;
} data_speed_pid;
#endif

/* 速度设置数据 */
#ifdef __CONTROL_C_
struct
{
	SWORD speed_pwm;
	SWORD speed_target;
} data_speed_settings = { 0x0000, 0x0000, };
#else
extern struct
{
	SWORD speed_pwm;
	SWORD speed_target;
} data_speed_settings;
#endif

/* 方向舵机位置数据 */
#ifdef __CONTROL_C_
WORD helm_data_record = STEER_HELM_CENTER;
#endif

extern void PitISR(void);
extern void set_speed_pwm(int16_t speed_pwm);
extern void set_door_pwm(int16_t speed_pwm);
extern void contorl_speed_encoder_bb(void);
extern void set_speed_target(SWORD speed_target);
extern void set_speed_KP(float kp);
extern void set_speed_KI(float ki);
extern void set_speed_KD(float kd);
extern void set_steer_helm(SWORD helmData);
extern void control_speed_encoder_pid(void);
extern DWORD diff_time_basis_PIT(const DWORD new_time, const DWORD old_time);
extern void set_steer_helm_basement_center(WORD helmData);
extern void set_steer_helm_basement_left_limit(WORD helmData);
extern void set_steer_helm_basement_right_limit(WORD helmData);
extern int update_steer_helm_basement_to_steer_helm(void);
extern void set_steer_helm_basement(WORD helmData);
extern void control_angle_steer_helm(int angle_target);
extern void control_speed_motor(int speed_target);
extern int abs(int data);
extern void set_steer_helm(SWORD helmData);
extern void Road_Stop(void);
extern void Steer_PDSet(void);
extern void SET_motor(int leftSpeed,int rightSpeed);
extern void DoubleSpeedControl(void);
extern void DoubleSpeedControl2(void);
extern void SpeedCount(void);

extern int csl,csr;//currentspeedleft=0,currentspeedright=0;
extern int tsl,tsr;//targetspeedleft=0,targetspeedright=0;
extern int targetspeed; 
extern int	Motor_PWM_MAX; 
extern int	Motor_PWM_MIN;
extern int cyclespeed,turnspeed,straightspeed,cyclespeedleft,cyclespeedright;
extern unsigned int speedcounter1,speedcounter2,speedcounter3,speedcounter4;
//**********************差速参数***************************/
extern signed int Speed_kc;
extern signed int wheel_distance;//半车距8
extern signed int RPID;
extern double r;
//**********************电机PID参数**********************************************;	
extern double Speed_kp_Left,Speed_ki_Left,Speed_kd_Left;//12,0.6
extern double Speed_kp_Right,Speed_ki_Right,Speed_kd_Right;	//12,0.85

extern int ErrorLeft,PreErrorLeft,Pre2ErrorLeft,SumErrorLeft,ErrorRight,PreErrorRight,Pre2ErrorRight,SumErrorRight;
extern int intErrorLeft,intErrorRight;
extern float Pwm_Delta_Left,Pwm_Delta_Right; 
extern int tsl_PWM,tsr_PWM,tsr_Delta,error_Delta;







#endif
