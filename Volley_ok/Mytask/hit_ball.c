#include "hit_ball.h"
//exp_param up_exp;
//exp_param down_exp;
//static uint8_t traj_started = 0;


//RobStride_t stride_up;
//RobStride_t stride_down;
//PID2 up_pid;
//PID2 down_pid;
//PID2 up_vel_pid;
//PID2 down_vel_pid;

//Vec2 O1 = { 0.0875f, 0.21841f };
//Vec2 O2 = { 0.2775f, 0.11341f };
//Vec2 Cw = { 0.0f , 0.0f };
//Vec2 Op = { 0.0f , 0.0f };
//float motor1_angle = 0.0f;
//float motor2_angle = 0.0f;
//int take = 0;
//int flag1 = 0;

float exp_angle = 0;
TaskHandle_t Hit_Task_Handle;

void Hit_Task(void *pvParameters)
{
	//up_pid.Kp = 0.0f;
	//up_pid.Ki = 0.0f;
	//up_pid.Kd = 0.0f;
	//up_pid.limit = 100000.0f;
	//up_pid.output_limit = 30.0f;

	//down_pid.Kp = 0.0f; 
	//down_pid.Ki = 0.0f;
	//down_pid.Kd = 0.0f;
	//down_pid.limit = 100000.0f;
	//down_pid.output_limit = 30.0f;

	//up_vel_pid.Kp = 0.0f;
	//up_vel_pid.Ki = 0.0f;
	//up_vel_pid.Kd = 0.0f;
	//up_vel_pid.limit = 100000.0f;
	//up_vel_pid.output_limit = 30.0f;

	//down_vel_pid.Kp = 0.0f;
	//down_vel_pid.Ki = 0.0f;
	//down_vel_pid.Kd = 0.0f;
	//down_vel_pid.limit = 10000.00f;
	//down_vel_pid.output_limit = 30.0f;

//	vTaskDelay(2000);
//	RobStrideInit(&stride_up,&hcan1,0x01,RobStride_MotionControl,RobStride_04);
//	RobStrideInit(&stride_down,&hcan1,0x02,RobStride_MotionControl,RobStride_04);
//	RobStrideSetMode(&stride_up,RobStride_MotionControl);
//	RobStrideSetMode(&stride_down,RobStride_MotionControl);
//	vTaskDelay(200);
//	RobStrideEnable(&stride_up);
//	RobStrideEnable(&stride_down);
//	vTaskDelay(2000);
//	
//	  CubicParam_t trajectory;
//    float p0 = 0.0f;
//    float v0 = 0.0f;
//    float a0 = 0.0f;
//    float pT = 1.0f;
//    float vT = 0.0f;
//    float aT = 0.0f;
//    float duration = 2.0f;
//    uint32_t current_tick = HAL_GetTick();
		
	TickType_t Last_wake_time = xTaskGetTickCount();
	for(;;)
	{
//		Cubic_SetTrajectory(&trajectory, p0, v0, pT, vT, duration, current_tick);
//		CubicParam_t traj_up;
//		CubicParam_t traj_down;
//        TrajectoryState_t traj_up_state;
//		TrajectoryState_t traj_down_state;
//		
//		float theta = atan2f(-(Cw.x - Op.x),(Cw.y - Op.y));
//			
//		float c = cosf(theta);
//		float s = sinf(theta);

//		Vec2 P1w = {
//			Op.x - HALF_P * c,
//			Op.y - HALF_P * s
//		};

//	    Vec2 P2w = {
//			Op.x + HALF_P * c,
//			Op.y + HALF_P * s
//		};
//			
//		Vec2 r1 = {
//			P1w.x - O1.x,
//			P1w.y - O1.y
//		};

//		float d1 = sqrtf(r1.x*r1.x + r1.y*r1.y);
//		float phi1 = atan2f(r1.y, r1.x);
//		float alpha1 = acosf(d1 / (2.0f * L_OA));

//		motor1_angle = phi1 + alpha1;
//			
//		Vec2 r2 = {
//		    P2w.x - O2.x,
//			P2w.y - O2.y
//		};

//		float d2 = sqrtf(r2.x * r2.x + r2.y * r2.y);
//		float phi2 = atan2f(r2.y, r2.x);
//		float alpha2 = acosf(d2 / (2.0f * L_OA));

//		motor2_angle = phi2 - alpha2; 
//			
//			
//if(take == 1 && traj_started == 0)
//{
//Quintic_SetTrajectory(
//	&traj_up,
//    stride_up.state.rad,
//	stride_up.state.omega,
//	0,
//	motor1_angle,
//	0,
//	0,
//	0.17f,
//	xTaskGetTickCount()
//	);
//	
//	Quintic_SetTrajectory(
//	&traj_down,
//stride_down.state.rad,
//	stride_down.state.omega,
//	0,
//	motor2_angle,
//	0,
//	0,
//	0.17f,
//	xTaskGetTickCount()
//	);
//	traj_started = 1;
////  RobStrideMotionControl(&stride_up, 0x01, 0, 0, 0, 0, 0);
////	RobStrideMotionControl(&stride_down, 0x02, 0, 0, 0, 0, 0);

//}	
//		if(take == 1)
//  	{
//	Quintic_GetFullState(&traj_up,xTaskGetTickCount(),&traj_up_state);
//	Quintic_GetFullState(&traj_down,xTaskGetTickCount(),&traj_down_state);
//	RobStrideMotionControl(&stride_up, 0x01, up_exp.exp_tor, traj_up_state.pos, up_exp.exp_omega, up_exp.exp_kp, up_exp.exp_kd);
//	RobStrideMotionControl(&stride_down, 0x02, down_exp.exp_tor, traj_down_state.pos, down_exp.exp_omega, down_exp.exp_kp, down_exp.exp_kd);
//			}
//   if(flag1 == 0 && take == 0)
//	 {
//    RobStrideMotionControl(&stride_up, 0x01, 0, 0, 0, 0, 0);
//	RobStrideMotionControl(&stride_down, 0x02, 0, 0, 0, 0, 0);
//	 }
//		if(flag1 == 0)
//		{
//		////RobStrideGet(&stride_up,up_cmd);
//		////RobStrideGet(&stride_down,down_cmd);
//		//PID_Control2(stride_up.state.rad,up_rad,&up_pid);
//		//PID_Control2(stride_down.state.rad,down_rad,&down_pid)

//		//PID_Control2(stride_up.state.omega,up_pid.pid_out,&up_vel_pid);
//		//PID_Control2(stride_down.state.omega,down_pid.pid_out,&down_vel_pid);
//		////RobStrideMotionControl(&stride_up,  0x01,  up_tor,   up_vel_pid.pid_out,   up_omega,   up_kp,   up_kd);
//		////RobStrideMotionControl(&stride_down,0x02,down_tor, down_rad, down_vel_pid.pid_out, down_kp, down_kd);

//		//	
//   RobStrideMotionControl(&stride_up,  0x01,  0,   stride_up.state.rad,  0,   0,   0);
//   RobStrideMotionControl(&stride_down,0x02,0, stride_down.state.rad,0, 0, 0);
//		}
//		if (flag1 == 1)
//		{
////		PID_Control2(stride_up.state.rad,up_rad,&up_pid);
////		PID_Control2(stride_down.state.rad,down_rad,&down_pid);//λû

////		PID_Control2(stride_up.state.omega,up_pid.pid_out,&up_vel_pid);
////		PID_Control2(stride_down.state.omega,down_pid.pid_out,&down_vel_pid);//ٶȻ
//			
//		RobStrideMotionControl(&stride_up,  0x01,  up_tor,   up_rad,  up_omega,   up_kp,   up_kd);
//		RobStrideMotionControl(&stride_down,0x02,down_tor, down_rad,down_omega, down_kp, down_kd);

////		RobStrideTorqueControl(&stride_up  ,up_vel_pid.pid_out);
////		RobStrideTorqueControl(&stride_down,down_vel_pid.pid_out);
//		}
//	 }
	vTaskDelayUntil(&Last_wake_time, pdMS_TO_TICKS(2));
  }
}

//void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
//{
//	uint8_t buf[8];
//	uint32_t ID = CAN_Receive_DataFrame(&hcan1, buf);
//	RobStrideRecv_Handle(&stride_up, hcan, ID, buf);
//	RobStrideRecv_Handle(&stride_down, hcan, ID, buf);
//}
