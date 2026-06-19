
#define PI 3.14159265359f
#define MAX_VELOCITY 15.0f	  
#define MAX_OMEGA PI*15	 	
#define LENGTH 0.457f	  
#define WHEEL_RADIUS 0.075f  
#define MODE_t  1		 
#define ANGLE2RAD(x) (x) * PI / 180.0f
#define MAX_ROBOT_VEL 10.0f // m/s
#define KEY_RISING_EDGE(cur, last, field)  ((cur.field == 1) && (last.field == 0))

#define MAX_ROBOT_OMEGA ANGLE2RAD(90.0f)
#define REMOTE_FIGER 1647.0f
#define EXCHANGE_WHEEL_CONFIG 23.8f
#define OMNI_WHEEL_FACTOR 0.5f
#define SQRT3_OVER_2 0.866f
#define MOTOR_ANGLE_OFFSET_DEG 6810.95f
#define UP_VOLLEY_MOTOR_MS 1000.0f

