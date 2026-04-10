#ifndef NAVIGATION_H
#define NAVIGATION_H

/*INCLUDES*/
#include "main.h"
/*END INCLUDES*/

#ifdef __cplusplus
extern "C" {
#endif

/*TYPEDEFS*/
typedef enum {
    ROBOT_START,
    ROBOT_FORWARD,
    ROBOT_TURN_RIGHT,
    ROBOT_TURN_LEFT,
    ROBOT_STOP
} RobotState;

typedef struct Nav {
	// state tracking
	RobotState curr_state;
	float heading_rad; // where the robot is pointing

	// turn execution variables
	float target_yaw_rate; // desired spin speed (dps)
	float current_yaw_rate; // ramped spin speed (dps)
	float target_heading; // goal angle for a turn (rad)

} Nav;

/*END TYPEDEFS*/

/*FUNCTIONS*/
void Nav_Init(Nav *nav);
void Nav_UpdateStrategy(Nav *nav, float front_dist, float left_dist, float right_dist);
float Nav_UpdateSteering(Nav *nav, float current_gyro_z, float dt);
/*END FUNCTIONS*/

#ifdef __cplusplus
}
#endif

#endif
