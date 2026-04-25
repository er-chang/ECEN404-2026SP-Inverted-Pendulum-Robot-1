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
    ROBOT_HOVER,       // Critical for Mode 2 braking
	ROBOT_REVERSE,
    ROBOT_TURN_RIGHT,
    ROBOT_TURN_LEFT,
    ROBOT_STOP
} RobotState;

typedef struct Nav {
    // state tracking
    RobotState curr_state;
    float heading_rad; // where the robot is pointing

    // turn execution variables
    float target_yaw_rate;  // desired spin speed (dps)
    float current_yaw_rate; // ramped spin speed (dps)
    float target_heading;   // goal angle for a turn (rad)

    // Forward drive variables
    float target_tilt_bias;      // Used in Mode 2 (With Rod)
    float target_forward_effort; // Used in Mode 3 (Without Rod)
    uint32_t state_timer;        // Used for hover delays
} Nav;
/*END TYPEDEFS*/

/*FUNCTIONS*/
void Nav_Init(Nav *nav);
void Nav_UpdateStrategy(Nav *nav, float front_dist, float left_dist, float right_dist, uint8_t mode);
float Nav_UpdateSteering(Nav *nav, float current_gyro_z, float dt);
/*END FUNCTIONS*/

#ifdef __cplusplus
}
#endif

#endif
