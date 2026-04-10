#include "navigation.h"
#include <math.h>

// initialize all values to 0
void Nav_Init(Nav *nav) {
    nav->curr_state = ROBOT_START;
    nav->heading_rad = 0.0f;
    nav->target_yaw_rate = 0.0f;
    nav->current_yaw_rate = 0.0f;
    nav->target_heading = 0.0f;
}

// decision layer (w/ ultrasonic sensor)
void Nav_UpdateStrategy(Nav *nav, float front_dist, float left_dist, float right_dist) {

    switch(nav->curr_state) {

        case ROBOT_START:
            // wait for sensors to stabilize, then go
            nav->curr_state = ROBOT_FORWARD;
            break;

        case ROBOT_FORWARD:
            nav->target_yaw_rate = 0.0f; // drive straight, no turning

            // check for a wall ahead (e.g., less than 20cm)
            if (front_dist > 0.0f && front_dist < 0.20f) {

                // simple maze logic: decide which way to turn
            	// if there is more room to the left, turn left. otherwise, turn right
                if (left_dist > right_dist) {
                    nav->curr_state = ROBOT_TURN_LEFT; // set robot state to turn left
                    nav->target_heading = nav->heading_rad + 1.57f; // +90 degrees in radians
                    nav->target_yaw_rate = 45.0f; // spin left at 45 dps
                } else {
                    nav->curr_state = ROBOT_TURN_RIGHT; // set robot state to turn right
                    nav->target_heading = nav->heading_rad - 1.57f; // -90 degrees in radians
                    nav->target_yaw_rate = -45.0f; // spin right at 45 dps
                }
            }
            break;

        // perform turns slowly such that rod is still able to be balanced
        case ROBOT_TURN_RIGHT:
            // check if we have reached or passed the target heading
            if (nav->heading_rad <= nav->target_heading) {
                nav->heading_rad = nav->target_heading; // snap to exact 90 deg. angle
                nav->target_yaw_rate = 0.0f;            // stop spinning
                nav->curr_state = ROBOT_FORWARD;     	// resume driving
            }
            break;

        case ROBOT_TURN_LEFT:
            // check if we have reached or passed the target heading
            if (nav->heading_rad >= nav->target_heading) {
                nav->heading_rad = nav->target_heading; // snap to exact angle
                nav->target_yaw_rate = 0.0f;            // stop spinning
                nav->curr_state = ROBOT_FORWARD;     	// resume driving
            }
            break;

        // end of maze or encounter error.
        case ROBOT_STOP:
            nav->target_yaw_rate = 0.0f;
            break;
    }
}

// execution layer - manage turning while also maintaining balance of the rod
float Nav_UpdateSteering(Nav *nav, float current_gyro_z, float dt) {

    // dead reckoning
    float yaw_rad_s = current_gyro_z * (3.14159f / 180.0f);
    nav->heading_rad += yaw_rad_s * dt;

    // @@@@@@@@@@ angle wrapping (-PI to +PI) here if needed later @@@@@@@@@@

    // Trapezoidal Velocity Profile
    // limits how fast the robot is allowed to start spinning
    const float max_accel = 90.0f; // limit acceleration to 90 degrees/sec^2

    // HAVE WE REACHED GOAL ANGLE?
    // YES -> stop spinning, return to forward state
    // NO -> keep spinning
    if (nav->current_yaw_rate < nav->target_yaw_rate) {
        nav->current_yaw_rate += max_accel * dt;
        if (nav->current_yaw_rate > nav->target_yaw_rate) nav->current_yaw_rate = nav->target_yaw_rate;
    }
    else if (nav->current_yaw_rate > nav->target_yaw_rate) {
        nav->current_yaw_rate -= max_accel * dt;
        if (nav->current_yaw_rate < nav->target_yaw_rate) nav->current_yaw_rate = nav->target_yaw_rate;
    }

    // YAW RATE PID
    // P: calculates effort based on the difference between desired and actual spin
    float error = nav->current_yaw_rate - current_gyro_z;

    // Yaw Kp Gain: start small (e.g., 0.02) and tune up until turns are snappy but stable
    float kp_yaw = 0.02f;
    float effort_turn = error * kp_yaw;

    return effort_turn;
}

