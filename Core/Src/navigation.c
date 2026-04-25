#include "navigation.h"
#include <math.h>

// Obstacle thresholds
#define AVOID_DIST_CM 40.0f
#define CLEAR_DIST_CM 55.0f

// Consider robot dimensions for turning
#define SWING_CLEARANCE_CM 20.0f // 15cm minimum + 5cm safety buffer

void Nav_Init(Nav *nav) {
    nav->curr_state = ROBOT_START;
    nav->heading_rad = 0.0f;
    nav->target_yaw_rate = 0.0f;
    nav->current_yaw_rate = 0.0f;
    nav->target_heading = 0.0f;

    nav->target_tilt_bias = 0.0f;
    nav->target_forward_effort = 0.0f;
    nav->state_timer = 0;
}

void Nav_UpdateStrategy(Nav *nav, float front_dist, float left_dist, float right_dist, uint8_t mode) {
    // Safety: If a sensor misses an echo, it often reads 0. Treat 0 as open space.
    if (front_dist < 1.0f) front_dist = 999.0f;
    if (left_dist < 1.0f) left_dist = 999.0f;
    if (right_dist < 1.0f) right_dist = 999.0f;


    // ═══════════════════════════════════════════════════════════════════
    // MODE 2: INVERTED PENDULUM (BALANCING)
    // ═══════════════════════════════════════════════════════════════════
    if (mode == 2) {
        switch(nav->curr_state) {
            case ROBOT_START:
                nav->target_tilt_bias = 0.0f;
                nav->target_yaw_rate = 0.0f;
                nav->curr_state = ROBOT_FORWARD;
                break;

            case ROBOT_FORWARD:
                // INVERTED: Negative bias commands a forward lean.
                nav->target_tilt_bias = -0.018f; // Bumped speed slightly
                nav->target_yaw_rate = 0.0f;

                if (front_dist < AVOID_DIST_CM) {
					if (left_dist > right_dist && left_dist > SWING_CLEARANCE_CM) {
						nav->curr_state = ROBOT_TURN_LEFT;
					}
					else if (right_dist > left_dist && right_dist > SWING_CLEARANCE_CM) {
						nav->curr_state = ROBOT_TURN_RIGHT;
					}
					else {
						nav->curr_state = ROBOT_REVERSE;
					}
				}
                break;

            case ROBOT_HOVER:
                nav->target_tilt_bias = 0.0f; // Return to perfect vertical to brake
                nav->target_yaw_rate = 0.0f;

                if (HAL_GetTick() - nav->state_timer > 1500) {
					if (left_dist > right_dist && left_dist > SWING_CLEARANCE_CM) {
						nav->curr_state = ROBOT_TURN_LEFT;
					}
					else if (right_dist > left_dist && right_dist > SWING_CLEARANCE_CM) {
						nav->curr_state = ROBOT_TURN_RIGHT;
					}
					else {
						nav->curr_state = ROBOT_TURN_LEFT;
						nav->target_heading += 3.14159f; // Command a 180 spin
					}
				}
                break;

            case ROBOT_REVERSE:
                // Lean backward! (Opposite of forward bias). Bumped up slightly.
                nav->target_tilt_bias = 0.020f;
                nav->target_yaw_rate = 0.0f;

                // Reverse until we have enough clear space in front
                if (front_dist > CLEAR_DIST_CM) {
                    if (left_dist > right_dist) {
                        nav->curr_state = ROBOT_TURN_LEFT;
                    } else {
                        nav->curr_state = ROBOT_TURN_RIGHT;
                    }
                }
                break;

            case ROBOT_TURN_LEFT:
                nav->target_tilt_bias = 0.0f; // Stay vertical while spinning
                nav->target_yaw_rate = 55.0f; // Spin left faster

                if (front_dist > CLEAR_DIST_CM) {
                    nav->curr_state = ROBOT_FORWARD;
                }
                break;

            case ROBOT_TURN_RIGHT:
                nav->target_tilt_bias = 0.0f;
                nav->target_yaw_rate = -55.0f; // Spin right faster

                if (front_dist > CLEAR_DIST_CM) {
                    nav->curr_state = ROBOT_FORWARD;
                }
                break;

            default:
                nav->curr_state = ROBOT_FORWARD;
                break;
        }
    }

    // ═══════════════════════════════════════════════════════════════════
    // MODE 3: STANDARD ROVER (NO ROD)
    // ═══════════════════════════════════════════════════════════════════
    else if (mode == 3) {
        switch(nav->curr_state) {
            case ROBOT_START:
            case ROBOT_HOVER:
                nav->curr_state = ROBOT_FORWARD;
                break;

            case ROBOT_FORWARD:
                // INVERTED: Negative effort is physical forward.
				nav->target_forward_effort = -0.80f; // Bumped forward speed
				nav->target_yaw_rate = 0.0f;

				// Rover can stop and turn instantly
				if (front_dist < AVOID_DIST_CM) {
					if (left_dist > right_dist && left_dist > SWING_CLEARANCE_CM) {
						nav->curr_state = ROBOT_TURN_LEFT;
					}
					else if (right_dist > left_dist && right_dist > SWING_CLEARANCE_CM) {
						nav->curr_state = ROBOT_TURN_RIGHT;
					}
					else {
						nav->curr_state = ROBOT_REVERSE;
					}
				}
				break;

            case ROBOT_REVERSE:
                // Drive backward!
                nav->target_forward_effort = 0.25f; // Bumped reverse speed
                nav->target_yaw_rate = 0.0f;

                // Back up until clear
                if (front_dist > CLEAR_DIST_CM) {
                    if (left_dist > right_dist) {
                        nav->curr_state = ROBOT_TURN_LEFT;
                    } else {
                        nav->curr_state = ROBOT_TURN_RIGHT;
                    }
                }
                break;

            case ROBOT_TURN_LEFT:
                nav->target_forward_effort = 0.0f; // Stop forward drive
                nav->target_yaw_rate = 100.0f;      // Turn much faster to break friction

                if (front_dist > CLEAR_DIST_CM) {
                    nav->curr_state = ROBOT_FORWARD;
                }
                break;

            case ROBOT_TURN_RIGHT:
                nav->target_forward_effort = 0.0f;
                nav->target_yaw_rate = -100.0f;

                if (front_dist > CLEAR_DIST_CM) {
                    nav->curr_state = ROBOT_FORWARD;
                }
                break;

            default:
                nav->curr_state = ROBOT_FORWARD;
                break;
        }
    }
}

// ═══════════════════════════════════════════════════════════════════
// Execution Layer
// ═══════════════════════════════════════════════════════════════════
float Nav_UpdateSteering(Nav *nav, float current_gyro_z, float dt) {
    float yaw_rad_s = current_gyro_z * (3.14159f / 180.0f);
    nav->heading_rad += yaw_rad_s * dt;

    const float max_accel = 120.0f;

    // Ramp up the target yaw rate so it doesn't jerk the chassis
    if (nav->current_yaw_rate < nav->target_yaw_rate) {
        nav->current_yaw_rate += max_accel * dt;
        if (nav->current_yaw_rate > nav->target_yaw_rate) nav->current_yaw_rate = nav->target_yaw_rate;
    }
    else if (nav->current_yaw_rate > nav->target_yaw_rate) {
        nav->current_yaw_rate -= max_accel * dt;
        if (nav->current_yaw_rate < nav->target_yaw_rate) nav->current_yaw_rate = nav->target_yaw_rate;
    }

    // Simple P-controller to chase the target spin speed
    float error = nav->current_yaw_rate - current_gyro_z;

    // INCREASED MULTIPLIER: Raise to overpower rubber tire friction
    float turn_effort = error * 0.025f;

    // Hard stop to prevent drifting
    if (nav->target_yaw_rate == 0.0f && fabs(current_gyro_z) < 3.0f) {
        turn_effort = 0.0f;
    }

    return turn_effort;
}
