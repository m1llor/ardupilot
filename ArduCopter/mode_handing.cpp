#include "Copter.h"

/*
 * ModeHanding: simple vertical takeoff/hold mode.
 * Pilot commands climb or descent with throttle stick; controller integrates
 * the requested climb rate into a vertical position target and holds it.
 * No roll/pitch/yaw motion is commanded so the vehicle should stay level
 * and maintain heading while holding altitude.
 */

 bool ModeHanding::init(bool ignore_checks)
    {
    // initialise the vertical position controller
    if (!pos_control->is_active_U()) {
        pos_control->init_U_controller();
    }

    // hold the current altitude immediately on entry
    pos_control->set_pos_target_U_from_climb_rate_m(0.0f);
    
    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_U_m(-get_pilot_speed_dn_ms(), get_pilot_speed_up_ms(), get_pilot_accel_U_mss());
    pos_control->set_correction_speed_accel_U_m(-get_pilot_speed_dn_ms(), get_pilot_speed_up_ms(), get_pilot_accel_U_mss());

    return true;
}

void ModeHanding::run()
{
    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_U_m(-get_pilot_speed_dn_ms(), get_pilot_speed_up_ms(), get_pilot_accel_U_mss());

    // pilot desired climb rate (m/s)
    float target_climb_rate_ms = get_pilot_desired_climb_rate_ms();
    target_climb_rate_ms = constrain_float(target_climb_rate_ms, -get_pilot_speed_dn_ms(), get_pilot_speed_up_ms());

    // determine state using shared altitude-hold logic
    HandingModeState handing_state = get_handing_state_U_ms(target_climb_rate_ms);

    switch (handing_state) {

    case HandingModeState::MotorStopped:
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->reset_yaw_target_and_rate(false);
        pos_control->relax_U_controller(0.0f);
        break;

    case HandingModeState::Landed_Ground_Idle:
        attitude_control->reset_yaw_target_and_rate();
        FALLTHROUGH;

    case HandingModeState::Landed_Pre_Takeoff:
        attitude_control->reset_rate_controller_I_terms_smoothly();
        pos_control->relax_U_controller(0.0f);
        break;

    case HandingModeState::Takeoff:
        // initiate take-off
        if (!takeoff.running()) {
            takeoff.start_m(constrain_float(g.pilot_takeoff_alt_cm * 0.01f, 0.0f, 10.0f));
        }
        
                // use avoidance adjusted climb rate
        target_climb_rate_ms = get_avoidance_adjusted_climbrate_ms(target_climb_rate_ms);

        // set position controller targets adjusted for pilot input
        takeoff.do_pilot_takeoff_ms(target_climb_rate_ms);
                break;

    case HandingModeState::Flying:
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // use avoidance adjusted climb rate
        target_climb_rate_ms = get_avoidance_adjusted_climbrate_ms(target_climb_rate_ms);

#if AP_RANGEFINDER_ENABLED
        // update the vertical offset based on the surface measurement
        copter.surface_tracking.update_surface_offset();
#endif
        // send the commanded climb rate to the position controller
        pos_control->set_pos_target_U_from_climb_rate_m(target_climb_rate_ms);
        break;
    }

    // keep the vehicle level and heading-stable (no yaw rate command)
    const float roll_target_rad = 0.0f;
    const float pitch_target_rad = 0.0f;
    const float yaw_rate_target_rads = 0.0f;
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_rad(roll_target_rad, pitch_target_rad, yaw_rate_target_rads);

    // run the vertical position controller and set output throttle
    pos_control->update_U_controller();
}

