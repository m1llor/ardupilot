// #include "Copter.h"

// /*
//  * ModeHanding: simple vertical takeoff/hold mode.
//  * Pilot commands climb or descent with throttle stick; controller integrates
//  * the requested climb rate into a vertical position target and holds it.
//  * Horizontal position (X/Y) is held using NE position controller; yaw is held.
//  */

// // maximum altitude above home allowed in HANDING mode (meters)
// float HANDING_ALT_MAX_M = 150.0f;

// bool ModeHanding::init(bool ignore_checks)
// {
//     if (!ignore_checks && !motors->armed()) {
//         return false;
//     }

//     // initialise vertical controller if needed
//     if (!pos_control->is_active_U()) {
//         pos_control->init_U_controller();
//     }

//     // initialise horizontal controller to a stopping point at current position
//     pos_control->init_NE_controller_stopping_point();
//     pos_control->relax_velocity_controller_NE();

//     // hold current altitude on entry
//     pos_control->set_pos_target_U_from_climb_rate_m(0.0f);

//     // constrain initial target altitude to the HANDING limit
//     if (pos_control->get_pos_target_U_m() > HANDING_ALT_MAX_M) {
//         pos_control->set_alt_target_with_slew_m(HANDING_ALT_MAX_M);
//     }

//     // set vertical speed and acceleration limits
//     pos_control->set_max_speed_accel_U_m(-get_pilot_speed_dn_ms(),
//                                          get_pilot_speed_up_ms(),
//                                          get_pilot_accel_U_mss());
//     pos_control->set_correction_speed_accel_U_m(-get_pilot_speed_dn_ms(),
//                                                 get_pilot_speed_up_ms(),
//                                                 get_pilot_accel_U_mss());

//     // lock current yaw
//     attitude_control->reset_yaw_target_and_rate();

//     return true;
// }

// void ModeHanding::run()
// {
//     // set vertical speed and acceleration limits
//     pos_control->set_max_speed_accel_U_m(-get_pilot_speed_dn_ms(),
//                                          get_pilot_speed_up_ms(),
//                                          get_pilot_accel_U_mss());

//     // pilot desired climb rate (m/s)
//     float target_climb_rate_ms = get_pilot_desired_climb_rate_ms();
//     target_climb_rate_ms = constrain_float(target_climb_rate_ms,
//                                            -get_pilot_speed_dn_ms(),
//                                             get_pilot_speed_up_ms());

//     // block further ascent once the altitude target reaches the HANDING limit
//     if ((pos_control->get_pos_target_U_m() >= HANDING_ALT_MAX_M) && (target_climb_rate_ms > 0.0f)) {
//         target_climb_rate_ms = 0.0f;
//     }


//     // determine state using shared altitude-hold logic
//     HandingModeState handing_state = get_handing_state_U_ms(target_climb_rate_ms);

//     // roll/pitch targets будем брать из NE-контроллера, по умолчанию 0
//     float roll_target_rad  = 0.0f;
//     float pitch_target_rad = 0.0f;

//     switch (handing_state) {

//     case HandingModeState::MotorStopped:
//         attitude_control->reset_rate_controller_I_terms();
//         attitude_control->reset_yaw_target_and_rate(false);
//         pos_control->relax_U_controller(0.0f);
//         pos_control->relax_velocity_controller_NE();
//         break;

//     case HandingModeState::Landed_Ground_Idle:
//         attitude_control->reset_yaw_target_and_rate();
//         FALLTHROUGH;

//     case HandingModeState::Landed_Pre_Takeoff:
//         attitude_control->reset_rate_controller_I_terms_smoothly();
//         pos_control->relax_U_controller(0.0f);
//         // pos_control->relax_velocity_controller_NE();
//         break;

//     case HandingModeState::Takeoff:
//         if (!takeoff.running()) {
//             takeoff.start_m(constrain_float(g.pilot_takeoff_alt_cm * 0.01f, 0.0f, 10.0f));
//         }

//         target_climb_rate_ms = get_avoidance_adjusted_climbrate_ms(target_climb_rate_ms);
//         takeoff.do_pilot_takeoff_ms(target_climb_rate_ms);

//         // по X/Y — удержание позиции
//         pos_control->update_NE_controller();
//         roll_target_rad  = pos_control->get_roll_rad();
//         pitch_target_rad = pos_control->get_pitch_rad();
//         break;

//     case HandingModeState::Flying:
//         motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

//         target_climb_rate_ms = get_avoidance_adjusted_climbrate_ms(target_climb_rate_ms);

//         pos_control->set_pos_target_U_from_climb_rate_m(target_climb_rate_ms);

//         // по X/Y — держим текущую цель, считаем компенсацию наклона
//         // ensure altitude target never exceeds the HANDING maximum
//         if (pos_control->get_pos_target_U_m() > HANDING_ALT_MAX_M) {
//             pos_control->set_alt_target_with_slew_m(HANDING_ALT_MAX_M);
//         }
//         pos_control->update_NE_controller();
//         roll_target_rad  = pos_control->get_roll_rad();
//         pitch_target_rad = pos_control->get_pitch_rad();
        
//         break;
//     }

//     const float yaw_rate_target_rads = 0.0f;

//     attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_rad(
//         roll_target_rad,
//         pitch_target_rad,
//         yaw_rate_target_rads
//     );

//     pos_control->update_U_controller();
// }


#include "Copter.h"

/*
 * ModeHanding: simple vertical takeoff/hold mode.
 * Pilot commands climb or descent with throttle stick; controller integrates
 * the requested climb rate into a vertical position target and holds it.
 * Horizontal position (X/Y) is held using NE position controller; yaw is held.
 */

// maximum allowed altitude above home in meters
static constexpr float HANDING_ALT_MAX_M = 150.0f;

bool ModeHanding::init(bool ignore_checks)
{
    if (!ignore_checks && !motors->armed()) {
        return false;
    }

    // initialise vertical controller if needed
    if (!pos_control->is_active_U()) {
        pos_control->init_U_controller();
    }

    // initialise horizontal controller to a stopping point at current position
    pos_control->init_NE_controller_stopping_point();
    pos_control->relax_velocity_controller_NE();

    // hold current altitude on entry
    pos_control->set_pos_target_U_from_climb_rate_m(0.0f);

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_U_m(-get_pilot_speed_dn_ms(),
                                         get_pilot_speed_up_ms(),
                                         get_pilot_accel_U_mss());
    pos_control->set_correction_speed_accel_U_m(-get_pilot_speed_dn_ms(),
                                                get_pilot_speed_up_ms(),
                                                get_pilot_accel_U_mss());

    // lock current yaw
    attitude_control->reset_yaw_target_and_rate();

    return true;
}

void ModeHanding::run()
{
    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_U_m(-get_pilot_speed_dn_ms(),
                                         get_pilot_speed_up_ms(),
                                         get_pilot_accel_U_mss());

    // либо удерживаем пилотскую команду, либо по RC3>=1500 ведём в потолок HANDING_ALT_MAX_M
    float target_climb_rate_ms = 0.0f;

    const bool takeoff_command = rc().has_valid_input() &&
                                 (copter.channel_throttle != nullptr) &&
                                 (copter.channel_throttle->get_radio_in() >= 1500);

    if (takeoff_command) {
        // формируем профиль скорости до заданной высоты
        const float current_alt_m = copter.current_loc.alt * 0.01f;
        const float remaining_to_ceiling_m = HANDING_ALT_MAX_M - current_alt_m;

        if (remaining_to_ceiling_m > 0.0f) {
            target_climb_rate_ms = sqrt_controller(remaining_to_ceiling_m,
                                                   pos_control->get_pos_U_p().kP(),
                                                   pos_control->get_max_accel_U_mss(),
                                                   G_Dt);
        }
    } else {
        // pilot desired climb rate (m/s)
        target_climb_rate_ms = get_pilot_desired_climb_rate_ms();
    }

    target_climb_rate_ms = constrain_float(target_climb_rate_ms,
                                           -get_pilot_speed_dn_ms(),
                                            get_pilot_speed_up_ms());

    // determine state using shared altitude-hold logic
    HandingModeState handing_state = get_handing_state_U_ms(target_climb_rate_ms);

    // slow down as we approach the hard altitude ceiling to avoid overshoot
    if ((handing_state == HandingModeState::Takeoff || handing_state == HandingModeState::Flying) &&
        !is_negative(target_climb_rate_ms)) {
        const float current_alt_m = copter.current_loc.alt * 0.01f;
        const float target_alt_m = pos_control->get_pos_target_U_m();
        const float limited_current_target_m = MAX(current_alt_m, target_alt_m);
        const float remaining_to_max_m = HANDING_ALT_MAX_M - limited_current_target_m;

        if (remaining_to_max_m <= 0.0f) {
            target_climb_rate_ms = 0.0f;
        } else {
            const float slowdown_rate_ms = sqrt_controller(remaining_to_max_m,
                                                           pos_control->get_pos_U_p().kP(),
                                                           pos_control->get_max_accel_U_mss(),
                                                           G_Dt);
            target_climb_rate_ms = MIN(target_climb_rate_ms, slowdown_rate_ms);
        }
    }

    // roll/pitch targets будем брать из NE-контроллера, по умолчанию 0
    float roll_target_rad  = 0.0f;
    float pitch_target_rad = 0.0f;

    switch (handing_state) {

    case HandingModeState::MotorStopped:
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->reset_yaw_target_and_rate(false);
        pos_control->relax_U_controller(0.0f);
        pos_control->relax_velocity_controller_NE();
        break;

    case HandingModeState::Landed_Ground_Idle:
        attitude_control->reset_yaw_target_and_rate();
        FALLTHROUGH;

    case HandingModeState::Landed_Pre_Takeoff:
        attitude_control->reset_rate_controller_I_terms_smoothly();
        pos_control->relax_U_controller(0.0f);
        // pos_control->relax_velocity_controller_NE();
        break;

    case HandingModeState::Takeoff:
        if (!takeoff.running()) {
            takeoff.start_m(constrain_float(g.pilot_takeoff_alt_cm * 0.01f, 0.0f, 10.0f));
        }

        target_climb_rate_ms = get_avoidance_adjusted_climbrate_ms(target_climb_rate_ms);
        takeoff.do_pilot_takeoff_ms(target_climb_rate_ms);

        // по X/Y — удержание позиции
        pos_control->update_NE_controller();
        roll_target_rad  = pos_control->get_roll_rad();
        pitch_target_rad = pos_control->get_pitch_rad();
        break;

    case HandingModeState::Flying:
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        target_climb_rate_ms = get_avoidance_adjusted_climbrate_ms(target_climb_rate_ms);

#if AP_RANGEFINDER_ENABLED
        copter.surface_tracking.update_surface_offset();
#endif
        pos_control->set_pos_target_U_from_climb_rate_m(target_climb_rate_ms);

        // по X/Y — держим текущую цель, считаем компенсацию наклона
        pos_control->update_NE_controller();
        roll_target_rad  = pos_control->get_roll_rad();
        pitch_target_rad = pos_control->get_pitch_rad();
        break;
    }

    const float yaw_rate_target_rads = 0.0f;

    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_rad(
        roll_target_rad,
        pitch_target_rad,
        yaw_rate_target_rads
    );

    pos_control->update_U_controller();
}
