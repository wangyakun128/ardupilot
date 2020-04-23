#include "Copter.h"

/*
 * Init and run calls for guided flight mode
 */

// guided_init - initialise guided controller
bool Copter::ModeRoute::init(bool ignore_checks)
{

    if (copter.position_ok() || ignore_checks) {

        mode = Route_ROUTE;
        // initialise yaw
        auto_yaw.set_mode_to_default(false);

        pathNum = 0;
        creat_path();

        // start in position control mode
        pos_control_start();
        return true;
    }else{
        return false;
    }
}

void Copter::ModeRoute::creat_path()
{
    float radius_cm = 1000.0;

    wp_nav->get_wp_stopping_point(path[0]);

    path[1] = path[0] + Vector3f(1.0f,0,0)*radius_cm;
    path[2] = path[0] + Vector3f(-cosf(radians(36.0f)),-sinf(radians(36.0f)),0)*radius_cm;
    path[3] = path[0] + Vector3f(sinf(radians(18.0f)),cosf(radians(18.0f)),0)*radius_cm;
    path[4] = path[0] + Vector3f(sinf(radians(18.0f)),-cosf(radians(18.0f)),0)*radius_cm;
    path[5] = path[0] + Vector3f(-cosf(radians(36.0f)),sinf(radians(36.0f)),0)*radius_cm;
    path[6] = path[1];

}

// initialise guided mode's position controller//Î»ÖÃ¿ØÖÆ
void Copter::ModeRoute::pos_control_start()
{

    // initialise waypoint and spline controller
    wp_nav->wp_and_spline_init();

    // initialise wpnav to stopping point
    Vector3f stopping_point;
    wp_nav->get_wp_stopping_point(stopping_point);

    // no need to check return status because terrain data is not used
    wp_nav->set_wp_destination(path[0], false);

    // initialise yaw
    auto_yaw.set_mode_to_default(false);
}

void Copter::ModeRoute::go_run()
{
    if(pathNum<=6){
        if(wp_nav->reached_wp_destination()){
            pathNum++;
            wp_nav->set_wp_destination(path[pathNum],false);
        }
    }else{
        rtl_start();
    }
    pos_control_run();
}

// auto_rtl_start - initialises RTL in AUTO flight mode
void Copter::ModeRoute::rtl_start()
{
    mode = Route_RTL;

    // call regular rtl flight mode initialisation and ask it to ignore checks
    copter.mode_rtl.init(true);
}

void Copter::ModeRoute::rtl_run()
{
    // call regular rtl flight mode run function
    copter.mode_rtl.run(false);
}

// guided_run - runs the guided controller
// should be called at 100hz or more
void Copter::ModeRoute::run()
{
    switch (mode) {

    case Route_ROUTE:
        go_run();
        break;

    case Route_RTL:
        rtl_run();
        break;
    }
 }

// guided_pos_control_run - runs the guided position controller
// called from guided_run
void Copter::ModeRoute::pos_control_run()
{
    // if not auto armed or motors not enabled set throttle to zero and exit immediately
    if (!motors->armed() || !ap.auto_armed || !motors->get_interlock() || ap.land_complete) {
        zero_throttle_and_relax_ac();
        return;
    }

    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!copter.failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
        if (!is_zero(target_yaw_rate)) {
            auto_yaw.set_mode(AUTO_YAW_HOLD);
        }
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // run waypoint controller
    copter.failsafe_terrain_set_status(wp_nav->update_wpnav());

    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control->update_z_controller();

    // call attitude controller
    if (auto_yaw.mode() == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), target_yaw_rate);
    } else if (auto_yaw.mode() == AUTO_YAW_RATE) {
        // roll & pitch from waypoint controller, yaw rate from mavlink command or mission item
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), auto_yaw.rate_cds());
    } else {
        // roll, pitch from waypoint controller, yaw heading from GCS or auto_heading()
        attitude_control->input_euler_angle_roll_pitch_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), auto_yaw.yaw(), true);
    }
}
