#include "Copter.h"

/*
 * Init and run calls for circle flight mode
 */

// circle_init - initialise circle controller flight mode
bool Copter::circle_init(bool ignore_checks)
{
#if FRAME_CONFIG == HELI_FRAME
    // do not allow helis to enter Loiter if the Rotor Runup is not complete
    if (!ignore_checks && !motors.rotor_runup_complete()){
	pole_detected = false;
	pole_ascend = false;
	pole_descend = false;
	pole_rotate = false;
	pole_end = false;
	pole_rotate_counter = 0;

        return false;
    }
#endif

    if (position_ok() || ignore_checks) {

	circle_pilot_yaw_override = false;
        // set target to current position
        wp_nav.init_loiter_target();

        // initialize vertical speed and acceleration
        pos_control.set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
        pos_control.set_accel_z(g.pilot_accel_z);

        // initialise position and desired velocity
        if (!pos_control.is_active_z()) {
            pos_control.set_alt_target_to_current_alt();
            pos_control.set_desired_velocity_z(inertial_nav.get_velocity_z());
        }

	//Initialize Circle_Nav_Target
	circle_nav.init();
	pole_detected = false;
	pole_ascend = false;
	pole_descend = false;
	pole_rotate = false;
	pole_end = false;
	pole_rotate_counter = 0;

        return true;
    }else{
	pole_detected = false;
	pole_ascend = false;
	pole_descend = false;
	pole_rotate = false;
	pole_end = false;
	pole_rotate_counter = 0;
        return false;
    }
}

// circle_run - runs the circle flight mode
// should be called at 100hz or more
void Copter::circle_run()
{
    
    LoiterModeState loiter_state;
    float target_yaw_rate = 0.0;
    float target_climb_rate = 0.0;
    float takeoff_climb_rate = 0.0;
    float roll_percent = 0.0;
    float pitch_percent =0.0;
    float input_angle_rate=0.0;
    float input_radius_change=0.0;
    float max_angle_rate=1.5; //rad/s
    float max_radius_change=500.0; //cm/s
    float yaw_cmd=0.0;
    float pole_distance=0.0;
    float pole_min_offset=circle_nav.min_off();
    float pole_climb_rate=circle_nav.clmb_rate();
    float pole_descent_rate=circle_nav.descent_rate();
    float turn_velocity=circle_nav.turn_vel();
    float pole_height=circle_nav.pole_height();
    float obs_dist_fwd=(float)rangefinder.distance_cm(1);
    float obs_dist_blw=(float)rangefinder.distance_cm(2);
    float obstacle_dist_off = circle_nav.obs();
    float obstacle_back_rate = circle_nav.obsrt();

    // initialize speeds and accelerations
    pos_control.set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    pos_control.set_accel_z(g.pilot_accel_z);
    
    // process pilot inputs
    if (!failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
        if (!is_zero(target_yaw_rate)) {
            circle_pilot_yaw_override = true;
        }
	
	get_pilot_desired_lean_percent(channel_roll->get_control_in(), channel_pitch->get_control_in(), roll_percent, pitch_percent);
	input_angle_rate=-1.0*roll_percent*max_angle_rate;
	input_radius_change=pitch_percent*max_radius_change;

        // get pilot desired climb rate
        target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
	target_climb_rate = constrain_float(target_climb_rate, -g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    }
    else {
	wp_nav.clear_pilot_desired_acceleration();
    }

    if (ap.land_complete_maybe) {
	wp_nav.loiter_soften_for_landing();
    }
    bool takeoff_triggered = ap.land_complete && (target_climb_rate > 0.0f);

    // Loiter State Machine Determination
    if (!motors.armed() || !motors.get_interlock()) {
    	loiter_state = Loiter_MotorStopped;
    } else if (takeoff_state.running || takeoff_triggered) {
	loiter_state = Loiter_Takeoff;
    } else if (!ap.auto_armed || ap.land_complete) {
	loiter_state = Loiter_Landed;
    } else {
	loiter_state = Loiter_Flying;
    }

    // Loiter State Machine
    switch (loiter_state) {

    case Loiter_MotorStopped:

	motors.set_desired_spool_state(AP_Motors::DESIRED_SHUT_DOWN);
	wp_nav.init_loiter_target();
	circle_nav.init();
	attitude_control.reset_rate_controller_I_terms();
	attitude_control.set_yaw_target_to_current_heading();
	pos_control.relax_alt_hold_controllers(0.0f); // Forces throttle output to go to zero
	circle_nav.update(0.0f,0.0f);
	attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), target_yaw_rate, get_smoothing_gain());
	pos_control.update_z_controller();
	pole_detected = true;
	break;

    case Loiter_Takeoff:
        // set motors to full range
        motors.set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

        // initiate take-off
        if (!takeoff_state.running) {
            takeoff_timer_start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
            // indicate we are taking off
            set_land_complete(false);
            // clear i term when we're taking off
            set_throttle_takeoff();
        }

        // get takeoff adjusted pilot and takeoff climb rates
        takeoff_get_climb_rates(target_climb_rate, takeoff_climb_rate);

        // run loiter controller
        circle_nav.update(input_angle_rate,input_radius_change);

        // call attitude controller
        attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), target_yaw_rate, get_smoothing_gain());

        // update altitude target and call position controller
        pos_control.set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        pos_control.add_takeoff_climb_rate(takeoff_climb_rate, G_Dt);
        pos_control.update_z_controller();
	pole_detected = true;
        break;

    case Loiter_Landed:
        // set motors to spin-when-armed if throttle below deadzone, otherwise full range (but motors will only spin at min throttle)
        if (target_climb_rate < 0.0f) {
            motors.set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        } else {
            motors.set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
        }
        wp_nav.init_loiter_target();
	circle_nav.init();
        attitude_control.reset_rate_controller_I_terms();
        attitude_control.set_yaw_target_to_current_heading();
        attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(0, 0, 0, get_smoothing_gain());
        pos_control.relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
        pos_control.update_z_controller();
	pole_detected = true;
        break;

    case Loiter_Flying:

        // set motors to full range
        motors.set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

        // run circle controller
        if(pole_rotate == false) {circle_nav.update(input_angle_rate,input_radius_change);}

	
	if(!pole_detected) {
		pole_detected = pole_detection(G_Dt, 15.0, 90.0);
		yaw_cmd = circle_nav.get_yaw() + heading_add;
		if(pole_detected) {
			circle_nav.set_center_heading(heading_pole,min_distance);
			pole_ascend = true;
			altitude_start=inertial_nav.get_altitude();
			pole_leash_altitude = pole_height+altitude_start;
		}
	}
	else if (circle_nav.automatic() <= 0.0) {
		yaw_cmd = circle_nav.get_yaw();
	}
	else if (pole_rotate == true) {
		if (obs_dist_fwd <= obstacle_dist_off) {circle_nav.change_radius(obstacle_back_rate*G_Dt);}
		circle_nav.update((turn_velocity+turn_velocity*-2.0*roll_percent),0.0);
		yaw_cmd = circle_nav.get_yaw() + heading_add;
		if (inertial_nav.get_altitude() <= altitude_start) {
			pole_detected = false;
				pole_ascend = false;
				pole_descend = false;
				pole_rotate = false;
				pole_end = false;
				pole_rotate_counter = 0;
		}
		else {
			target_climb_rate -= pole_descent_rate+pole_descent_rate*-2.0*roll_percent;
			yaw_cmd = circle_nav.get_yaw() + heading_add;
		}
	}
	else if (pole_ascend == true) {
		if(pole_rotate_counter == 0) {
			if (pole_detection(G_Dt, 4.0, 30.0)) {
				pole_distance=min_distance;
				if (pole_distance <= pole_min_offset) {circle_nav.change_radius(pole_min_offset-pole_distance);}
				if (pole_distance >= 2000.0) {pole_leash_altitude = inertial_nav.get_altitude();}
			}
			else if (obs_dist_fwd <= obstacle_dist_off) {circle_nav.change_radius(obstacle_back_rate*G_Dt);}
		}
		else {
			heading_add = 0.0;
			if (obs_dist_fwd <= obstacle_dist_off) {circle_nav.change_radius(obstacle_back_rate*G_Dt);}
		}
		if (inertial_nav.get_altitude() >= pole_leash_altitude) {
			pole_ascend = false;
			pole_rotate = true;
			pole_descend = true;
			pole_rotate_counter+=1;
			if(pole_rotate_counter == 3) {pole_end = true;}
		}
		else {
			target_climb_rate += pole_climb_rate;
			yaw_cmd = circle_nav.get_yaw() + heading_add;
		}
	}
	else {yaw_cmd = circle_nav.get_yaw();}

        // call attitude controller
        attitude_control.input_euler_angle_roll_pitch_yaw(circle_nav.get_roll(), circle_nav.get_pitch(), yaw_cmd, true, get_smoothing_gain());
	
    	// update altitude target and call position controller
    	pos_control.set_alt_target_from_climb_rate(target_climb_rate, G_Dt, false);
    	pos_control.update_z_controller();
	break;
    }
}
