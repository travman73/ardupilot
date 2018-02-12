#include "Copter.h"

// get_smoothing_gain - returns smoothing gain to be passed into attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw
//      result is a number from 2 to 12 with 2 being very sluggish and 12 being very crisp
float Copter::get_smoothing_gain()
{
    return (2.0f + (float)g.rc_feel_rp/10.0f);
}

// get_pilot_desired_angle - transform pilot's roll or pitch input into a desired lean angle
// returns desired angle in centi-degrees
void Copter::get_pilot_desired_lean_angles(float roll_in, float pitch_in, float &roll_out, float &pitch_out, float angle_max)
{
    // sanity check angle max parameter
    aparm.angle_max = constrain_int16(aparm.angle_max,1000,8000);

    // limit max lean angle
    angle_max = constrain_float(angle_max, 1000, aparm.angle_max);

    // scale roll_in, pitch_in to ANGLE_MAX parameter range
    float scaler = aparm.angle_max/(float)ROLL_PITCH_YAW_INPUT_MAX;
    roll_in *= scaler;
    pitch_in *= scaler;

    // do circular limit
    float total_in = norm(pitch_in, roll_in);
    if (total_in > angle_max) {
        float ratio = angle_max / total_in;
        roll_in *= ratio;
        pitch_in *= ratio;
    }

    // do lateral tilt to euler roll conversion
    roll_in = (18000/M_PI) * atanf(cosf(pitch_in*(M_PI/18000))*tanf(roll_in*(M_PI/18000)));

    // return
    roll_out = roll_in;
    pitch_out = pitch_in;
}

// get_pilot_desired_desired_lean_percent - transform pilot's roll or pitch input into a desired percentage
// returns desired percent (i.e. 1 = 100%)
void Copter::get_pilot_desired_lean_percent(float roll_in, float pitch_in, float &roll_out, float &pitch_out)
{
    // scale roll_in, pitch_in to ANGLE_MAX parameter range
    float scaler = 1.0/(float)ROLL_PITCH_YAW_INPUT_MAX;
    roll_in *= scaler;
    pitch_in *= scaler;

    // do circular limit
    float total_in = norm(pitch_in, roll_in);
    if (total_in > 1.0) {
        float ratio = 1.0 / total_in;
        roll_in *= ratio;
        pitch_in *= ratio;
    }

    // return
    roll_out = roll_in;
    pitch_out = pitch_in;
}


// get_pilot_desired_heading - transform pilot's yaw input into a
// desired yaw rate
// returns desired yaw rate in centi-degrees per second
float Copter::get_pilot_desired_yaw_rate(int16_t stick_angle)
{
    float yaw_request;

    // calculate yaw rate request
    if (g2.acro_y_expo <= 0) {
        yaw_request = stick_angle * g.acro_yaw_p;
    } else {
        // expo variables
        float y_in, y_in3, y_out;

        // range check expo
        if (g2.acro_y_expo > 1.0f || g2.acro_y_expo < 0.5f) {
            g2.acro_y_expo = 1.0f;
        }

        // yaw expo
        y_in = float(stick_angle)/ROLL_PITCH_YAW_INPUT_MAX;
        y_in3 = y_in*y_in*y_in;
        y_out = (g2.acro_y_expo * y_in3) + ((1.0f - g2.acro_y_expo) * y_in);
        yaw_request = ROLL_PITCH_YAW_INPUT_MAX * y_out * g.acro_yaw_p;
    }
    // convert pilot input to the desired yaw rate
    return yaw_request;
}

/*************************************************************
 * yaw controllers
 *************************************************************/

// get_roi_yaw - returns heading towards location held in roi_WP
// should be called at 100hz
float Copter::get_roi_yaw()
{
    static uint8_t roi_yaw_counter = 0;     // used to reduce update rate to 100hz

    roi_yaw_counter++;
    if (roi_yaw_counter >= 4) {
        roi_yaw_counter = 0;
        yaw_look_at_WP_bearing = pv_get_bearing_cd(inertial_nav.get_position(), roi_WP);
    }

    return yaw_look_at_WP_bearing;
}

float Copter::get_look_ahead_yaw()
{
    const Vector3f& vel = inertial_nav.get_velocity();
    float speed = norm(vel.x,vel.y);
    // Commanded Yaw to automatically look ahead.
    if (position_ok() && (speed > YAW_LOOK_AHEAD_MIN_SPEED)) {
        yaw_look_ahead_bearing = degrees(atan2f(vel.y,vel.x))*100.0f;
    }
    return yaw_look_ahead_bearing;
}

/*************************************************************
 *  throttle control
 ****************************************************************/

// update estimated throttle required to hover (if necessary)
//  called at 100hz
void Copter::update_throttle_hover()
{
#if FRAME_CONFIG != HELI_FRAME
    // if not armed or landed exit
    if (!motors.armed() || ap.land_complete) {
        return;
    }

    // do not update in manual throttle modes or Drift
    if (mode_has_manual_throttle(control_mode) || (control_mode == DRIFT)) {
        return;
    }

    // do not update while climbing or descending
    if (!is_zero(pos_control.get_desired_velocity().z)) {
        return;
    }

    // get throttle output
    float throttle = motors.get_throttle();

    // calc average throttle if we are in a level hover
    if (throttle > 0.0f && abs(climb_rate) < 60 && labs(ahrs.roll_sensor) < 500 && labs(ahrs.pitch_sensor) < 500) {
        // Can we set the time constant automatically
        motors.update_throttle_hover(0.01f);
    }
#endif
}

// set_throttle_takeoff - allows parents to tell throttle controller we are taking off so I terms can be cleared
void Copter::set_throttle_takeoff()
{
    // tell position controller to reset alt target and reset I terms
    pos_control.init_takeoff();
}

// transform pilot's manual throttle input to make hover throttle mid stick
// used only for manual throttle modes
// thr_mid should be in the range 0 to 1
// returns throttle output 0 to 1
float Copter::get_pilot_desired_throttle(int16_t throttle_control, float thr_mid)
{
    if (thr_mid <= 0.0f) {
        thr_mid = motors.get_throttle_hover();
    }

    int16_t mid_stick = channel_throttle->get_control_mid();
    // protect against unlikely divide by zero
    if (mid_stick <= 0) {
        mid_stick = 500;
    }

    // ensure reasonable throttle values
    throttle_control = constrain_int16(throttle_control,0,1000);

    // calculate normalised throttle input
    float throttle_in;
    if (throttle_control < mid_stick) {
        // below the deadband
        throttle_in = ((float)throttle_control)*0.5f/(float)mid_stick;
    }else if(throttle_control > mid_stick) {
        // above the deadband
        throttle_in = 0.5f + ((float)(throttle_control-mid_stick)) * 0.5f / (float)(1000-mid_stick);
    }else{
        // must be in the deadband
        throttle_in = 0.5f;
    }

    float expo = constrain_float(-(thr_mid-0.5)/0.375, -0.5f, 1.0f);
    // calculate the output throttle using the given expo function
    float throttle_out = throttle_in*(1.0f-expo) + expo*throttle_in*throttle_in*throttle_in;
    return throttle_out;
}

// get_pilot_desired_climb_rate - transform pilot's throttle input to climb rate in cm/s
// without any deadzone at the bottom
float Copter::get_pilot_desired_climb_rate(float throttle_control)
{
    // throttle failsafe check
    if( failsafe.radio ) {
        return 0.0f;
    }

    float desired_rate = 0.0f;
    float mid_stick = channel_throttle->get_control_mid();
    float deadband_top = mid_stick + g.throttle_deadzone;
    float deadband_bottom = mid_stick - g.throttle_deadzone;

    // ensure a reasonable throttle value
    throttle_control = constrain_float(throttle_control,0.0f,1000.0f);

    // ensure a reasonable deadzone
    g.throttle_deadzone = constrain_int16(g.throttle_deadzone, 0, 400);

    // check throttle is above, below or in the deadband
    if (throttle_control < deadband_bottom) {
        // below the deadband
        desired_rate = g.pilot_velocity_z_max * (throttle_control-deadband_bottom) / deadband_bottom;
    }else if (throttle_control > deadband_top) {
        // above the deadband
        desired_rate = g.pilot_velocity_z_max * (throttle_control-deadband_top) / (1000.0f-deadband_top);
    }else{
        // must be in the deadband
        desired_rate = 0.0f;
    }

    // desired climb rate for logging
    desired_climb_rate = desired_rate;

    return desired_rate;
}

// get_non_takeoff_throttle - a throttle somewhere between min and mid throttle which should not lead to a takeoff
float Copter::get_non_takeoff_throttle()
{
    return MAX(0,motors.get_throttle_hover()/2.0f);
}

// get_surface_tracking_climb_rate - hold copter at the desired distance above the ground
//      returns climb rate (in cm/s) which should be passed to the position controller
float Copter::get_surface_tracking_climb_rate(int16_t target_rate, float current_alt_target, float dt)
{
#if RANGEFINDER_ENABLED == ENABLED
    static uint32_t last_call_ms = 0;
    float distance_error;
    float velocity_correction;
    float current_alt = inertial_nav.get_altitude();

    uint32_t now = millis();

    // reset target altitude if this controller has just been engaged
    if (now - last_call_ms > RANGEFINDER_TIMEOUT_MS) {
        target_rangefinder_alt = rangefinder_state.alt_cm + current_alt_target - current_alt;
    }
    last_call_ms = now;

    // adjust rangefinder target alt if motors have not hit their limits
    if ((target_rate<0 && !motors.limit.throttle_lower) || (target_rate>0 && !motors.limit.throttle_upper)) {
        target_rangefinder_alt += target_rate * dt;
    }

    // do not let target altitude get too far from current altitude above ground
    // Note: the 750cm limit is perhaps too wide but is consistent with the regular althold limits and helps ensure a smooth transition
    target_rangefinder_alt = constrain_float(target_rangefinder_alt,rangefinder_state.alt_cm-pos_control.get_leash_down_z(),rangefinder_state.alt_cm+pos_control.get_leash_up_z());

    // calc desired velocity correction from target rangefinder alt vs actual rangefinder alt (remove the error already passed to Altitude controller to avoid oscillations)
    distance_error = (target_rangefinder_alt - rangefinder_state.alt_cm) - (current_alt_target - current_alt);
    velocity_correction = distance_error * g.rangefinder_gain;
    velocity_correction = constrain_float(velocity_correction, -THR_SURFACE_TRACKING_VELZ_MAX, THR_SURFACE_TRACKING_VELZ_MAX);

    // return combined pilot climb rate + rate to correct rangefinder alt error
    return (target_rate + velocity_correction);
#else
    return (float)target_rate;
#endif
}

// upper_surface_tracking - offset copter at the desired distance below roof
//      returns climb rate (in cm/s) which should be passed to the position controller
//	also can modify desired altitude directly
float Copter::upper_surface_tracking(int16_t target_rate, float current_alt_target, float dt)
{

    float current_alt = inertial_nav.get_altitude();	//Altitude above ground from GPS/Barometer/Etc..
    float climb_rate_z = inertial_nav.get_velocity_z(); //Current_Climb_Rate
    float abords =1.0;					//Checks Rangefinder health (basic check if above minimum return value);
    float pos_crl_range_cm=optflow.get_slow_down_cm();	//Distance in addition to offset distance in which copter should begin to slow
							//dowm
    float offset_dist_cm=optflow.get_obstacle_offset_cm();	//Minimum clearance copter should maintain below overhead obstacle
    float max_crl_cms=40.0;				//Maximum climb rate when copter starts getting close to overhead obstacle
    float min_crl_cms=0.0;				//Maximum climb rate just before copter crosses the minimum clearance distance
    float temp_target_rate_cms=0.0;			//Buffer for comparing target_rate
    float temp_target_cm=0.0;				//Buffer for comparing current_alt_target
    float alt_correction_cm=0.0;			//Altitude Correction Desired

    //int16_t rang_alt = rangefinder.distance_cm();	//Get Overhead Rangefinder Distance
    if(rangefinder.distance_cm() <= 5) {abords=0.0;}			//See if Rangefinder is healthy (if not set aboards=0.0)
    float rang_alt = rangefinder.distance_cm();	//Compensate for copter tilt angle
    //if(overhead_obstacle_effort_cm < 100.0) {overhead_obstacle_effort_cm=overhead_obstacle_effort_cm+50.0*dt;}
    if((rang_alt < (offset_dist_cm + pos_crl_range_cm)) && (abords==1.0)) {	//If copter is within distance of overhead obstacle
	if(rang_alt > offset_dist_cm){						//If we haven't crossed the minimum overhead clearance
										//start scaling the maximum ascent distance
		temp_target_rate_cms=((rang_alt-offset_dist_cm)/pos_crl_range_cm*(max_crl_cms-min_crl_cms));
		if(temp_target_rate_cms < target_rate) {target_rate = temp_target_rate_cms;}
		//Added to reduce overshoot
		if((climb_rate_z >= 100.0) && ((current_alt+30.0) <= current_alt_target)) {pos_control.set_alt_target(current_alt);}
		if(target_rate >= 100.0) {target_rate = 100.0;}
	}
	else{
	/*
		if(target_rate > 0) {target_rate=0;}
		temp_target_cm=rang_alt+current_alt-offset_dist_cm;
		if(temp_target_cm < current_alt_target) {
			alt_correction_cm=current_alt_target-temp_target_cm;
			if(overhead_obstacle_effort_cm >= alt_correction_cm) {
				overhead_obstacle_effort_cm=overhead_obstacle_effort_cm-alt_correction_cm;
				pos_control.set_alt_target(temp_target_cm);
			}
			else{
				temp_target_cm=current_alt_target-overhead_obstacle_effort_cm;
				pos_control.set_alt_target(temp_target_cm);
				overhead_obstacle_effort_cm=0.0;
			}
		}
	}
	*/
	//Less aggressive Altitude Standoff Controller
		temp_target_cm=rang_alt+current_alt-offset_dist_cm;
		if(target_rate >= 0.0) {target_rate = 0.0;}	//cm/s
//Modified Stuff Here
/*
		if(temp_target_cm < current_alt_target) {
			if(target_rate >= -50.0) {target_rate = -50.0;} //cm/s
		}
*/
		if (dt > 0.0) {
		temp_target_rate_cms=(temp_target_cm-current_alt_target)/dt;
		}
		else {temp_target_rate_cms = -50.0;}
		if(temp_target_rate_cms <= -50.0) {temp_target_rate_cms = -50.0;}
		if(target_rate >= temp_target_rate_cms) {target_rate = temp_target_rate_cms;};
	}
	
     }

     temp_target_rate_cms=(rang_alt-offset_dist_cm)/dt;
     if((temp_target_rate_cms < target_rate) && (abords==1.0) && (temp_target_rate_cms >= 0.0)) {target_rate=temp_target_rate_cms;}
    return (target_rate);
}

// upper_surface_tracking_v2 - offset copter at the desired distance below roof
//      returns climb rate (in cm/s) which should be passed to the position controller
//	also can modify desired altitude directly
float Copter::upper_surface_tracking_v2(int16_t target_rate, float current_alt_target, float dt)
{

    float current_alt = inertial_nav.get_altitude();	//Altitude above ground from GPS/Barometer/Etc..
    float climb_rate_z = inertial_nav.get_velocity_z(); //Current_Climb_Rate
    float abords =1.0;					//Checks Rangefinder health (basic check if above minimum return value);
    float pos_crl_range_cm=optflow.get_slow_down_cm();	//Distance in addition to offset distance in which copter should begin to slow
    float deadband=optflow.rng_deadband();
							//dowm
    float offset_dist_cm=optflow.get_obstacle_offset_cm();	//Minimum clearance copter should maintain below overhead obstacle
    float max_crl_cms=40.0;				//Maximum climb rate when copter starts getting close to overhead obstacle
    float min_crl_cms=0.0;				//Maximum climb rate just before copter crosses the minimum clearance distance
    float temp_target_rate_cms=0.0;			//Buffer for comparing target_rate
    float temp_target_cm=0.0;				//Buffer for comparing current_alt_target
    float alt_correction_cm=0.0;			//Altitude Correction Desired
    float diff_hgt;
    float temp_obs_height;
    //int16_t rang_alt = rangefinder.distance_cm();	//Get Overhead Rangefinder Distance
    if(rangefinder_state.alt_cm <= 5) {abords=0.0;}			//See if Rangefinder is healthy (if not set aboards=0.0)
    float rang_alt = rangefinder_state.alt_cm;	//Compensate for copter tilt angle
    temp_obs_height = rang_alt +current_alt-offset_dist_cm;
    if (rang_alt >= 3000.0) {return (target_rate);}
    else if (abords == 0.0) {return (target_rate);}
    else if ( rang_alt >= (pos_crl_range_cm + offset_dist_cm)) {
	filt_obs_height = temp_obs_height;
	return (target_rate);
    }
    else if ( temp_obs_height >= filt_obs_height ) {
	if ((temp_obs_height - filt_obs_height) >= deadband) {
		filt_obs_height += 50.0*dt;
	}
	if (current_alt_target >= filt_obs_height) {
		if(target_rate >= -50.0) {target_rate = -50.0;}
	}
	if (target_rate >= 50.0) {target_rate = 50.0;}
	if (current_alt_target+target_rate*dt >= filt_obs_height) {
		if (target_rate >= 0.0) {target_rate = 0.0;}
	}
	return (target_rate);
	}
    else {
	filt_obs_height -=50.0*dt;
	if (target_rate >= -50.0) {target_rate = -50.0;}
	return (target_rate);
	}
}

// upper_surface_tracking - offset copter at the desired distance below roof
//      returns climb rate (in cm/s) which should be passed to the position controller
//	also can modify desired altitude directly
float Copter::upper_surface_tracking_v3(int16_t target_rate, float current_alt_target, float dt)
{

    float current_alt = inertial_nav.get_altitude();	//Altitude above ground from GPS/Barometer/Etc..
    float climb_rate_z = inertial_nav.get_velocity_z(); //Current_Climb_Rate
    float abords =1.0;					//Checks Rangefinder health (basic check if above minimum return value);
    float pos_crl_range_cm=optflow.get_slow_down_cm();	//Distance in addition to offset distance in which copter should begin to slow
							//dowm
    float offset_dist_cm=optflow.get_obstacle_offset_cm();	//Minimum clearance copter should maintain below overhead obstacle
    float max_crl_cms=40.0;				//Maximum climb rate when copter starts getting close to overhead obstacle
    float min_crl_cms=0.0;				//Maximum climb rate just before copter crosses the minimum clearance distance
    float temp_target_rate_cms=0.0;			//Buffer for comparing target_rate
    float temp_target_cm=0.0;				//Buffer for comparing current_alt_target
    float alt_correction_cm=0.0;			//Altitude Correction Desired
    filt_obs_height=rangefinder_state.alt_cm;
    //int16_t rang_alt = rangefinder.distance_cm();	//Get Overhead Rangefinder Distance
    if(rangefinder_state.alt_cm <= 5) {abords=0.0;}			//See if Rangefinder is healthy (if not set aboards=0.0)
    float rang_alt = rangefinder_state.alt_cm;	//Compensate for copter tilt angle
    //if(overhead_obstacle_effort_cm < 100.0) {overhead_obstacle_effort_cm=overhead_obstacle_effort_cm+50.0*dt;}
    if((rang_alt < (offset_dist_cm + pos_crl_range_cm)) && (abords==1.0)) {	//If copter is within distance of overhead obstacle
	if(rang_alt > offset_dist_cm){						//If we haven't crossed the minimum overhead clearance
										//start scaling the maximum ascent distance
		temp_target_rate_cms=((rang_alt-offset_dist_cm)/pos_crl_range_cm*(max_crl_cms-min_crl_cms));
		if(temp_target_rate_cms < target_rate) {target_rate = temp_target_rate_cms;}
		//Added to reduce overshoot
		if((climb_rate_z >= 100.0) && ((current_alt+30.0) <= current_alt_target)) {pos_control.set_alt_target(current_alt);}
		if(target_rate >= 100.0) {target_rate = 100.0;}
	}
	else{
	/*
		if(target_rate > 0) {target_rate=0;}
		temp_target_cm=rang_alt+current_alt-offset_dist_cm;
		if(temp_target_cm < current_alt_target) {
			alt_correction_cm=current_alt_target-temp_target_cm;
			if(overhead_obstacle_effort_cm >= alt_correction_cm) {
				overhead_obstacle_effort_cm=overhead_obstacle_effort_cm-alt_correction_cm;
				pos_control.set_alt_target(temp_target_cm);
			}
			else{
				temp_target_cm=current_alt_target-overhead_obstacle_effort_cm;
				pos_control.set_alt_target(temp_target_cm);
				overhead_obstacle_effort_cm=0.0;
			}
		}
	}
	*/
	//Less aggressive Altitude Standoff Controller
		temp_target_cm=rang_alt+current_alt-offset_dist_cm;
		if(target_rate >= 0.0) {target_rate = 0.0;}	//cm/s
//Modified Stuff Here
/*
		if(temp_target_cm < current_alt_target) {
			if(target_rate >= -50.0) {target_rate = -50.0;} //cm/s
		}
*/
		if (dt > 0.0) {
		temp_target_rate_cms=(temp_target_cm-current_alt_target)/dt;
		}
		else {temp_target_rate_cms = -50.0;}
		if(temp_target_rate_cms <= -50.0) {temp_target_rate_cms = -50.0;}
		if(target_rate >= temp_target_rate_cms) {target_rate = temp_target_rate_cms;};
	}
	
     }

     temp_target_rate_cms=(rang_alt-offset_dist_cm)/dt;
     if((temp_target_rate_cms < target_rate) && (abords==1.0) && (temp_target_rate_cms >= 0.0)) {target_rate=temp_target_rate_cms;}
    return (target_rate);
}

float Copter::horizontal_obstacle_avoid()
{
	int16_t dist = rangefinder.distance_cm();
	float absolute=100.0;
	float begin=400.0;
	float angle_add=0.0;
	float max_angle=2000.0;
	if (dist <= absolute) {angle_add=max_angle;}
	else if (dist <= begin) {angle_add=(begin-dist)/(begin-absolute)*max_angle;}
	return angle_add;
}

bool Copter::pole_detection(float dt, float sweep_time, float sweep_angle) {
	int16_t dist = rangefinder.distance_cm();
	sweep_time=sweep_time/1.82;
	sweep_angle=sweep_angle/2.0;
	if(yaw_time >= (sweep_time*1.82)) {
		heading_add = 0.0;
		yaw_time = 0.0;
		return true;
	}
/* old code
	else if(yaw_time == 0.0 ) {
		min_distance= dist;
		heading_pole=ahrs.yaw;
		yaw_time+=dt;
		heading_add = sweep_angle*50.0*sinf((1.0/sweep_time)*yaw_time*6.28);
		return false;
	}
	else {
		if (min_distance > dist) {
			min_distance = dist;
			heading_pole = ahrs.yaw;
		}
		yaw_time+=dt;
		heading_add = sweep_angle*50.0*sinf((1.0/sweep_time)*yaw_time*6.28);
		return false;
	}
*/
	else if(yaw_time == 0.0 ) {
		min_distance= dist;
		heading_pole=ahrs.yaw;
		yaw_time+=dt;
		heading_add = -50.0*sweep_angle+sweep_angle*50.0*cosf((1.0/sweep_time)*yaw_time*6.28);
		return false;
	}
	else {
		if (min_distance > dist) {
			min_distance = dist;
			heading_pole = ahrs.yaw;
		}
		yaw_time+=dt;
		if(yaw_time < (0.75*sweep_time)) {
			heading_add = -50.0*sweep_angle+sweep_angle*50.0*cosf((1.0/sweep_time)*yaw_time*6.28);
		}
		else if(yaw_time < (0.75*sweep_time+sweep_time/3.14)){
			heading_add += sweep_angle*50.0*6.28/sweep_time*dt;
		}
		else {
			heading_add = sweep_angle*50.0+sweep_angle*50.0*cosf((1.0/sweep_time)*(yaw_time-(sweep_time/3.14))*6.28);
		}
		return false;
	}
}

float Copter::pole_rotation_alg(int i, float radius, float angle, float vel)
{
	float angular_rate = 0.0;
	if (angle >= ((((float)i - 1.0) * (2.0*3.14/3.0))+(3.14/6.0))) {
		pole_rotate=false;
	}
	else {angular_rate = vel;}
	return angular_rate;
}

// overhead_surface_tracking_climb_rate - lower copter at the desired distance away from overhang
//      returns climb rate (in cm/s) which should be passed to the position controller
float Copter::overhead_surface_tracking_climb_rate(int16_t target_rate, float current_overhead_target_cm, float dt)
{
#if RANGEFINDER_ENABLED == ENABLED
    static uint32_t last_call_ms = 0;
    float velocity_correction;
    float abords =1.0;

    int16_t temp_alt = rangefinder.distance_cm();
    if(temp_alt <= 5) {abords=0.0;}
    temp_alt = (float)temp_alt * MAX(0.707f, ahrs.get_rotation_body_to_ned().c.z);

    temp_alt=temp_alt-current_overhead_target_cm;

    velocity_correction = temp_alt * g.rangefinder_gain * 10.0*abords;
    velocity_correction = constrain_float(velocity_correction, -150.0, 0.0);

    // return combined pilot climb rate + rate to correct rangefinder alt error
    return (target_rate + velocity_correction);
#else
    return (float)target_rate;
#endif
}

// set_accel_throttle_I_from_pilot_throttle - smoothes transition from pilot controlled throttle to autopilot throttle
void Copter::set_accel_throttle_I_from_pilot_throttle()
{
    // get last throttle input sent to attitude controller
    float pilot_throttle = constrain_float(attitude_control.get_throttle_in(), 0.0f, 1.0f);
    // shift difference between pilot's throttle and hover throttle into accelerometer I
    g.pid_accel_z.set_integrator((pilot_throttle-motors.get_throttle_hover()) * 1000.0f);
}

// updates position controller's maximum altitude using fence and EKF limits
void Copter::update_poscon_alt_max()
{
    float alt_limit_cm = 0.0f;  // interpreted as no limit if left as zero

#if AC_FENCE == ENABLED
    // set fence altitude limit in position controller
    if ((fence.get_enabled_fences() & AC_FENCE_TYPE_ALT_MAX) != 0) {
        alt_limit_cm = pv_alt_above_origin(fence.get_safe_alt()*100.0f);
    }
#endif

    // get alt limit from EKF (limited during optical flow flight)
    float ekf_limit_cm = 0.0f;
    if (inertial_nav.get_hgt_ctrl_limit(ekf_limit_cm)) {
        if ((alt_limit_cm <= 0.0f) || (ekf_limit_cm < alt_limit_cm)) {
            alt_limit_cm = ekf_limit_cm;
        }
    }

    // pass limit to pos controller
    pos_control.set_alt_max(alt_limit_cm);
}

// rotate vector from vehicle's perspective to North-East frame
void Copter::rotate_body_frame_to_NE(float &x, float &y)
{
    float ne_x = x*ahrs.cos_yaw() - y*ahrs.sin_yaw();
    float ne_y = x*ahrs.sin_yaw() + y*ahrs.cos_yaw();
    x = ne_x;
    y = ne_y;
}
