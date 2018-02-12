#include "Copter.h"

void Copter::init_barometer(bool full_calibration)
{
    gcs_send_text(MAV_SEVERITY_INFO, "Calibrating barometer");
    if (full_calibration) {
        barometer.calibrate();
    }else{
        barometer.update_calibration();
    }
    gcs_send_text(MAV_SEVERITY_INFO, "Barometer calibration complete");
}

// return barometric altitude in centimeters
void Copter::read_barometer(void)
{
    barometer.update();
    if (should_log(MASK_LOG_IMU)) {
        Log_Write_Baro();
    }
    baro_alt = barometer.get_altitude() * 100.0f;
    baro_climbrate = barometer.get_climb_rate() * 100.0f;

    motors.set_air_density_ratio(barometer.get_air_density_ratio());
}

void Copter::init_rangefinder(void)
{
#if RANGEFINDER_ENABLED == ENABLED
   rangefinder.init();
   //rangefinder_state.alt_cm_filt.set_cutoff_frequency(RANGEFINDER_WPNAV_FILT_HZ);
	rangefinder_state.alt_cm_filt.set_cutoff_frequency(0.0);
   rangefinder_state.enabled = (rangefinder.num_sensors() >= 1);
#endif
}

// return rangefinder altitude in centimeters
void Copter::read_rangefinder(void)
{
#if RANGEFINDER_ENABLED == ENABLED
    rangefinder.update();

    rangefinder_state.alt_healthy = ((rangefinder.status() == RangeFinder::RangeFinder_Good) && (rangefinder.range_valid_count() >= RANGEFINDER_HEALTH_MAX));

    int16_t temp_alt = rangefinder.distance_cm()+rangefinder.offset1();

 #if RANGEFINDER_TILT_CORRECTION == ENABLED
    // correct alt for angle of the rangefinder
    temp_alt = (float)temp_alt * MAX(0.707f, ahrs.get_rotation_body_to_ned().c.z);
 #endif

    
    //float z_vel=inertial_nav.get_velocity_z(); //For Camera Down
    float z_vel=-1.0*inertial_nav.get_velocity_z(); //For Camera Up
    float M11 = rangefinder.m11();
    float M12 = rangefinder.m12();
    float M21 = rangefinder.m21();
    float M22 = rangefinder.m22();

    rangefinder_state.alt_cm = rangefinder_state.alt_cm+0.005*range_vel+M11*(temp_alt-rangefinder_state.alt_cm)+M12*(z_vel-range_vel);
    range_vel = range_vel+M21*(temp_alt-rangefinder_state.alt_cm)+M22*(z_vel-range_vel);

    // filter rangefinder for use by AC_WPNav
    uint32_t now = AP_HAL::millis();
    range_dt=(float)now-range_ms;
    range_ms=(float)now;

    if (rangefinder_state.alt_healthy) {
        if (now - rangefinder_state.last_healthy_ms > RANGEFINDER_TIMEOUT_MS) {
            // reset filter if we haven't used it within the last second
            //rangefinder_state.alt_cm_filt.reset(rangefinder_state.alt_cm);
        } else {
            //rangefinder_state.alt_cm_filt.apply(rangefinder_state.alt_cm, 0.05f);
        }
        rangefinder_state.last_healthy_ms = now;
    }

    // send rangefinder altitude and health to waypoint navigation library
    //wp_nav.set_rangefinder_alt(rangefinder_state.enabled, rangefinder_state.alt_healthy, rangefinder_state.alt_cm_filt.get());
    wp_nav.set_rangefinder_alt(rangefinder_state.enabled, rangefinder_state.alt_healthy, rangefinder_state.alt_cm);

#else
    rangefinder_state.enabled = false;
    rangefinder_state.alt_healthy = false;
    rangefinder_state.alt_cm = 0;
#endif
}

// return true if rangefinder_alt can be used
bool Copter::rangefinder_alt_ok()
{
    return (rangefinder_state.enabled && rangefinder_state.alt_healthy);
}

/*
  update RPM sensors
 */
void Copter::rpm_update(void)
{
    rpm_sensor.update();
    if (rpm_sensor.enabled(0) || rpm_sensor.enabled(1)) {
        if (should_log(MASK_LOG_RCIN)) {
            DataFlash.Log_Write_RPM(rpm_sensor);
        }
    }
}

// initialise compass
void Copter::init_compass()
{
    if (!compass.init() || !compass.read()) {
        // make sure we don't pass a broken compass to DCM
        cliSerial->println("COMPASS INIT ERROR");
        Log_Write_Error(ERROR_SUBSYSTEM_COMPASS,ERROR_CODE_FAILED_TO_INITIALISE);
        return;
    }
    ahrs.set_compass(&compass);
}

// initialise optical flow sensor
void Copter::init_optflow()
{
#if OPTFLOW == ENABLED
    // exit immediately if not enabled
    if (!optflow.enabled() && !optflow.ovr()) {
        return;
    }

    // initialise optical flow sensor
    optflow.init();
#endif      // OPTFLOW == ENABLED
}

// called at 200hz
#if OPTFLOW == ENABLED
void Copter::update_optical_flow(void)
{
    static uint32_t last_of_update = 0;

    // exit immediately if not enabled
    if (!optflow.enabled() && !optflow.ovr()) {
        return;
    }

    // read from sensor
    optflow.update();

    // write to log and send to EKF if new data has arrived
    if (optflow.last_update() != last_of_update) {
        last_of_update = optflow.last_update();
        uint8_t flowQuality = optflow.quality();
        Vector2f flowRate = optflow.flowRate();
        Vector2f bodyRate = optflow.bodyRate();
	Vector3f gyro=ahrs.get_gyro();
///////////////////////////////////////////////////////////////////////////////////////
//////// For Optical Flow Navigation //////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
	Vector2f of_ned_cms;
	Vector2f of_bf_cms;
//For Camera Facing down and FLOW_ORIENT_YAW = 90 degrees
/*
	of_ned_cms.x = ((flowRate.y-bodyRate.y)*rangefinder.distance_cm())*ahrs.cos_yaw()-(-1.0*(flowRate.x-bodyRate.x)*rangefinder.distance_cm())*ahrs.sin_yaw();
	of_ned_cms.y = ((flowRate.y-bodyRate.y)*rangefinder.distance_cm())*ahrs.sin_yaw()+(-1.0*(flowRate.x-bodyRate.x)*rangefinder.distance_cm())*ahrs.cos_yaw();
*/
//For camera facing up and FLOW_ORIENT_YAW = -90 degrees
/*
	of_ned_cms.x = ((flowRate.y-bodyRate.y)*rangefinder.distance_cm())*ahrs.cos_yaw()-((flowRate.x-bodyRate.x)*rangefinder.distance_cm())*ahrs.sin_yaw();
	of_ned_cms.y = ((flowRate.y-bodyRate.y)*rangefinder.distance_cm())*ahrs.sin_yaw()+((flowRate.x-bodyRate.x)*rangefinder.distance_cm())*ahrs.cos_yaw();
*/
//Camera Facing Ceiling
	//Rotates Coordinate System about x-axis 180 degrees
	flowRate.y=flowRate.y*(-1.0);
	bodyRate.y=bodyRate.y*(-1.0);

	if((flowRate.y <= optflow.flowdb()) && (flowRate.y >= -1.0*optflow.flowdb())) {flowRate.y=0.0;}
	if((flowRate.x <= optflow.flowdb()) && (flowRate.x >= -1.0*optflow.flowdb())) {flowRate.x=0.0;}
	if((bodyRate.y <= optflow.gyrodb()) && (bodyRate.y >= -1.0*optflow.gyrodb())) {bodyRate.y=0.0;}
	if((bodyRate.x <= optflow.gyrodb()) && (bodyRate.x >= -1.0*optflow.gyrodb())) {bodyRate.x=0.0;}
	
        /*
	float rngof = ((float)rangefinder.distance_cm() - 15.0)* MAX(0.707f, ahrs.get_rotation_body_to_ned().c.z);
	float rangefd= rngof;
	//For Overhead Obstacle
	float temp_obs_height = rngof+inertial_nav.get_altitude();
	//For Standard Setup
	//float temp_obs_height = inertial_nav.get_altitude() - rngof;
	float tmp_hgt;
	if(rngof >= 4000.0) {filt_obs_height = temp_obs_height;}
	else if(temp_obs_height >= filt_obs_height) {
		tmp_hgt = filt_obs_height*(1.0-optflow.alpha_a())+temp_obs_height*optflow.alpha_a();
		if((temp_obs_height-filt_obs_height) >= optflow.rng_deadband()){
			filt_obs_height = filt_obs_height*(1.0-optflow.alpha_a2())+temp_obs_height*optflow.alpha_a2();
		}
		else { filt_obs_height = tmp_hgt; }
	}
	else {
		tmp_hgt = filt_obs_height*(1.0-optflow.alpha_d())+temp_obs_height*optflow.alpha_d();
		if((filt_obs_height-temp_obs_height) >= optflow.rng_deadband()){
			filt_obs_height = filt_obs_height*(1.0-optflow.alpha_d2())+temp_obs_height*optflow.alpha_d2();
		}
		else { filt_obs_height = tmp_hgt; }
	}
	if(filt_obs_height <= 0.0) {filt_obs_height = 0.0;}
	
	//For Overhead Obstruction	
	rngof=filt_obs_height-inertial_nav.get_altitude();
	//For Off the ground Distance
	//rngof=inertial_nav.get_altitude()-filt_obs_height;	
	*/
	float rngof = rangefinder_state.alt_cm;
	float opt_dist = rngof/(ahrs.cos_roll()*ahrs.cos_pitch());
	//float opt_dist = rngof;
	//float opt_dist = (float)rangefinder.distance_cm();
	//float optical_height_x = 0.733*rngof/(0.35836795*(1.0-2.0*ahrs.sin_pitch()*ahrs.sin_pitch())+0.36651914);
	//float optical_height_y = 0.733*rngof/(0.35836795*(1.0-2.0*ahrs.sin_roll()*ahrs.sin_roll())+0.36651914);
	//Yaw Rotation about Z-axis
	of_bf_cms.x=-1.0*(flowRate.y*optflow.focal()-bodyRate.y-optflow.offsety())*opt_dist;
	of_bf_cms.y=(flowRate.x*optflow.focal()-bodyRate.x-optflow.offsetx())*opt_dist; //For_Camera_Down
	//of_bf_cms.y=-1.0*(flowRate.x-bodyRate.x)*opt_dist; //For Camera Up

	//For Attitude Compensation
	if((ahrs.cos_pitch() != 0.0) && (ahrs.cos_roll() != 0.0) && (optflow.en_ac() >= 0.0)) {
	of_bf_cms.x=of_bf_cms.x/(ahrs.cos_pitch());
	of_bf_cms.y=of_bf_cms.y/(ahrs.cos_roll())-ahrs.sin_roll()*ahrs.sin_pitch()*of_bf_cms.x/ahrs.cos_roll();
	}
	
	//For Coreolis Effect
	float a=optflow.offseta();
	float b=optflow.offsetb();
	float c=optflow.offsetc();
	float corx = (a*gyro.y-b*gyro.x)*(ahrs.sin_roll()*ahrs.sin_yaw()+ahrs.cos_roll()*ahrs.cos_yaw()*ahrs.sin_pitch())+(a*gyro.z-c*gyro.x)*(ahrs.cos_roll()*ahrs.sin_yaw()-ahrs.cos_yaw()*ahrs.sin_roll()*ahrs.sin_pitch())+(b*gyro.z-c*gyro.y)*ahrs.cos_yaw()*ahrs.cos_pitch();
	float cory = (b*gyro.z-c*gyro.y)*ahrs.cos_pitch()*ahrs.sin_yaw()-(a*gyro.z-c*gyro.x)*(ahrs.cos_roll()*ahrs.cos_yaw()+ahrs.sin_roll()*ahrs.sin_yaw()*ahrs.sin_pitch())-(a*gyro.y-b*gyro.x)*(ahrs.cos_yaw()*ahrs.sin_roll()-ahrs.cos_roll()*ahrs.sin_yaw()*ahrs.sin_pitch());


	//NED Conversion
	of_ned_cms.x=of_bf_cms.x*ahrs.cos_yaw()-of_bf_cms.y*ahrs.sin_yaw();
	of_ned_cms.y=of_bf_cms.y*ahrs.cos_yaw()+of_bf_cms.x*ahrs.sin_yaw();

	//Add Coreolis Effect
	of_ned_cms.x = of_ned_cms.x + corx;
	of_ned_cms.y = of_ned_cms.y + cory;

	//of_ned_cms.x = -1.0*(((flowRate.y-bodyRate.y)*ahrs.cos_yaw()+(flowRate.x-bodyRate.x)*ahrs.sin_yaw())*opt_dist);
	//of_ned_cms.y = -1.0*((flowRate.y-bodyRate.y)*ahrs.sin_yaw()-(flowRate.x-bodyRate.x)*ahrs.cos_yaw())*opt_dist;
	//of_ned_cms.x = -1.0*(((flowRate.y-bodyRate.y)*optical_height_x*ahrs.cos_yaw()+(flowRate.x-bodyRate.x)*optical_height_y*ahrs.sin_yaw()));
	//of_ned_cms.y = -1.0*((flowRate.y-bodyRate.y)*optical_height_x*ahrs.sin_yaw()-(flowRate.x-bodyRate.x)*optical_height_y*ahrs.cos_yaw());

	if(rngof > 0.0){
		NavVelGainScaler=400.0/MAX(rngof,400.0);
	}
	else {NavVelGainScaler = 1.0;}

	float tmo = AP_HAL::micros64();
///////////////////////////////////////////////////////////////////////////////////////
	if (optflow.enabled()) {
        	ahrs.writeOptFlowMeas(flowQuality, flowRate, bodyRate, last_of_update);
	}
///////////////////////////////////////////////////////////////////////////////////////
//////// For Optical Flow Navigation //////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
	if (optflow.enabled() || optflow.ovr()) {
		ahrs.update_flow_nav(of_ned_cms, (float)rangefinder.distance_cm(), tmo, 1.0, optflow.n11(), optflow.n12(), optflow.n21(), optflow.n22(),optflow.maxvel(),optflow.dist());
	}
///////////////////////////////////////////////////////////////////////////////////////
        if ((g.log_bitmask & MASK_LOG_OPTFLOW) || optflow.ovr()) {
            Log_Write_Optflow();
	    Log_Write_Proximity();
        }
    }
}
#endif  // OPTFLOW == ENABLED

// read_battery - check battery voltage and current and invoke failsafe if necessary
// called at 10hz
void Copter::read_battery(void)
{
    battery.read();

    // update compass with current value
    if (battery.has_current()) {
        compass.set_current(battery.current_amps());
    }

    // update motors with voltage and current
    if (battery.get_type() != AP_BattMonitor::BattMonitor_TYPE_NONE) {
        motors.set_voltage(battery.voltage());
    }
    if (battery.has_current()) {
        motors.set_current(battery.current_amps());
    }

    // check for low voltage or current if the low voltage check hasn't already been triggered
    // we only check when we're not powered by USB to avoid false alarms during bench tests
    if (!ap.usb_connected && !failsafe.battery && battery.exhausted(g.fs_batt_voltage, g.fs_batt_mah)) {
        failsafe_battery_event();
    }

    // log battery info to the dataflash
    if (should_log(MASK_LOG_CURRENT)) {
        Log_Write_Current();
    }
}

// read the receiver RSSI as an 8 bit number for MAVLink
// RC_CHANNELS_SCALED message
void Copter::read_receiver_rssi(void)
{
    receiver_rssi = rssi.read_receiver_rssi_uint8();
}

void Copter::compass_cal_update()
{
    static uint32_t compass_cal_stick_gesture_begin = 0;

    if (!hal.util->get_soft_armed()) {
        compass.compass_cal_update();
    }

    if (compass.is_calibrating()) {
        if (channel_yaw->get_control_in() < -4000 && channel_throttle->get_control_in() > 900) {
            compass.cancel_calibration_all();
        }
    } else {
        bool stick_gesture_detected = compass_cal_stick_gesture_begin != 0 && !motors.armed() && channel_yaw->get_control_in() > 4000 && channel_throttle->get_control_in() > 900;
        uint32_t tnow = millis();

        if (!stick_gesture_detected) {
            compass_cal_stick_gesture_begin = tnow;
        } else if (tnow-compass_cal_stick_gesture_begin > 1000*COMPASS_CAL_STICK_GESTURE_TIME) {
#ifdef CAL_ALWAYS_REBOOT
            compass.start_calibration_all(true,true,COMPASS_CAL_STICK_DELAY,true);
#else
            compass.start_calibration_all(true,true,COMPASS_CAL_STICK_DELAY,false);
#endif
        }
    }
}

void Copter::accel_cal_update()
{
    if (hal.util->get_soft_armed()) {
        return;
    }
    ins.acal_update();
    // check if new trim values, and set them
    float trim_roll, trim_pitch;
    if(ins.get_new_trim(trim_roll, trim_pitch)) {
        ahrs.set_trim(Vector3f(trim_roll, trim_pitch, 0));
    }

#ifdef CAL_ALWAYS_REBOOT
    if (ins.accel_cal_requires_reboot()) {
        hal.scheduler->delay(1000);
        hal.scheduler->reboot(false);
    }
#endif
}

#if EPM_ENABLED == ENABLED
// epm update - moves epm pwm output back to neutral after grab or release is completed
void Copter::epm_update()
{
    epm.update();
}
#endif

/*
  update AP_Button
 */
void Copter::button_update(void)
{
    g2.button.update();
}

// initialise proximity sensor
void Copter::init_proximity(void)
{
#if PROXIMITY_ENABLED == ENABLED
    g2.proximity.init();
#endif
}

// update proximity sensor
void Copter::update_proximity(void)
{
#if PROXIMITY_ENABLED == ENABLED
    g2.proximity.update();
#endif
}

