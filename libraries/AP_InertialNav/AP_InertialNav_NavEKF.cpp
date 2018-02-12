#include <AP_HAL/AP_HAL.h>
#include "AP_InertialNav.h"

#if AP_AHRS_NAVEKF_AVAILABLE

/*
  A wrapper around the AP_InertialNav class which uses the NavEKF
  filter if available, and falls back to the AP_InertialNav filter
  when EKF is not available
 */

/**
   update internal state
*/
void AP_InertialNav_NavEKF::update(float dt)
{
    // get the NE position relative to the local earth frame origin
    Vector2f posNE;
    if (_ahrs_ekf.get_relative_position_NE(posNE)) {
        _relpos_cm.x = posNE.x * 100; // convert from m to cm
        _relpos_cm.y = posNE.y * 100; // convert from m to cm
    }

// get the velocity relative to the local earth frame
    Vector3f velNED;
    if (_ahrs_ekf.get_velocity_NED(velNED)) {
        _velocity_cm = velNED * 100; // convert to cm/s
        _velocity_cm.z = -_velocity_cm.z; // convert from NED to NEU
    }
///////////////////////////////////////////////////////////////////////////////////
/// Added for OF NAV //////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////
/*
    // get the NE position relative to the local earth frame origin
    Vector2f ofposNE;
    if (_ahrs_ekf.get_of_relative_position_NE(ofposNE)) {
        _of_relpos_cm.x = ofposNE.x;
        _of_relpos_cm.y = ofposNE.y;
    }
*/
    

    float dist;
    float m11=0.0049;//0.0021;
    float m12=0.0024;//0.0025;
    float m13=0.0005;//0.0010;
    float m21=0.0089;//0.0021;
    float m22=0.1056;//0.2;
    float m23=0.0009;//0.0010;
    if (_ahrs_ekf.get_range_distance(dist)) {
	//_standoff_distance_cm_dot = (dist-_standoff_distance_cm)/dt;
	_standoff_distance_cm = dist;

	if((_standoff_distance_cm <= 400.0) && (_standoff_distance_cm >= 10.0)) {
		if(_in_range == false) {
			_of_ovrhng = _relpos_cm.z + _standoff_distance_cm;
		}
		_in_range = true;
		//_offset_z = _relpos_cm.z - _of_ovrhng + _standoff_distance_cm;
		//_offset_z_dot = _velocity_cm.z + _standoff_distance_cm_dot;
		_of_z=_of_z+0.0025*_of_z_dot+(_of_ovrhng-_standoff_distance_cm-_of_z)*m11+m12*(_velocity_cm.z-_of_z_dot)+m13*(_relpos_cm.z-_of_z);
		_of_z_dot=_of_z_dot+(_of_ovrhng-_standoff_distance_cm-_of_z)*m21+m22*(_velocity_cm.z-_of_z_dot)+m23*(_relpos_cm.z-_of_z);
	}
	else {
		if((_of_z-_relpos_cm.z)*(_of_z-_relpos_cm.z) >= 400.0) {
			_of_z=_relpos_cm.z*0.1+_of_z*0.9;
		}
		else{
		_of_z=_relpos_cm.z;
		}
		_of_z_dot=_velocity_cm.z;
	}


	if (_standoff_distance_cm >=  _ahrs_ekf.get_maxdist() || (_of_velocity_cm.x*_of_velocity_cm.x+_of_velocity_cm.y*_of_velocity_cm.y) >= (_ahrs_ekf.get_maxvel() * _ahrs_ekf.get_maxvel())) {
		if (_optical_flow_nav) {
			_switch = true;
			_nav_adjust.x =+ (_relpos_cm.x - _of_relpos_cm.x);
			_nav_adjust.y =+ (_relpos_cm.y - _of_relpos_cm.y);
			_of_relpos_cm.x = _relpos_cm.x;
			_of_relpos_cm.y = _relpos_cm.y;
			_of_velocity_cm.x = _velocity_cm.x; // 
        		_of_velocity_cm.y = _velocity_cm.y;
			_sys_update = AP_HAL::millis();
		}
		_optical_flow_nav = false;
		_of_relpos_cm.x = _relpos_cm.x;
		_of_relpos_cm.y = _relpos_cm.y;
		_of_velocity_cm.x = _velocity_cm.x; // 
        	_of_velocity_cm.y = _velocity_cm.y;
	}
	else {
		float n11=_ahrs_ekf.get_n11();
		float n12=_ahrs_ekf.get_n12();
		float n21=_ahrs_ekf.get_n21();
		float n22=_ahrs_ekf.get_n22();
		Vector2f ofvelNED;
		_ahrs_ekf.get_of_velocity_NED(ofvelNED);
		const Vector3f &accelef=_ahrs_ekf.get_accel_ef_blended();
		Vector3f efaccel;
		efaccel.x=accelef.x*100.0;
		efaccel.y=accelef.y*100.0;
		efaccel.z=accelef.z*100.0;
		float prevvx=_of_velocity_cm.x;
		float prevvy=_of_velocity_cm.y;
		//_of_velocity_cm.x=_of_velocity_cm.x+_of_accel_cmss.x*dt+n11*(ofvelNED.x-_of_velocity_cm.x)+n12*(efaccel.x-_of_accel_cmss.x);
		//_of_velocity_cm.y=_of_velocity_cm.y+_of_accel_cmss.y*dt+n11*(ofvelNED.y-_of_velocity_cm.y)+n12*(efaccel.y-_of_accel_cmss.y);
		//_of_accel_cmss.x=_of_accel_cmss.x+n21*(ofvelNED.x-prevvx)+n22*(efaccel.x-_of_accel_cmss.x);
		//_of_accel_cmss.y=_of_accel_cmss.y+n21*(ofvelNED.y-prevvy)+n22*(efaccel.y-_of_accel_cmss.y);
		_of_velocity_cm.x=_of_velocity_cm.x+n11*(ofvelNED.x-_of_velocity_cm.x)+efaccel.x*(1.0-n22);
		_of_velocity_cm.y=_of_velocity_cm.y+n11*(ofvelNED.y-_of_velocity_cm.y)+efaccel.y*(1.0-n22);
		/*
    		if (_ahrs_ekf.get_of_velocity_NED(ofvelNED)) {
       			 _of_velocity_cm.x = ofvelNED.x; // 
       			 _of_velocity_cm.y = ofvelNED.y; // 
   		 }
		*/
		_optical_flow_nav = true;
		_of_relpos_cm.x += _of_velocity_cm.x * dt;//+_of_accel_cmss.x*0.5*dt*dt;
 		_of_relpos_cm.y += _of_velocity_cm.y * dt;//+_of_accel_cmss.y*0.5*dt*dt;
	}
    }
///////////////////////////////////////////////////////////////////////////////////

    // get the D position relative to the local earth frame origin
    float posD;
    if (_ahrs_ekf.get_relative_position_D(posD)) {
        _relpos_cm.z = - posD * 100; // convert from m in NED to cm in NEU
    }

    // get the absolute WGS-84 position
    _haveabspos = _ahrs_ekf.get_position(_abspos);

    

    // Get a derivative of the vertical position which is kinematically consistent with the vertical position is required by some control loops.
    // This is different to the vertical velocity from the EKF which is not always consistent with the verical position due to the various errors that are being corrected for.
    if (_ahrs_ekf.get_vert_pos_rate(_pos_z_rate)) {
        _pos_z_rate *= 100; // convert to cm/s
        _pos_z_rate = - _pos_z_rate; // InertialNav is NEU
    }
}

/**
 * get_filter_status : returns filter status as a series of flags
 */
nav_filter_status AP_InertialNav_NavEKF::get_filter_status() const
{
    nav_filter_status status;
    _ahrs_ekf.get_filter_status(status);
    return status;
}

/**
 * get_origin - returns the inertial navigation origin in lat/lon/alt
 */
struct Location AP_InertialNav_NavEKF::get_origin() const
{
    struct Location ret;
     if (!_ahrs_ekf.get_origin(ret)) {
         // initialise location to all zeros if EKF1 origin not yet set
         memset(&ret, 0, sizeof(ret));
     }
    return ret;
}

/**
 * get_position - returns the current position relative to the home location in cm.
 *
 * @return
 */
const Vector3f &AP_InertialNav_NavEKF::get_position(void) const 
{
    return _relpos_cm;
}

///////////////////////////////////////////////////////////////////////////////////
/// Added for OF NAV //////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////

/**
 * get_position - returns the current position relative to the home location in cm.
 *
 * @return
 */
const Vector2f &AP_InertialNav_NavEKF::get_of_position(void) const 
{
    return _of_relpos_cm;
}

/**
 * get_velocity - returns the current velocity in cm/s
 *
 * @return velocity vector:
 *      		.x : latitude  velocity in cm/s
 * 				.y : longitude velocity in cm/s
 * 				.z : vertical  velocity in cm/s
 */
const Vector2f &AP_InertialNav_NavEKF::get_of_velocity(void) const
{
    return _of_velocity_cm;
}


uint32_t AP_InertialNav_NavEKF::nav_switch(void) const
{
	return _sys_update;
}

const Vector2f &AP_InertialNav_NavEKF::get_switch(void) const
{
	
	return _nav_adjust;	
}

bool AP_InertialNav_NavEKF::flownav(void) const
{
	return _optical_flow_nav;
}
///////////////////////////////////////////////////////////////////////////////////

/**
 * get_location - updates the provided location with the latest calculated location
 *  returns true on success (i.e. the EKF knows it's latest position), false on failure
 */
bool AP_InertialNav_NavEKF::get_location(struct Location &loc) const
{
    return _ahrs_ekf.get_location(loc);
}

/**
 * get_latitude - returns the latitude of the current position estimation in 100 nano degrees (i.e. degree value multiplied by 10,000,000)
 */
int32_t AP_InertialNav_NavEKF::get_latitude() const
{
    return _abspos.lat;
}

/**
 * get_longitude - returns the longitude of the current position estimation in 100 nano degrees (i.e. degree value multiplied by 10,000,000)
 * @return
 */
int32_t AP_InertialNav_NavEKF::get_longitude() const
{
    return _abspos.lng;
}

/**
 * get_velocity - returns the current velocity in cm/s
 *
 * @return velocity vector:
 *      		.x : latitude  velocity in cm/s
 * 				.y : longitude velocity in cm/s
 * 				.z : vertical  velocity in cm/s
 */
const Vector3f &AP_InertialNav_NavEKF::get_velocity() const
{
    return _velocity_cm;
}

float AP_InertialNav_NavEKF::get_of_velocity_z() const
{
    return _of_z_dot;
}

float AP_InertialNav_NavEKF::get_of_altitude() const
{
    return _of_z;
}

/**
 * get_velocity_xy - returns the current horizontal velocity in cm/s
 *
 * @returns the current horizontal velocity in cm/s
 */
float AP_InertialNav_NavEKF::get_velocity_xy() const
{
    return norm(_velocity_cm.x, _velocity_cm.y);
}

/**
 * get_pos_z_derivative - returns the derivative of the z position in cm/s
*/
float AP_InertialNav_NavEKF::get_pos_z_derivative() const
{
    return _pos_z_rate;
}

/**
 * get_altitude - get latest altitude estimate in cm
 * @return
 */
float AP_InertialNav_NavEKF::get_altitude() const
{
    return _relpos_cm.z;
}

/**
 * getHgtAboveGnd - get latest height above ground level estimate in cm and a validity flag
 *
 * @return
 */
bool AP_InertialNav_NavEKF::get_hagl(float &height) const
{
    // true when estimate is valid
    bool valid = _ahrs_ekf.get_hagl(height);
    // convert height from m to cm
    height *= 100.0f;
    return valid;
}

/**
 * get_hgt_ctrl_limit - get maximum height to be observed by the control loops in cm and a validity flag
 * this is used to limit height during optical flow navigation
 * it will return invalid when no limiting is required
 * @return
 */
bool AP_InertialNav_NavEKF::get_hgt_ctrl_limit(float& limit) const
{
    // true when estimate is valid
    if (_ahrs_ekf.get_hgt_ctrl_limit(limit)) {
        // convert height from m to cm
        limit *= 100.0f;
        return true;
    }
    return false;
}

/**
 * get_velocity_z - returns the current climbrate.
 *
 * @see get_velocity().z
 *
 * @return climbrate in cm/s
 */
float AP_InertialNav_NavEKF::get_velocity_z() const
{
    return _velocity_cm.z;
}

#endif // AP_AHRS_NAVEKF_AVAILABLE
