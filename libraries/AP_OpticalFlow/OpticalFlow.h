/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

/*
 *       OpticalFlow.h - OpticalFlow Base Class for Ardupilot
 *       Code by Randy Mackay. DIYDrones.com
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

class OpticalFlow_backend;
class AP_AHRS_NavEKF;

class OpticalFlow
{
    friend class OpticalFlow_backend;

public:
    // constructor
    OpticalFlow(AP_AHRS_NavEKF& ahrs);

    // init - initialise sensor
    void init(void);

    // enabled - returns true if optical flow is enabled
    bool enabled() const { return _enabled; }

    float n11() const { return _n11; }
    float n12() const { return _n12; }
    float n21() const { return _n21; }
    float n22() const { return _n22; }
    float maxvel() const { return _velmax; }
    float dist() const { return _dist; }
    float offsetx() const { return _offset_x; }
    float offsety() const { return _offset_y; }
    float offseta() const { return _offset_a; }
    float offsetb() const { return _offset_b; }
    float offsetc() const { return _offset_c; }
    float focal() const { return _focal;}
    float gyrodb() const { return _gdb;}
    float flowdb() const { return _fdb;}
    float ovrhead() const { return _overhead;}
    float sonarhz() const { return _sonarhz;}
    float barohz() const { return _barohz;}
    float maxzvel() const { return _maxzvel;}
    float hyz() const { return _hyz;}

    //override - use optical flow, but not for EKF
    bool ovr() const { return _ovr; }

    //return alpha for ascending
    float alpha_a() const {return _asc_alpha; }

    //return alpha for descending
    float alpha_d() const {return _des_alpha; }

    //return alpha for ascending
    float alpha_a2() const {return _asc_alpha2; }

    //return alpha for descending
    float alpha_d2() const {return _des_alpha2; }

    float vel_alpha() const {return _opt_vel_alpha; }

    float en_ac() const {return _en_ac; }


    // healthy - return true if the sensor is healthy
    bool healthy() const { return backend != NULL && _flags.healthy; }

    // read latest values from sensor and fill in x,y and totals.
    void update(void);

    // quality - returns the surface quality as a measure from 0 ~ 255
    uint8_t quality() const { return _state.surface_quality; }

    // raw - returns the raw movement from the sensor
    const Vector2f& flowRate() const { return _state.flowRate; }

    // velocity - returns the velocity in m/s
    const Vector2f& bodyRate() const { return _state.bodyRate; }

    //flowrate x
    float flowRatex() const { return _state.flowRate.x; }

    //flowrate y
    float flowRatey() const { return _state.flowRate.y; }

    //bodyrate x
    float bodyRatex() const { return _state.bodyRate.x; }

    //bodyrate y
    float bodyRatey() const { return _state.bodyRate.y; }

    //added
    float get_slow_down_cm() const { return _slow_dist_cm; }
    float get_obstacle_offset_cm() const { return _obs_offset_cm; }
    float rng_deadband() const { return _deadbnd; }


    // device_id - returns device id
    uint8_t device_id() const { return _state.device_id; }

    // last_update() - returns system time of last sensor update
    uint32_t last_update() const { return _last_update_ms; }

    // parameter var info table
    static const struct AP_Param::GroupInfo var_info[];

    struct OpticalFlow_state {
        uint8_t device_id;          // device id
        uint8_t  surface_quality;   // image quality (below TBD you can't trust the dx,dy values returned)
        Vector2f flowRate;          // optical flow angular rate in rad/sec measured about the X and Y body axis. A RH rotation about a sensor axis produces a positive rate.
        Vector2f bodyRate;          // body inertial angular rate in rad/sec measured about the X and Y body axis. A RH rotation about a sensor axis produces a positive rate.
    };

    // support for HIL/SITL
    void setHIL(const struct OpticalFlow_state &state);

private:
    OpticalFlow_backend *backend;

    struct AP_OpticalFlow_Flags {
        uint8_t healthy     : 1;    // true if sensor is healthy
    } _flags;

    // parameters
    AP_Int8  _enabled;              // enabled/disabled flag
    AP_Int16 _flowScalerX;          // X axis flow scale factor correction - parts per thousand
    AP_Int16 _flowScalerY;          // Y axis flow scale factor correction - parts per thousand
    AP_Int16 _yawAngle_cd;          // yaw angle of sensor X axis with respect to vehicle X axis - centi degrees
    AP_Int8 _ovr;		    //enable/disable override flag
    AP_Float _asc_alpha;	    //alpha for ascending altitude filter
    AP_Float _des_alpha;	    //alpha for descending altitude filter
    AP_Float _asc_alpha2;	    //alpha for ascending altitude filter
    AP_Float _des_alpha2;	    //alpha for descending altitude filter
    AP_Float _deadbnd;	            //deadband crossover for altitude filter
    AP_Float _slow_dist_cm;
    AP_Float _obs_offset_cm;
    AP_Float _opt_vel_alpha;
    AP_Float _n11;
    AP_Float _n12;
    AP_Float _n21;
    AP_Float _n22;
    AP_Float _dist;
    AP_Float _velmax;
    AP_Float _offset_x;
    AP_Float _offset_y;
    AP_Float _offset_a;
    AP_Float _offset_b;
    AP_Float _offset_c;
    AP_Float _en_ac;
    AP_Float _focal;
    AP_Float _gdb;
    AP_Float _fdb;
    AP_Float _overhead;
    AP_Float _sonarhz;
    AP_Float _barohz;
    AP_Float _maxzvel;
    AP_Float _hyz;
    // state filled in by backend
    struct OpticalFlow_state _state;

    uint32_t _last_update_ms;        // millis() time of last update
};

#include "OpticalFlow_backend.h"
#include "AP_OpticalFlow_HIL.h"
#include "AP_OpticalFlow_PX4.h"
#include "AP_OpticalFlow_Linux.h"
