#include "OpticalFlow.h"
#include "AP_OpticalFlow_Onboard.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo OpticalFlow::var_info[] = {
    // @Param: _ENABLE
    // @DisplayName: Optical flow enable/disable
    // @Description: Setting this to Enabled(1) will enable optical flow. Setting this to Disabled(0) will disable optical flow
    // @Values: 0:Disabled, 1:Enabled
    // @User: Standard
    AP_GROUPINFO("_ENABLE", 0,  OpticalFlow,    _enabled,   0),

    // @Param: _FXSCALER
    // @DisplayName: X axis optical flow scale factor correction
    // @Description: This sets the parts per thousand scale factor correction applied to the flow sensor X axis optical rate. It can be used to correct for variations in effective focal length. Each positive increment of 1 increases the scale factor applied to the X axis optical flow reading by 0.1%. Negative values reduce the scale factor.
    // @Range: -200 +200
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_FXSCALER", 1,  OpticalFlow,    _flowScalerX,   0),

    // @Param: _FYSCALER
    // @DisplayName: Y axis optical flow scale factor correction
    // @Description: This sets the parts per thousand scale factor correction applied to the flow sensor Y axis optical rate. It can be used to correct for variations in effective focal length. Each positive increment of 1 increases the scale factor applied to the Y axis optical flow reading by 0.1%. Negative values reduce the scale factor.
    // @Range: -200 +200
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_FYSCALER", 2,  OpticalFlow,    _flowScalerY,   0),

    // @Param: _ORIENT_YAW
    // @DisplayName: Flow sensor yaw alignment
    // @Description: Specifies the number of centi-degrees that the flow sensor is yawed relative to the vehicle. A sensor with its X-axis pointing to the right of the vehicle X axis has a positive yaw angle.
    // @Range: -18000 +18000
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_ORIENT_YAW", 3,  OpticalFlow,    _yawAngle_cd,   0),

    // @Param: _ENABLE
    // @DisplayName: Optical flow override
    // @Description: Setting this to Enabled(1) will enable optical flow override. Setting this to Disabled(0) will disable optical flow
    // @Values: 0:Disabled, 1:Override
    // @User: Standard
    AP_GROUPINFO("_OVR", 4,  OpticalFlow,    _ovr,   0),

    // @Param: _ENABLE
    // @DisplayName: Optical flow altidude alpha ascent
    // @Description: Controls the filtering of distance rangefinder for optical flow in the ascending direction
    // @Range: 0 +1
    // @Increment: 0.0001
    // @User: Standard
    AP_GROUPINFO("_ALPHA_A", 5,  OpticalFlow,    _asc_alpha,   0),

    // @Param: _ENABLE
    // @DisplayName: Optical flow altidude alpha descent
    // @Description: Controls the filtering of distance rangefinder for optical flow in the descending direction
    // @Range: 0 +1
    // @Increment: 0.0001
    // @User: Standard
    AP_GROUPINFO("_ALPHA_D", 6,  OpticalFlow,    _des_alpha,   0),

    // @Param: _ENABLE
    // @DisplayName: Optical flow altidude alpha ascent
    // @Description: Controls the filtering of distance rangefinder for optical flow in the ascending direction
    // @Range: 0 +4000
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_SLOW_DN", 7,  OpticalFlow,    _slow_dist_cm,   0),

    // @Param: _ENABLE
    // @DisplayName: Optical flow altidude alpha descent
    // @Description: Controls the filtering of distance rangefinder for optical flow in the descending direction
    // @Range: 0 +4000
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_OBS_OFF", 8,  OpticalFlow,    _obs_offset_cm,   0),

    // @Param: _ENABLE
    // @DisplayName: Optical flow altidude alpha ascent
    // @Description: Controls the filtering of distance rangefinder for optical flow in the ascending direction
    // @Range: 0 +1
    // @Increment: 0.0001
    // @User: Standard
    AP_GROUPINFO("_ALPHA_A2", 9,  OpticalFlow,    _asc_alpha2,   0),

    // @Param: _ENABLE
    // @DisplayName: Optical flow altidude alpha descent
    // @Description: Controls the filtering of distance rangefinder for optical flow in the descending direction
    // @Range: 0 +1
    // @Increment: 0.0001
    // @User: Standard
    AP_GROUPINFO("_ALPHA_D2", 10,  OpticalFlow,    _des_alpha2,   0),


    // @Param: _ENABLE
    // @DisplayName: Optical flow altidude alpha descent
    // @Description: Controls the filtering of distance rangefinder for optical flow in the descending direction
    // @Range: 0 +4000
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_DEADBND", 11,  OpticalFlow,    _deadbnd,   0),

// @Param: _ENABLE
    // @DisplayName: Optical flow altidude alpha descent
    // @Description: Controls the filtering of distance rangefinder for optical flow in the descending direction
    // @Range: 0 +1
    // @Increment: 0.0001
    // @User: Standard
    AP_GROUPINFO("_VEL_ALPH", 12,  OpticalFlow,    _opt_vel_alpha,   0),
    AP_GROUPINFO("_n11", 13,  OpticalFlow,    _n11,   1.0),
    AP_GROUPINFO("_n12", 14,  OpticalFlow,    _n12,   0.0),
    AP_GROUPINFO("_n21", 15,  OpticalFlow,    _n21,   0.0),
    AP_GROUPINFO("_n22", 16,  OpticalFlow,    _n22,   1.0),
    AP_GROUPINFO("_offx", 17,  OpticalFlow,    _offset_x,   0.0),
    AP_GROUPINFO("_offy", 18,  OpticalFlow,    _offset_y,   0.0),
    AP_GROUPINFO("_offa", 19,  OpticalFlow,    _offset_a,   0.0),
    AP_GROUPINFO("_offb", 20,  OpticalFlow,    _offset_b,   0.0),
    AP_GROUPINFO("_offc", 21,  OpticalFlow,    _offset_c,   0.0),
    AP_GROUPINFO("_en_ac", 22,  OpticalFlow,    _en_ac,   0.0),
    AP_GROUPINFO("_maxvel", 23,  OpticalFlow,    _velmax,   1600.0),
    AP_GROUPINFO("_maxdist", 24,  OpticalFlow,    _dist,   5000.0),
    AP_GROUPINFO("_focal", 25,  OpticalFlow,    _focal,   1.0),
    AP_GROUPINFO("_gyrodb", 26,  OpticalFlow,    _gdb,   0.0),
    AP_GROUPINFO("_flowdb", 27,  OpticalFlow,    _fdb,   0.0),
    AP_GROUPINFO("_ovrhead", 28,  OpticalFlow,    _overhead,   0.0),
    AP_GROUPINFO("_sonarhz", 29,  OpticalFlow,    _sonarhz,   72.0),
    AP_GROUPINFO("_barohz", 30,  OpticalFlow,    _barohz,   72.0),
    AP_GROUPINFO("_maxzvel", 31,  OpticalFlow,    _maxzvel,   0.0),
    AP_GROUPINFO("_hyz", 32,  OpticalFlow,    _hyz,   0.0),

    AP_GROUPEND
};

// default constructor
OpticalFlow::OpticalFlow(AP_AHRS_NavEKF &ahrs)
    : _last_update_ms(0)
{
    AP_Param::setup_object_defaults(this, var_info);

    memset(&_state, 0, sizeof(_state));

    // healthy flag will be overwritten on update
    _flags.healthy = false;

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BEBOP ||\
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_MINLURE ||\
    CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BBBMINI
    backend = new AP_OpticalFlow_Onboard(*this, ahrs);
#endif
}

void OpticalFlow::init(void)
{
    if (!backend) {
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
        backend = new AP_OpticalFlow_PX4(*this);
#elif CONFIG_HAL_BOARD == HAL_BOARD_SITL
        backend = new AP_OpticalFlow_HIL(*this);
#elif CONFIG_HAL_BOARD == HAL_BOARD_LINUX
        backend = new AP_OpticalFlow_Linux(*this, hal.i2c_mgr->get_device(HAL_OPTFLOW_PX4FLOW_I2C_BUS, HAL_OPTFLOW_PX4FLOW_I2C_ADDRESS));
#endif
    }

    if (backend != NULL) {
        backend->init();
    } else {
        _enabled = 0;
	_ovr =0;
    }
}

void OpticalFlow::update(void)
{
    if (backend != NULL) {
        backend->update();
    }
    // only healthy if the data is less than 0.5s old
    _flags.healthy = (AP_HAL::millis() - _last_update_ms < 500);
}

void OpticalFlow::setHIL(const struct OpticalFlow::OpticalFlow_state &state)
{
    if (backend) {
        backend->_update_frontend(state);
    }
}
