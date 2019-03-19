
#include <px4_config.h>
#include <px4_defines.h>
#include <px4_module_params.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <px4_module.h>
#include <drivers/drv_hrt.h>
#include <systemlib/hysteresis/hysteresis.h>
#include <commander/px4_custom_mode.h>

#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>
#include <matrix/matrix/math.hpp>
#include <float.h>
#include <mathlib/mathlib.h>
#include <systemlib/mavlink_log.h>
#include <controllib/blocks.hpp>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/optical_flow_tau_theta.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_status.h>

#include <lib/FlightTasks/FlightTasks.hpp>

#include "Utility/ControlMath.hpp"
#include <modules/mc_pos_control/PositionControl.hpp>

extern "C" __EXPORT int perching_control_main(int argc, char *argv[]);

class MulticopterPerchingControl : public ModuleBase<MulticopterPerchingControl>, public control::SuperBlock, public ModuleParams
{
public:
    MulticopterPerchingControl();
    ~MulticopterPerchingControl();
    /** @see ModuleBase */
    static int task_spawn(int argc, char *argv[]);

    /** @see ModuleBase */
    static MulticopterPerchingControl *instantiate(int argc, char *argv[]);

    /** @see ModuleBase */
    static int custom_command(int argc, char *argv[]);

    /** @see ModuleBase */
    static int print_usage(const char *reason = nullptr);

    /** @see ModuleBase::run() */
    void run() override;

    /** @see ModuleBase::print_status() */
    int print_status() override;

private:
    matrix::Vector3f _thr_sp{};

    optical_flow_tau_theta_s ofd;

    optical_flow_s of;

    distance_sensor_s dis;

    vehicle_attitude_s att;

    vehicle_attitude_setpoint_s _att_sp{};

    orb_advert_t    _att_sp_pub{nullptr};           /**< attitude setpoint publication */

    int optical_front_sub{-1};

    int optical_downward_sub{-1}; 

    int dis_z_sub{-1};

    int att_sub{-1};

    int _params_sub{-1};

    control::BlockDerivative _h_deriv;
    control::BlockDerivative _vel_z_deriv;
    control::BlockDerivative _xmotion_deriv;
    FlightTasks _flight_tasks;
    PositionControl _positioncontrol;
    vehicle_constraints_s constraints = _flight_tasks.getConstraints();
    _positioncontrol.updateConstraints(constraints);
    float output_h;//filtered h
    float previous_h; //for fi
    float sf = 0.7; //filter smooh factor
    float _thr_int_z, _thr_int_y;
    float roll;
    float pitch;
    float yaw;
    float yaw_sp;
    DEFINE_PARAMETERS(
        (ParamFloat<px4::params::PC_H) h_sp,
        (ParamFloat<px4::params::PC_Z_KP>) k_p,
        (ParamFloat<px4::params::PC_Z_VEL_KP>) k_vel_p,
        (ParamFloat<px4::params::PC_Z_VEL_KI>) k_vel_i,
        (ParamFloat<px4::params::PC_Z_VEL_KD>) k_vel_d,
        (ParamFloat<px4::params::PC_OFD_KP>) k_ofd_p,
        (ParamFloat<px4::params::PC_NP_KP>) k_m_p,
        (ParamFloat<px4::params::PC_NP_KI>) k_m_i,
        (ParamFloat<px4::params::PC_NP_KD>) k_m_d,
        (ParamFloat<px4::params::PC_YAW_KP>) k_yaw,
        (ParamFloat<px4::params::PC_YAW_RATE_KP>) k_yaw_rate
    );
    int parameters_update(bool force);

}

int MulticopterPerchingControl::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
### Description
The perching controller use PID on z direction and non perching direction to maintain the altitude and the trajectory. 
Z is estimated by distance sensor, non perching direction movement is estimated by downward facing optical flow.
A constant optical flow divergence control policy is used for perching direction control. OFD is from front facing optical flow array.
The multicopter will alway fly towards it's own heading, and yaw angle is directed by the front facing optical flow array.
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("perching_control", "controller");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

MulticopterPerchingControl::MulticopterPerchingControl() :
    SuperBlock(nullptr, "PC"),
    ModuleParams(nullptr),
    _h_deriv(this, "HD"),
    _vel_z_deriv(this, "VELD"),
    _xmotion_deriv(this, "MD"),
    _control(this)
{
    parameters_update(true);
}

MulticopterPerchingControl::~MulticopterPerchingControl(){

}

int MulticopterPerchingControl::parameters_update(bool force)
{
    bool updated;
    struct parameter_update_s param_upd;

    orb_check(_params_sub, &updated);

    if(updated) {
        orb_copy(ORB_ID(parameter_update), _params_sub, &param_upd);
    }

    if(updated || force){
        ModuleParams::updateParams();
        SuperBlock::updateParams();

        _flight_tasks.handleParameterUpdate();  
    }

    return OK;
}

int
MulticopterPerchingControl::print_status()
{
    if (_flight_tasks.isAnyTaskActive()) {
        PX4_INFO("Running, active flight task: %i", _flight_tasks.getActiveTask());
    } else {
        PX4_INFO("Running, no flight task active");
    }
    return 0;
}

void MulticopterPerchingControl::run()
{
    PX4_INFO("perching controller start");
    hrt_abstime time_stamp_last_loop = hrt_absolute_time();

    /* subscribe to customised optical_flow_tayu_theta topic */
    optical_front_sub = orb_subscribe(ORB_ID(optical_flow_tau_theta));

    optical_downward_sub = orb_subscribe(ORB_ID(optical_flow));

    dis_z_sub = orb_subscribe(ORB_ID(distance_sensor));

    att_sub = orb_subscribe(ORB_ID(vehicle_attitude));

    /*matrix::Vector3f _thr_sp{};

    optical_flow_tau_theta_s ofd;

    optical_flow_s of;

    distance_sensor_s dis;

    vehicle_attitude_s att;

    vehicle_attitude_setpoint_s _att_sp{};

    control::BlockDerivative _h_deriv;
    control::BlockDerivative _vel_z_deriv;
    control::BlockDerivative _xmotion_deriv;
    FlightTasks _flight_tasks;
    PositionControl _positioncontrol;
    vehicle_constraints_s constraints = _flight_tasks.getConstraints();
    _positioncontrol.updateConstraints(constraints);
    float output_h;//filtered h
    float previous_h; //for fi
    float sf = 0.7; //filter smooh factor
    float _thr_int_z, _thr_int_y;
    float roll;
    float pitch;
    float yaw;
    float yaw_sp;*/
    /* limit the update rate to 100 Hz */
    //orb_set_interval(optical_front_sub, 10);
    //orb_set_interval(optical_downward_sub, 10);
    parameters_update(true);
    /* one could wait for multiple topics with this technique, just using one here */
    px4_pollfd_struct_t fds[] = {
        { .fd = optical_front_sub,      .events = POLLIN },
        { .fd = optical_downward_sub,   .events = POLLIN },
        { .fd = att_sub,                .events = POLLIN },
        { .fd = dis_z_sub,              .events = POLLIN },
        /* there could be more file descriptors here, in the form like:
         * { .fd = other_sub_fd,   .events = POLLIN },
         */};

    int error_counter = 0;

    while(!should_exit()){
        /* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
        int poll_ret = px4_poll(&fds, 1, 50);

        /* handle the poll result */
        if (poll_ret == 0) {
            /* this means none of our providers is giving us data */
            PX4_ERR("Got no data within 50ms");

        }
        else if (poll_ret < 0) {
            /* this is seriously bad - should be an emergency */
                PX4_ERR("ERROR return value from poll(): %d", poll_ret);
        }
        
        //set _dt
        const hrt_abstime time_stamp_current = hrt_absolute_time();
        setDt((time_stamp_current - time_stamp_last_loop) / 1e6f);
        time_stamp_last_loop = time_stamp_current;


        /*message subscriber*/
        bool updated;
        orb_check(optical_front_sub, &updated);
        if(updated){
            
            orb_copy(ORB_ID(optical_flow_tau_theta), optical_front_sub, &ofd);
            float flow_divergence = ofd.tau;
            float direction_guidence = ofd.theta;
        }

       orb_check(optical_downward_sub, &updated);
       if(updated){
            
            orb_copy(ORB_ID(optical_flow), optical_downward_sub, &of);
            float xmotion = of.pixel_flow_x_integral;
            float ymotion = of.pixel_flow_y_integral;
       }
       orb_check(att_sub, &updated);
       if (updated) {
            if (orb_copy(ORB_ID(vehicle_attitude), att_sub, &att) == PX4_OK && PX4_ISFINITE(att.q[0])) {
               yaw = Eulerf(Quatf(att.q)).psi();
               roll = Eulerf(Quatf(att.q)).phi();
               pitch = Eulerf(Quatf(att.q)).theta();
            }
       }
       orb_check(dis_z_sub, &updated);
       if(updated){ 
             orb_copy(ORB_ID(distance_sensor), dis_z_sub, &dis);
             float h = dis.current_distance * cos(roll) * cos(pitch);
             if(h>=200){h = 2.0;} else if(h<=0){h=0.0;} else{h = h/100.0;}
             output_h = sf * h + (1-sf)* previous_h;
             previous_h = output_h;
       }


    /*thrust in D direction*/
    float H = -output_h - 0.06; //compensate for roll and pitch, and add offset
    float vel_z_sp = k_p * (h_sp - H);//h_sp k_p param;
    float vel_z = _h_deriv.update(h);
    float acc_z = _vel_z_deriv.update(vel_z);
    float vel_err_z = vel_z_sp - vel_z;
    float thrust_desired_D = k_vel_p * vel_err_z + k_vel_d * acc_z + _thr_int_z - MPC_THR_HOVER.get(); //k_vel_p, k_vel_d
    float uMax = -MPC_THR_MIN.get();
    float uMin = -MPC_THR_MAX.get();
    bool stop_integral_D = (thrust_desired_D >= uMax && vel_err_z >= 0.0f) ||
                   (thrust_desired_D <= uMin && vel_err_z <= 0.0f);
    if (!stop_integral_D){
        _thr_int_z += vel_err_z * k_vel_i * _dt;        //k_vel_i
        // limit thrust integral
        _thr_int_z = math::min(fabsf(_thr_int_z, MPC_THR_MAX.get()) * math::sign(_thr_int_z));
    }

    _thr_sp(2) = math::constrain(thrust_desired_D, uMin, uMax);
    
    /*thrust in perching and non perching direction */

    float motion_err_y = 0 - xmotion;
    float acc_y = xmotion_deriv.update();
    float ofd_err = desired_ofd - flow_divergence;
    // PID-velocity controller for NE-direction.
        Vector2f thrust_desired_xy;
        thrust_desired_xy(0) = k_ofd_p * (ofd_err); //kofd_p
        thrust_desired_xy(1) = k_m_p * motion_err_y + k_m_d * acc_y + _thr_int_y; //k_m_p, k_m_d, k_m_i

        // Get maximum allowed thrust in NE based on tilt and excess thrust.
        float thrust_max_tilt = fabsf(_thr_sp(2)) * tanf(_constraints.tilt);
        float thrust_max = sqrtf(MPC_THR_MAX.get() * MPC_THR_MAX.get() - _thr_sp(2) * _thr_sp(2));
        thrust_max = math::min(thrust_max_NE_tilt, thrust_max_NE);

        // Saturate thrust in NE-direction.
        _thr_sp(0) = thrust_desired_xy(0);
        _thr_sp(1) = thrust_desired_xy(1);

        if (thrust_desired_xy * thrust_desired_xy > thrust_max* thrust_max) {
            float mag = thrust_desired_xy.length();
            _thr_sp(0) = thrust_desired_xy(0) / mag * thrust_max;
            _thr_sp(1) = thrust_desired_xy(1) / mag * thrust_max;
        }

        // Use tracking Anti-Windup for NE-direction: during saturation, the integrator is used to unsaturate the output
        // see Anti-Reset Windup for PID controllers, L.Rundqwist, 1990
        float arw_gain = 2.f / k_m_p;

        float vel_err_lim;
        vel_err_lim = motion_err_y - (thrust_desired_xy(1) - _thr_sp(1)) * arw_gain;

        // Update integral
        _thr_int_y += k_m_i * vel_err_lim * _dt;  //k_m_i

        /*yaw setpoint control */

        yaw_sp = yaw + direction_guidence * k_yaw; //k_yaw;
        yaw_rate_sp = k_yaw_rate * direction_guidence ; //k_yaw_rate

        _att_sp = ControlMath::thrustToAttitude(_thr_sp, yaw_sp);
        _att_sp.yaw_sp_move_rate = yaw_rate_sp;
        _att_sp.fw_control_yaw = false;
        _att_sp.apply_flaps = false;

        _att_sp.timestamp = hrt_absolute_time();

        if (_att_sp_pub != nullptr) {
            orb_publish(_attitude_setpoint_id, _att_sp_pub, &_att_sp);

         } else if (_attitude_setpoint_id) {
              _att_sp_pub = orb_advertise(_attitude_setpoint_id, &_att_sp);
         }

    orb_unsubscribe(optical_front_sub);
    orb_unsubscribe(optical_downward_sub);
    orb_unsubscribe(dis_z_sub);
    orb_unsubscribe(att_sub);
  }
    PX4_INFO("exiting");

    return 0;

}

int MulticopterPerchingControl::task_spawn(int argc, char *argv[])
{
    _task_id = px4_task_spawn_cmd("perching_control",
                       SCHED_DEFAULT,
                       SCHED_PRIORITY_POSITION_CONTROL,
                       1900,
                       (px4_main_t)&run_trampoline,
                       (char *const *)argv);

    if (_task_id < 0) {
        _task_id = -1;
        return -errno;
    }

    return 0;
}

MulticopterPerchingControl *MulticopterPerchingControl::instantiate(int argc, char *argv[])
{
    return new MulticopterPerchingControl();
}

int MulticopterPerchingControl::custom_command(int argc, char *argv[])
{
    return print_usage("unknown command");
}

int perching_control_main(int argc, char *argv[])
{
    return MulticopterPerchingControl::main(argc, argv);
}