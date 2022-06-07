/*! @package pid_controller
    Code Information:
        Maintainer: Eng. Davidson Daniel Rojas Cediel
        Mail: davidson@kiwibot.com
        Kiwi Campus / AI & Robotics Team
*/

#include "motion_control/pid_controller.hpp"

PIDController::PIDController() {}

float PIDController::ThrottlePID(float ref_vx, float cur_vx, double dt)
{
    /********************************************
     * DEFINE YOUR AMAZING PID CONTROLLER
     * Find Documentation here:
     * https://www.elprocus.com/the-working-of-a-pid-controller/
     * 
     * The implementation is based on discrete PID implementation
     * applying the "Backward Euler" structure.
     * U_pid = ( Kp * E )
     *         + ( Ki  * dt * z / ( z - 1 )) * E
     *         + ( Kd / dt ) * ( z - 1 ) * E / z
     * in terms of difference equations for its implementation by epochs is:
     * 
     * up = Kp * e_k
     * ui = Ki  * dt * e_k + ui_k1
     * ud = ( Kd / dt ) * ( e_k - e_k1 )
     * u_pid = up + ui + ud
     ********************************************/

    if(!m_throttle_ctrl) {
        m_vx_int_error = 0.0;
        m_vx_prop_ek1 = 0.0f;
        m_prev_prop_error = 0.0;

        return ref_vx;
    }

    if(ref_vx==0) {
        m_vx_int_error = 0.0;
        m_vx_prop_ek1 = 0.0f;
        m_prev_prop_error = 0.0;
    }

    double m_prop_error = ref_vx - cur_vx; // e_k
    
    float up = m_kp_thr * m_prop_error;
    float ui = m_ki_thr * dt * m_prop_error + m_vx_int_error;
    float ud = ( m_kd_thr / dt ) * (m_prop_error - m_prev_prop_error);
    float u_pid = up + ui + ud;

    if (u_pid > m_max_linear_spd) u_pid = m_max_linear_spd;
    else if (u_pid < -m_max_linear_spd) u_pid = -m_max_linear_spd;
    
    /*****************************
     * In order to saturate the control action within the limits imposed by the system,
     * a direct saturation is applied, which is then fed back into the integral action,
     * so the past integral action must take into account the real implications of
     * the saturation. This is to prevent the integral action from increasing too much
     * and leading to system instabilities.
     * **************************/

    m_vx_int_error = u_pid - up - ud;
    m_prev_prop_error = m_prop_error;
    
    return u_pid;
    /********************************************
     * END CODE
     *  ********************************************/
}

float PIDController::SteeringPID(float ref_wz, float cur_wz, double dt)
{
    /********************************************
     * DEFINE YOUR AMAZING PID CONTROLLER
     * Find Documentation here:
     * https://www.elprocus.com/the-working-of-a-pid-controller/
     * */

    if(!m_steering_ctrl) {
        m_wz_int_error = 0.0;
        m_wz_prop_ek1 = 0.0f;
        m_prev_prop_w_error = 0.0;

        return ref_wz;
    }

    if(ref_wz==0) {
        m_wz_int_error = 0.0;
        m_wz_prop_ek1 = 0.0f;
        m_prev_prop_w_error = 0.0;
    }

    double m_prop_error = ref_wz - cur_wz; // e_k
    
    float up = m_kp_str * m_prop_error;
    float ui = m_ki_str * dt * m_prop_error + m_wz_int_error;
    float ud = ( m_kd_str / dt ) * (m_prop_error - m_prev_prop_w_error);
    float u_pid = up + ui + ud;

    /********************************************
     * FeedForward:
     * https://zhuanlan.zhihu.com/p/382010500#:~:text=In%20many%20applications,dynamic%20models%20used.
     * "Combined FeedForward and Feedback Control"
     ********************************************/
    float uff = m_kff_str * ref_wz;

    float u_pid_ff = u_pid + uff;

    if (u_pid_ff > m_max_angular_spd) u_pid_ff = m_max_angular_spd;
    else if (u_pid_ff < -m_max_angular_spd) u_pid_ff = -m_max_angular_spd;
    
    m_wz_int_error = u_pid_ff - up - ud - uff;
    m_prev_prop_w_error = m_prop_error;

    return u_pid_ff;
    /********************************************
     * END CODE
     *  ********************************************/
}
