/*! @package rpm_converter
    Code Information:
        Maintainer: Eng. Davidson Daniel Rojas Cediel
        Mail: davidson@kiwibot.com
        Kiwi Campus / AI & Robotics Team
*/
#ifndef RPM_CONVERTER_H_INCLUDED
#define RPM_CONVERTER_H_INCLUDED

// STD libraries
#include <math.h>
#include <memory>
#include <utility>
#include <vector>
#include <algorithm>
#include <cmath>
#include <iostream>

// ROS2 Default
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"

// ROS2 Transformations
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>
#include <rclcpp/rclcpp.hpp>

// Transform broadcasting
#include <tf2_ros/transform_broadcaster.h>
#include "geometry_msgs/msg/transform_stamped.hpp"

// Custom libraries
#include "utils/console.hpp"

// ROS2 Messages
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"

// ROS2 Services
#include "std_srvs/srv/set_bool.hpp"

// Custom Messages
#include "usr_msgs/msg/motors_rpm.hpp"

using std::placeholders::_1;

class RpmConverter : public rclcpp::Node
{
public:
    /*!
        RpmConverter Class constructor. Inherits from rclcpp::Node
        @param options: rclcpp::NodeOptions.
    */
    RpmConverter(rclcpp::NodeOptions const &options);
    /*!
        RpmConverter Class destructor
        @param void
    */
    ~RpmConverter(){};

    /*!
        Publishes the motors RPM control commands to ROS2.
        @param void.
        @return void.
    */
    void PublishMotorsControl();
    int m_publish_time = getEnv("CHASSIS_PUBLISH_TIME", 150);

private:
    // Structures
    struct speed_cmd
    {
        float linear_vx = 0.0f;
        float angular_wz = 0.0f;
    } speed_ctrl;

    // Publishers
    rclcpp::Publisher<usr_msgs::msg::MotorsRPM>::SharedPtr
        m_motors_rpm_out_pub; /*!< Publish at topic /rpm_converter/motors_rpm_out */

    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_spd_ctrl_out_sub; /*!< @sa SpeedControlOutCb() */

    // Timers
    rclcpp::TimerBase::SharedPtr m_publish_data_tmr; /*!< @sa PublisherTmrCb() */

    // Utils functions
    /*!
        Converts wheels angular velocity into RPMs.
        @param rads: Wheels angular velocities in floats.
        @return float: Corresponding RPMs in the range [-255, 255].
    */
    float RadsToDigital(float rads);

    /*!
        Function to calculate the right Angular Velocity
        @param lin_vx: const float& Linear velocity
        @param lin_wz: const float& Angular velocity
        @return std::vector<float>
    */
    std::vector<float> EnsureAngularVelocity(const float &lin_vx, const float &ang_wz);

    // Subscribers callbacks
    /*!
        Speed controller output subscription callback for the topic
       /motion_control/speed_controller/output_cmd.
        @param msg: Twist message.
        @return void.
    */
    void SpeedControlOutCb(const geometry_msgs::msg::Twist::SharedPtr msg);

    // Timer CallBacks
    /*!
        Timer Cb to publish data in case
        @param void.
        @return void.
    */
    void PublisherTmrCb();

    /* Vectors for checking the motors state */
    std::vector<float> m_motors_rpm{0.0f, 0.0f, 0.0f, 0.0f};

    /* ROS messages for the robot state */
    usr_msgs::msg::MotorsRPM m_motors_rpm_feedback_msg;

    // Environment variables
    double m_robot_track = getEnv("ODOMETRY_CHASSIS_TRACK", 0.392f);
    double m_wheel_rad = getEnv("ODOMETRY_WHEEL_RADIUS", 0.079f);
    double m_wheel_max_rpm = getEnv("CONVERTER_WHEEL_MAX_RPM", 165.0f);
    double m_wheel_spd_factor = getEnv("CONVERTER_WHEEL_SPD_FACTOR", 17.6f);
};
#endif /* End of RPM_CONVERTER_H_INCLUDED */