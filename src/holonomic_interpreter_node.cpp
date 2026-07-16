#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>

class HolonomicInterpreter : public rclcpp::Node
{
  public:
    HolonomicInterpreter() : Node("holonomic_interpreter")
    {
      // QoS profile
      auto sub_qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_default);

      // Subscribers
      cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("holo_cmd_vel", sub_qos,
        std::bind(&HolonomicInterpreter::callback_cmd, this, std::placeholders::_1));

      odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", sub_qos,
        std::bind(&HolonomicInterpreter::callback_odom, this, std::placeholders::_1));

      // Publishers
      cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("nonholo_cmd_vel",10);
      
      // Desired state
      desired_heading_ = 0;
      desired_speed_ = 0;

      //Current state
      yaw_ = 0;
      yaw_dot_ = 0;

      // PID controllers
      this->declare_parameter("heading_kp", 1.0);
      this->get_parameter("heading_kp",heading_kp_);

      this->declare_parameter("heading_ki", 0.00);
      this->get_parameter("heading_ki", heading_ki_);

      this->declare_parameter("heading_kd", 0.1);
      this->get_parameter("heading_kd", heading_kd_);

      this->declare_parameter("max_lin_vel", 0.5);
      this->get_parameter("max_lin_vel", max_lin_vel_);

      this->declare_parameter("origin_rotation", -0.6021965548550632);
      this->get_parameter("origin_rotation", origin_rotation_);

      this->declare_parameter<std::int64_t>("control_loop_frequency", 100);
      this->get_parameter("control_loop_frequency", control_loop_frequency_);

      if (control_loop_frequency_ <= 0.0)
      {
        throw std::invalid_argument(
          "control_loop_frequency must be greater than zero."
        );
      }
      
      double period = 1.0/static_cast<double>(control_loop_frequency_);
      auto rate = std::chrono::duration<double>(period);

      timer_ = this->create_wall_timer(rate, std::bind(&HolonomicInterpreter::callback_timer, this));
    }

  private:
    void callback_timer()
    {
      desired_heading_ = std::atan2(lin_vel_y_, lin_vel_x_); 
      desired_speed_ = std::sqrt((lin_vel_x_*lin_vel_x_) + (lin_vel_y_*lin_vel_y_));

      double heading_error = wrap_angle(desired_heading_ - yaw_);
      double heading_correction = heading_kp_*heading_error - heading_kd_*yaw_dot_;

      if (safety_check(cmd_epoch_))
      {
        if (safety_check(odom_epoch_))
        {
          auto twist = geometry_msgs::msg::Twist();

          speed_ = desired_speed_; 

          if (desired_speed_ < 1e-6)
          {
            publish_zeros();
            return;
          }

          if (speed_ >= max_lin_vel_) {
              speed_ = max_lin_vel_;
          } else if (speed_ <= -max_lin_vel_) {
              speed_ = -max_lin_vel_;
          } //else {
              // No change needed, as speed_ remains the same
          //}

          twist.angular.z = heading_correction;
          twist.linear.x = speed_;
          
          // Publish cmd_vel to robot
          cmd_pub_->publish(twist);
        }
        else
        {
         // RCLCPP_WARN(this->get_logger(), "MISSING: HEADING!");
         // Publish zeros in here since odom safety check failed
         publish_zeros();
        }
      }
      else
      {
        // RCLCPP_WARN(this->get_logger(), "MISSING: SETPOINT!");
        // Publish zeros in here since cmd safety check failed
        publish_zeros();
      }
    }

    void callback_cmd(const std::shared_ptr<geometry_msgs::msg::Twist> msg)
    {
        cmd_epoch_ = now();

        // RCLCPP_WARN(this->get_logger(), "Got HEADING!");
        
        lin_vel_x_ = msg->linear.x;
        lin_vel_y_ = msg->linear.y;
    }

    void callback_odom(const std::shared_ptr<nav_msgs::msg::Odometry> msg)
    {
      // RCLCPP_WARN(this->get_logger(), "Got IMU!");
      odom_epoch_ = now();
      geometry_msgs::msg::Quaternion orientation = msg->pose.pose.orientation;
      yaw_ = getYawFromQuaternion(orientation);
      yaw_ = yaw_ - origin_rotation_;
      // RCLCPP_WARN(this->get_logger(), "Yaw:\t%.2f", yaw_);

      yaw_dot_ = msg->twist.twist.angular.z;

    }

    double getYawFromQuaternion(const geometry_msgs::msg::Quaternion &q_msg) {
      tf2::Quaternion q(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
      double roll, pitch, yaw;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
      return yaw;
    }

    bool safety_check(rclcpp::Time epoch)
    {
      if ((now().nanoseconds() - epoch.nanoseconds()) * 1e-9 > 1.0)
      {
        // RCLCPP_INFO(this->get_logger(), "Missing sensor update rate!");
        return false;
      }
      return true;
    }

    double wrap_angle (double angle)
    {
      angle = fmod(angle + M_PI, 2.0 * M_PI);

      if (angle < 0)
      {
        angle += 2.0*M_PI;
      }

      return angle - M_PI;
    }

    void publish_zeros()
    {
      auto zero_twist = geometry_msgs::msg::Twist();
      // Set all fields to zero
      zero_twist.linear.x = 0.0;
      zero_twist.linear.y = 0.0;
      zero_twist.linear.z = 0.0;
      zero_twist.angular.x = 0.0;
      zero_twist.angular.y = 0.0;
      zero_twist.angular.z = 0.0;
      
      // Send zero command to robot
      cmd_pub_->publish(zero_twist);
    }

    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

    // ROS parameters
    double heading_kp_;
    double heading_ki_;
    double heading_kd_;
    double max_lin_vel_;
    double origin_rotation_;
    std::int64_t control_loop_frequency_;

    // Safety checks
    rclcpp::Time cmd_epoch_;
    rclcpp::Time odom_epoch_;

    // Desired state
    double lin_vel_x_{0.0};
    double lin_vel_y_{0.0};

    double desired_heading_;
    double desired_speed_;
    double speed_;

    //Current state
    double yaw_;
    double yaw_dot_;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto holo_interpret = std::make_shared<HolonomicInterpreter>();

  rclcpp::spin(holo_interpret);

  rclcpp::shutdown();

  return 0;
}