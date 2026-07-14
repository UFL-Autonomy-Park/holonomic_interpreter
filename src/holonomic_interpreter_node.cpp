#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <limits>
#include <math.h>
#include <stdexcept>

class HolonomicInterpreter : public rclcpp::Node
{
  public:
    HolonomicInterpreter() : Node("holonomic_interpreter")
    {
      //Qos profile
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
      last_speed_ = 0.0;

      //Current state
      yaw_ = 0;
      yaw_dot_ = 0;

      // ROS Parameters
      this->declare_parameter("heading_kp", 1.0);
      this->get_parameter("heading_kp",heading_kp_);

      this->declare_parameter("heading_ki", 0.00);
      this->get_parameter("heading_ki", heading_ki_);

      this->declare_parameter("heading_kd", 0.1);
      this->get_parameter("heading_kd", heading_kd_);

      this->declare_parameter("max_vel", 0.5);
      this->get_parameter("max_vel", max_vel_);

      this->declare_parameter("max_ang_acc", 0.05);
      this->get_parameter("max_ang_acc", max_ang_acc_);

      this->declare_parameter("control_frequency", 50);
      this->get_parameter("control_frequency", control_frequency_);

      if (control_frequency_ <= 0) {
        throw std::runtime_error(
          "Controller frequency must be strictly greater than zero"
        );
      }

      this->declare_parameter("control_timeout", 1.0);
      this->get_parameter("control_timeout", control_timeout_);

      if (control_timeout_ <= 0) {
        throw std::runtime_error(
          "Controller timeout must be strictly greater than zero"
        );
      }

      // Update rate
      double rate_ = 1/static_cast<double>(control_frequency_);

      // Angular acceleration
      max_ang_delta_ = max_ang_acc_ * rate_;

      std::chrono::milliseconds rate_ms((int)(rate_*1000));
      timer_ = this->create_wall_timer(rate_ms, std::bind(&HolonomicInterpreter::callback_timer, this));
    }

  private:
    void callback_timer()
    {
      desired_heading_ = std::atan2(lin_vel_y_, lin_vel_x_); 
      desired_speed_ = std::sqrt((lin_vel_x_*lin_vel_x_) + (lin_vel_y_*lin_vel_y_));

      double heading_error = wrap_angle(desired_heading_ - yaw_);
      double heading_correction = heading_kp_*heading_error - heading_kd_*yaw_dot_;    //heading_pid_->calculate(-heading_error, -yaw_dot_);

      //RCLCPP_WARN(this->get_logger(), "Heading: %f", yaw_);
      //RCLCPP_WARN(this->get_logger(), "Setpoint: %f", desired_heading_);

      heading_correction = last_speed_ + std::min(std::max(heading_correction - last_speed_, -max_ang_delta_), max_ang_delta_);
      //WRCLCPP_WARN(this->get_logger(), "Input: %f", heading_correction);

      if (safety_check(cmd_epoch_))
      {
        if (safety_check(odom_epoch_))
        {
          auto twist = geometry_msgs::msg::Twist();

          //if (desired_speed_ != 0.0)
          //{

          //double val = std::max(std::min(heading_correction, M_PI/24), -M_PI/24);
          //val = std::abs(val/(M_PI/24));
          speed_ = desired_speed_; //(desired_speed_-val*desired_speed_)+min_vel_;

          if (speed_ >= max_vel_) {
              speed_ = max_vel_;
          } else if (speed_ <= -max_vel_) {
              speed_ = -max_vel_;
          } //else {
              // No change needed, as speed_ remains the same
          //}

          twist.angular.z = heading_correction;
          twist.linear.x = speed_;

          cmd_pub_->publish(twist);
          last_speed_ = heading_correction;
        }
         else
         {
             RCLCPP_WARN(this->get_logger(), "MISSING: HEADING!");
         }
      }
       else
       {
           RCLCPP_WARN(this->get_logger(), "MISSING: SETPOINT!");
       }
    }

    void callback_cmd(const std::shared_ptr<geometry_msgs::msg::Twist> msg)
    {
        cmd_epoch_ = now();

        //RCLCPP_WARN(this->get_logger(), "Got HEADING!");
        
        lin_vel_x_ = msg->linear.x;
        lin_vel_y_ = msg->linear.y;
        command_receieved_ = true;
    }

    void callback_odom(const std::shared_ptr<nav_msgs::msg::Odometry> msg)
    {
      //RCLCPP_WARN(this->get_logger(), "Got IMU!");
      odom_epoch_ = now();
        
      yaw_ = quat_to_yaw(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

      yaw_dot_ = msg->twist.twist.angular.z;
      odom_received_ = true;
    }


    double quat_to_yaw(double x, double y, double z, double w) {
        // yaw (z-axis rotation)
        double siny_cosp = 2 * (w * z + x * y);
        double cosy_cosp = 1 - 2 * (y * y + z * z);
        double yaw = std::atan2(siny_cosp, cosy_cosp);
        return yaw;
    }

    bool safety_check(rclcpp::Time epoch)
    {
      if ((now().nanoseconds() - epoch.nanoseconds()) * 1e-9 > 1.0)
      {
        //RCLCPP_INFO(this->get_logger(), "Missing sensor update rate!");
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

    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;


    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

    // ROS parameters
    double heading_kp_;
    double heading_ki_;
    double heading_kd_;
    double max_vel_;
    double max_ang_acc_;
    int control_frequency_;
    double control_timeout_;

    double control_period_sec_;
    double max_ang_delta_;

    // Safety checks
    rclcpp::Time cmd_epoch_;
    rclcpp::Time odom_epoch_;

    // Desired state
    double lin_vel_x_;
    double lin_vel_y_;
    double last_speed_;

    double desired_heading_;
    double desired_speed_;
    double speed_;

    // Current state
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
