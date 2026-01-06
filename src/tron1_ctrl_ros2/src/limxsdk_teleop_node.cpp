#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "limxsdk/pointfoot.h"
#include <thread>
#include <atomic>
#include <mutex>
#include <chrono>
#include <vector>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>

class LimxSdkTeleopNode : public rclcpp::Node
{
public:
  LimxSdkTeleopNode() : Node("limxsdk_teleop_node")
  {
    // Parameters
    this->declare_parameter<std::string>("robot_ip", "127.0.0.1");
    this->get_parameter("robot_ip", robot_ip_);
    
    this->declare_parameter<bool>("use_keyboard", false);
    this->get_parameter("use_keyboard", use_keyboard_);

    // Initialize SDK
    pf_ = limxsdk::PointFoot::getInstance();
    RCLCPP_INFO(this->get_logger(), "Connecting to robot at %s ...", robot_ip_.c_str());
    if (!pf_->init(robot_ip_))
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize LimX SDK with IP: %s", robot_ip_.c_str());
      rclcpp::shutdown();
      return;
    }
    
    // Subscribe to diagnostics
    pf_->subscribeDiagnosticValue([this](const limxsdk::DiagnosticValueConstPtr& msg) {
      if (msg->level == limxsdk::DiagnosticValue::ERROR) {
        RCLCPP_ERROR(this->get_logger(), "Robot Diagnostic Error: [%s] %s", msg->name.c_str(), msg->message.c_str());
      }
    });

    RCLCPP_INFO(this->get_logger(), "LimX SDK Connected!");

    // Subscriber
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&LimxSdkTeleopNode::cmdVelCallback, this, std::placeholders::_1));

    // Get motor info
    motor_num_ = pf_->getMotorNumber();
    RCLCPP_INFO(this->get_logger(), "Robot has %d motors", motor_num_);

    // Init control parameters
    kp_.resize(motor_num_, 60.0); 
    kd_.resize(motor_num_, 3.0);
    
    // Init command vectors
    cmd_.resize(motor_num_);
    std::fill(cmd_.mode.begin(), cmd_.mode.end(), 0); // Hybrid Control
    
    // Start control thread
    running_ = true;
    control_thread_ = std::thread(&LimxSdkTeleopNode::controlLoop, this);
    
    // Start keyboard thread if enabled
    if (use_keyboard_) {
      keyboard_thread_ = std::thread(&LimxSdkTeleopNode::keyboardLoop, this);
    }
  }

  ~LimxSdkTeleopNode()
  {
    running_ = false;
    if (control_thread_.joinable())
    {
      control_thread_.join();
    }
    if (keyboard_thread_.joinable())
    {
      keyboard_thread_.join();
    }
  }

  /**
   * @brief Convert Twist command to Joint velocities and update control buffer
   * 
   * @param cmd Twist command from ROS2
   */
  void sendVelocityCommand(const geometry_msgs::msg::Twist& cmd)
  {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    
    // 1. Update timestamp for timeout protection
    last_cmd_time_ = std::chrono::steady_clock::now();
    
    // 2. Mapping Logic (Twist -> Joint Velocities)
    
    // Reset all commands first
    std::fill(cmd_.q.begin(), cmd_.q.end(), 0.0);
    std::fill(cmd_.dq.begin(), cmd_.dq.end(), 0.0);
    std::fill(cmd_.tau.begin(), cmd_.tau.end(), 0.0);
    std::fill(cmd_.Kp.begin(), cmd_.Kp.end(), 0.0); // No position control for velocity mode
    std::fill(cmd_.Kd.begin(), cmd_.Kd.end(), 4.0); // Damping

    if (motor_num_ >= 8) {
      // Assuming WheelFoot configuration:
      // Indices 3 (Left Wheel) and 7 (Right Wheel)
      
      double linear_x = cmd.linear.x;
      double angular_z = cmd.angular.z;
      
      double wheel_radius = 0.06; // Approximate, check URDF
      double track_width = 0.3;   // Approximate
      
      // Differential Drive Kinematics
      double v_left = linear_x - (angular_z * track_width / 2.0);
      double v_right = linear_x + (angular_z * track_width / 2.0);
      
      double w_left = v_left / wheel_radius;
      double w_right = v_right / wheel_radius;

      // Set Wheel Velocities
      
      // Left Wheel (Index 3)
      cmd_.dq[3] = w_left;
      cmd_.Kd[3] = 2.0; // Velocity gain
      
      // Right Wheel (Index 7)
      cmd_.dq[7] = w_right;
      cmd_.Kd[7] = 2.0; // Velocity gain
      
      // Keep legs stiff/damped
      for(int i : {0,1,2, 4,5,6}) {
         cmd_.Kp[i] = 40.0; // Hold position
         cmd_.Kd[i] = 3.0;
      }
    } 
  }

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    // Directly call the processing function
    sendVelocityCommand(*msg);
  }
  
  // Non-blocking character read
  int getch()
  {
    static struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);           // save old settings
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);         // disable buffering & echo
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);  // apply new settings

    int ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
    return ch;
  }

  void keyboardLoop()
  {
    RCLCPP_INFO(this->get_logger(), "Keyboard Control Enabled. Use W/A/S/D to move, Space to stop, Q to quit.");
    
    double speed = 0.5;
    double turn = 1.0;
    double x = 0;
    double th = 0;

    while(rclcpp::ok() && running_) {
      int c = getch();
      
      bool dirty = false;
      if (c == 'w') { x = speed; dirty = true; }
      else if (c == 's') { x = -speed; dirty = true; }
      else if (c == 'a') { th = turn; dirty = true; }
      else if (c == 'd') { th = -turn; dirty = true; }
      else if (c == ' ') { x = 0; th = 0; dirty = true; }
      else if (c == 'q') { 
        rclcpp::shutdown(); 
        break; 
      }
      
      if (dirty) {
        geometry_msgs::msg::Twist twist;
        twist.linear.x = x;
        twist.linear.y = 0; 
        twist.linear.z = 0;
        twist.angular.x = 0; 
        twist.angular.y = 0; 
        twist.angular.z = th;
        
        // Directly execute command
        sendVelocityCommand(twist);
        
        // Print status
        printf("\rCurrent: speed %.2f\tturn %.2f | ", x, th);
        fflush(stdout);
        
        // Auto-reset for safety (optional, mimicking key-down behavior is hard in terminal)
        // Here we just keep the command active until Space is pressed or Timeout happens.
        // But since sendVelocityCommand updates timeout, if user stops pressing, 
        // they might want it to stop? 
        // Standard teleop requires holding keys or toggling.
        // Let's implement auto-stop after short delay if no new key?
        // No, simplest is toggle mode (press W -> moves until Space).
      }
      
      // Small sleep
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  void controlLoop()
  {
    rclcpp::Rate rate(500); // 500 Hz control loop
    
    // Wait a bit for connection
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    pf_->subscribeRobotState([this](const limxsdk::RobotStateConstPtr& state) {
       (void)state;
    });

    RCLCPP_INFO(this->get_logger(), "Control loop started.");

    while (rclcpp::ok() && running_)
    {
      // 3. Timeout Protection
      {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - last_cmd_time_).count();
        
        if (elapsed > 0.5) {
           // Timeout: Stop robot (Damping Mode)
           std::fill(cmd_.dq.begin(), cmd_.dq.end(), 0.0);
           std::fill(cmd_.tau.begin(), cmd_.tau.end(), 0.0);
           std::fill(cmd_.Kp.begin(), cmd_.Kp.end(), 0.0);
           std::fill(cmd_.Kd.begin(), cmd_.Kd.end(), 5.0); // Pure damping
        }
      }
      
      // Update timestamp and Send
      cmd_.stamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::system_clock::now().time_since_epoch()).count();
          
      pf_->publishRobotCmd(cmd_);

      rate.sleep();
    }
  }

  std::string robot_ip_;
  bool use_keyboard_;
  
  limxsdk::PointFoot* pf_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  
  std::thread control_thread_;
  std::thread keyboard_thread_;
  std::atomic<bool> running_;
  
  std::mutex cmd_mutex_;
  std::chrono::steady_clock::time_point last_cmd_time_;
  
  uint32_t motor_num_;
  std::vector<float> kp_;
  std::vector<float> kd_;
  limxsdk::RobotCmd cmd_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LimxSdkTeleopNode>());
  rclcpp::shutdown();
  return 0;
}
