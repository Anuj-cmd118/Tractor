#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>

class VehicleDynamicsNode : public rclcpp::Node
{
public:
    VehicleDynamicsNode() : Node("vehicle_dynamics_node")
    {
        // Publishers
        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "vehicle/pose", 10);
        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "vehicle/velocity", 10);
        
        // Subscribers
        cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10,
            std::bind(&VehicleDynamicsNode::cmdVelCallback, this, std::placeholders::_1));
        
        // Parameters
        this->declare_parameter("wheelbase", 2.5);  // meters
        this->declare_parameter("max_speed", 5.0);  // m/s
        this->declare_parameter("max_steering_angle", 0.785);  // radians (45 degrees)
        this->declare_parameter("update_rate", 50.0);  // Hz
        
        wheelbase_ = this->get_parameter("wheelbase").as_double();
        max_speed_ = this->get_parameter("max_speed").as_double();
        max_steering_angle_ = this->get_parameter("max_steering_angle").as_double();
        
        double update_rate = this->get_parameter("update_rate").as_double();
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / update_rate)),
            std::bind(&VehicleDynamicsNode::updateDynamics, this));
        
        // Initialize state
        x_ = 0.0;
        y_ = 0.0;
        theta_ = 0.0;
        v_ = 0.0;
        steering_angle_ = 0.0;
        
        RCLCPP_INFO(this->get_logger(), "Vehicle Dynamics Node initialized");
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // Extract commanded linear velocity and angular velocity (steering)
        double cmd_v = std::max(-max_speed_, std::min(max_speed_, msg->linear.x));
        double cmd_steering = std::max(-max_steering_angle_, 
                                     std::min(max_steering_angle_, msg->angular.z));
        
        // Simple first-order dynamics
        double alpha = 0.1;  // Filter coefficient
        v_ = alpha * cmd_v + (1.0 - alpha) * v_;
        steering_angle_ = alpha * cmd_steering + (1.0 - alpha) * steering_angle_;
    }
    
    void updateDynamics()
    {
        double dt = 0.02;  // 50 Hz update rate
        
        // Bicycle model kinematics
        double dx = v_ * cos(theta_);
        double dy = v_ * sin(theta_);
        double dtheta = (v_ / wheelbase_) * tan(steering_angle_);
        
        // Integrate
        x_ += dx * dt;
        y_ += dy * dt;
        theta_ += dtheta * dt;
        
        // Normalize angle
        while (theta_ > M_PI) theta_ -= 2.0 * M_PI;
        while (theta_ < -M_PI) theta_ += 2.0 * M_PI;
        
        publishState();
    }
    
    void publishState()
    {
        // Publish pose
        auto pose_msg = geometry_msgs::msg::PoseWithCovarianceStamped();
        pose_msg.header.stamp = this->get_clock()->now();
        pose_msg.header.frame_id = "odom";
        pose_msg.pose.pose.position.x = x_;
        pose_msg.pose.pose.position.y = y_;
        pose_msg.pose.pose.position.z = 0.0;
        
        // Convert theta to quaternion
        pose_msg.pose.pose.orientation.w = cos(theta_ / 2.0);
        pose_msg.pose.pose.orientation.x = 0.0;
        pose_msg.pose.pose.orientation.y = 0.0;
        pose_msg.pose.pose.orientation.z = sin(theta_ / 2.0);
        
        pose_publisher_->publish(pose_msg);
        
        // Publish velocity
        auto vel_msg = geometry_msgs::msg::Twist();
        vel_msg.linear.x = v_;
        vel_msg.linear.y = 0.0;
        vel_msg.linear.z = 0.0;
        vel_msg.angular.x = 0.0;
        vel_msg.angular.y = 0.0;
        vel_msg.angular.z = (v_ / wheelbase_) * tan(steering_angle_);
        
        velocity_publisher_->publish(vel_msg);
    }
    
    // Publishers and subscribers
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Vehicle parameters
    double wheelbase_;
    double max_speed_;
    double max_steering_angle_;
    
    // Vehicle state
    double x_, y_, theta_;  // Position and orientation
    double v_;             // Linear velocity
    double steering_angle_; // Steering angle
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VehicleDynamicsNode>());
    rclcpp::shutdown();
    return 0;
}