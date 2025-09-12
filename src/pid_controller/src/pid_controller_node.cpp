#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <nav_msgs/msg/path.hpp>
#include <cmath>

class PIDControllerNode : public rclcpp::Node
{
public:
    PIDControllerNode() : Node("pid_controller_node")
    {
        // Publishers
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        
        // Subscribers
        pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "vehicle/pose", 10,
            std::bind(&PIDControllerNode::poseCallback, this, std::placeholders::_1));
        
        setpoint_subscription_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "setpoint", 10,
            std::bind(&PIDControllerNode::setpointCallback, this, std::placeholders::_1));
        
        // Parameters
        this->declare_parameter("linear_kp", 1.0);
        this->declare_parameter("linear_ki", 0.1);
        this->declare_parameter("linear_kd", 0.05);
        this->declare_parameter("angular_kp", 2.0);
        this->declare_parameter("angular_ki", 0.2);
        this->declare_parameter("angular_kd", 0.1);
        this->declare_parameter("max_linear_vel", 2.0);
        this->declare_parameter("max_angular_vel", 1.0);
        this->declare_parameter("position_tolerance", 0.1);
        this->declare_parameter("angle_tolerance", 0.1);
        
        linear_kp_ = this->get_parameter("linear_kp").as_double();
        linear_ki_ = this->get_parameter("linear_ki").as_double();
        linear_kd_ = this->get_parameter("linear_kd").as_double();
        angular_kp_ = this->get_parameter("angular_kp").as_double();
        angular_ki_ = this->get_parameter("angular_ki").as_double();
        angular_kd_ = this->get_parameter("angular_kd").as_double();
        max_linear_vel_ = this->get_parameter("max_linear_vel").as_double();
        max_angular_vel_ = this->get_parameter("max_angular_vel").as_double();
        position_tolerance_ = this->get_parameter("position_tolerance").as_double();
        angle_tolerance_ = this->get_parameter("angle_tolerance").as_double();
        
        // Initialize PID variables
        linear_error_integral_ = 0.0;
        angular_error_integral_ = 0.0;
        linear_error_prev_ = 0.0;
        angular_error_prev_ = 0.0;
        last_time_ = this->get_clock()->now();
        
        // Initialize setpoint
        target_x_ = 0.0;
        target_y_ = 0.0;
        has_setpoint_ = false;
        
        // Initialize current pose
        current_x_ = 0.0;
        current_y_ = 0.0;
        current_theta_ = 0.0;
        has_pose_ = false;
        
        RCLCPP_INFO(this->get_logger(), "PID Controller Node initialized");
    }

private:
    void poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        
        // Convert quaternion to yaw angle
        double qx = msg->pose.pose.orientation.x;
        double qy = msg->pose.pose.orientation.y;
        double qz = msg->pose.pose.orientation.z;
        double qw = msg->pose.pose.orientation.w;
        
        current_theta_ = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
        has_pose_ = true;
        
        if (has_setpoint_) {
            computeControl();
        }
    }
    
    void setpointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        target_x_ = msg->point.x;
        target_y_ = msg->point.y;
        has_setpoint_ = true;
        
        // Reset integral terms when new setpoint arrives
        linear_error_integral_ = 0.0;
        angular_error_integral_ = 0.0;
        
        RCLCPP_INFO(this->get_logger(), "New setpoint: (%.2f, %.2f)", target_x_, target_y_);
        
        if (has_pose_) {
            computeControl();
        }
    }
    
    void computeControl()
    {
        auto current_time = this->get_clock()->now();
        double dt = (current_time - last_time_).seconds();
        if (dt <= 0.0) dt = 0.02; // Default 50Hz
        
        // Calculate distance and angle errors
        double dx = target_x_ - current_x_;
        double dy = target_y_ - current_y_;
        double distance_error = sqrt(dx * dx + dy * dy);
        double target_angle = atan2(dy, dx);
        double angle_error = normalizeAngle(target_angle - current_theta_);
        
        // Check if we've reached the target
        if (distance_error < position_tolerance_) {
            publishZeroVelocity();
            return;
        }
        
        // Linear velocity PID
        linear_error_integral_ += distance_error * dt;
        double linear_error_derivative = (distance_error - linear_error_prev_) / dt;
        double linear_output = linear_kp_ * distance_error + 
                              linear_ki_ * linear_error_integral_ + 
                              linear_kd_ * linear_error_derivative;
        
        // Angular velocity PID
        angular_error_integral_ += angle_error * dt;
        double angular_error_derivative = (angle_error - angular_error_prev_) / dt;
        double angular_output = angular_kp_ * angle_error + 
                               angular_ki_ * angular_error_integral_ + 
                               angular_kd_ * angular_error_derivative;
        
        // Apply limits and reduce linear velocity when turning
        linear_output = std::max(-max_linear_vel_, std::min(max_linear_vel_, linear_output));
        angular_output = std::max(-max_angular_vel_, std::min(max_angular_vel_, angular_output));
        
        // Reduce linear velocity proportionally to angular error
        if (fabs(angle_error) > angle_tolerance_) {
            linear_output *= (1.0 - fabs(angle_error) / M_PI);
        }
        
        // Publish control command
        auto cmd_msg = geometry_msgs::msg::Twist();
        cmd_msg.linear.x = linear_output;
        cmd_msg.angular.z = angular_output;
        cmd_vel_publisher_->publish(cmd_msg);
        
        // Update previous values
        linear_error_prev_ = distance_error;
        angular_error_prev_ = angle_error;
        last_time_ = current_time;
    }
    
    void publishZeroVelocity()
    {
        auto cmd_msg = geometry_msgs::msg::Twist();
        cmd_msg.linear.x = 0.0;
        cmd_msg.angular.z = 0.0;
        cmd_vel_publisher_->publish(cmd_msg);
    }
    
    double normalizeAngle(double angle)
    {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }
    
    // Publishers and subscribers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr setpoint_subscription_;
    
    // PID parameters
    double linear_kp_, linear_ki_, linear_kd_;
    double angular_kp_, angular_ki_, angular_kd_;
    double max_linear_vel_, max_angular_vel_;
    double position_tolerance_, angle_tolerance_;
    
    // PID variables
    double linear_error_integral_, angular_error_integral_;
    double linear_error_prev_, angular_error_prev_;
    rclcpp::Time last_time_;
    
    // State variables
    double current_x_, current_y_, current_theta_;
    double target_x_, target_y_;
    bool has_pose_, has_setpoint_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PIDControllerNode>());
    rclcpp::shutdown();
    return 0;
}