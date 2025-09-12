#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>

class SetpointPublisherNode : public rclcpp::Node
{
public:
    SetpointPublisherNode() : Node("setpoint_publisher_node")
    {
        // Publishers
        setpoint_publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("setpoint", 10);
        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("planned_path", 10);
        
        // Parameters
        this->declare_parameter("waypoint_mode", "manual");  // manual, circle, square, line
        this->declare_parameter("publish_rate", 1.0);  // Hz
        this->declare_parameter("circle_radius", 10.0);  // meters
        this->declare_parameter("square_size", 20.0);   // meters
        this->declare_parameter("line_length", 15.0);   // meters
        this->declare_parameter("auto_advance", true);  // automatically advance to next waypoint
        this->declare_parameter("waypoint_timeout", 10.0);  // seconds
        
        waypoint_mode_ = this->get_parameter("waypoint_mode").as_string();
        publish_rate_ = this->get_parameter("publish_rate").as_double();
        circle_radius_ = this->get_parameter("circle_radius").as_double();
        square_size_ = this->get_parameter("square_size").as_double();
        line_length_ = this->get_parameter("line_length").as_double();
        auto_advance_ = this->get_parameter("auto_advance").as_bool();
        waypoint_timeout_ = this->get_parameter("waypoint_timeout").as_double();
        
        // Initialize waypoint tracking
        current_waypoint_index_ = 0;
        last_waypoint_time_ = this->get_clock()->now();
        
        // Generate waypoints based on mode
        generateWaypoints();
        
        // Create timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate_)),
            std::bind(&SetpointPublisherNode::publishCurrentSetpoint, this));
        
        RCLCPP_INFO(this->get_logger(), "Setpoint Publisher Node initialized in '%s' mode with %zu waypoints", 
                    waypoint_mode_.c_str(), waypoints_.size());
    }

private:
    void generateWaypoints()
    {
        waypoints_.clear();
        
        if (waypoint_mode_ == "circle") {
            generateCircleWaypoints();
        } else if (waypoint_mode_ == "square") {
            generateSquareWaypoints();
        } else if (waypoint_mode_ == "line") {
            generateLineWaypoints();
        } else if (waypoint_mode_ == "field_pattern") {
            generateFieldPatternWaypoints();
        } else {
            // Manual mode - add some default waypoints
            generateManualWaypoints();
        }
        
        // Publish the planned path
        publishPlannedPath();
    }
    
    void generateCircleWaypoints()
    {
        int num_points = 8;
        for (int i = 0; i < num_points; i++) {
            double angle = 2.0 * M_PI * i / num_points;
            geometry_msgs::msg::PointStamped point;
            point.header.frame_id = "odom";
            point.point.x = circle_radius_ * cos(angle);
            point.point.y = circle_radius_ * sin(angle);
            point.point.z = 0.0;
            waypoints_.push_back(point);
        }
    }
    
    void generateSquareWaypoints()
    {
        double half_size = square_size_ / 2.0;
        
        // Corner waypoints of a square
        std::vector<std::pair<double, double>> corners = {
            {half_size, half_size},    // Top-right
            {-half_size, half_size},   // Top-left  
            {-half_size, -half_size},  // Bottom-left
            {half_size, -half_size}    // Bottom-right
        };
        
        for (const auto& corner : corners) {
            geometry_msgs::msg::PointStamped point;
            point.header.frame_id = "odom";
            point.point.x = corner.first;
            point.point.y = corner.second;
            point.point.z = 0.0;
            waypoints_.push_back(point);
        }
    }
    
    void generateLineWaypoints()
    {
        int num_points = 3;
        for (int i = 0; i < num_points; i++) {
            geometry_msgs::msg::PointStamped point;
            point.header.frame_id = "odom";
            point.point.x = line_length_ * i / (num_points - 1);
            point.point.y = 0.0;
            point.point.z = 0.0;
            waypoints_.push_back(point);
        }
    }
    
    void generateFieldPatternWaypoints()
    {
        // Generate a back-and-forth farming pattern
        double field_width = 30.0;  // meters
        double field_length = 40.0; // meters
        double row_spacing = 3.0;   // meters between rows
        
        int num_rows = static_cast<int>(field_width / row_spacing);
        
        for (int row = 0; row < num_rows; row++) {
            double y = -field_width/2.0 + row * row_spacing;
            
            if (row % 2 == 0) {
                // Left to right
                for (double x = -field_length/2.0; x <= field_length/2.0; x += 5.0) {
                    geometry_msgs::msg::PointStamped point;
                    point.header.frame_id = "odom";
                    point.point.x = x;
                    point.point.y = y;
                    point.point.z = 0.0;
                    waypoints_.push_back(point);
                }
            } else {
                // Right to left
                for (double x = field_length/2.0; x >= -field_length/2.0; x -= 5.0) {
                    geometry_msgs::msg::PointStamped point;
                    point.header.frame_id = "odom";
                    point.point.x = x;
                    point.point.y = y;
                    point.point.z = 0.0;
                    waypoints_.push_back(point);
                }
            }
        }
    }
    
    void generateManualWaypoints()
    {
        // Some default manual waypoints
        std::vector<std::pair<double, double>> points = {
            {0.0, 0.0},
            {5.0, 0.0},
            {5.0, 5.0},
            {0.0, 5.0},
            {0.0, 0.0}
        };
        
        for (const auto& point : points) {
            geometry_msgs::msg::PointStamped wp;
            wp.header.frame_id = "odom";
            wp.point.x = point.first;
            wp.point.y = point.second;
            wp.point.z = 0.0;
            waypoints_.push_back(wp);
        }
    }
    
    void publishCurrentSetpoint()
    {
        if (waypoints_.empty()) return;
        
        // Check if we should advance to next waypoint
        if (auto_advance_) {
            auto current_time = this->get_clock()->now();
            double elapsed = (current_time - last_waypoint_time_).seconds();
            
            if (elapsed > waypoint_timeout_) {
                current_waypoint_index_ = (current_waypoint_index_ + 1) % waypoints_.size();
                last_waypoint_time_ = current_time;
                
                RCLCPP_INFO(this->get_logger(), "Advancing to waypoint %zu: (%.2f, %.2f)", 
                            current_waypoint_index_,
                            waypoints_[current_waypoint_index_].point.x,
                            waypoints_[current_waypoint_index_].point.y);
            }
        }
        
        // Publish current waypoint
        auto current_waypoint = waypoints_[current_waypoint_index_];
        current_waypoint.header.stamp = this->get_clock()->now();
        setpoint_publisher_->publish(current_waypoint);
    }
    
    void publishPlannedPath()
    {
        auto path_msg = nav_msgs::msg::Path();
        path_msg.header.stamp = this->get_clock()->now();
        path_msg.header.frame_id = "odom";
        
        for (const auto& waypoint : waypoints_) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = waypoint.header;
            pose.pose.position.x = waypoint.point.x;
            pose.pose.position.y = waypoint.point.y;
            pose.pose.position.z = waypoint.point.z;
            pose.pose.orientation.w = 1.0;  // No rotation
            path_msg.poses.push_back(pose);
        }
        
        path_publisher_->publish(path_msg);
        RCLCPP_INFO(this->get_logger(), "Published planned path with %zu waypoints", path_msg.poses.size());
    }
    
    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr setpoint_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Parameters
    std::string waypoint_mode_;
    double publish_rate_;
    double circle_radius_;
    double square_size_;
    double line_length_;
    bool auto_advance_;
    double waypoint_timeout_;
    
    // Waypoint tracking
    std::vector<geometry_msgs::msg::PointStamped> waypoints_;
    size_t current_waypoint_index_;
    rclcpp::Time last_waypoint_time_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SetpointPublisherNode>());
    rclcpp::shutdown();
    return 0;
}