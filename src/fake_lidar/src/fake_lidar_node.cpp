#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <random>
#include <cmath>

class FakeLidarNode : public rclcpp::Node
{
public:
    FakeLidarNode() : Node("fake_lidar_node")
    {
        // Publisher
        laser_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
        
        // Subscriber for vehicle pose (optional - for more realistic simulation)
        pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "vehicle/pose", 10,
            std::bind(&FakeLidarNode::poseCallback, this, std::placeholders::_1));
        
        // Parameters
        this->declare_parameter("scan_rate", 10.0);  // Hz
        this->declare_parameter("range_min", 0.1);   // meters
        this->declare_parameter("range_max", 30.0);  // meters
        this->declare_parameter("angle_min", -M_PI); // radians
        this->declare_parameter("angle_max", M_PI);  // radians
        this->declare_parameter("angle_increment", M_PI / 180.0);  // 1 degree
        this->declare_parameter("noise_stddev", 0.05);  // meters
        this->declare_parameter("obstacle_probability", 0.1);  // probability of fake obstacle
        
        scan_rate_ = this->get_parameter("scan_rate").as_double();
        range_min_ = this->get_parameter("range_min").as_double();
        range_max_ = this->get_parameter("range_max").as_double();
        angle_min_ = this->get_parameter("angle_min").as_double();
        angle_max_ = this->get_parameter("angle_max").as_double();
        angle_increment_ = this->get_parameter("angle_increment").as_double();
        noise_stddev_ = this->get_parameter("noise_stddev").as_double();
        obstacle_probability_ = this->get_parameter("obstacle_probability").as_double();
        
        // Setup random number generator
        generator_ = std::mt19937(std::random_device{}());
        noise_dist_ = std::normal_distribution<double>(0.0, noise_stddev_);
        obstacle_dist_ = std::uniform_real_distribution<double>(0.0, 1.0);
        range_dist_ = std::uniform_real_distribution<double>(2.0, 15.0);
        
        // Calculate number of scan points
        num_readings_ = static_cast<int>((angle_max_ - angle_min_) / angle_increment_) + 1;
        
        // Initialize vehicle pose
        vehicle_x_ = 0.0;
        vehicle_y_ = 0.0;
        vehicle_theta_ = 0.0;
        
        // Create timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / scan_rate_)),
            std::bind(&FakeLidarNode::publishScan, this));
        
        RCLCPP_INFO(this->get_logger(), "Fake LiDAR Node initialized with %d scan points", num_readings_);
    }

private:
    void poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        vehicle_x_ = msg->pose.pose.position.x;
        vehicle_y_ = msg->pose.pose.position.y;
        
        // Convert quaternion to yaw
        double qx = msg->pose.pose.orientation.x;
        double qy = msg->pose.pose.orientation.y;
        double qz = msg->pose.pose.orientation.z;
        double qw = msg->pose.pose.orientation.w;
        
        vehicle_theta_ = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
    }
    
    void publishScan()
    {
        auto scan_msg = sensor_msgs::msg::LaserScan();
        
        // Fill header
        scan_msg.header.stamp = this->get_clock()->now();
        scan_msg.header.frame_id = "laser_frame";
        
        // Fill scan parameters
        scan_msg.angle_min = angle_min_;
        scan_msg.angle_max = angle_max_;
        scan_msg.angle_increment = angle_increment_;
        scan_msg.time_increment = 0.0;
        scan_msg.scan_time = 1.0 / scan_rate_;
        scan_msg.range_min = range_min_;
        scan_msg.range_max = range_max_;
        
        // Generate scan data
        scan_msg.ranges.resize(num_readings_);
        scan_msg.intensities.resize(num_readings_);
        
        for (int i = 0; i < num_readings_; i++) {
            double angle = angle_min_ + i * angle_increment_;
            double range;
            
            // Simulate different environments
            if (obstacle_dist_(generator_) < obstacle_probability_) {
                // Random obstacle
                range = range_dist_(generator_);
            } else {
                // Open field - max range with some variation
                range = range_max_ * (0.8 + 0.2 * obstacle_dist_(generator_));
            }
            
            // Add some simple geometric features (walls, corners)
            range = addGeometricFeatures(angle, range);
            
            // Add noise
            range += noise_dist_(generator_);
            
            // Clamp to valid range
            range = std::max(range_min_, std::min(range_max_, range));
            
            scan_msg.ranges[i] = range;
            scan_msg.intensities[i] = 100.0;  // Constant intensity
        }
        
        laser_publisher_->publish(scan_msg);
    }
    
    double addGeometricFeatures(double angle, double base_range)
    {
        double range = base_range;
        
        // Add some walls at specific angles (simulate field boundaries)
        double global_angle = vehicle_theta_ + angle;
        
        // Simulate field boundaries
        double field_size = 50.0;  // 50m x 50m field
        
        // Calculate intersection with field boundaries
        double cos_angle = cos(global_angle);
        double sin_angle = sin(global_angle);
        
        if (cos_angle > 0.01) {  // Looking east
            double dist_to_east = (field_size/2.0 - vehicle_x_) / cos_angle;
            if (dist_to_east > 0 && dist_to_east < range) {
                range = dist_to_east;
            }
        } else if (cos_angle < -0.01) {  // Looking west
            double dist_to_west = (-field_size/2.0 - vehicle_x_) / cos_angle;
            if (dist_to_west > 0 && dist_to_west < range) {
                range = dist_to_west;
            }
        }
        
        if (sin_angle > 0.01) {  // Looking north
            double dist_to_north = (field_size/2.0 - vehicle_y_) / sin_angle;
            if (dist_to_north > 0 && dist_to_north < range) {
                range = dist_to_north;
            }
        } else if (sin_angle < -0.01) {  // Looking south
            double dist_to_south = (-field_size/2.0 - vehicle_y_) / sin_angle;
            if (dist_to_south > 0 && dist_to_south < range) {
                range = dist_to_south;
            }
        }
        
        // Add some random obstacles (trees, rocks, etc.)
        if (fabs(vehicle_x_) < 10 && fabs(vehicle_y_) < 10) {  // Near origin
            double obstacle_x = 5.0;
            double obstacle_y = 3.0;
            double obstacle_radius = 0.5;
            
            double dx = obstacle_x - vehicle_x_;
            double dy = obstacle_y - vehicle_y_;
            double obstacle_angle = atan2(dy, dx);
            double angle_diff = fabs(global_angle - obstacle_angle);
            
            if (angle_diff < 0.1) {  // Within beam width
                double obstacle_dist = sqrt(dx * dx + dy * dy) - obstacle_radius;
                if (obstacle_dist > 0 && obstacle_dist < range) {
                    range = obstacle_dist;
                }
            }
        }
        
        return range;
    }
    
    // Publishers and subscribers
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_subscription_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Parameters
    double scan_rate_;
    double range_min_, range_max_;
    double angle_min_, angle_max_;
    double angle_increment_;
    double noise_stddev_;
    double obstacle_probability_;
    int num_readings_;
    
    // Vehicle state
    double vehicle_x_, vehicle_y_, vehicle_theta_;
    
    // Random number generation
    std::mt19937 generator_;
    std::normal_distribution<double> noise_dist_;
    std::uniform_real_distribution<double> obstacle_dist_;
    std::uniform_real_distribution<double> range_dist_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FakeLidarNode>());
    rclcpp::shutdown();
    return 0;
}