# NAR - Semi-Autonomous Electric Tractor Control System

This ROS 2 workspace contains a modular semi-autonomous control system for electric tractors, built using ROS 2 Jazzy. The system integrates perception, decision-making, and actuation layers to enable assisted navigation and control.

## System Overview

The NAR (Navigation, Autonomy, and Robotics) system provides:

- **Modular Architecture**: Separate ROS 2 nodes for different vehicle subsystems
- **Simulation Environment**: Pure ROS 2-based simulation without Gazebo dependency
- **PID-based Control**: Robust navigation and steering control algorithms
- **Perception Simulation**: Fake sensor data for testing without physical hardware
- **Safety Systems**: Integrated braking and emergency stop mechanisms

## Package Structure

### Core Control Packages

- **`vehicle_dynamics`**: Vehicle dynamics modeling and state estimation
- **`pid_controller`**: PID-based navigation and steering control
- **`braking_system`**: Braking control and safety mechanisms
- **`force_actuator`**: Force actuation for steering and propulsion

### Simulation Packages

- **`fake_lidar`**: Simulated LiDAR sensor data publisher
- **`setpoint_publisher`**: Waypoint and setpoint generation for testing
- **`tractor_simulation`**: Main simulation framework and integration

## Quick Start

### Prerequisites

- ROS 2 Jazzy installation
- Ubuntu 22.04 or compatible
- C++ compiler with C++17 support

### Build Instructions

```bash
cd ~/nar
colcon build
source install/setup.bash
```

### Running the Simulation

1. **Start the vehicle dynamics simulation:**
   ```bash
   ros2 run vehicle_dynamics vehicle_dynamics_node
   ```

2. **Launch the PID controller:**
   ```bash
   ros2 run pid_controller pid_controller_node
   ```

3. **Start sensor simulation:**
   ```bash
   ros2 run fake_lidar fake_lidar_node
   ```

4. **Generate setpoints:**
   ```bash
   ros2 run setpoint_publisher setpoint_publisher_node --ros-args -p waypoint_mode:=circle
   ```

### Available Waypoint Modes

- `manual`: Default waypoints for basic testing
- `circle`: Circular path pattern
- `square`: Square/rectangular field pattern
- `line`: Straight line navigation
- `field_pattern`: Agricultural back-and-forth pattern

## System Architecture

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│ Setpoint        │    │ Fake LiDAR       │    │ Other Sensors   │
│ Publisher       │    │ Simulation       │    │ (Future)        │
└─────┬───────────┘    └──────┬───────────┘    └─────────────────┘
      │                       │                                  
      │ /setpoint             │ /scan                           
      │                       │                                  
      v                       v                                  
┌─────────────────────────────────────────────────────────────────┐
│                    PID Controller                                │
│                  (/cmd_vel output)                               │
└─────────────────────────┬───────────────────────────────────────┘
                          │                                        
                          │ /cmd_vel                              
                          │                                        
                          v                                        
┌─────────────────────────────────────────────────────────────────┐
│                 Vehicle Dynamics                                 │
│            (Bicycle Model Simulation)                           │
└─────────────────────────┬───────────────────────────────────────┘
                          │                                        
                          │ /vehicle/pose, /vehicle/velocity      
                          │                                        
                          └─────────────────────────────────────────
```

## Key Topics

- `/setpoint` - Target waypoints (geometry_msgs/PointStamped)
- `/cmd_vel` - Velocity commands (geometry_msgs/Twist)
- `/vehicle/pose` - Vehicle pose estimation (geometry_msgs/PoseWithCovarianceStamped)
- `/vehicle/velocity` - Vehicle velocity (geometry_msgs/Twist)
- `/scan` - LiDAR scan data (sensor_msgs/LaserScan)
- `/planned_path` - Planned trajectory (nav_msgs/Path)

## Configuration

Each node supports extensive parameter configuration:

### Vehicle Dynamics Parameters
- `wheelbase`: Vehicle wheelbase (default: 2.5m)
- `max_speed`: Maximum linear velocity (default: 5.0 m/s)
- `max_steering_angle`: Maximum steering angle (default: 45°)

### PID Controller Parameters
- `linear_kp`, `linear_ki`, `linear_kd`: Linear velocity PID gains
- `angular_kp`, `angular_ki`, `angular_kd`: Angular velocity PID gains
- `position_tolerance`: Position tolerance for waypoint completion

### LiDAR Simulation Parameters
- `scan_rate`: Scan frequency (default: 10 Hz)
- `range_max`: Maximum detection range (default: 30m)
- `noise_stddev`: Measurement noise standard deviation

## Safety Features

- **Emergency Stop**: Immediate velocity command zeroing
- **Steering Limits**: Hardware-consistent steering angle constraints
- **Speed Limiting**: Configurable maximum speeds for different operating modes
- **Collision Avoidance**: LiDAR-based obstacle detection (future enhancement)

## Development

### Adding New Packages

```bash
cd ~/nar/src
ros2 pkg create --build-type ament_cmake your_package_name --dependencies rclcpp std_msgs
```

### Testing

The system includes comprehensive testing capabilities:

```bash
# Run all tests
cd ~/nar
colcon test

# Run specific package tests  
colcon test --packages-select vehicle_dynamics
```

## Future Enhancements

- **Advanced Path Planning**: A* or RRT-based path planning algorithms
- **Sensor Fusion**: Integration of multiple sensor types (GPS, IMU, cameras)
- **Machine Learning**: AI-based decision making for complex scenarios  
- **Real Hardware Interface**: Integration with actual tractor hardware
- **Fleet Management**: Multi-vehicle coordination and task allocation

## Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Contact

For questions or support, please open an issue in the repository or contact the development team.