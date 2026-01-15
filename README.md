# STEVE Pan-Tilt Controller (ROS2)

This package provides control for the Dynamixel XH430-W250 pan-tilt unit on the STEVE robot. It includes support for both real hardware and Gazebo simulation, and a real-time JointState publisher for TF visualization.

## Installation

Ensure you have the Dynamixel SDK installed:

```bash
pip install dynamixel-sdk
```

##  Usage

### 1. Launching the Controller (Real Hardware)

Before running on the real robot, ensure you have permissions for the USB port:
```bash
sudo chmod 666 /dev/ttyUSB0
```

To start the controller in real hardware mode:
```bash
ros2 launch steve_pan_tilt_controller steve_pan_tilt_controller.launch.py use_sim:=false
```

**Parameters:**
- `pan_goal_position`: Initial pan angle in degrees (Default: 75.0)
- `tilt_goal_position`: Initial tilt angle in degrees (Default: 180.0)
- `profile_velocity`: Movement speed (Default: 50)
- `profile_acceleration`: Movement smoothness (Default: 10)

Example with custom goals:
```bash
ros2 launch steve_pan_tilt_controller steve_pan_tilt_controller.launch.py use_sim:=false pan_goal_position:=90 tilt_goal_position:=200
```

### 2. Simulation Testing

To test in simulation (Gazebo):

1. **Start the simulation environment**:
   ```bash
   ros2 launch neo_simulation2 simulation.launch.py my_robot:=mmo_700 world:=neo_workshop arm_type:=ur5e include_pan_tilt:=true
   ```

2. **Run the controller in simulation mode**:
   ```bash
   ros2 launch steve_pan_tilt_controller steve_pan_tilt_controller.launch.py use_sim:=true pan_goal_position:=90 tilt_goal_position:=180
   ```


## Hardware Details
- **Motors**: Dynamixel XL430-W250
- **Pan Limits**: [90°, 270°]
- **Tilt Limits**: [90°, 270°]
- **Default Port**: `/dev/ttyUSB0` (Configurable in `config/dynamixel_motors.yaml`)
