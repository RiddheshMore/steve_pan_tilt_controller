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
   ros2 launch steve_simulation simulation.launch.py my_robot:=mmo_700 world:=neo_workshop arm_type:=ur5e include_pan_tilt:=true
   ```

2. **Run the controller in simulation mode**:
   ```bash
   ros2 launch steve_pan_tilt_controller steve_pan_tilt_controller.launch.py use_sim:=true pan_goal_position:=90 tilt_goal_position:=180
   ```

### Sweep Mode
- **Controller**: `steve_pan_tilt_controller` (Sim) or Analytic Loop (Real)
- **Features**: Smooth approach from current position to sweep start; Continuous elliptical trajectory.
- **Parameters**: 
  - `pan_goals`: `[min, max]` (degrees, 0-centered)
  - `tilt_goals`: `[min, max]` (degrees, 0-centered)
  - `sweep_speed`: Degrees/second
- **Example**: `ros2 launch steve_pan_tilt_controller steve_pan_tilt_controller.launch.py sweep_speed:=20.0 pan_goals:="[-30, 30]"`

### 3. Sweep Mode Usage

The controller now supports continuous elliptical sweeping movement.

**Sweep Parameters:**
- `pan_goals`: List of angles. `[0.0]` = Center. `[-20, 20]` = Sweep from -20 to 20.
- `tilt_goals`: List of angles. `[0.0]` = Center. `[-10, 10]` = Sweep from -10 to 10.
- `sweep_speed`: Speed in degrees/second.
- `log_feedback`: Enable position logging (default: true).

**Example (Hardware & Simulation):**
```bash
ros2 launch steve_pan_tilt_controller steve_pan_tilt_controller.launch.py use_sim:=true pan_goals:="[-40.0, 40.0]" tilt_goals:="[10.0, 30.0]" sweep_speed:=15.0
```
*(Use `use_sim:=true` for simulation)*

**Note on Coordinates:**
- `pan_goals`/`tilt_goals` are **0-centered** relative to the robot's forward direction.
  - Fixed position: `pan_goals:="[30.0]"` → 30° right
  - Sweep: `pan_goals:="[-30.0, 30.0]"` → sweep ±30°
- `pan_goal_position`/`tilt_goal_position` (Legacy) are **180-centered** (Raw Dynamixel coordinates).
- **Hardware limits** (configured in `config/dynamixel_motors.yaml`): Default [90°, 270°] = ±90° from center

## Hardware Details
- **Motors**: Dynamixel XL430-W250
- **Pan Limits**: [90°, 270°]
- **Tilt Limits**: [90°, 270°]
- **Default Port**: `/dev/ttyUSB0` (Configurable in `config/dynamixel_motors.yaml`)
