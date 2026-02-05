#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import yaml
import time
import math
from dynamixel_sdk import * # Import all handlers

from steve_pan_tilt_controller.trajectory import create_single_goal_trajectory, create_elliptical_trajectory


class PanTiltController(Node):
    """ROS2 node for controlling Dynamixel pan-tilt motors using SyncWrite for smooth motion."""

    # Dynamixel Control Table Addresses
    ADDR_HARDWARE_ERROR_STATUS = 70
    ADDR_TORQUE_ENABLE = 64
    ADDR_PROFILE_ACCELERATION = 108
    ADDR_PROFILE_VELOCITY = 112
    ADDR_GOAL_POSITION = 116
    ADDR_PRESENT_POSITION = 132

    def __init__(self):
        super().__init__('steve_pan_tilt_controller')
        
        # Init vars to prevent __del__ error if init fails
        self.use_sim = False
        self.port_handler = None
        self.current_pan = 0.0
        self.current_tilt = 0.0
        self.has_feedback = False
        self.hardware_initialized = False  # Track if hardware init succeeded

        # Declare parameters
        self.declare_parameter('config_file', '')
        self.declare_parameter('use_sim', False)
        
        # Legacy params
        self._declare_flexible_parameter('pan_goal_position', 180.0)
        self._declare_flexible_parameter('tilt_goal_position', 180.0)
        
        # New sweep params (Default [0.0] implies 0-centered, unlike legacy 180-centered)
        # Use native list (DOUBLE_ARRAY) instead of string
        self.declare_parameter('pan_goals', [0.0]) 
        self.declare_parameter('tilt_goals', [0.0])
        self.declare_parameter('sweep_speed', 15.0)
        self.declare_parameter('log_feedback', True)
        
        self.declare_parameter('profile_velocity', 50)
        self.declare_parameter('profile_acceleration', 20)
        self.declare_parameter('publish_rate', 20.0)

        # Get params
        config_file = self.get_parameter('config_file').value
        self.use_sim = self.get_parameter('use_sim').value
        legacy_pan = self._get_flexible_float('pan_goal_position')
        legacy_tilt = self._get_flexible_float('tilt_goal_position')
        
        self.log_fb = self.get_parameter('log_feedback').value
        self.profile_velocity = self.get_parameter('profile_velocity').value
        self.profile_acceleration = self.get_parameter('profile_acceleration').value
        self.publish_rate = self.get_parameter('publish_rate').value
        
        self.joint_names = ['pan_tilt_pan_motor_joint', 'pan_tilt_tilt_motor_joint']
        self.sweep_speed_rad = math.radians(self.get_parameter('sweep_speed').value)

        # Parse Goals / Modes
        p_goals_str = self.get_parameter('pan_goals').value
        t_goals_str = self.get_parameter('tilt_goals').value
        
        self.pan_c, self.pan_a = self._resolve_goals(p_goals_str, legacy_pan, "Pan")
        self.tilt_c, self.tilt_a = self._resolve_goals(t_goals_str, legacy_tilt, "Tilt")
        
        self.sweep = (self.pan_a > 1e-6 or self.tilt_a > 1e-6)

        # Init (load motor configs before validating sweep)
        if self.use_sim:
            self.init_sim()
            # No hardware limits in sim
        else:
            self.init_real(config_file)
            # Validate sweep goals against configured hardware limits
            self._validate_sweep_limits()

        # Only create timers and start control if hardware initialized successfully
        if not self.use_sim and self.hardware_initialized:
            self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_joint_states)
        
        self.cmd_sub = self.create_subscription(Float64MultiArray, '/pan_tilt/command', self.command_callback, 10)
        
        if self.use_sim or self.hardware_initialized:
            self.get_logger().info(f'Pan-Tilt Controller initialized (Mode: {"SIM" if self.use_sim else "REAL"}, Sweep: {self.sweep})')
        else:
            self.get_logger().error('Pan-Tilt Controller failed to initialize hardware!')
            return
        
        # Start Control (only if hardware is ready)
        if self.sweep and (self.use_sim or self.hardware_initialized):
            if self.use_sim:
                self.sweep_start_timer = self.create_timer(2.0, self.start_sweep_sim_loop)
            else:
                self.start_sweep_real_loop()

        # Feedback Logging for Sim
        if self.use_sim and self.log_fb:
            self.create_subscription(JointState, 'joint_states', self.sim_feedback_callback, 10)
            self.last_log_time = self.get_clock().now()

    def sim_feedback_callback(self, msg):
        now = self.get_clock().now()
        # Always update internal state for control logic
        try:
            p_idx = msg.name.index('pan_tilt_pan_motor_joint')
            t_idx = msg.name.index('pan_tilt_tilt_motor_joint')
            self.current_pan = msg.position[p_idx]
            self.current_tilt = msg.position[t_idx]
            self.has_feedback = True
            
            # Log only if enabled and throttled
            if self.log_fb and (now - self.last_log_time).nanoseconds > 0.5 * 1e9:
                self.get_logger().info(f'Feedback: Pan={math.degrees(self.current_pan):.1f}, Tilt={math.degrees(self.current_tilt):.1f}')
                self.last_log_time = now
        except ValueError: pass

    def _ensure_list(self, val):
        if isinstance(val, str):
            try: return [float(x) for x in val.replace('[','').replace(']','').split(',') if x.strip()]
            except: return [0.0]
        return [float(x) for x in (val if isinstance(val, (list, tuple)) else [val])]

    def _resolve_goals(self, goals_str, legacy_val, name):
        goals = self._ensure_list(goals_str)
        # Check if user explicitly set goals (simplistic check: is it [0.0]?)
        # If goals is [0.0] (default) and legacy is NOT 180.0 (default), use legacy 180-centered value converted to 0-centered
        is_default_goal = (len(goals)==1 and abs(goals[0]) < 1e-6)
        is_legacy_set = (abs(legacy_val - 180.0) > 0.01)
        
        if is_default_goal and is_legacy_set:
            val = math.radians(legacy_val - 180.0)
            self.get_logger().info(f"{name}: Fixed (Legacy) {legacy_val} deg -> {val:.2f} rad")
            return val, 0.0
            
        return self._parse_mode_list(goals, name)
        
    def _parse_mode_list(self, goals, name):
        if not goals: return 0.0, 0.0
        if len(goals) == 1:
            self.get_logger().info(f"{name}: Fixed {goals[0]}")
            return math.radians(goals[0]), 0.0
        c = (goals[0] + goals[1]) / 2.0
        a = abs(goals[0] - goals[1]) / 2.0
        self.get_logger().info(f"{name}: Sweep {goals} (C={c:.1f}, A={a:.1f})")
        return math.radians(c), math.radians(a)

    def init_sim(self):
        """Initialize connection to Gazebo trajectory controller."""
        self.sim_client = ActionClient(self, FollowJointTrajectory, '/pan_tilt_controller/follow_joint_trajectory')
        
        # Only hold position if NOT sweeping. If sweeping, the loop will take over.
        if not self.sweep:
            self.timer = self.create_timer(1.0, lambda: self.send_sim_command(self.pan_c, self.tilt_c))

    def init_real(self, config_file):
        """Initialize connection to actual Dynamixel hardware with SyncWrite support."""
        if not config_file:
            self.get_logger().error('No config_file parameter provided!')
            return

        try:
            with open(config_file, 'r') as file:
                config = yaml.safe_load(file)
        except Exception as e:
            self.get_logger().error(f'Failed to load config file: {e}')
            return

        self.motor_ids = config['motors']['ids']
        self.baud_rate = config['motors']['baud_rate']
        self.device_name = config['motors']['device_name']
        
        # Parse motor-specific position limits from config
        self.motor_configs = self._parse_motor_limits(config)
        
        self.port_handler = PortHandler(self.device_name)
        self.packet_handler = PacketHandler(2.0)

        # Check if port is already open and close it
        if self.port_handler.is_open:
            self.get_logger().warn(f'Port {self.device_name} already open, closing first...')
            self.port_handler.closePort()
            time.sleep(0.5)

        if not self.port_handler.openPort():
            self.get_logger().error(f'Failed to open port {self.device_name}. Check if device is connected and not in use by another process.')
            self.get_logger().error('You may need to: 1) Check USB connection, 2) Close other programs using the port, or 3) Run: sudo chmod 666 /dev/ttyUSB0')
            return

        if not self.port_handler.setBaudRate(self.baud_rate):
            self.get_logger().error(f'Failed to set baud rate {self.baud_rate}')
            self.port_handler.closePort()
            return

        # Initialize SyncWrite and SyncRead handlers for efficiency
        self.group_sync_write = GroupSyncWrite(self.port_handler, self.packet_handler, self.ADDR_GOAL_POSITION, 4)
        self.group_sync_read = GroupSyncRead(self.port_handler, self.packet_handler, self.ADDR_PRESENT_POSITION, 4)
        
        for motor_id in self.motor_ids:
            self.group_sync_read.addParam(motor_id)

        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        
        # Try to initialize hardware
        if self.init_hardware():
            self.hardware_initialized = True
            self.get_logger().info('Hardware initialized successfully')
        else:
            self.get_logger().error('Hardware initialization failed!')
            self.port_handler.closePort()

    def init_hardware(self):
        """Initialize motor settings and enable torque."""
        success_count = 0
        for motor_id in self.motor_ids:
            try:
                # Check and handle hardware errors
                if self._check_hardware_error(motor_id):
                    self.get_logger().warn(f'Motor {motor_id} has hardware error, attempting reboot...')
                    reboot_result = self.packet_handler.reboot(self.port_handler, motor_id)
                    if reboot_result != COMM_SUCCESS:
                        self.get_logger().error(f'Failed to reboot motor {motor_id}: {self.packet_handler.getTxRxResult(reboot_result)}')
                        continue
                    time.sleep(3.5)
                    
                    # Re-check after reboot
                    if self._check_hardware_error(motor_id):
                        self.get_logger().error(f'Motor {motor_id} still has hardware error after reboot')
                        continue
                
                # Set profiles for smooth motion
                vel_result, vel_err = self.packet_handler.write4ByteTxRx(self.port_handler, motor_id, self.ADDR_PROFILE_VELOCITY, int(self.profile_velocity))
                acc_result, acc_err = self.packet_handler.write4ByteTxRx(self.port_handler, motor_id, self.ADDR_PROFILE_ACCELERATION, int(self.profile_acceleration))
                torque_result, torque_err = self.packet_handler.write1ByteTxRx(self.port_handler, motor_id, self.ADDR_TORQUE_ENABLE, 1)
                
                if vel_result != COMM_SUCCESS or acc_result != COMM_SUCCESS or torque_result != COMM_SUCCESS:
                    self.get_logger().error(f'Failed to configure motor {motor_id}')
                    continue
                    
                success_count += 1
                self.get_logger().info(f'Motor {motor_id} initialized successfully')

            except Exception as e:
                self.get_logger().error(f"Error during motor {motor_id} initialization: {str(e)}")
                continue

        # Only proceed with initial movement if at least one motor succeeded
        if success_count == 0:
            self.get_logger().error('Failed to initialize any motors!')
            return False
            
        if success_count < len(self.motor_ids):
            self.get_logger().warn(f'Only {success_count}/{len(self.motor_ids)} motors initialized successfully')

        # Initial move using SyncWrite (Only if fixed mode, or to start of sweep)
        try:
            pan_ticks = int(2048 + (self.pan_c * 4096 / (2*math.pi)))
            tilt_ticks = int(2048 + (self.tilt_c * 4096 / (2*math.pi)))
            self._sync_write_position(pan_ticks, tilt_ticks)
        except Exception as e:
            self.get_logger().error(f'Failed to set initial position: {e}')
            return False
            
        return True

    
    def _parse_motor_limits(self, config):
        """Parse motor position limits from config and convert to ticks."""
        motor_configs = {}
        
        if 'dxl_info' not in config['motors']:
            return motor_configs
            
        for motor_id in self.motor_ids:
            # Match config entry by ID (e.g., "id_1" for motor 1)
            config_key = f'id_{motor_id}'
            if config_key in config['motors']['dxl_info']:
                info = config['motors']['dxl_info'][config_key]
                
                # Convert degree limits to ticks (Dynamixel XL430: 4096 ticks = 360°)
                min_deg = info.get('min_position_limit', 0)
                max_deg = info.get('max_position_limit', 360)
                
                motor_configs[motor_id] = {
                    'min_ticks': int(min_deg * 4096 / 360.0),
                    'max_ticks': int(max_deg * 4096 / 360.0)
                }
                
                self.get_logger().info(
                    f'Motor {motor_id}: Position limits [{min_deg}°, {max_deg}°] '
                    f'({motor_configs[motor_id]["min_ticks"]}-{motor_configs[motor_id]["max_ticks"]} ticks)'
                )
        
        return motor_configs
    
    def _validate_sweep_limits(self):
        """Validate that sweep goals don't exceed configured hardware limits."""
        if not self.sweep or not self.motor_configs:
            return
            
        # Convert limits from ticks to radians (centered at 0)
        # Formula: radians = (ticks - 2048) * 2π / 4096
        for idx, (motor_id, axis_name, center, amplitude) in enumerate([
            (self.motor_ids[0], "Pan", self.pan_c, self.pan_a),
            (self.motor_ids[1], "Tilt", self.tilt_c, self.tilt_a)
        ]):
            if motor_id in self.motor_configs:
                cfg = self.motor_configs[motor_id]
                min_rad = (cfg['min_ticks'] - 2048) * (2 * math.pi) / 4096
                max_rad = (cfg['max_ticks'] - 2048) * (2 * math.pi) / 4096
                
                # Check if sweep range exceeds limits
                sweep_min = center - amplitude
                sweep_max = center + amplitude
                
                if sweep_min < min_rad or sweep_max > max_rad:
                    self.get_logger().warn(
                        f'{axis_name} sweep [{math.degrees(sweep_min):.1f}°, {math.degrees(sweep_max):.1f}°] '
                        f'EXCEEDS configured limits [{math.degrees(min_rad):.1f}°, {math.degrees(max_rad):.1f}°]. '
                        f'Positions will be clamped for safety!'
                    )
    
    # --- Sweep Loop Methods ---

    def start_sweep_sim_loop(self):
        """Start sweep motion for simulation."""
        if hasattr(self, 'timer') and self.timer:
            self.timer.cancel()
        if hasattr(self, 'sweep_start_timer') and self.sweep_start_timer:
            self.sweep_start_timer.cancel()
            self.sweep_start_timer = None
        
        # Move to sweep start position
        start_pan = self.pan_c + self.pan_a
        start_tilt = self.tilt_c
        curr_p = self.current_pan if self.has_feedback else 0.0
        curr_t = self.current_tilt if self.has_feedback else 0.0
        
        self.get_logger().info(
            f'Approaching Sweep Start: ({math.degrees(curr_p):.1f}°, {math.degrees(curr_t):.1f}°) -> '
            f'({math.degrees(start_pan):.1f}°, {math.degrees(start_tilt):.1f}°)'
        )
        
        points, duration = create_single_goal_trajectory(curr_p, curr_t, start_pan, start_tilt, self.sweep_speed_rad)
        self._send_traj(points, on_success=self._execute_sim_sweep_cycle)

    def _execute_sim_sweep_cycle(self):
        """Execute one sweep cycle for simulation."""
        points, period = create_elliptical_trajectory(
            self.pan_c, self.pan_a, self.tilt_c, self.tilt_a, self.sweep_speed_rad
        )
        self.get_logger().debug(f'Sweep cycle (T={period:.1f}s)')
        self._send_traj(points, on_success=self._execute_sim_sweep_cycle)

    def _send_traj(self, points, on_success):
        """Send trajectory to simulation controller."""
        if not self.sim_client.server_is_ready():
            self.sim_client.wait_for_server(timeout_sec=1.0)
            
        msg = FollowJointTrajectory.Goal()
        msg.trajectory.joint_names = self.joint_names
        msg.trajectory.points = points
        
        future = self.sim_client.send_goal_async(msg)
        future.add_done_callback(lambda f: self._on_goal(f, on_success))

    def _on_goal(self, future, on_success):
        """Handle goal acceptance."""
        gh = future.result()
        if not gh.accepted:
            self.get_logger().warn('Goal Rejected')
            return
        gh.get_result_async().add_done_callback(lambda f: self._on_res(f, on_success))

    def _on_res(self, future, on_success):
        """Handle goal result and loop sweep."""
        res = future.result().result
        if res.error_code == 0:
            on_success()
        else:
            self.get_logger().error(f'Goal Failed: {res.error_code}')

    def start_sweep_real_loop(self):
        """Start sweep motion for hardware."""
        max_amp = max(self.pan_a, self.tilt_a)
        self.period = 2.0 if max_amp < 1e-6 else 2 * math.pi / (self.sweep_speed_rad / max_amp)
        
        # Move to sweep start
        start_pan = self.pan_c + self.pan_a
        start_tilt = self.tilt_c
        pan_ticks = int(2048 + (start_pan * 4096 / (2 * math.pi)))
        tilt_ticks = int(2048 + (start_tilt * 4096 / (2 * math.pi)))
        
        self.get_logger().info('Approaching sweep start (hardware)')
        self._sync_write_position(pan_ticks, tilt_ticks)
        
        # Schedule sweep loop
        duration = (max_amp / self.sweep_speed_rad) + 1.0 if self.sweep_speed_rad > 0 else 2.0
        self.create_timer(duration, self._start_real_analytic_loop)

    def _start_real_analytic_loop(self):
        """Start continuous sweep for hardware."""
        self.get_logger().info(f'Starting sweep (T={self.period:.1f}s)')
        self.sweep_start_time = self.get_clock().now()
        self.control_timer = self.create_timer(1.0/self.publish_rate, self._real_sweep_tick)
        
    def _real_sweep_tick(self):
        """Compute and send next position for hardware sweep."""
        t = (self.get_clock().now() - self.sweep_start_time).nanoseconds / 1e9
        omega = 2 * math.pi / self.period if self.period > 1e-6 else 0
        
        p_rad = self.pan_c + self.pan_a * math.cos(omega * t)
        t_rad = self.tilt_c + self.tilt_a * math.sin(omega * t)
        
        pan_ticks = int(2048 + (p_rad * 4096 / (2 * math.pi)))
        tilt_ticks = int(2048 + (t_rad * 4096 / (2 * math.pi)))
        
        self._sync_write_position(pan_ticks, tilt_ticks)

    def _sync_write_position(self, pan_ticks, tilt_ticks):
        """Write both motor positions simultaneously with safety limits."""
        # Don't attempt write if hardware not initialized
        if not self.use_sim and not self.hardware_initialized:
            return
            
        # Apply configured position limits (defaults to full range if not configured)
        pan_cfg = self.motor_configs.get(self.motor_ids[0], {'min_ticks': 0, 'max_ticks': 4095})
        tilt_cfg = self.motor_configs.get(self.motor_ids[1], {'min_ticks': 0, 'max_ticks': 4095})

        pan_ticks = max(pan_cfg['min_ticks'], min(pan_cfg['max_ticks'], pan_ticks))
        tilt_ticks = max(tilt_cfg['min_ticks'], min(tilt_cfg['max_ticks'], tilt_ticks))

        # Encode positions to bytes
        param_pan = [DXL_LOBYTE(DXL_LOWORD(pan_ticks)), DXL_HIBYTE(DXL_LOWORD(pan_ticks)), 
                     DXL_LOBYTE(DXL_HIWORD(pan_ticks)), DXL_HIBYTE(DXL_HIWORD(pan_ticks))]
        
        param_tilt = [DXL_LOBYTE(DXL_LOWORD(tilt_ticks)), DXL_HIBYTE(DXL_LOWORD(tilt_ticks)), 
                      DXL_LOBYTE(DXL_HIWORD(tilt_ticks)), DXL_HIBYTE(DXL_HIWORD(tilt_ticks))]
        
        # Ensure we have enough IDs
        if len(self.motor_ids) >= 2:
            self.group_sync_write.addParam(self.motor_ids[1], param_pan)
            self.group_sync_write.addParam(self.motor_ids[0], param_tilt)
            
            comm_result = self.group_sync_write.txPacket()
            if comm_result != COMM_SUCCESS:
                self.get_logger().warn(f'SyncWrite Failed: {self.packet_handler.getTxRxResult(comm_result)}')
                
            self.group_sync_write.clearParam()
        else:
            self.get_logger().error("Hardware ID Mismatch: Need at least 2 motor IDs configured.")

    def command_callback(self, msg):
        """Handle position commands (radians) and send via SyncWrite."""
        if len(msg.data) < 2:
            return

        if self.use_sim:
            self.send_sim_command(msg.data[0], msg.data[1])
        else:
            # Conversion from radians to ticks (180 center)
            pan_ticks = int(2048 + (msg.data[0] * 4096 / (2 * math.pi)))
            tilt_ticks = int(2048 + (msg.data[1] * 4096 / (2 * math.pi)))
            self._sync_write_position(pan_ticks, tilt_ticks)

    def publish_joint_states(self):
        """Optimized joint state publishing using SyncRead."""
        # Don't attempt to read if hardware not initialized
        if not self.hardware_initialized:
            return
            
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = self.joint_names
        
        try:
            dxl_comm_result = self.group_sync_read.txRxPacket()
            if dxl_comm_result != COMM_SUCCESS:
                # self.get_logger().warn('SyncRead Failed') # Optional: reduce spam
                return

            positions = []
            for motor_id in self.motor_ids:
                if self.group_sync_read.isAvailable(motor_id, self.ADDR_PRESENT_POSITION, 4):
                    pos_ticks = self.group_sync_read.getData(motor_id, self.ADDR_PRESENT_POSITION, 4)
                    positions.append((pos_ticks - 2048) * (2 * math.pi / 4096))
                else:
                    positions.append(0.0)
            
            joint_state.position = positions
            self.joint_state_pub.publish(joint_state)
        except Exception as e:
            self.get_logger().error(f'Error publishing joint states: {e}')
            # Disable timer to prevent further errors
            if hasattr(self, 'timer') and self.timer:
                self.timer.cancel()

    def send_sim_command(self, pan_rad, tilt_rad, duration=1.5):
        """Send trajectory goal to simulation Action Server."""
        from control_msgs.action import FollowJointTrajectory
        from trajectory_msgs.msg import JointTrajectoryPoint
        from builtin_interfaces.msg import Duration

        if not self.sim_client.server_is_ready():
            return
        
        if hasattr(self, 'timer') and self.timer:
            self.timer.cancel()

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = [float(pan_rad), float(tilt_rad)]
        point.time_from_start = Duration(sec=int(duration), nanosec=int((duration % 1) * 1e9))
        goal_msg.trajectory.points = [point]
        self.sim_client.send_goal_async(goal_msg)

    def _check_hardware_error(self, motor_id: int) -> bool:
        error_status, result, error = self.packet_handler.read1ByteTxRx(self.port_handler, motor_id, self.ADDR_HARDWARE_ERROR_STATUS)
        return result != COMM_SUCCESS or error_status != 0

    def _declare_flexible_parameter(self, name, default_value):
        try:
            self.declare_parameter(name, float(default_value))
        except rclpy.exceptions.InvalidParameterTypeException:
            self.declare_parameter(name, int(default_value))

    def _get_flexible_float(self, name):
        param = self.get_parameter(name)
        return float(param.value) if param.value is not None else 0.0

    def destroy_node(self):
        # Graceful shutdown: Disable Torque
        if not self.use_sim and hasattr(self, 'port_handler') and self.port_handler:
            self.get_logger().info("Shutting down: Disabling Torque...")
            for motor_id in self.motor_ids:
                self.packet_handler.write1ByteTxRx(self.port_handler, motor_id, self.ADDR_TORQUE_ENABLE, 0)
            self.port_handler.closePort()
        super().destroy_node()

    def __del__(self):
        pass # Handle in destroy_node


def main(args=None):
    rclpy.init(args=args)
    node = PanTiltController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # Only shutdown if context is still valid
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
