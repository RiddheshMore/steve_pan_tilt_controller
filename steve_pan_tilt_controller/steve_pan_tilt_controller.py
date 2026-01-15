#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import yaml
import time
import math
from dynamixel_sdk import * # Import all handlers


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

        # Declare parameters
        self.declare_parameter('config_file', '')
        self.declare_parameter('use_sim', False)
        self._declare_flexible_parameter('pan_goal_position', 180.0)
        self._declare_flexible_parameter('tilt_goal_position', 180.0)
        self.declare_parameter('profile_velocity', 50)
        self.declare_parameter('profile_acceleration', 20)
        self.declare_parameter('publish_rate', 20.0)

        # Get parameters
        config_file = self.get_parameter('config_file').value
        self.use_sim = self.get_parameter('use_sim').value
        self.pan_goal_position = self._get_flexible_float('pan_goal_position')
        self.tilt_goal_position = self._get_flexible_float('tilt_goal_position')
        self.profile_velocity = self.get_parameter('profile_velocity').value
        self.profile_acceleration = self.get_parameter('profile_acceleration').value
        self.publish_rate = self.get_parameter('publish_rate').value

        self.joint_names = ['pan_tilt_pan_motor_joint', 'pan_tilt_tilt_motor_joint']

        if self.use_sim:
            self.init_sim()
        else:
            self.init_real(config_file)

        if not self.use_sim:
            self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_joint_states)
        
        self.cmd_sub = self.create_subscription(Float64MultiArray, '/pan_tilt/command', self.command_callback, 10)
        self.get_logger().info(f'Pan-Tilt Controller initialized (Mode: {"SIM" if self.use_sim else "REAL"})')

    def init_sim(self):
        """Initialize connection to Gazebo trajectory controller."""
        from rclpy.action import ActionClient
        from control_msgs.action import FollowJointTrajectory
        
        self.sim_client = ActionClient(self, FollowJointTrajectory, '/pan_tilt_controller/follow_joint_trajectory')
        pan_rad = (self.pan_goal_position - 180.0) * (math.pi / 180.0)
        tilt_rad = (self.tilt_goal_position - 180.0) * (math.pi / 180.0)
        self.timer = self.create_timer(1.0, lambda: self.send_sim_command(pan_rad, tilt_rad))

    def init_real(self, config_file):
        """Initialize connection to actual Dynamixel hardware with SyncWrite support."""
        if not config_file:
            self.get_logger().error('No config_file parameter provided!')
            return

        with open(config_file, 'r') as file:
            config = yaml.safe_load(file)

        self.motor_ids = config['motors']['ids']
        self.baud_rate = config['motors']['baud_rate']
        self.device_name = config['motors']['device_name']
        
        self.port_handler = PortHandler(self.device_name)
        self.packet_handler = PacketHandler(2.0)

        if not self.port_handler.openPort():
            self.get_logger().error(f'Failed to open port {self.device_name}')
            return

        if not self.port_handler.setBaudRate(self.baud_rate):
            self.get_logger().error(f'Failed to set baud rate {self.baud_rate}')
            return

        # Initialize SyncWrite and SyncRead handlers for efficiency
        self.group_sync_write = GroupSyncWrite(self.port_handler, self.packet_handler, self.ADDR_GOAL_POSITION, 4)
        self.group_sync_read = GroupSyncRead(self.port_handler, self.packet_handler, self.ADDR_PRESENT_POSITION, 4)
        
        for motor_id in self.motor_ids:
            self.group_sync_read.addParam(motor_id)

        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.init_hardware()

    def init_hardware(self):
        """Initialize motor settings and enable torque."""
        for motor_id in self.motor_ids:
            try:
                if self._check_hardware_error(motor_id):
                    self.get_logger().warn(f'Motor {motor_id} has hardware error, attempting reboot...')
                    self.packet_handler.reboot(self.port_handler, motor_id)
                    time.sleep(3.5)
                
                # Set profiles for smooth motion
                self.packet_handler.write4ByteTxRx(self.port_handler, motor_id, self.ADDR_PROFILE_VELOCITY, int(self.profile_velocity))
                self.packet_handler.write4ByteTxRx(self.port_handler, motor_id, self.ADDR_PROFILE_ACCELERATION, int(self.profile_acceleration))
                self.packet_handler.write1ByteTxRx(self.port_handler, motor_id, self.ADDR_TORQUE_ENABLE, 1)

            except Exception as e:
                self.get_logger().error(f"Error during motor {motor_id} initialization: {str(e)}")

        # Initial move using SyncWrite
        pan_ticks = int(self.pan_goal_position * 4096 / 360.0)
        tilt_ticks = int(self.tilt_goal_position * 4096 / 360.0)
        self._sync_write_position(pan_ticks, tilt_ticks)

    def _sync_write_position(self, pan_ticks, tilt_ticks):
        """Write both motor positions simultaneously."""
        # Pan
        param_pan = [DXL_LOBYTE(DXL_LOWORD(pan_ticks)), DXL_HIBYTE(DXL_LOWORD(pan_ticks)), 
                     DXL_LOBYTE(DXL_HIWORD(pan_ticks)), DXL_HIBYTE(DXL_HIWORD(pan_ticks))]
        # Tilt
        param_tilt = [DXL_LOBYTE(DXL_LOWORD(tilt_ticks)), DXL_HIBYTE(DXL_LOWORD(tilt_ticks)), 
                      DXL_LOBYTE(DXL_HIWORD(tilt_ticks)), DXL_HIBYTE(DXL_HIWORD(tilt_ticks))]
        
        self.group_sync_write.addParam(self.motor_ids[1], param_pan)
        self.group_sync_write.addParam(self.motor_ids[0], param_tilt)
        
        self.group_sync_write.txPacket()
        self.group_sync_write.clearParam()

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
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = self.joint_names
        
        dxl_comm_result = self.group_sync_read.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
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

    def __del__(self):
        if not self.use_sim and hasattr(self, 'port_handler'):
            if hasattr(self.port_handler, 'ser') and self.port_handler.ser is not None:
                self.port_handler.closePort()


def main(args=None):
    rclpy.init(args=args)
    node = PanTiltController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
