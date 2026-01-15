#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import yaml
import time
import math
from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS


class PanTiltController(Node):
    """ROS2 node for controlling Dynamixel pan-tilt motors and publishing joint states."""

    # Dynamixel XH430-W250 Control Table Addresses
    ADDR_HARDWARE_ERROR_STATUS = 70
    ADDR_TORQUE_ENABLE = 64
    ADDR_PROFILE_ACCELERATION = 108
    ADDR_PROFILE_VELOCITY = 112
    ADDR_GOAL_POSITION = 116
    ADDR_PRESENT_POSITION = 132

    # Dynamixel Protocol 2.0 Instructions
    INST_REBOOT = 0x08

    def __init__(self):
        super().__init__('steve_pan_tilt_controller')

        # Declare parameters
        self.declare_parameter('config_file', '')
        self.declare_parameter('pan_goal_position', 90.0)
        self.declare_parameter('tilt_goal_position', 180.0)
        self.declare_parameter('profile_velocity', 50)
        self.declare_parameter('profile_acceleration', 10)
        self.declare_parameter('publish_rate', 20.0)

        # Get parameters
        config_file = self.get_parameter('config_file').value
        self.profile_velocity = self.get_parameter('profile_velocity').value
        self.profile_acceleration = self.get_parameter('profile_acceleration').value
        self.publish_rate = self.get_parameter('publish_rate').value

        # Load config from YAML
        if not config_file:
            self.get_logger().error('No config_file parameter provided!')
            return

        with open(config_file, 'r') as file:
            config = yaml.safe_load(file)

        self.motor_ids = config['motors']['ids']
        self.baud_rate = config['motors']['baud_rate']
        self.device_name = config['motors']['device_name']
        
        # Joint names should match URDF
        self.joint_names = ['pan_tilt_pan_motor_joint', 'pan_tilt_tilt_motor_joint']

        # Initialize PortHandler and PacketHandler
        self.port_handler = PortHandler(self.device_name)
        self.packet_handler = PacketHandler(2.0)

        # Open port
        if not self.port_handler.openPort():
            self.get_logger().error(f'Failed to open port {self.device_name}')
            return

        # Set port baud rate
        if not self.port_handler.setBaudRate(self.baud_rate):
            self.get_logger().error(f'Failed to set baud rate {self.baud_rate}')
            return

        # Setup Publishers and Subscribers
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.cmd_sub = self.create_subscription(Float64MultiArray, '/pan_tilt/command', self.command_callback, 10)
        
        # Initial Hardware setup
        self.init_hardware()

        # Create Timer for Joint State Publishing
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_joint_states)
        
        self.get_logger().info('Pan-Tilt Controller initialized and publishing joint states.')

    def init_hardware(self):
        """Initialize motor settings and enable torque."""
        for motor_id in self.motor_ids:
            # Check and clear hardware errors
            if self._check_hardware_error(motor_id):
                self.get_logger().warn(f'Motor {motor_id} has hardware error, attempting reboot...')
                self._reboot_motor(motor_id)
                time.sleep(1)

            # Configure profiles
            self._set_profile_velocity(motor_id, self.profile_velocity)
            self._set_profile_acceleration(motor_id, self.profile_acceleration)

            # Enable torque
            self._set_motor_torque(motor_id, True)

    def command_callback(self, msg):
        """Callback to handle position commands in radians."""
        if len(msg.data) < 2:
            self.get_logger().warn('Command message must have at least [pan, tilt] values')
            return

        pan_rad = msg.data[0]
        tilt_rad = msg.data[1]

        # Convert radians to Dynamixel ticks (0-4095)
        # Assuming 2048 is 0 radians (center)
        pan_ticks = int(2048 + (pan_rad * 4096 / (2 * math.pi)))
        tilt_ticks = int(2048 + (tilt_rad * 4096 / (2 * math.pi)))

        self._set_motor_position(self.motor_ids[0], pan_ticks)
        self._set_motor_position(self.motor_ids[1], tilt_ticks)

    def publish_joint_states(self):
        """Read present positions and publish JointState message."""
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = self.joint_names
        
        positions = []
        for motor_id in self.motor_ids:
            pos_ticks = self._get_present_position(motor_id)
            if pos_ticks is not None:
                # Convert ticks to radians
                rad = (pos_ticks - 2048) * (2 * math.pi / 4096)
                positions.append(rad)
            else:
                positions.append(0.0)
        
        joint_state.position = positions
        self.joint_state_pub.publish(joint_state)

    def _get_present_position(self, motor_id: int):
        """Read the present position of a motor."""
        pos, result, error = self.packet_handler.read4ByteTxRx(
            self.port_handler, motor_id, self.ADDR_PRESENT_POSITION
        )
        if result != COMM_SUCCESS:
            return None
        return pos

    def _check_hardware_error(self, motor_id: int) -> bool:
        error_status, result, error = self.packet_handler.read1ByteTxRx(
            self.port_handler, motor_id, self.ADDR_HARDWARE_ERROR_STATUS
        )
        return result != COMM_SUCCESS or error_status != 0

    def _reboot_motor(self, motor_id: int) -> None:
        self.packet_handler.reboot(self.port_handler, motor_id)

    def _set_motor_torque(self, motor_id: int, enable: bool) -> None:
        torque_value = 1 if enable else 0
        self.packet_handler.write1ByteTxRx(
            self.port_handler, motor_id, self.ADDR_TORQUE_ENABLE, torque_value
        )

    def _set_motor_position(self, motor_id: int, position: int) -> None:
        self.packet_handler.write4ByteTxRx(
            self.port_handler, motor_id, self.ADDR_GOAL_POSITION, position
        )

    def _set_profile_velocity(self, motor_id: int, velocity: int) -> None:
        self.packet_handler.write4ByteTxRx(
            self.port_handler, motor_id, self.ADDR_PROFILE_VELOCITY, velocity
        )

    def _set_profile_acceleration(self, motor_id: int, acceleration: int) -> None:
        self.packet_handler.write4ByteTxRx(
            self.port_handler, motor_id, self.ADDR_PROFILE_ACCELERATION, acceleration
        )

    def __del__(self):
        if hasattr(self, 'port_handler'):
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
