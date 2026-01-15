#!/usr/bin/env python3
"""
Test script for controlling pan-tilt motors in Gazebo simulation.
Uses ros2_control trajectory controller interface.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time


class PanTiltSimController(Node):
    """Test pan-tilt controller in Gazebo simulation using ros2_control."""

    def __init__(self):
        super().__init__('pan_tilt_sim_controller')
        
        # Create action client for pan-tilt controller
        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/pan_tilt_controller/follow_joint_trajectory'
        )
        
        self.get_logger().info('Waiting for pan_tilt_controller action server...')
        self.action_client.wait_for_server()
        self.get_logger().info('Connected to pan_tilt_controller!')

    def send_pan_tilt_goal(self, pan_position, tilt_position, duration_sec=3.0):
        """
        Send a goal to move pan and tilt joints.
        
        Args:
            pan_position: Pan position in radians
            tilt_position: Tilt position in radians
            duration_sec: Time to reach the position
        """
        goal_msg = FollowJointTrajectory.Goal()
        
        # Define joint names (must match URDF)
        goal_msg.trajectory.joint_names = [
            'pan_tilt_pan_motor_joint',
            'pan_tilt_tilt_motor_joint'
        ]
        
        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = [pan_position, tilt_position]
        point.velocities = [0.0, 0.0]
        point.time_from_start = Duration(sec=int(duration_sec), nanosec=int((duration_sec % 1) * 1e9))
        
        goal_msg.trajectory.points = [point]
        
        self.get_logger().info(f'Sending goal: pan={pan_position:.2f}rad, tilt={tilt_position:.2f}rad')
        
        # Send goal
        send_goal_future = self.action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return False
        
        self.get_logger().info('Goal accepted, waiting for result...')
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result().result
        self.get_logger().info(f'Result: {result.error_code}')
        return result.error_code == FollowJointTrajectory.Result.SUCCESSFUL

    def run_test_sequence(self):
        """Run a test sequence of pan-tilt movements."""
        import math
        
        self.get_logger().info('Starting pan-tilt test sequence...')
        
        # Test sequence: move to different positions
        # Pan limits: approx [-π/2, π/2], Tilt limits: approx [-π/4, π/4]
        
        test_positions = [
            (0.0, 0.0, 'Center'),  # Center position
            (0.5, 0.0, 'Pan right'),  # Pan right
            (-0.5, 0.0, 'Pan left'),  # Pan left
            (0.0, 0.3, 'Tilt up'),  # Tilt up
            (0.0, -0.3, 'Tilt down'),  # Tilt down
            (0.3, 0.3, 'Diagonal up-right'),  # Diagonal
            (-0.3, -0.3, 'Diagonal down-left'),  # Diagonal
            (0.0, 0.0, 'Return to center'),  # Return to center
        ]
        
        for pan, tilt, description in test_positions:
            self.get_logger().info(f'\n--- {description} ---')
            success = self.send_pan_tilt_goal(pan, tilt, duration_sec=2.0)
            if not success:
                self.get_logger().error(f'Failed to execute: {description}')
            time.sleep(3)  # Wait between movements
        
        self.get_logger().info('\nTest sequence completed!')


def main(args=None):
    rclpy.init(args=args)
    
    controller = PanTiltSimController()
    
    try:
        # Run the test sequence
        controller.run_test_sequence()
    except KeyboardInterrupt:
        controller.get_logger().info('Test interrupted by user')
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
