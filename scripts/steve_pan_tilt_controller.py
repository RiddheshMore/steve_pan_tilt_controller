#!/usr/bin/env python3

import rospy
import yaml
from dynamixel_sdk import * 

def set_motor_torque(port_handler, packet_handler, motor_id, enable):
    torque_value = 1 if enable else 0
    result, error = packet_handler.write1ByteTxRx(port_handler, motor_id, 64, torque_value)
    if result != COMM_SUCCESS:
        rospy.logerr(f"Failed to {'enable' if enable else 'disable'} torque for motor {motor_id}: {packet_handler.getTxRxResult(result)}")
    elif error != 0:
        rospy.logerr(f"Error occurred for motor {motor_id}: {packet_handler.getRxPacketError(error)}")
    else:
        rospy.loginfo(f"Motor {motor_id} torque {'enabled' if enable else 'disabled'}")

def set_motor_position(port_handler, packet_handler, motor_id, position):
    result, error = packet_handler.write4ByteTxRx(port_handler, motor_id, 116, position)
    if result != COMM_SUCCESS:
        rospy.logerr(f"Failed to move motor {motor_id}: {packet_handler.getTxRxResult(result)}")
    elif error != 0:
        rospy.logerr(f"Error occurred for motor {motor_id}: {packet_handler.getRxPacketError(error)}")
    else:
        rospy.loginfo(f"Motor {motor_id} moved to position {position}")

def main():
    rospy.init_node('steve_pan_tilt_controller')

    # Load parameters from yaml file
    config_file = rospy.get_param('~config_file')
    rospy.loginfo(f"Loading config file: {config_file}")
    with open(config_file, 'r') as file:
        config = yaml.safe_load(file)
    
    motor_ids = config['motors']['ids']
    baud_rate = config['motors']['baud_rate']
    device_name = config['motors']['device_name']

    # Get goal positions from parameters
    pan_goal_position = rospy.get_param('~pan_goal_position', config['motors']['dxl_info']['id_1']['min_position_limit'])
    tilt_goal_position = rospy.get_param('~tilt_goal_position', config['motors']['dxl_info']['id_2']['max_position_limit'])

    # Initialize PortHandler and PacketHandler
    port_handler = PortHandler(device_name)
    packet_handler = PacketHandler(2.0)

    # Open port
    if not port_handler.openPort():
        rospy.logerr("Failed to open the port")
        return

    # Set port baud rate
    if not port_handler.setBaudRate(baud_rate):
        rospy.logerr("Failed to set baud rate")
        return

    try:
        pan_position = int(pan_goal_position / 360.0 * 4095)
        tilt_position = int(tilt_goal_position / 360.0 * 4095)

        # Enable torque for both motors
        set_motor_torque(port_handler, packet_handler, motor_ids[0], True)
        set_motor_torque(port_handler, packet_handler, motor_ids[1], True)

        # Set motor positions
        set_motor_position(port_handler, packet_handler, motor_ids[0], pan_position)
        rospy.sleep(2)
        set_motor_position(port_handler, packet_handler, motor_ids[1], tilt_position)

        rospy.sleep(10)

        # set_motor_position(port_handler, packet_handler, motor_ids[0], int(180 / 360.0 * 4095))
        # rospy.sleep(2)
        set_motor_position(port_handler, packet_handler, motor_ids[1], int(180 / 360.0 * 4095))

        rospy.sleep(2)

        # Disable torque for both motors
        # set_motor_torque(port_handler, packet_handler, motor_ids[0], False)
        # set_motor_torque(port_handler, packet_handler, motor_ids[1], False)
    except Exception as e:
        rospy.logerr(f"Exception occurred: {e}")
    finally:
        # Close port
        port_handler.closePort()

if __name__ == '__main__':
    main()
