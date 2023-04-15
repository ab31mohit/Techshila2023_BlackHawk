#!/usr/bin/env python
from pymavlink import mavutil
import time
import rospy
from geometry_msgs.msg import Vector3
 
rospy.init_node('drones')
# Start a connection listening to a UDP port
udp_add = rospy.get_param("~udp")
d = rospy.get_param("~d")
# Start a connection listening to a UDP port
the_connection = mavutil.mavlink_connection(udp_add)

def callback(data):
    x = data.x
    y = data.y
    z = data.z
    global the_connection
    # Movement
    print('Movement:\n')
    the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10,
    the_connection.target_system,
    the_connection.target_component,
    mavutil.mavlink.MAV_FRAME_LOCAL_NED,
    int(0b010111111000), x, -y, -z, 0, 0, 0, 0, 0, 0, 0, 0))

    while 1:
        msg = the_connection.recv_match(
        type='NAV_CONTROLLER_OUTPUT', blocking=True)
        print(msg)
        if abs(msg.wp_dist)<=0.01:
            break

    ack_msg = the_connection.recv_match(type='COMMAND_ACK', condition=lambda msg: msg.command == mavutil.mavlink.MAV_CMD_SET_POSITION_TARGET_LOCAL_NED)
    print(ack_msg)
    

rospy.Subscriber(d, Vector3, callback)
# This sets the system and component ID of remote system for the link
the_connection.wait_heartbeat()
print("drone 1 :\n")
print("Heartbeat from system (system %u component %u)" %
    (the_connection.target_system, the_connection.target_component))
# arm
print('Arm:')
the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
print(msg)
# set mode guided
print('Guided:\n')
mode = 'GUIDED'
mode_id = the_connection.mode_mapping()[mode]
the_connection.mav.set_mode_send(the_connection.target_system,mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,mode_id)
msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
print(msg)
# takeoff
print('Takeoff:\n')
the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 5)
while 1:
    msg = the_connection.recv_match(type='LOCAL_POSITION_NED', blocking=True)
    print(msg)
    if abs(msg.z*(-1)-5)<=0.05:
         break

print("Hover Done")
rospy.spin()
    
