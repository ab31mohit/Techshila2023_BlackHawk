from pymavlink import mavutil
import time
import numpy as np
import rospy as rp
from geometry_msgs.msg import Vector3, Pose

# drone-local frame coordinate in world frame
x0 = 0
y0 = 0
z0 = 0

# target point coordinate in world frame
# xt=2 
# yt=3
# zt=6

# waypoints you should give to drone in its local coordinate frame so that it reaches desired target point in world frame
to_move = False
x = 0
y = 0
z = 0
 
def _move(target):
    global x, y, z, to_move
    x = target.x
    y = target.y
    z = target.z
    to_move = True

if __name__ == '__main__':
    rp.init_node("slave")
    rp.Subscriber('D1', Vector3, _move)
    # Start a connection listening to a UDP port
    the_connection = mavutil.mavlink_connection('udpin:localhost:14551')

    # This sets the system and component ID of remote system for the link
    the_connection.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" %
        (the_connection.target_system, the_connection.target_component))

    # arm
    print('Arming:')
    the_connection.mav.command_long_send(the_connection.target_system,
    the_connection.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0, 1, 0, 0, 0, 0, 0, 0)

    msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg)
    print("\n")

    
    # set mode guided
    print('Setting Mode Guided:')
    mode = 'GUIDED'
    mode_id = the_connection.mode_mapping()[mode]
    the_connection.mav.set_mode_send(the_connection.target_system,mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,mode_id)

    msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg)
    print("\n")

    # takeoff
    print('Taking off:')
    the_connection.mav.command_long_send(the_connection.target_system,
    the_connection.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0, 0, 0, 0, 0, 0, 0, 3)

    msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg)
    
    while 1: 
        pose = the_connection.recv_match(type='LOCAL_POSITION_NED', blocking=True)
        # Extract the position data from the message
        pos_x = pose.x  # meters
        pos_y = -pose.y  # meters
        pos_z = -pose.z  # meters (convert to upward positive convention)

        # Print the position data
        print(f"X: {pos_x + x0:.2f} m, Y: {pos_y + y0:.2f} m, Z: {pos_z + z0:.2f} m")
        if (abs(pos_z-3) <= 0.01):
            break
        
    print("\n")
    time.sleep(4)
    

    # Movement
    rt = rp.Rate(1)
    while True:
        print("sleeping")
        rt.sleep()
        if to_move:
            break

    print('Moving to a point:')

    the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10,
    the_connection.target_system, the_connection.target_component, 
    mavutil.mavlink.MAV_FRAME_LOCAL_NED, int(0b010111111000), x, -y, -z, 0, 0, 0, 0, 0, 0, 0, 0))


    while 1:
        pose = the_connection.recv_match(type='LOCAL_POSITION_NED', blocking=True)
        # Extract the position data from the message
        pos_x = pose.x  # meters
        pos_y = -pose.y  # meters
        pos_z = -pose.z  # meters (convert to upward positive convention)

        # Print the position data
        print(f"X: {pos_x + x0:.2f} m, Y: {pos_y + y0:.2f} m, Z: {pos_z + z0:.2f} m")

        # msg = the_connection.recv_match(type='NAV_CONTROLLER_OUTPUT', blocking=True)
        

        if (sum(np.power([(pos_x-x),(pos_y-y),(pos_z-z)],2)) <= 0.01):
            break


    msg = the_connection.recv_match(type='COMMAND_ACK', condition=lambda msg: msg.command == mavutil.mavlink.MAV_CMD_SET_POSITION_TARGET_LOCAL_NED)
    print(msg)
    print("\n")
    time.sleep(2)


    # landing
    print('Landing :\n')
    the_connection.mav.command_long_send(the_connection.target_system,
    the_connection.target_component, mavutil.mavlink.MAV_CMD_NAV_LAND,
    0, 0, 0, 0, 0, 0, 0, 0)

    msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg)

    while 1:
        pose = the_connection.recv_match(type='LOCAL_POSITION_NED', blocking=True)
        # Extract the position data from the message
        pos_x = pose.x  # meters
        pos_y = -pose.y  # meters
        pos_z = -pose.z  # meters (convert to upward positive convention)

        # Print the position data
        print(f"X: {pos_x + x0:.2f} m, Y: {pos_y + y0:.2f} m, Z: {pos_z + z0:.2f} m")

        # msg = the_connection.recv_match(type='NAV_CONTROLLER_OUTPUT', blocking=True)

        if (pos_z + z0 <= 0.01):
            break

    print("\n")
    rp.spin()

