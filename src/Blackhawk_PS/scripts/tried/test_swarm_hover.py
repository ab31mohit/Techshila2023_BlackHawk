from pymavlink import mavutil
import numpy as np 
import time

class Drone:
    
    drone_id = 0
    def __init__(self, udpin):
        Drone.drone_id += 1
        self.the_connection = mavutil.mavlink_connection(f'udpin:localhost:{udpin}')

    ###############################################
    
    def heartbeat(self):
        # This sets the system and component ID of remote system for the link
        print(f"drone {Drone.drone_id} :\n")

        self.the_connection.wait_heartbeat()

        print("Heartbeat from system (system %u component %u)" %
            (self.the_connection.target_system, self.the_connection.target_component))
        print('\n')   

    ###############################################

    def arm_throttle(self):
        # arm
        self.the_connection.mav.command_long_send(self.the_connection.target_system, self.the_connection.target_component,
                                            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)

        msg = self.the_connection.recv_match(type='COMMAND_ACK', blocking=True)

        print(f"drone {Drone.drone_id} :\n")
        print(msg)
        print('\n')

    ################################################

    def set_mode(self):
        mode = 'GUIDED'
        mode_id = self.the_connection.mode_mapping()[mode]
        self.the_connection.mav.set_mode_send(self.the_connection.target_system,mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,mode_id)

        msg = self.the_connection.recv_match(type='COMMAND_ACK', blocking=True)
        print(f"drone { Drone.drone_id} :\n")
        print(msg)
        print('\n')

    ##################################################
    
    def takeoff(self, height):
        # takeoff
        self.the_connection.mav.command_long_send(self.the_connection.target_system, self.the_connection.target_component,
                                            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, height)

        msg = self.the_connection.recv_match(type='COMMAND_ACK', blocking=True)
        print(f"drone {Drone.drone_id} :\n")
        print(msg)
        print('\n')



if __name__ == '__main__':
    
    udpin_arr = list(range(14551, 14671, 10))

    for i in range(12):
        drone = Drone(udpin_arr[i])
        drone.heartbeat()
        drone.arm_throttle()
        drone.set_mode()
        drone.takeoff(3)
        time.sleep(2)

