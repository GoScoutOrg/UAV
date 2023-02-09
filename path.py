#!/usr/bin/env python3

from movement import *
import sys
from vision.target_finder import TargetFinder
from vision.camera import Webcam

def print_gps(vehicle):
        # Printing Vehicle's Latitude
    print("Vehicle's Latitude              =  ", vehicle.location.global_relative_frame.lat)
    # print("Size of lat = ", sys.getsizeof(vehicle.location.global_relative_frame.lat))

    # Printing Vehicle's Longitude
    print("Vehicle's Longitude             =  ", vehicle.location.global_relative_frame.lon)
    # print("Size of lon = ", sys.getsizeof(vehicle.location.global_relative_frame.lon))


    # Printing Vehicle's Altitude
    print("Vehicle's Altitude (in meters)  =  ", vehicle.location.global_relative_frame.alt)
    # print("Size of alt = ", sys.getsizeof(vehicle.location.global_relative_frame.alt))

def main():

    # Initialize vehicle object.
    vehicle = startup()
    print(vehicle.parameters['RC3_TRIM'])
    print(vehicle.parameters['RC3_MIN'])
    print(vehicle.parameters['RC3_MAX'])

    print("Taking off to 5...")
    arm_and_takeoff(5, vehicle)


    print("Starting path")

    msg_blank = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        0, 0, 0, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    forward_vel = 0.1
    lateral_vel = 1
    
    num_sweeps = 12
    
    tf = TargetFinder(Webcam())
    
    for i in range(num_sweeps):
        
        if send_vel_poll_target(forward_vel, lateral_vel, 5, vehicle, tf) == True:
            # send coordinates to rover here
            print_gps(vehicle)
            print(tf.frame_coord)
            return
        vehicle.send_mavlink(msg_blank)
        time.sleep(2) 
        lateral_vel = lateral_vel * -1
        
        # turn around, optional
        if i == num_sweeps // 2:
            forward_vel = forward_vel * -1
        


    # Close vehicle object before exiting script
    print("Close vehicle object")
    vehicle.close()


if __name__ == '__main__':
   main()
