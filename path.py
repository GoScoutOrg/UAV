#!/usr/bin/env python3

from movement import *
import sys
from time import sleep

def main():

    # Initialize vehicle object.
    vehicle = startup()
    print(vehicle.parameters['RC3_TRIM'])
    print(vehicle.parameters['RC3_MIN'])
    print(vehicle.parameters['RC3_MAX'])

    # Printing Vehicle's Latitude
    print("Vehicle's Latitude              =  ", vehicle.location.global_relative_frame.lat)
    print("Size of lat = ", sys.getsizeof(vehicle.location.global_relative_frame.lat))

    # Printing Vehicle's Longitude
    print("Vehicle's Longitude             =  ", vehicle.location.global_relative_frame.lon)
    print("Size of lon = ", sys.getsizeof(vehicle.location.global_relative_frame.lon))


    # Printing Vehicle's Altitude
    print("Vehicle's Altitude (in meters)  =  ", vehicle.location.global_relative_frame.alt)
    print("Size of alt = ", sys.getsizeof(vehicle.location.global_relative_frame.alt))

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

    for i in range(3):
        offset = i*3
        print(f"offset={offset}")
        if send_ned_velocity(0, 1, 0, 3+offset, vehicle) == True:
            return
        vehicle.send_mavlink(msg_blank)
        time.sleep(2) 
        condition_yaw(90, vehicle)
        time.sleep(2) 

        if send_ned_velocity(0, 1, 0, 3+offset, vehicle) == True:
            return
        vehicle.send_mavlink(msg_blank)
        time.sleep(2) 
        condition_yaw(90, vehicle)
        time.sleep(2)

    input("> ")

    # Close vehicle object before exiting script
    print("Close vehicle object")
    vehicle.close()


if __name__ == '__main__':
   main()
