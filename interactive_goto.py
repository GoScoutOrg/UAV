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


   while True:
      print(80*"*")
      commands = ["goto", "altgoto", "takeoff", "land", "path", "exit"]
      print("Commands:")
      for c in commands:
         print(f"  {c}")

      command = input("> ")

      if command == "goto":
         try:
            heading = float(input("Enter heading (deg, 0=North): "))
            distance = float(input("Enter distance (m): "))
            groundspeed = float(input("Enter groundspeed (m/s): "))
         except (ValueError, EOFError):
            print("Invalid input.")
            continue
         dN = distance * math.cos(heading / 180 * math.pi)
         dE = distance * math.sin(heading / 180 * math.pi)
         print(f"Goto: N={dN}, E={dE}")
         goto(dN, dE, groundspeed, vehicle)
      
      elif command == "altgoto":
         try:
            heading = float(input("Enter relative heading (deg, 0=Forward): "))
            distance = float(input("Enter distance (m): "))
            speed = float(input("Enter speed (m/s): "))
         except (ValueError, EOFError):
            print("Invalid input.")
            continue
         vX = speed * math.cos(heading / 180 * math.pi)
         vY = speed * math.sin(heading / 180 * math.pi)
         vZ = 0
         print(f"vX = {vX}, vY = {vY}, vZ = {vZ}, time = {round(distance / speed)}")
         send_ned_velocity(vX, vY, vZ, distance / speed, vehicle)

      elif command == "takeoff":
         try:
            altitude = float(input("Enter altitude (m): "))
         except (ValueError, EOFError):
            print("Invalid input.")
            continue
         print(f"Taking off to {altitude} meters")
         arm_and_takeoff(altitude, vehicle)

      elif command == "path":

         msg_blank = vehicle.message_factory.set_position_target_local_ned_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, # frame
            0b0000111111000111, # type_mask (only speeds enabled)
            0, 0, 0, # x, y, z positions (not used)
            0, 0, 0, # x, y, z velocity in m/s
            0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

         for i in range(10):
            offset = i*3
            print(f"offset={offset}")
            send_ned_velocity(0, 0.5, 0, 3+offset, vehicle)
            vehicle.send_mavlink(msg_blank)
            time.sleep(2) 
            condition_yaw(90, vehicle)
            time.sleep(2) 

            send_ned_velocity(0, 0.5, 0, 3+offset, vehicle)
            vehicle.send_mavlink(msg_blank)
            time.sleep(2) 
            condition_yaw(90, vehicle)
            time.sleep(2)

         # offset = 0
         # print("first")
         # send_ned_velocity(0, 0.5, 0, 2+offset, vehicle)
         # vehicle.send_mavlink(msg_blank)
         # time.sleep(2) 
         # condition_yaw(90, vehicle)
         # time.sleep(2) 

         # print("second")
         # send_ned_velocity(0, 0.5, 0, 2+offset, vehicle)
         # vehicle.send_mavlink(msg_blank)
         # time.sleep(2) 
         # condition_yaw(90, vehicle)
         # time.sleep(2)

         # print("third")
         # send_ned_velocity(0, (0.5), 0, 4+offset, vehicle)
         # vehicle.send_mavlink(msg_blank)
         # time.sleep(2) 
         # condition_yaw(90, vehicle)
         # time.sleep(2)

         # print("fourth")
         # send_ned_velocity(0, (0.5), 0, 4+offset, vehicle)
         # vehicle.send_mavlink(msg_blank)
         # time.sleep(2) 
         # condition_yaw(90, vehicle)
         # time.sleep(2) 

         # print("fifth")
         # send_ned_velocity(0, 0.5, 0, 6+offset, vehicle)
         # vehicle.send_mavlink(msg_blank)
         # time.sleep(2) 
         # condition_yaw(90, vehicle)
         # time.sleep(2)

         # for i in range(2):
         #    offset = i * 2
         #    print("first")
         #    send_ned_velocity(0, 0.5, 0, 6+offset, vehicle)
         #    vehicle.send_mavlink(msg_blank)
         #    time.sleep(2) 
         #    condition_yaw(90, vehicle)
         #    time.sleep(2) 

         #    print("second")
         #    send_ned_velocity(0, 0.5, 0, 8+offset, vehicle)
         #    vehicle.send_mavlink(msg_blank)
         #    time.sleep(2) 
         #    condition_yaw(90, vehicle)
         #    time.sleep(2)

         #    print("third")
         #    send_ned_velocity(0, (0.5), 0, 8+offset, vehicle)
         #    vehicle.send_mavlink(msg_blank)
         #    time.sleep(2) 
         #    condition_yaw(90, vehicle)
         #    time.sleep(2)

         #    print("fourth")
         #    send_ned_velocity(0, (0.5), 0, 10+offset, vehicle)
         #    vehicle.send_mavlink(msg_blank)
         #    time.sleep(2) 
         #    condition_yaw(90, vehicle)
         #    time.sleep(2) 

         #    print("fifth")
         #    send_ned_velocity(0, 0.5, 0, 10+offset, vehicle)
         #    vehicle.send_mavlink(msg_blank)
         #    time.sleep(2) 
         #    condition_yaw(90, vehicle)
         #    time.sleep(2)

         # dN = 10 * math.cos(0 / 180 * math.pi)
         # dE = 10 * math.sin(0 / 180 * math.pi)
         # goto(dN, dE, 10, vehicle)
         # dN = 10 * math.cos(90 / 180 * math.pi)
         # dE = 10 * math.sin(90 / 180 * math.pi)
         # goto(dN, dE, 10, vehicle)
         # dN = 10 * math.cos(180 / 180 * math.pi)
         # dE = 10 * math.sin(180 / 180 * math.pi)
         # goto(dN, dE, 10, vehicle)
         # dN = 10 * math.cos(270 / 180 * math.pi)
         # dE = 10 * math.sin(270 / 180 * math.pi)
         # goto(dN, dE, 10, vehicle)

      elif command == "land":
         print("Landing")
         vehicle.mode = VehicleMode("LAND")

      elif command == "exit":
         break

# Close vehicle object before exiting script
   print("Close vehicle object")
   vehicle.close()


if __name__ == '__main__':
   main()
