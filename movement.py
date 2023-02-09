import time
import math
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative, mavutil, Command
import argparse
# from vision.webcam import check_cam
# from vision.webcam import check_cam
# from vision.camera import Webcam

#from QRCode import findQR, CameraData


def get_distance_metres(aLocation1, aLocation2):
   """
   Returns the ground distance in metres between two LocationGlobal objects.

   This method is an approximation, and will not be accurate over large distances and close to the
   earth's poles. It comes from the ArduPilot test code:
   https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
   """
   dlat = aLocation2.lat - aLocation1.lat
   dlong = aLocation2.lon - aLocation1.lon
   return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5


def get_location_metres(original_location, dNorth, dEast):
   """
   Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
   specified `original_location`. The returned LocationGlobal has the same `alt` value
   as `original_location`.

   The function is useful when you want to move the vehicle around specifying locations relative to 
   the current vehicle position.

   The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.

   For more information see:
   http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
   """
   earth_radius = 6378137.0 #Radius of "spherical" earth
   #Coordinate offsets in radians
   dLat = dNorth/earth_radius
   dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

   #New position in decimal degrees
   newlat = original_location.lat + (dLat * 180/math.pi)
   newlon = original_location.lon + (dLon * 180/math.pi)
   if type(original_location) is LocationGlobal:
      targetlocation=LocationGlobal(newlat, newlon,original_location.alt)
   elif type(original_location) is LocationGlobalRelative:
      targetlocation=LocationGlobalRelative(newlat, newlon,original_location.alt)
   else:
      raise Exception("Invalid Location object passed")
      
   return targetlocation


# Set vehicle mode to GUIDED, arm, and takeoff to the specified altitude.
def arm_and_takeoff(aTargetAltitude, vehicle):
   print("Basic pre-arm checks")
   # Don't try to arm until autopilot is ready
   while not vehicle.is_armable:
      print(" Waiting for vehicle to initialise...")
      time.sleep(1)

   print("Arming motors")
   # Copter should arm in GUIDED mode
   vehicle.mode = VehicleMode("GUIDED")
   vehicle.armed = True

   # Confirm vehicle armed before attempting to take off
   while not vehicle.armed:
      print(" Waiting for arming...")
      time.sleep(1)

   print("Taking off!")
   vehicle.simple_takeoff(aTargetAltitude)
   #vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

   # Wait until the vehicle reaches a safe height before processing the goto 
   # (otherwise the command after Vehicle.simple_takeoff will execute immediately).
   while True:
      print()
      print("Altitude (global relative frame): %s" 
         % vehicle.location.global_relative_frame.alt)
      print("Altitude (NED frame): %s" 
         % vehicle.location.local_frame.down)

      # Break and return from function just below target altitude.
      if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
      #if -vehicle.location.local_frame.down >= aTargetAltitude * 0.95:
         print("Reached target altitude")
         break
      time.sleep(1)


# Navigate using North/East/Down velocity vectors.
# TODO: this appears not to work reliably. Needs more standalone testing.
def send_vel_poll_target(velocity_x, velocity_y, duration, vehicle, tf):
   """
   Move vehicle in direction based on specified velocity vectors.
   """
   msg = vehicle.message_factory.set_position_target_local_ned_encode(
      0,       # time_boot_ms (not used)
      0, 0,    # target system, target component
      mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, # frame
      0b0000111111000111, # type_mask (only speeds enabled)
      0, 0, 0, # x, y, z positions (not used)
      velocity_x, velocity_y, 0, # x, y, z velocity in m/s
      0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
      0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)


   msg_blank = vehicle.message_factory.set_position_target_local_ned_encode(
      0,       # time_boot_ms (not used)
      0, 0,    # target system, target component
      mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, # frame
      0b0000111111000111, # type_mask (only speeds enabled)
      0, 0, 0, # x, y, z positions (not used)
      0, 0, 0, # x, y, z velocity in m/s
      0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
      0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

   # send command to vehicle on 1 Hz cycle
   for x in range(round(duration)):
      vehicle.send_mavlink(msg)
      print("Altitude (NED frame): %s" % vehicle.location.local_frame.down)
      for y in range(10):
         if tf.check_for_target() == True:
            vehicle.send_mavlink(msg_blank)
            return True
         time.sleep(0.1) 
   return False


def condition_yaw(heading, vehicle):
   is_relative=1 #yaw relative to direction of travel
   # create the CONDITION_YAW command using command_long_encode()
   msg = vehicle.message_factory.command_long_encode(
      0, 0,    # target system, target component
      mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
      0, #confirmation
      heading,    # param 1, yaw in degrees
      0,          # param 2, yaw speed deg/s
      1,          # param 3, direction -1 ccw, 1 cw
      is_relative, # param 4, relative offset 1, absolute angle 0
      0, 0, 0)    # param 5 ~ 7 not used
   # send command to vehicle
   vehicle.send_mavlink(msg)
   


# Parse connection string, if it exists.
# For example:  --connect tcp:127.0.0.1:5762
# If no string, connect to Pixhawk over serial.
def startup():
   parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
   parser.add_argument('--connect',
                  help="Vehicle connection target string. If not specified, SITL automatically started and used.")
   args = parser.parse_args()

   connection_string = args.connect

   # Connect to the Vehicle
   if not connection_string:
      print('auto connect')
      vehicle = connect('127.0.0.1:14550', wait_ready=True)
      # vehicle = connect('/dev/serial0', baud=57600, heartbeat_timeout=15)
   else:
      print('Connecting to vehicle on: %s' % connection_string)
      vehicle = connect(connection_string, wait_ready=True)

   return vehicle


def land(vehicle):
   vehicle.mode = VehicleMode("LAND")
   print("Landing...")

if __name__ == '__main__':
   print('Import this script to use its functions.')
