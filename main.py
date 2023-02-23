from vision.target_finder import TargetFinder
from vision.camera import Arducam
from movement import *


from multiprocessing import Process

import sys
sys.path.append("../Communications")
import communications as c

import math

UAV, comms = Pipe()


#----------------------------------------------------------------#
# START TARGET COORDINATE CODE
#----------------------------------------------------------------#


# Takes in target coordinates given current coordinates and current orientation (compass direction in degrees)
# Returns the turn angle required in degrees
def coords_to_delta_theta(target_long, target_lat, curr_long, curr_lat, curr_theta_deg):
    delta_x = target_long - curr_long
    delta_y = target_lat - curr_lat
    delta_theta_deg = 0
    if(delta_y < 0):
        delta_theta_deg = 180 + math.degrees(math.atan(float(delta_x) / float(delta_y))) - curr_theta_deg
    else:
        delta_theta_deg = math.degrees(math.atan(float(delta_x) / float(delta_y))) - curr_theta_deg
    return delta_theta_deg

# Takes in target coordinates given current coordinates
# Returns the distance for direct travel (the hypotenuse)
def coords_to_target_distance(target_long, target_lat, curr_long, curr_lat):
    METER_TO_COORD_DEG_RATIO = 111139
    delta_x = target_long - curr_long
    delta_y = target_lat - curr_lat
    return math.sqrt((delta_x**2)+(delta_y**2)) * METER_TO_COORD_DEG_RATIO

# This converts relative image positioning into a physical offset (in meters) from the center of the image.
# The offset uses a coordinate transformation to translate relative offsets into a global x and y scale (like a compass coordinate plane)
# DISCLAIMER: assumes that the bottom left of the image is (0,0) and the top right of the image is (resolution_x, resolution_y)
def image_to_offset(altitude, curr_orientation, target_x, target_y, FOV, resolution_x, resolution_y):
    physical_image_width = 2 * altitude * math.tan((FOV / 2.0) * math.pi / 180.0)
    physical_pixel_width = physical_image_width / resolution_x

    unadjusted_x_offset = (target_x - (resolution_x / 2.0)) * physical_pixel_width
    unadjusted_y_offset = (target_y - (resolution_y / 2.0)) * physical_pixel_width

    adjusted_x_offset = unadjusted_x_offset * math.cos(math.radians(curr_orientation)) + unadjusted_y_offset * math.sin(math.radians(curr_orientation))
    adjusted_y_offset = (-unadjusted_x_offset) * math.sin(math.radians(curr_orientation)) + unadjusted_y_offset * math.cos(math.radians(curr_orientation))

    return [adjusted_x_offset, adjusted_y_offset]

# This function takes in physical distances (in meters) as offsets from the current lat and lon, returning the lat and lon
# of the offset target
def offset_to_target_coords(offset_y, offset_x, curr_lat, curr_lon):
    # Earth’s radius, sphere
    R=6378137

    # offsets in meters
    dn = offset_y
    de = offset_x

    # Coordinate offsets in radians
    dLat = dn/R
    dLon = de/(R*math.cos(math.pi*curr_lat/180))

    # OffsetPosition, decimal degrees
    latO = curr_lat + dLat * 180/math.pi
    lonO = curr_lon + dLon * 180/math.pi 

    return [latO, lonO]

# This function simply returns the physical distance between two physical offsets (Think simple pythagorian theorem)
def offset_to_distance(offset_x, offset_y):
    return math.sqrt((offset_x * offset_x) + (offset_y * offset_y))

# This function is essentially a wrapper that calls both image_to_offset() and offset_to_target_coords(). If this function is called, neither 
# image_to_offset() nor offset_to_target() need to be called individually.
def get_target_coords(curr_lat, curr_lon, altitude, curr_orientation, target_x, target_y, FOV, resolution_x, resolution_y):
    transformed_offset = image_to_offset(altitude, curr_orientation, target_x, target_y, FOV, resolution_x, resolution_y)
    target_coords = offset_to_target_coords(transformed_offset[1], transformed_offset[0], curr_lat, curr_lon)
    return target_coords



def target_callback(vehicle):
        # Printing Vehicle's Latitude
    lat = vehicle.location.global_relative_frame.lat
    lon = vehicle.location.global_relative_frame.lon
    alt = vehicle.location.global_relative_frame.alt
    print("Vehicle's Latitude              =  ", lat)
    # print("Size of lat = ", sys.getsizeof(vehicle.location.global_relative_frame.lat))

    # Printing Vehicle's Longitude
    print("Vehicle's Longitude             =  ", lon)
    # print("Size of lon = ", sys.getsizeof(vehicle.location.global_relative_frame.lon))


    # Printing Vehicle's Altitude
    print("Vehicle's Altitude (in meters)  =  ", alt)
    # print("Size of alt = ", sys.getsizeof(vehicle.location.global_relative_frame.alt))
    gpsinfo = str(lat) + ":" + str(lon)
    c.send_packet(flag = "GPS", args= [gpsinfo]) 


#----------------------------------------------------------------#
# END TARGET COORDINATE CODE
#----------------------------------------------------------------#


def main():
    tf = TargetFinder(Arducam())
    vehicle = startup()

    function_set = {
        "TEST": print("Hello")
    }

    communications = Process(target=c.parent_proc, args=("192.168.4.10",7777, "192.168.4.1", 7676, function_set))
    communications.start()
    while True:
        #if red square is found: 
        while(tf.check_for_target() == False):
            pass
            #gps_coor = gps.gps_setup()
            #calibrate_coordinates(gps_coor)
        
        target_callback(vehicle)

        communications.join()
        
        


    communications.join()

if __name__ == "__main__":
    main()