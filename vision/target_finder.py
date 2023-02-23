import cv2
import numpy as np
import math

class TargetFinder():
    
    class FrameCoord:
        
        def __init__(self, x=0, y=0) -> None:
            self.x = x
            self.y = y
            
        def __repr__(self) -> str:
            return f"({self.x}, {self.y})"
            
        def set(self, x, y):
            self.x = x
            self.y = y
    
    def __init__(self, camera=None) -> None:
        self.src = camera
        self.frame_coord = self.FrameCoord()
        
    # START TARGET COORDINATE CODE

    # Takes in target coordinates given current coordinates and current orientation (compass direction in degrees)
    # Returns the turn angle required in degrees
    def coords_to_delta_theta(self, target_long, target_lat, curr_long, curr_lat, curr_theta_deg):
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
    def coords_to_target_distance(self, target_long, target_lat, curr_long, curr_lat):
        # Approximate radius of earth in km
        R = 6373.0

        curr_lat = math.radians(curr_lat)
        curr_long = math.radians(curr_long)
        target_lat = math.radians(target_lat)
        target_long = math.radians(target_long)

        dlon = target_long - curr_long
        dlat = target_lat - curr_lat

        a = math.sin(dlat / 2)**2 + math.cos(curr_lat) * math.cos(target_lat) * math.sin(dlon / 2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        distance = R * c

        return distance / 1000

    # This converts relative image positioning into a physical offset (in meters) from the center of the image.
    # The offset uses a coordinate transformation to translate relative offsets into a global x and y scale (like a compass coordinate plane)
    # DISCLAIMER: assumes that the bottom left of the image is (0,0) and the top right of the image is (resolution_x, resolution_y)
    def image_to_offset(self, altitude, curr_orientation, target_x, target_y, FOV, resolution_x, resolution_y):
        physical_image_width = 2 * altitude * math.tan((FOV / 2.0) * math.pi / 180.0)
        physical_pixel_width = physical_image_width / resolution_x

        unadjusted_x_offset = (target_x - (resolution_x / 2.0)) * physical_pixel_width
        unadjusted_y_offset = (target_y - (resolution_y / 2.0)) * physical_pixel_width

        adjusted_x_offset = unadjusted_x_offset * math.cos(math.radians(curr_orientation)) + unadjusted_y_offset * math.sin(math.radians(curr_orientation))
        adjusted_y_offset = (-unadjusted_x_offset) * math.sin(math.radians(curr_orientation)) + unadjusted_y_offset * math.cos(math.radians(curr_orientation))

        return [adjusted_x_offset, adjusted_y_offset]

    # This function takes in physical distances (in meters) as offsets from the current lat and lon, returning the lat and lon
    # of the offset target
    def offset_to_target_coords(self, offset_y, offset_x, curr_lat, curr_lon):
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
    def offset_to_distance(self, offset_x, offset_y):
        return math.sqrt((offset_x * offset_x) + (offset_y * offset_y))

    # This function is essentially a wrapper that calls both image_to_offset() and offset_to_target_coords(). If this function is called, neither 
    # image_to_offset() nor offset_to_target() need to be called individually.
    def get_target_coords(self, curr_lat, curr_lon, altitude, curr_orientation, target_x, target_y, FOV, resolution_x, resolution_y):
        transformed_offset = self.image_to_offset(altitude, curr_orientation, target_x, target_y, FOV, resolution_x, resolution_y)
        target_coords = self.offset_to_target_coords(transformed_offset[1], transformed_offset[0], curr_lat, curr_lon)
        return target_coords

    # END TARGET COORDINATE CODE

    def check_for_target(self):
        
        imageFrame = self.src.fetch_frame()

        hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)

        # Set range for red color and
        # define mask
        red_lower = np.array([136, 87, 111], np.uint8)
        red_upper = np.array([180, 255, 255], np.uint8)
        red_mask = cv2.inRange(hsvFrame, red_lower, red_upper)

        # Morphological Transform, Dilation
        # for each color and bitwise_and operator
        # between imageFrame and mask determines
        # to detect only that particular color
        kernal = np.ones((5, 5), "uint8")

        # For red color
        red_mask = cv2.dilate(red_mask, kernal)
        res_red = cv2.bitwise_and(imageFrame, imageFrame,mask = red_mask)

        # Creating contour to track red color
        contours, hierarchy = cv2.findContours(red_mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        
        for pic, contour in enumerate(contours):
            # cv2.approxPloyDP() function to approximate the shape
            approx = cv2.approxPolyDP(contour, 0.01 * cv2.arcLength(contour, True), True)
            # using drawContours() function, can comment out
            cv2.drawContours(imageFrame, [contour], 0, (0, 0, 255), 5)

            # finding center point of shape
            M = cv2.moments(contour)
            # calc moment of our shape
            if M['m00'] != 0.0:
                x_M = int(M['m10']/M['m00'])
                y_M = int(M['m01']/M['m00'])
            area = cv2.contourArea(contour)
            # if area is red and quadrilateral
            if(area > 300 and len(approx) >= 3 and len(approx) < 5):
                x, y, w, h = cv2.boundingRect(contour)
                imageFrame = cv2.rectangle(imageFrame, (x, y),
                (x + w, y + h),
                (0, 0, 255), 2)
                print(f"({x_M}, {y_M})\n")
                self.frame_coord.set(x_M, y_M)
                cv2.putText(imageFrame, "Red Square", (x, y),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                (0, 0, 255))
                # cv2.imshow("Multiple Color Detection in Real-TIme", imageFrame)
                cv2.imwrite("test.png", imageFrame)
                return True
        # cv2.imshow("Multiple Color Detection in Real-TIme", imageFrame)
        return False
