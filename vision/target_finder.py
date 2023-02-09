import cv2
import numpy as np

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
    
    def __init__(self, camera) -> None:
        self.src = camera
        self.frame_coord = self.FrameCoord()

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
                cv2.imshow("Multiple Color Detection in Real-TIme", imageFrame)
                cv2.imwrite("test.png", imageFrame)
                return True
        cv2.imshow("Multiple Color Detection in Real-TIme", imageFrame)
        return False