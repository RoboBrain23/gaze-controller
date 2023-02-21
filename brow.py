from math import hypot
import numpy as np
import cv2
from eye import Eye

class Brows:
    """ 
    a class to represent the Brows of a face.
    """
    LEFT_BROW = [22,23,24,25,26]     
    RIGHT_BROW= [17,18,19,20,21] 

    def __init__(self, landmarks):
        self.__landmarks = landmarks
        self.__left_brow_region = np.array([(self.__landmarks.part(point).x, self.__landmarks.part(point).y) for point in Brows.LEFT_BROW]).astype(np.int32)
        self.__right_brow_region = np.array([(self.__landmarks.part(point).x, self.__landmarks.part(point).y) for point in Brows.RIGHT_BROW]).astype(np.int32)
        self.__brow_distance = self.calculate_brow_distance()
        self.__threshold = 0

    def draw_brows(self, frame):
        """
        Draw the brow region on the frame.
        """
        self.__frame = frame
        cv2.polylines(self.__frame, [self.__left_brow_region], True, 255,2 )
        cv2.polylines(self.__frame, [self.__right_brow_region], True, 255,2 )
    
    def calculate_brow_distance(self):
        """
        Calculate the distance between the top of the eye and the top of the brow.
        """
        # right brow distance
        top_right_eye_point = Eye.mid_point_v2(self.__landmarks.part(46),self.__landmarks.part(47))
        top_right_brow_point = (self.__landmarks.part(24).x,self.__landmarks.part(24).y)
        right_brow_distance = hypot(top_right_brow_point[0]-top_right_eye_point[0], top_right_brow_point[1]-top_right_eye_point[1])
        
        # left brow distance
        top_left_eye_point = Eye.mid_point_v2(self.__landmarks.part(42),self.__landmarks.part(43))
        top_left_brow_point = (self.__landmarks.part(19).x,self.__landmarks.part(19).y)
        left_brow_distance = hypot(top_left_brow_point[0]-top_left_eye_point[0], top_left_brow_point[1]-top_left_eye_point[1])

        # average brow distance
        # brow_distance = (right_brow_distance + left_brow_distance)/2
        brow_distance = right_brow_distance
        return brow_distance

    def get_brow_distance(self):
        """
        Get the distance between the top of the eye and the top of the brow.
        """
        return self.__brow_distance
    
    def set_brows_threshold(self, threshold):
        """
        Set the threshold for the brow distance.
        """
        self.__threshold = threshold

    def is_up(self):
        """
        Check if the brow is up.
        """
        if self.__brow_distance >= self.__threshold:
            return True
        else:
            return False