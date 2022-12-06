import math
import numpy as np
import cv2


class Eye():
    LEFT_EYE_POINTS = [36, 37, 38, 39, 40, 41]
    RIGHT_EYE_POINTS = [42, 43, 44, 45, 46, 47]

    def __init__(self, frame,side):
        self.__frame = frame
        self.__side = side
        if self.__side.lower == "left":
            self.__eye = self.LEFT_EYE_POINTS
        elif self.__side.lower() == "right":
            self.__eye = self.RIGHT_EYE_POINTS


    @staticmethod
    def mid_point(p1,p2):
        """Returns the middle point (x,y) between two points

        Arguments:
            p1 (dlib.point): First point
            p2 (dlib.point): Second point
        """

        return (int((p1.x+p2.x)/2),int((p1.y+p2.y)/2))
