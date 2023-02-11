import math
import cv2
import numpy as np


class Eye:
    """
    This class creates a new frame to isolate the eye and
    get eye diminutions
    """
    LEFT_EYE_POINTS = [36, 37, 38, 39, 40, 41]
    RIGHT_EYE_POINTS = [42, 43, 44, 45, 46, 47]

    def __init__(self, frame, side, landmarks):
        """
        Initialize Eye object
        :param frame: frame to isolate the eye
        :param side: eye side (left or right)
        :param landmarks: landmarks of the eye
        """
        self.__frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        self.__side = side
        if self.__side.lower() == "left":
            self.__eye = self.LEFT_EYE_POINTS
        elif self.__side.lower() == "right":
            self.__eye = self.RIGHT_EYE_POINTS
        self.__landmarks = landmarks

    @staticmethod
    def mid_point(p1, p2):
        """
        Get the middle point (x,y) between two points
        :param p1: point 1
        :param p2: point 2
        :return: middle point (x,y)
        """

        return int((p1[0] + p2[0]) / 2), int((p1[1] + p2[1]) / 2)

    def get_eye_region(self):
        """
        Get Eye region from points
        :return: eye region (numpy array) of points
        """
        points = self.__eye
        eye_region = np.array([(self.__landmarks.part(points[0]).x, self.__landmarks.part(points[0]).y),
                               (self.__landmarks.part(points[1]).x, self.__landmarks.part(points[1]).y),
                               (self.__landmarks.part(points[2]).x, self.__landmarks.part(points[2]).y),
                               (self.__landmarks.part(points[3]).x, self.__landmarks.part(points[3]).y),
                               (self.__landmarks.part(points[4]).x, self.__landmarks.part(points[4]).y),
                               (self.__landmarks.part(points[5]).x, self.__landmarks.part(points[5]).y)], np.int32)

        return eye_region

    # todo: Fix division by zero error
    def blink_ratio(self):
        """
        Get blinking ratio
        :return: blink ratio
        """
        horizontal_distance = self.get_eye_width()
        vertical_distance = self.get_eye_height()
        ratio = horizontal_distance / vertical_distance
        return ratio

    def get_eye_width(self):
        """
        Get Width of Eye
        :return: width of eye
        """
        points = self.get_eye_region()
        left_point = points[0]
        right_point = points[3]
        horizontal_distance = math.hypot(left_point[0] - right_point[0],
                                         left_point[1] - right_point[1])
        return horizontal_distance

    def get_blink_ratio(self):
        """
        Get blinking ratio based on EAR
        :return: blink ratio
        """
        eye_region = self.get_eye_region()
        a = np.linalg.norm(eye_region[1] - eye_region[5])
        b = np.linalg.norm(eye_region[2] - eye_region[4])
        c = np.linalg.norm(eye_region[0] - eye_region[3])
        ear = (a + b) / (2.0 * c)
        return ear

    def get_eye_height(self):
        """
        Get Height of Eye
        :return: height of eye
        """
        points = self.get_eye_region()
        mid_point_up = self.mid_point(points[1], points[2])
        mid_point_down = self.mid_point(points[4], points[5])
        vertical_distance = math.hypot(mid_point_up[0] - mid_point_down[0],
                                       mid_point_up[1] - mid_point_down[1])
        return vertical_distance
