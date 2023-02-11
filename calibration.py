import cv2
import numpy as np


class Calibration:
    """Calibration class for calibrating eye threshold and blink threshold"""

    def __init__(self):
        """
        Initialize Calibration object
        """
        self.blink_threshold = 10
        self.blink_ratios = []

    @staticmethod
    def threshold_calibrate(eye_gaze, thresh):
        """
        Calibrate eye threshold
        :param eye_gaze: Eye object
        :param thresh: current threshold
        :return: thresh the updated threshold
        """
        if cv2.countNonZero(eye_gaze.get_eye_threshold(eye_gaze.eye_region)) < 1400 and thresh > 0:
            thresh -= 1
        elif cv2.countNonZero(eye_gaze.get_eye_threshold(eye_gaze.eye_region)) > 3000 and thresh < 255:
            thresh += 1
        return thresh

    def blink_calibrate(self):
        """
        Calibrate blink threshold
        :return: None
        """
        blinks = np.array(self.blink_ratios)
        self.blink_threshold = np.mean(blinks)

    def add_blink_ratio(self, blink_ratio):
        """
        Add blink ratio to list
        :param blink_ratio: blink ratio
        :return: None
        """
        if blink_ratio > 4:
            self.blink_ratios.append(blink_ratio)

    def get_cal_blink_threshold(self):
        """
        Get calibrated blink threshold
        :return: blink threshold
        """
        return self.blink_threshold

    def get_cal_eye_threshold(self):
        """
        Get calibrated eye threshold
        :return: eye threshold
        """
        return self.eye_threshold
