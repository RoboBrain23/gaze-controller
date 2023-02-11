import cv2
import numpy as np


class Calibration:
    """Calibration class for calibrating eye threshold and blink threshold"""

    def __init__(self, calibration_frames=200):
        """
        Initialize Calibration object
        """
        self.blink_threshold = 10
        self.blink_ratios = []
        self.calibration_frames = calibration_frames
        self.is_calibrated = False
        self.done = False
        self.left_eye_thresh = 42
        self.right_eye_thresh = 42

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

    def get_cal_frames(self):
        """
        Get calibration frames
        :return: calibration frames
        """
        return self.calibration_frames

    def set_cal_frames(self, frames):
        """
        Set calibration frames
        :param frames: calibration frames
        :return: None
        """
        self.calibration_frames = frames

    def calibrate(self, gaze_left, gaze_right, blinking_ratio):
        """
        Calibrate eye threshold and blink threshold
        :param gaze_left:  Eye object
        :param gaze_right: Eye object
        :param blinking_ratio:  blink ratio value
        :return: None
        """
        if self.calibration_frames > 100:
            self.left_eye_thresh = self.threshold_calibrate(gaze_left, self.left_eye_thresh)
            self.right_eye_thresh = self.threshold_calibrate(gaze_right, self.right_eye_thresh)
            self.calibration_frames -= 1
            print(f'Left eye threshold: {self.left_eye_thresh}')
            print(f'Right eye threshold: {self.right_eye_thresh}')
        elif 100 >= self.calibration_frames > 0:
            # todo: calibrate blinking threshold
            self.calibration_frames -= 1
            self.add_blink_ratio(blinking_ratio)
            print(f'Blink ratio: {blinking_ratio}')
        elif self.calibration_frames == 0:
            self.blink_calibrate()
            self.is_calibrated = True
        if self.is_calibrated and not self.done:
            print(f'Blinking threshold: {self.get_cal_blink_threshold()}')
            self.done = True

    def set_left_eye_thresh(self, thresh):
        """
        Set left eye threshold
        :param thresh: threshold of left eye
        :return:  None
        """
        self.left_eye_thresh = thresh

    def set_right_eye_thresh(self, thresh):
        """
        Set right eye threshold
        :param thresh: threshold of right eye
        :return:  None
        """
        self.right_eye_thresh = thresh

    def get_cal_left_eye_thresh(self):
        """
        Get calibrated left eye threshold
        :return: left eye threshold
        """
        return self.left_eye_thresh

    def get_cal_right_eye_thresh(self):
        """
        Get calibrated right eye threshold
        :return: right eye threshold
        """
        return self.right_eye_thresh
