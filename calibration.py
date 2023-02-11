from eye import Eye
import cv2
import numpy as np


class Calibration:
    def __init__(self):
        self.blink_threshold = 10
        self.blink_ratios = []

    @staticmethod
    def threshold_calibrate(eye_gaze, thresh):
        # print(cv2.countNonZero(gaze_left.get_eye_threshold(gaze_left.eye_region)),
        #       cv2.countNonZero(gaze_right.get_eye_threshold(gaze_right.eye_region)))
        if cv2.countNonZero(eye_gaze.get_eye_threshold(eye_gaze.eye_region)) < 1400 and thresh > 0:
            thresh -= 1
            # eye_gaze.set_threshold(thresh)
        elif cv2.countNonZero(eye_gaze.get_eye_threshold(eye_gaze.eye_region)) > 3000 and thresh < 255:
            thresh += 1
            # eye_gaze.set_threshold(thresh)
        return thresh
        # return self.eye_threshold

        # if cv2.countNonZero(gaze_right.get_eye_threshold(gaze_right.eye_region)) < 1400:
        #     right_eye_thresh -= 1
        # elif cv2.countNonZero(gaze_right.get_eye_threshold(gaze_right.eye_region)) > 3000:
        #     right_eye_thresh += 1
        # print(left_eye_thresh,right_eye_thresh)

    def blink_calibrate(self):
        blinks = np.array(self.blink_ratios)
        self.blink_threshold = np.mean(blinks)

    def add_blink_ratio(self, blink_ratio):
        if blink_ratio > 4:
            self.blink_ratios.append(blink_ratio)

    def get_cal_blink_threshold(self):
        return self.blink_threshold

    def get_cal_eye_threshold(self):
        return self.eye_threshold
