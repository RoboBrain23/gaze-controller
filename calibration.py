from eye import Eye
import cv2


class Calibration:

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

    @staticmethod
    def blink_calibrate(blink_ratio):
        if blink_ratio > 5.7:
            blink_ratio = 5.7
        elif blink_ratio < 4.5:
            blink_ratio = 4.5
        return blink_ratio
