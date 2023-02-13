import math
import numpy as np
import cv2
import dlib
from gaze import Gaze
from eye import Eye
from blink import Blink
from calibration import Calibration


class Movement:
    def __init__(self, gaze_ratio):
        self.total_blinks = 0
        self.blinks_counter = 0
        self.left_counter = 0
        self.right_counter = 0
        self.center_counter = 0
        self.flag = 0
        self.closed_eyes_frame = 10
        self.eye_direction_frame = 10
        self.left_eye_thresh = 100
        self.right_eye_thresh = 100
        self.is_right = False
        self.is_left = False
        self.is_center = False
        self.gaze_ratio = gaze_ratio
        self.is_running = False

    def run(self):
        if 2 <= self.total_blinks <= 3:
            self.is_running = True
        elif self.total_blinks>3:
            self.total_blinks = 0
            self.is_running = False
        else:
            self.is_running = False

    def driver(self):
        self.run()
        if self.is_running:
            if self.gaze_ratio == 1:
                self.move_center()
            elif self.gaze_ratio == 0:
                self.move_left()
            elif self.gaze_ratio == 2:
                self.move_right()
            else:
                self.stop()

    def move_counter_reset(self):
        self.right_counter = 0
        self.left_counter = 0
        self.center_counter = 0

    def move_center(self):
        # if self.gaze_ratio == 1:
        self.is_center = True
        self.center_counter += 1
        # cv2.putText(frame, "STATE : LEFT", (50, 100), FONT, 1, (0, 0, 255), 3)
        if self.center_counter > self.eye_direction_frame:
            self.stop()
            print('Left')

    def move_left(self):
        # if self.gaze_ratio == 0:
        self.is_center = True
        self.left_counter += 1
        # cv2.putText(frame, "STATE : LEFT", (50, 100), FONT, 1, (0, 0, 255), 3)
        if self.left_counter > self.eye_direction_frame:
            self.stop()
            print('Left')

    def move_right(self):
        # if self.gaze_ratio == 2:
        self.is_center = True
        self.right_counter += 1
        # cv2.putText(frame, "STATE : LEFT", (50, 100), FONT, 1, (0, 0, 255), 3)
        if self.right_counter > self.eye_direction_frame:
            self.stop()
            print('Left')

    def stop(self):
        self.is_center = False
        self.is_left = False
        self.is_right = False
        self.move_counter_reset()

if __name__ == "__main__":
    TOTAL_BLINKS = 0
    BLINKS_COUNTER = 0
    LEFT_COUNTER = 0
    RIGHT_COUNTER = 0
    CENTER_COUNTER = 0
    FLAG = 0
    CLOSED_EYES_FRAME = 10
    EYE_DIRECTION_FRAME = 10
    MODEL = "shape_predictor_68_face_landmarks.dat"
    FONT = cv2.FONT_HERSHEY_SIMPLEX

    left_eye_thresh = 100
    right_eye_thresh = 100
    # blinking_threshold = 5.2
    blinks = []
    k = True
    calibrate = Calibration()
    try:
        cap = cv2.VideoCapture(0)
        detector = dlib.get_frontal_face_detector()
        predictor = dlib.shape_predictor(MODEL)

        while True:
            try:
                ret, frame = cap.read()
                frame = cv2.flip(frame, 1)
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                faces = detector(gray)
                height, width, _ = frame.shape
                mask = np.zeros((height, width), dtype=np.uint8)

                for face in faces:
                    x, y = face.left(), face.top()
                    x1, y1 = face.right(), face.bottom()
                    cv2.rectangle(frame, (x, y), (x1, y1), (0, 255, 0), 2)
                    landmarks = predictor(gray, face)
                    left_eye = Eye(frame, 'left', landmarks)
                    right_eye = Eye(frame, 'right', landmarks)
                    # Detect blinking
                    left_eye_ratio = left_eye.blink_ratio()
                    right_eye_ratio = right_eye.blink_ratio()
                    blinking_ratio = (left_eye_ratio + right_eye_ratio) / 2
                    blink = Blink(blinking_ratio)

                    blink.set_blinking_threshold(calibrate.get_cal_blink_threshold())

                    if blink.is_blinking() and calibrate.is_calibrated:
                        is_closed = True
                        cv2.putText(frame, "BLINKING", (50, 150), FONT, 3, (255, 0, 0))
                    else:
                        is_closed = False

                    if is_closed:
                        BLINKS_COUNTER += 1
                    else:
                        BLINKS_COUNTER = 0

                    if BLINKS_COUNTER == CLOSED_EYES_FRAME:
                        FLAG = 1
                        BLINKS_COUNTER = 0

                    if FLAG == 1 and not is_closed:
                        TOTAL_BLINKS += 1
                        FLAG = 0

                    cv2.putText(frame, f'Total Blinks: {TOTAL_BLINKS}', (50, 350), FONT, 2, (0, 0, 255), 3)

                    gaze_right = Gaze(right_eye.get_eye_region(), mask, gray)
                    gaze_right.set_threshold(calibrate.get_cal_right_eye_thresh())  # change right eye threshold
                    gaze_left = Gaze(left_eye.get_eye_region(), mask, gray)
                    gaze_left.set_threshold(calibrate.get_cal_left_eye_thresh())  # change left eye threshold

                    # Calibrate
                    calibrate.calibrate(gaze_left, gaze_right, blinking_ratio)
                    if calibrate.is_cal_threshold:
                        cv2.putText(frame, "Calibrating Threshold", (150, 50), FONT, 1, (200, 0, 200), 2)
                    elif calibrate.is_cal_blink:
                        cv2.putText(frame, "Calibrating Blinking", (150, 50), FONT, 1, (200, 0, 200), 2)

                    gaze_ratio = math.ceil((gaze_right.get_gaze_ratio() + gaze_left.get_gaze_ratio()) / 2)

                    if TOTAL_BLINKS == 2 or TOTAL_BLINKS == 3:
                        if gaze_ratio == 0 and not blink.is_blinking():
                            LEFT_COUNTER += 1
                            cv2.putText(frame, "STATE : LEFT", (50, 100), FONT, 1, (0, 0, 255), 3)
                            if LEFT_COUNTER > EYE_DIRECTION_FRAME:  # frame=20
                                RIGHT_COUNTER = 0
                                LEFT_COUNTER = 0
                                CENTER_COUNTER = 0
                                print('Left')
                        elif gaze_ratio == 2 and not blink.is_blinking():
                            RIGHT_COUNTER += 1
                            cv2.putText(frame, "STATE : RIGHT", (50, 100), FONT, 1, (0, 0, 255), 3)
                            if RIGHT_COUNTER > EYE_DIRECTION_FRAME:
                                RIGHT_COUNTER = 0
                                LEFT_COUNTER = 0
                                CENTER_COUNTER = 0
                                print('right')
                        elif gaze_ratio == 1 and not blink.is_blinking():
                            CENTER_COUNTER += 1
                            cv2.putText(frame, "STATE : CENTER", (50, 100), FONT, 1, (0, 0, 255), 3)
                            if CENTER_COUNTER > EYE_DIRECTION_FRAME:
                                RIGHT_COUNTER = 0
                                LEFT_COUNTER = 0
                                CENTER_COUNTER = 0
                                print('forward')
                        else:
                            cv2.putText(frame, "STATE : STOP", (50, 100), FONT, 1, (0, 0, 255), 3)
                            RIGHT_COUNTER = 0
                            LEFT_COUNTER = 0
                            CENTER_COUNTER = 0
                            # print('stop')

                    if TOTAL_BLINKS > 3:
                        TOTAL_BLINKS = 0

                cv2.imshow('frame', frame)
                # cv2.imshow('mask',mask)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            except Exception as e:
                print("Error processing the frame:", e)
                pass
        cap.release()
        cv2.destroyAllWindows()
    except Exception as e:
        print("Error initializing the camera or the facial landmark model:", e)
