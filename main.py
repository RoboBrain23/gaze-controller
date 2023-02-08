import math
import numpy as np
import cv2
import dlib
from gaze import Gaze
from eye import Eye
from blink import Blink
from calibration import Calibration

if __name__ == "__main__":
    TOTAL_BLINKS = 0
    BLINKS_COUNTER = 0
    LEFT_COUNTER = 0
    RIGHT_COUNTER = 0
    CENTER_COUNTER = 0
    FLAG = 0
    CLOSED_EYES_FRAME = 10
    EYE_DIRECTION_FRAME = 10
    CALIBRATION_FRAME = 200
    is_calibrated = False
    MODEL = "shape_predictor_68_face_landmarks.dat"
    FONT = cv2.FONT_HERSHEY_SIMPLEX

    left_eye_thresh = 100
    right_eye_thresh = 100
    blink_ratio = 4

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

                    blink.set_blinking_threshold(blink_ratio)

                    if blink.is_blinking() and is_calibrated:
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
                    gaze_right.set_threshold(right_eye_thresh)  # change right eye threshold
                    gaze_left = Gaze(left_eye.get_eye_region(), mask, gray)
                    gaze_left.set_threshold(left_eye_thresh)  # change left eye threshold
                    # print(gaze_left.get_threshold())

                    # Calibrate
                    if CALIBRATION_FRAME > 100:
                        calibrate = Calibration()
                        left_eye_thresh = calibrate.threshold_calibrate(gaze_left, left_eye_thresh)
                        right_eye_thresh = calibrate.threshold_calibrate(gaze_right, right_eye_thresh)
                        CALIBRATION_FRAME -= 1
                        cv2.putText(frame, "CALIBRATING THRESHOLD", (50, 100), FONT, 2, (255, 0, 0))
                        print(f'Left eye threshold: {left_eye_thresh}')
                        print(f'Right eye threshold: {right_eye_thresh}')
                    elif 100 >= CALIBRATION_FRAME > 0:
                        CALIBRATION_FRAME -= 1
                        cv2.putText(frame, "CALIBRATING BLINK", (50, 100), FONT, 2, (255, 0, 0))
                        # blink_ratio = calibrate.blink_calibrate(blinking_ratio)
                        blink_ratio = blinking_ratio
                        print(f'Blink ratio: {blink_ratio}')
                        is_calibrated = True


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
