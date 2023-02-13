<<<<<<< HEAD
import cv2
import dlib
import numpy as np
from math import hypot,ceil
import random
# counters
frame_counter =0
BLINKS_COUNTER =0
TOTAL_BLINKS =0
LEFT_COUNTER=0
TOTAL_LEFT=0
RIGHT_COUNTER=0
# last_order="IDLE"
# constants
CLOSED_EYES_FRAME =15

LEFT_EYE_POINTS = [36, 37, 38, 39, 40, 41]
RIGHT_EYE_POINTS = [42, 43, 44, 45, 46, 47]


def mid_point(p1,p2):
    return (int((p1.x+p2.x)/2),int((p1.y+p2.y)/2))


def get_gaze_ratio(points,landmarks,frame,gray):
    eye_threshold = get_eye_threshold(points,landmarks,frame,gray)

    height,width = eye_threshold.shape
    left_side_eye_threshold = eye_threshold[0:height,0:int(width/2)]
    left_side_eye_threshold_white = cv2.countNonZero(left_side_eye_threshold)

    right_side_eye_threshold = eye_threshold[0:height,int(width/2):]
    right_side_eye_threshold_white = cv2.countNonZero(right_side_eye_threshold)
    if left_side_eye_threshold_white == 0:
        gaze_ratio = 0.3
    elif right_side_eye_threshold_white == 0:
        gaze_ratio = 5
    else:
        gaze_ratio = left_side_eye_threshold_white / right_side_eye_threshold_white
    return gaze_ratio

def get_blink_ratio(points,landmarks):
    #Right Eye Right and Left points
    left_point = (landmarks.part(points[0]).x,landmarks.part(points[0]).y)
    right_point = (landmarks.part(points[3]).x,landmarks.part(points[3]).y)
    #Right
    mid_point_up = mid_point(landmarks.part(points[1]),landmarks.part(points[2]))
    mid_point_down = mid_point(landmarks.part(points[4]),landmarks.part(points[5]))
    vertical_distance = hypot(mid_point_up[0]-mid_point_down[0],
                                      mid_point_up[1]-mid_point_down[1])
    horizontal_distance = hypot(left_point[0]-right_point[0],
                                      left_point[1]-right_point[1])
    ratio = horizontal_distance/vertical_distance

    return ratio

def get_gaze_ratio_v2(points,landmarks,frame,gray,mask):
        eye_threshold = get_eye_threshold(points,landmarks,frame,gray,mask)
        height,width = eye_threshold.shape
        left_side_eye_threshold = eye_threshold[0:height,0:int(width/3)]
        left_side_eye_threshold_white = cv2.countNonZero(left_side_eye_threshold)

        center_side_eye_threshold = eye_threshold[0:height,int(width/3):int(2*width/3)]
        center_side_eye_threshold_white = cv2.countNonZero(center_side_eye_threshold)

        right_side_eye_threshold = eye_threshold[0:height,int(2*width/3):]
        right_side_eye_threshold_white = cv2.countNonZero(right_side_eye_threshold)

        #left
        if left_side_eye_threshold_white < right_side_eye_threshold_white and left_side_eye_threshold_white< center_side_eye_threshold_white:
            gaze_ratio = 0
        #right
        elif left_side_eye_threshold_white > right_side_eye_threshold_white and right_side_eye_threshold_white < center_side_eye_threshold_white:
            gaze_ratio = 2
        #center
        else:
            gaze_ratio = 1
        return gaze_ratio

def get_min_max_eye_region(eye_region):
    min_x = np.min(eye_region[:,0])
    max_x = np.max(eye_region[:,0])
    min_y = np.min(eye_region[:,1])
    max_y = np.max(eye_region[:,1])
    return min_x,max_x,min_y,max_y


def get_eye_region(points,landmarks):
    eye_region = np.array([ (landmarks.part(points[0]).x,landmarks.part(points[0]).y),
                        (landmarks.part(points[1]).x,landmarks.part(points[1]).y),
                        (landmarks.part(points[2]).x,landmarks.part(points[2]).y),
                        (landmarks.part(points[3]).x,landmarks.part(points[3]).y),
                        (landmarks.part(points[4]).x,landmarks.part(points[4]).y),
                        (landmarks.part(points[5]).x,landmarks.part(points[5]).y)],np.int32)
    
    return eye_region

def get_eye_threshold(points,landmarks,frame,gray,mask):
    eye_region = get_eye_region(points,landmarks)
    min_x,max_x,min_y,max_y = get_min_max_eye_region(eye_region)
    cv2.polylines(mask,[eye_region],True,0,5)
    cv2.fillPoly(mask,[eye_region],255)
    gray_eye = cv2.bitwise_and(gray,gray,mask=mask)
    eye = gray_eye[min_y:max_y,min_x:max_x]
    eye = cv2.resize(eye, None,fx=5,fy=5)

    _ , eye_threshold = cv2.threshold(eye,45,255,cv2.THRESH_BINARY)
    cv2.imshow('eye_threshold',eye_threshold)
    return eye_threshold


cap = cv2.VideoCapture(1)
detector = dlib.get_frontal_face_detector()
predictor = dlib.shape_predictor("shape_predictor_68_face_landmarks.dat")
font = cv2.FONT_HERSHEY_SIMPLEX


while True:
    ret, frame = cap.read()
    frame= cv2.flip(frame, 1)
    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    faces = detector(gray)
    height,width,_ = frame.shape
    mask = np.ones((height,width),dtype=np.uint8)*0

    for face in faces:
        x,y = face.left(),face.top()
        x1,y1 = face.right(),face.bottom()
        cv2.rectangle(frame, (x,y), (x1,y1), (0,255,0),2)
        landmarks = predictor(gray,face)

        # Detect blinking
        left_eye_ratio = get_blink_ratio([36, 37, 38, 39, 40, 41], landmarks)
        right_eye_ratio = get_blink_ratio([42, 43, 44, 45, 46, 47], landmarks)
        blinking_ratio = (left_eye_ratio + right_eye_ratio) / 2
        
        if blinking_ratio > 5.7:
            BLINKS_COUNTER +=1
            cv2.putText(frame, "BLINKING", (50, 150), font,3,(255,0,0))
            
        else:
            if BLINKS_COUNTER>CLOSED_EYES_FRAME:
                TOTAL_BLINKS +=1
                BLINKS_COUNTER =0
        cv2.putText(frame,  f'Total Blinks: {TOTAL_BLINKS}',(50, 350), font, 2, (0, 0, 255), 3)


        left_ratio = get_blink_ratio(LEFT_EYE_POINTS,landmarks)
        right_ratio = get_blink_ratio(RIGHT_EYE_POINTS,landmarks)
        blink_ratio = (left_ratio+right_ratio)/2
        # print(blink_ratio)
        if(blink_ratio>5.7):
            cv2.putText(frame,"Blinking",(50,150),font,3,(255,0,0))
        #Need Calibration
        #gaze_ratio = (get_gaze_ratio(RIGHT_EYE_POINTS,landmarks)+get_gaze_ratio(LEFT_EYE_POINTS,landmarks))/2
        right_rat=get_gaze_ratio_v2(RIGHT_EYE_POINTS,landmarks,frame,gray,mask)
        left_rat = get_gaze_ratio_v2(LEFT_EYE_POINTS,landmarks,frame,gray,mask)
        print("left : ",left_rat," right : ",right_rat)
       
        gaze_ratio = ceil((get_gaze_ratio_v2(RIGHT_EYE_POINTS,landmarks,frame,gray,mask)+get_gaze_ratio_v2(LEFT_EYE_POINTS,landmarks,frame,gray,mask))//2)
        print(gaze_ratio)
       
        if gaze_ratio == 2:
            cv2.putText(frame, "STATE : RIGHT", (350, 50), font, 1, (0, 0, 255), 3)
        elif gaze_ratio == 1:
            cv2.putText(frame, "STATE : CENTER", (350, 50), font, 1, (0, 0, 255), 3)
        else:
            # LEFT_COUNTER+=1
            cv2.putText(frame, "STATE : LEFT", (350, 50), font, 1, (0, 0, 255), 3)


        # #######################
        # if gaze_ratio == 0:
        #     LEFT_COUNTER +=1
        #     cv2.putText(frame, "LEFT", (50, 100), font, 1, (0, 0, 255), 3)  
        # else:
        #     if LEFT_COUNTER>CLOSED_EYES_FRAME:
        #         last_order= "LEFT"
        #         LEFT_COUNTER =0

        # if gaze_ratio == 2:
        #     LEFT_COUNTER +=1
        #     cv2.putText(frame, "RIGHT", (50, 100), font, 1, (0, 0, 255), 3)  
        # else:
        #     if LEFT_COUNTER>CLOSED_EYES_FRAME:
        #         last_order= "RIGHT"
        #         LEFT_COUNTER =0
       
        # cv2.putText(frame, ("LAST Order"+last_order), (50, 200), font, 1, (0, 0, 255), 3)
        # ########################
 
    # cv2.polylines(frame,[get_eye_region(LEFT_EYE_POINTS,landmarks),get_eye_region(RIGHT_EYE_POINTS,landmarks)],True,(100,100,0),1)
    cv2.imshow('frame',frame)
    # cv2.imshow('mask',mask)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
=======
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
>>>>>>> Gaze/main
