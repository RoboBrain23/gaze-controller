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