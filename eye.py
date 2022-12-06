import math
import numpy as np
import cv2,dlib


class Eye:
    LEFT_EYE_POINTS = [36, 37, 38, 39, 40, 41]
    RIGHT_EYE_POINTS = [42, 43, 44, 45, 46, 47]
    FONT = cv2.FONT_HERSHEY_SIMPLEX
    MODEL = "shape_predictor_68_face_landmarks.dat"

    def __init__(self, frame,side,landmarks):
        self.__frame = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        self.__side = side
        if self.__side.lower() == "left":
            self.__eye = self.LEFT_EYE_POINTS
        elif self.__side.lower() == "right":
            self.__eye = self.RIGHT_EYE_POINTS
        self.__landmarks = landmarks
        # self.__detector = None
        # self.__predictor = None
        # self.mask = None


    @staticmethod
    def mid_point(p1,p2):
        """Returns the middle point (x,y) between two points

        Arguments:
            p1 (dlib.point): First point
            p2 (dlib.point): Second point
        """

        return (int((p1[0]+p2[0])/2),int((p1[1]+p2[1])/2))

    def get_eye_region(self,points):
        eye_region = np.array([ (self.__landmarks.part(points[0]).x,self.__landmarks.part(points[0]).y),
                            (self.__landmarks.part(points[1]).x,self.__landmarks.part(points[1]).y),
                            (self.__landmarks.part(points[2]).x,self.__landmarks.part(points[2]).y),
                            (self.__landmarks.part(points[3]).x,self.__landmarks.part(points[3]).y),
                            (self.__landmarks.part(points[4]).x,self.__landmarks.part(points[4]).y),
                            (self.__landmarks.part(points[5]).x,self.__landmarks.part(points[5]).y)],np.int32)
        
        return eye_region

    def __analysis(self,frame):
        self.__detector = dlib.get_frontal_face_detector()
        self.__predictor = dlib.shape_predictor(self.MODEL)
        faces = self.__detector(self.__frame)
        height,width,_ = frame.shape
        self.mask = np.zeros((height,width),dtype=np.uint8)

    def blink_ratio(self):
        """Returns blinking ratio
        """

        horizontal_distance = self.get_eye_width()
        vertical_distance = self.get_eye_height()
        ratio = horizontal_distance/vertical_distance
        return ratio

    def get_eye_width(self):
        """Returns Width of Eye

        Arguments:
            points : Eye points
        """
        points = self.get_eye_region(self.__eye)
        left_point = points[0]
        right_point = points[3]
        horizontal_distance = math.hypot(left_point[0]-right_point[0],
                                        left_point[1]-right_point[1])
        return horizontal_distance
    
    def get_eye_height(self):
        """Returns Height of Eye

        Arguments:
            points : Eye points
        """
        points = self.get_eye_region(self.__eye)
        mid_point_up = self.mid_point(points[1],points[2])
        mid_point_down = self.mid_point(points[4],points[5])
        vertical_distance = math.hypot(mid_point_up[0]-mid_point_down[0],
                                        mid_point_up[1]-mid_point_down[1])
        return vertical_distance

class blink:
    def __init__(self,blink_ratio):
        self.blink_ratio = blink_ratio

    

if __name__ == "__main__":
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
            left_eye = Eye(frame,'left',landmarks)
            right_eye = Eye(frame,'right',landmarks)
            # Detect blinking
            left_eye_ratio = left_eye.blink_ratio()
            print(left_eye_ratio)
            right_eye_ratio = right_eye.blink_ratio()
            blinking_ratio = (left_eye_ratio + right_eye_ratio) / 2
            
            # if blinking_ratio > 5.7:
            #     BLINKS_COUNTER +=1
            #     cv2.putText(frame, "BLINKING", (50, 150), font,3,(255,0,0))
                
            # else:
            #     if BLINKS_COUNTER>CLOSED_EYES_FRAME:
            #         TOTAL_BLINKS +=1
            #         BLINKS_COUNTER =0
            # cv2.putText(frame,  f'Total Blinks: {TOTAL_BLINKS}',(50, 350), font, 2, (0, 0, 255), 3)


            # left_ratio = get_blink_ratio(LEFT_EYE_POINTS,landmarks)
            # right_ratio = get_blink_ratio(RIGHT_EYE_POINTS,landmarks)
            # blink_ratio = (left_ratio+right_ratio)/2
            # # print(blink_ratio)
            if(blinking_ratio>5.7):
                cv2.putText(frame,"Blinking",(50,150),font,3,(255,0,0))
            # #Need Calibration
            # #gaze_ratio = (get_gaze_ratio(RIGHT_EYE_POINTS,landmarks)+get_gaze_ratio(LEFT_EYE_POINTS,landmarks))/2
            # right_rat=get_gaze_ratio_v2(RIGHT_EYE_POINTS,landmarks,frame,gray,mask)
            # left_rat = get_gaze_ratio_v2(LEFT_EYE_POINTS,landmarks,frame,gray,mask)
            # print("left : ",left_rat," right : ",right_rat)
        
            # gaze_ratio = ceil((get_gaze_ratio_v2(RIGHT_EYE_POINTS,landmarks,frame,gray,mask)+get_gaze_ratio_v2(LEFT_EYE_POINTS,landmarks,frame,gray,mask))//2)
            # print(gaze_ratio)
        
            # if gaze_ratio == 2:
            #     cv2.putText(frame, "STATE : RIGHT", (350, 50), font, 1, (0, 0, 255), 3)
            # elif gaze_ratio == 1:
            #     cv2.putText(frame, "STATE : CENTER", (350, 50), font, 1, (0, 0, 255), 3)
            # else:
            #     # LEFT_COUNTER+=1
            #     cv2.putText(frame, "STATE : LEFT", (350, 50), font, 1, (0, 0, 255), 3)
        cv2.imshow('frame',frame)
        # cv2.imshow('mask',mask)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()