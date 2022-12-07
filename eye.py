import math
import numpy as np
import cv2,dlib


class Eye:
    """
    This class creates a new frame to isolate the eye and
    get eye daimintions 
    """

    LEFT_EYE_POINTS = [36, 37, 38, 39, 40, 41]
    RIGHT_EYE_POINTS = [42, 43, 44, 45, 46, 47]

    def __init__(self, frame,side,landmarks):
        self.__frame = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        self.__side = side
        if self.__side.lower() == "left":
            self.__eye = self.LEFT_EYE_POINTS
        elif self.__side.lower() == "right":
            self.__eye = self.RIGHT_EYE_POINTS
        self.__landmarks = landmarks


    @staticmethod
    def mid_point(p1,p2):
        """Returns the middle point (x,y) between two points

        Arguments:
            p1 (dlib.point): First point
            p2 (dlib.point): Second point
        """

        return (int((p1[0]+p2[0])/2),int((p1[1]+p2[1])/2))

    def get_eye_region(self):
        """Returns Eye region from points

        Arguments:
            points: Eye points
        """
        points = self.__eye
        eye_region = np.array([ (self.__landmarks.part(points[0]).x,self.__landmarks.part(points[0]).y),
                            (self.__landmarks.part(points[1]).x,self.__landmarks.part(points[1]).y),
                            (self.__landmarks.part(points[2]).x,self.__landmarks.part(points[2]).y),
                            (self.__landmarks.part(points[3]).x,self.__landmarks.part(points[3]).y),
                            (self.__landmarks.part(points[4]).x,self.__landmarks.part(points[4]).y),
                            (self.__landmarks.part(points[5]).x,self.__landmarks.part(points[5]).y)],np.int32)
        
        return eye_region

    def get_eye_width(self):
        """Returns Width of Eye

        Arguments:
            points : Eye points
        """
        points = self.get_eye_region()
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
        points = self.get_eye_region()
        mid_point_up = self.mid_point(points[1],points[2])
        mid_point_down = self.mid_point(points[4],points[5])
        vertical_distance = math.hypot(mid_point_up[0]-mid_point_down[0],
                                        mid_point_up[1]-mid_point_down[1])
        return vertical_distance

class Blink:
    """
    This class deals with eye blinking
    """

    def __init__(self,blink_ratio):
        self.blink_ratio = blink_ratio
        self.__threshold = 5.5

    def set_blinking_threshold(self,blink_threshold):
        """set eye blinking threshold

        Arguments:
            blinking threshold : blink_threshold
        """
        self.__threshold = blink_threshold

    def is_blinking(self):       
        """Check if the eye is blinking"""
        if self.blink_ratio > self.__threshold:
            return True
        return False

class Gaze:
    """
    This class deals with eye gazing
    """
    def __init__(self,eye_region,mask,gray):
        self.eye_region = eye_region
        self.__threshold = 42
        self.mask = mask
        self.gray = gray

    def get_gaze_ratio(self):
        eye_threshold = self.get_eye_threshold(self.eye_region)
        height,width = eye_threshold.shape
        left_side_eye_threshold = eye_threshold[0:height,0:int(width/4)]
        left_side_eye_threshold_white = cv2.countNonZero(left_side_eye_threshold)

        center_side_eye_threshold = eye_threshold[0:height,int(width/4):int(3*width/4)]
        center_side_eye_threshold_white = cv2.countNonZero(center_side_eye_threshold)

        right_side_eye_threshold = eye_threshold[0:height,int(3*width/4):]
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

    def get_min_max_eye_region(self):
        min_x = np.min(self.eye_region[:,0])
        max_x = np.max(self.eye_region[:,0])
        min_y = np.min(self.eye_region[:,1])
        max_y = np.max(self.eye_region[:,1])
        return min_x,max_x,min_y,max_y


    def get_eye_threshold(self,eye_region):
        min_x,max_x,min_y,max_y = self.get_min_max_eye_region()
        cv2.polylines(self.mask,[eye_region],True,0,5)
        cv2.fillPoly(self.mask,[eye_region],255)
        gray_eye = cv2.bitwise_and(self.gray,self.gray,mask=self.mask)
        eye = gray_eye[min_y:max_y,min_x:max_x]
        eye = cv2.resize(eye, None,fx=5,fy=5)
        _ , eye_threshold = cv2.threshold(eye,self.__threshold,255,cv2.THRESH_BINARY)
        cv2.imshow('eye_threshold',eye_threshold)
        return eye_threshold

    def set_threshold(self,threshold):
        self.__threshold = threshold
    
    def get_threshold(self):
        return self.__threshold

if __name__ == "__main__":
    TOTAL_BLINKS =0
    BLINKS_COUNTER =0
    LEFT_COUNTER = 0
    RIGHT_COUNTER = 0
    CENTER_COUNTER = 0

    # last_order="IDLE"
    # constants
    CLOSED_EYES_FRAME =10
    EYE_DIRECTION_FRAME =20
    MODEL = "shape_predictor_68_face_landmarks.dat"
    FONT = cv2.FONT_HERSHEY_SIMPLEX

    cap = cv2.VideoCapture(1)
    detector = dlib.get_frontal_face_detector()
    predictor = dlib.shape_predictor(MODEL)
    

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
            # print(left_eye_ratio)
            right_eye_ratio = right_eye.blink_ratio()
            blinking_ratio = (left_eye_ratio + right_eye_ratio) / 2
            blink = Blink(blinking_ratio)
            blink.set_blinking_threshold(5.3)
            if blink.is_blinking():
                is_closed = True
                cv2.putText(frame, "BLINKING", (50, 150), FONT,3,(255,0,0))
            else:
                is_closed = False

            if is_closed:
                BLINKS_COUNTER +=1
            else:
                BLINKS_COUNTER =0

            if BLINKS_COUNTER == CLOSED_EYES_FRAME:
                TOTAL_BLINKS +=1
                BLINKS_COUNTER = 0



            cv2.putText(frame,  f'Total Blinks: {TOTAL_BLINKS}',(50, 350), FONT, 2, (0, 0, 255), 3)

            gaze_right = Gaze(right_eye.get_eye_region(),mask,gray)
            gaze_left = Gaze(left_eye.get_eye_region(),mask,gray)
            gaze_ratio = math.ceil((gaze_right.get_gaze_ratio()+gaze_left.get_gaze_ratio())/2)
            # if gaze_ratio == 2:
            #     cv2.putText(frame, "STATE : RIGHT", (350, 50), font, 1, (0, 0, 255), 3)
            # elif gaze_ratio == 1:
            #     cv2.putText(frame, "STATE : CENTER", (350, 50), font, 1, (0, 0, 255), 3)
            # else:
            #     # LEFT_COUNTER+=1
            #     cv2.putText(frame, "STATE : LEFT", (350, 50), font, 1, (0, 0, 255), 3)
            
            if TOTAL_BLINKS==2 or TOTAL_BLINKS==3:
                if gaze_ratio == 0:
                    LEFT_COUNTER +=1
                    cv2.putText(frame, "STATE : LEFT", (50, 100), FONT, 1, (0, 0, 255), 3)
                    if LEFT_COUNTER>EYE_DIRECTION_FRAME: #frame=20
                        RIGHT_COUNTER =0
                        LEFT_COUNTER =0
                        CENTER_COUNTER =0
                        print(f"Left")
                    # last_order= "LEFT"
                elif gaze_ratio == 2:
                    RIGHT_COUNTER +=1
                    cv2.putText(frame, "STATE : RIGHT", (50, 100), FONT, 1, (0, 0, 255), 3)
                    if RIGHT_COUNTER>EYE_DIRECTION_FRAME:
                        RIGHT_COUNTER =0
                        LEFT_COUNTER =0
                        CENTER_COUNTER =0
                        print('right')
                        # last_order= "RIGHT"
                elif gaze_ratio == 1 and not blink.is_blinking():
                    CENTER_COUNTER +=1
                    cv2.putText(frame, "STATE : CENTER", (50, 100), FONT, 1, (0, 0, 255), 3)
                    if CENTER_COUNTER>EYE_DIRECTION_FRAME:
                        RIGHT_COUNTER =0
                        LEFT_COUNTER =0
                        CENTER_COUNTER =0
                        print('forward')
            if TOTAL_BLINKS> 3:
                TOTAL_BLINKS=0

        cv2.imshow('frame',frame)
        # cv2.imshow('mask',mask)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        

    cap.release()
    cv2.destroyAllWindows()