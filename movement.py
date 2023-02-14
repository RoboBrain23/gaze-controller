class Movement:
    def __init__(self):
        """
        Initialize Movement object
        """
        self.__total_blinks = 0
        self.__left_counter = 0
        self.__right_counter = 0
        self.__center_counter = 0
        self.__eye_direction_frame = 10
        self.__is_right = False
        self.__is_left = False
        self.__is_forward = False
        self.__gaze_ratio = 4
        self.__is_running = False

    def run(self):
        """
        Run the driver function if the total blinks are between 2 and 3 and stop if the total blinks are more than 3.
        :return: None
        """
        if 2 <= self.__total_blinks <= 3:
            self.__is_running = True
        elif self.__total_blinks > 3:
            self.__total_blinks = 0
            self.__is_running = False
        else:
            self.__is_running = False

    def driver(self):
        """
        Driver function to control the movement.
        :return: None
        """
        self.run()
        if self.__is_running:
            if self.__gaze_ratio == 1:
                self.move_forward()
            elif self.__gaze_ratio == 0:
                self.move_left()
            elif self.__gaze_ratio == 2:
                self.move_right()
            else:
                self.stop()

    def move_counter_reset(self):
        """
        Reset direction counters to 0 for a certain number of frames and not blinking.
        :return: None
        """
        self.__right_counter = 0
        self.__left_counter = 0
        self.__center_counter = 0

    def move_forward(self):
        """
        Move forward if gaze ratio is 1 for a certain number of frames and not blinking.
        :return: None
        """
        # if self.gaze_ratio == 1:
        self.__is_forward = True
        self.__center_counter += 1
        # cv2.putText(frame, "STATE : FORWARD", (50, 100), FONT, 1, (0, 0, 255), 3)
        if self.__center_counter > self.__eye_direction_frame:
            self.stop()
            print('Forward')

    def move_left(self):
        """
        Move left if gaze ratio is 0 for a certain number of frames and not blinking.
        :return: None
        """
        # if self.gaze_ratio == 0:
        self.__is_forward = True
        self.__left_counter += 1
        # cv2.putText(frame, "STATE : LEFT", (50, 100), FONT, 1, (0, 0, 255), 3)
        if self.__left_counter > self.__eye_direction_frame:
            self.stop()
            print('Left')

    def move_right(self):
        """
        Move right if gaze ratio is 2 for a certain number of frames and not blinking.
        :return: None
        """
        # if self.gaze_ratio == 2:
        self.__is_forward = True
        self.__right_counter += 1
        # cv2.putText(frame, "STATE : RIGHT", (50, 100), FONT, 1, (0, 0, 255), 3)
        if self.__right_counter > self.__eye_direction_frame:
            self.stop()
            print('Right')

    def stop(self):
        """
        Stop if gaze ratio is not 1, 0 or 2 and reset direction counters.
        :return:
        """
        # cv2.putText(frame, "STATE : STOP", (50, 100), FONT, 1, (0, 0, 255), 3)
        self.__is_forward = False
        self.__is_left = False
        self.__is_right = False
        self.move_counter_reset()

    def set_gaze_ratio(self, gaze_ratio):
        """
        Set gaze ratio to a certain value
        :param gaze_ratio: gaze ratio value
        :return: None
        """
        self.__gaze_ratio = gaze_ratio

    def is_forward(self):
        """
        Return if it is forward or not
        :return: is_forward
        """
        return self.__is_forward

    def is_left(self):
        """
        Return if it is left or not
        :return: is_left
        """
        return self.__is_left

    def is_right(self):
        """
        Return if it is right or not
        :return: is_right
        """
        return self.__is_right

    def is_stopped(self):
        """
        Return if it is stopped or not
        :return: is_stopped
        """
        return not self.__is_forward and not self.__is_left and not self.__is_right

    def set_total_blinks(self, total_blinks):
        """
        Set total blinks to a certain value
        :param total_blinks: total blinks value
        :return: None
        """
        self.__total_blinks = total_blinks % 4

    def set_eye_direction_frame(self, eye_direction_frame):
        """
        Set eye direction frame to a certain value
        :param eye_direction_frame: eye direction frame value
        :return: None
        """
        self.__eye_direction_frame = eye_direction_frame
