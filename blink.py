class Blink:
    """
    This class deals with eye blinking
    """

    def __init__(self, blink_ratio):
        self.blink_ratio = blink_ratio
        self.__threshold = 5.5

    def set_blinking_threshold(self, blink_threshold):
        """
        set eye blinking threshold
        :param blink_threshold: threshold value
        :return: None
        """
        self.__threshold = blink_threshold

    def is_blinking(self):
        """
        Check if the eye is blinking
        :return: True if the eye is blinking else False
        """
        if self.blink_ratio > self.__threshold:
            return True
        return False

    def is_blinking_v2(self):
        """
        Check if the eye is blinking on Ear
        :return: True if the eye is blinking else False
        """
        if self.blink_ratio < self.__threshold:
            return True
        return False

    def get_blinking_threshold(self):
        """
        get eye blinking threshold
        :return: blinking threshold
        """
        return self.__threshold
