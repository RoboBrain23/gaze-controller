class Blink:
    """
    This class deals with eye blinking
    """

    def __init__(self, blink_ratio):
        self.blink_ratio = blink_ratio
        self.__threshold = 5.5

    def set_blinking_threshold(self, blink_threshold):
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

    def is_blinking_v2(self):
        """Check if the eye is blinking on Ear"""
        if self.blink_ratio < self.__threshold:
            return True
        return False
    def get_blinking_threshold(self):
        return self.__threshold