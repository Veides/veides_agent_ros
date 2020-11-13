import logging


class RosNodeConnectedLogger(logging.Logger):
    def __init__(self, rospy, name=''):
        self.rospy = rospy
        self.name = name

    def log(self, level, message, *_):
        if level == logging.DEBUG:
            self.debug(message)
        elif level == logging.INFO:
            self.info(message)
        elif level == logging.WARN:
            self.warning(message)
        elif level == logging.ERROR:
            self.error(message)
        else:
            self.critical(message)

    def critical(self, message, *_):
        self.rospy.logfatal(self._format_message(message))

    def error(self, message, *_):
        self.rospy.logerr(self._format_message(message))

    def warning(self, message, *_):
        self.rospy.logwarn(self._format_message(message))

    def info(self, message, *_):
        self.rospy.loginfo(self._format_message(message))

    def debug(self, message, *_):
        self.rospy.logdebug(self._format_message(message))

    def _format_message(self, message):
        return '{} {}'.format(self.name, message)
