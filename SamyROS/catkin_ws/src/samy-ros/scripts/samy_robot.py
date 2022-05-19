from pubsub import pub
import yaml
import logging

from moveit_robot import MoveItRobot


class SAMY_Robot():
    def __init__(self, global_settings):
        self.logger = logging.getLogger(__name__)
        self.logger.setLevel(logging.DEBUG)
        log_handler = logging.StreamHandler()
        log_handler.setFormatter(logging.Formatter("%(levelname)s %(filename)s - %(message)s"))
        self.logger.addHandler(log_handler)

        self.global_settings = global_settings
        self.moveit_robot = MoveItRobot()

        # Subscribe to Topics
        self.logger.info("Subscribing to topics")
        pub.subscribe(self.move_to, "MoveTo")
        pub.subscribe(self.popup, "Message")
        pub.subscribe(self.get_status, "GetStatus")
        pub.subscribe(self.set_actuators, "ActuateJoints")

    def popup(self, data):
        self.logger.info("Got Message")

    def move_to(self,data):
        self.logger.info("Got Move To command")

    def get_status(self, data):
        self.moveit_robot.print_ros_robot_info()

    def set_actuators(self, data):
        pass
        