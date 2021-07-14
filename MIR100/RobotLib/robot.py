
import logging
import requests
import json


class Robot(object):
    # This class represents the robot.

    def __init__(self, host):
        # create a logger for the robot object
        self.logger = logging.getLogger(__name__)
        self.logger.setLevel(logging.INFO)
        fh = logging.StreamHandler()
        fh.setFormatter(logging.Formatter("%(levelname)s %(filename)s - %(message)s"))
        self.logger.addHandler(fh)

        self.host = host
        self.url = "http://" + host + "/api/v2.0.0/"

        self.headers = {}
        self.headers["Conetent-Type"] = "application/json"
        self.headers["Authorization"] = "Basic ZGlzdHJpYnV0b3I6NjJmMmYwZjFlZmYxMGQzMTUyYzk1ZjZmMDU5NjU3NmU0ODJiYjhlNDQ4MDY0MzNmNGNmOTI5NzkyODM0YjAxNA=="
        self.headers["Accept_language"] = "en_US"

        self.missions = requests.get(self.url + "missions", headers=self.headers)
    # Add robot specific methodes here:

    def start_mission(self, message):
        # Search for message in available missions
        # get guid of that mission
        # post mission
        print(type(self.missions.text))
        missions = json.loads(self.missions.text)
        mission_id_post = {}
        for mission in missions:
            if message == mission["name"]:
                mission_id_post["mission_id"] =  mission["guid"]
        post_mission = requests.post(self.url + "mission_queue", json=mission_id_post, headers=self.headers)



    def delete_queue(self):
        delete = requests.delete(self.url + "mission_queue", headers=self.headers)
