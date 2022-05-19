import requests
import json
import time


class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

# Key for REST Communication with the MiR100
accept = 'application/json'
authorizationkey = 'Basic ZGlzdHJpYnV0b3I6NjJmMmYwZjFlZmYxMGQzMTUyYzk1ZjZmMDU5NjU3NmU0ODJiYjhlNDQ4MDY0MzNmNGNmOTI5NzkyODM0YjAxNA=='     #set the authorization key
#authorizationkey = 'Basic ZGlzdHJpYnV0b3I6NjJmMmYwZjFlZmYxMGQzMTUyYzk1ZjZmMDU5NjU3NmU0ODJiYjhlNDQ4MDY0MzNmNGNmOTI5NzkyODM0YjAxNA=='     #set the authorization key
language = 'en_US'

#Basic header for all commands
headers = {
    'accept': accept,
    'Authorization': authorizationkey,
    'Accept-Language': language,
    'Content-Type': 'application/json',
}

#Functions of the MiR100
'''
Move:
	move to x y w
	move to coordinate
	relative move
	switch map
Battery
	charging
Logic
	break
	continue
	if
	loop
	pause
	return
	wait
	while
Error handling
	create log
	throw error
	try/catch
Sound/light
	Play Sound
	Show light
UR
	Run UR program
'''

class MiR_Robot(object): #MiR REST class
    
    def __init__(self, address, key):
        #Basic url for requests
        url = 'http://' + address + '/api/v2.0.0/'
        self.url = url
        print(self.url)
        #Specific url
        self.status_url = url + 'status'
        self.missions_url = url + 'missions'
        self.mission_queue_url = url + 'mission_queue'
        self.maps_url = url + 'maps'
        self.positions = url +'positions'
        self.mission_groups_url = url+'mission_groups'
        self.mission_action_url = url+'missions/be6aa215-308e-11eb-b922-94c6911cf2ef/actions'
        self.moveto_action_url = url+'missions/move_to_position/actions'
        self.relativemove_url = url+'missions/relative_move/actions'
        self.registers_url = url + 'registers'		

        self.authorizationkey = key								

    def printStatus(self, battery, batteryTime, model):                 #prints the actual status of the robot
        print("Status of the robot " + model)
        print("Battery: %02.1f " %battery)
        print("Remaining battery time: %02.1f hours" %batteryTime)
        print('\n')

    def getRegisters(self, regNr):
        response = requests.get(self.registers_url + '/' + str(regNr), headers=headers)
        statusMiR = response.json().get('value')
        return statusMiR

    def waitForMiR(self):												#waiting for MiR100 to finish the just now executing mission
        if self.checkStatus() != "Ready":
            print("Wait for robot to finish mission...")
            while 1:
                #print(self.checkStatus())
                if self.checkStatus() == "Ready":						#if Robot finished the mission its status gets "Ready"
                    print(self.checkStatus())
                    return 1
        return -1

    def waitForRunning(self):
        print("Waiting to start mission...")
        while 1:
            if self.checkStatus() == "Executing":
                break

    def checkStatus(self):                                              #returns the status of the robot
        response = requests.get(self.status_url, headers = headers)
        statusMiR = response.json().get('state_text')
        return statusMiR

    def checkConnection(self):                                          #checks connection
        while 1:
            timeout = 5
            try:
            	response = requests.get(self.url, timeout=timeout)
            	print(bcolors.OKGREEN + "Connected to robot on address " + url + "\n" + bcolors.ENDC)
            	x = True
            except:# (response.ConnectionError, response.Timeout) as exception:
                print(bcolors.FAIL + "No connection to robot. Please check connection" + bcolors.ENDC)
                time.sleep(1)
                x = False
            if(x): break

    def getStatus(self):                                                #prints status of robot
        response = requests.get(self.status_url, headers = headers)
        if response.status_code == 200:
            model = response.json().get('robot_model')
            battery = float(response.json().get('battery_percentage'))
            batteryTime = float(response.json().get('battery_time_remaining'))
            batteryTime = batteryTime/3600
            self.printStatus(battery, batteryTime, model)
        elif response.status_code == 404:
            print("Get status error.\n")
        else:
            print("Connection Error")

    def printMissions(self):                                            #prints all missions on the robot
        print('Get missions')
        response = requests.get(self.missions_url, headers=headers)
        #print(response.text)
        if response.status_code == 200:
            print('All created Missions:\n')
            cnt = 0
            for x in response.json():
                if(response.json()[cnt].get('name') != response.json()[cnt-1].get('name')):
                    print(response.json()[cnt].get('name'))
                    cnt = cnt + 1
        elif response.status_code == 404:
            print('Get mission error.\n')

    def findName(self, name, response):                                 #finds name in json response and return guid of the mission
        cnt = 0
        while response.json()[cnt].get('name') != name:
            cnt = cnt + 1
        guid = response.json()[cnt].get('guid')
        return guid

    def postToMissionQueue(self, mission):                              #post mission to queue of the MiR100 robot
        response = requests.get(self.missions_url, headers=headers)
        guid = self.findName(mission, response)
        print(guid)
        print('Post mission to queue')
        response = requests.post(mission_queue_url, headers=headers, data = '{ "mission_id": "'+ guid + '"}')
        if response.status_code == 201:
            print('The element has been created successfully!\n')
        elif response.status_code == 400:
            print('Argument error or Missing content type application/json on the header or Bad request or Invalid JSON\n')
        elif response.status_code == 409:
            print('Duplicate entry\n')

    def getPositionsOnMap(self, map, pos):                              #get one position on the map (needs the name of the map) and return guid of specific position
        response = requests.get(self.maps_url, headers=headers)
        guid = self.findName(map, response)
        response = requests.get(self.url+'maps/'+guid+'/positions', headers=headers)
        print(response.text)
        guid = self.findName(pos, response)
        response = requests.get(self.url+'positions/'+guid+'', headers=headers)
        pos_x = float(response.json().get('pos_x'))
        pos_y = float(response.json().get('pos_y'))
        pos_w = float(response.json().get('orientation'))
        if response.status_code == 200:
            print("Postion: " + pos)
            print("Map: " + map)
            print("Position x: %02.3f" %pos_x)
            print("Position y: %02.3f" %pos_y)
            print("Orientation: %02.3f" %pos_w)
            print("\n")
        elif response.status_code == 404:
            print('Get mission error.\n')
        return guid

    def getAllPositions(self, map):                                     #prints all positions on the map, needs first the name of the map
        response = requests.get(self.maps_url, headers=headers)
        guid = self.findName(map, response)
        response = requests.get(self.url+'maps/'+guid+'/positions', headers=headers)
        cnt = 0
        for x in response.json():
            if(response.json()[cnt].get('name') != response.json()[cnt-1].get('name')):
                print(response.json()[cnt].get('name'))
            cnt = cnt + 1

    def getRobotActions(self):											#prints all actions of robot
        response = requests.get(self.url+'actions', headers=headers)
        print(response.text)

    def getMissionGroups(self):											#prints all mission groups
        response = requests.get(self.mission_groups_url, headers=headers)
        print(response.text)

    def deleteMissionQueues(self):										#prints all mission queues
        response = requests.delete(self.mission_queue_url, headers=headers)
        print(response.text)

    def newMission(self, name, group):    										#creates a new Mission and returns the guid of the mission
        response = requests.get(self.mission_groups_url, headers=headers)
        guid = self.findName(group, response)
        print(response.text)
        data = '{ "name": "'+name+'", "group_id": "'+guid+'"}'
        print(data)
        response = requests.post(self.url+'missions', headers=headers, data=data)
        print(response.text)
        guid = response.json().get('guid')
        return guid

    def getMarkers(self, guid, priority, marker):						#prints all markers of the robot (in progress)
        print(response.text)

    def urAction(self, guid, name):										#add an UR5 action to the given mission (guid)
        data = '{"action_type": "run_ur_program", "mission_id": "'+guid+'", "priority": 0, "parameters": [ {"id": "program_name","input_name": null,"value": "'+name+'"} ]}'
        response = requests.post(self.url+'missions/be6aa215-308e-11eb-b922-94c6911cf2ef/actions', headers=headers, data=data)
        #print(response.text)

    def loopAction(self, guid):											#add an loop action to the given mission (guid)
        data = '{"action_type": "loop", "mission_id": "'+guid+'", "priority": 0, "parameters": [{"id": "iterations", "input_name": null, "value": null}, {"id": "content", "input_name": null, "value": ""}]}'
        response = requests.post(self.mission_action_url, headers=headers, data=data)
        #print(response.text)

    def moveAction(self, guid, priority, x, y, w):						#add an move action to the given mission (guid)
        data = '{"action_type": "move_to_position", "mission_id": "'+guid+'", "priority": '+priority+', "parameters": [{"id": "x","input_name": null,"value": '+x+'}, {"id": "y","input_name": null,"value": '+y+'}, {"id": "orientation","input_name": null,"value": '+w+'}, {"id": "retries","input_name": null,"value": 10},{"id": "distance_threshold","input_name": null, "value": 0.1}]}'
        response = requests.post(self.mission_action_url, headers=headers, data=data)
        #print(response.text)

    def dockingAction(self, guid_mission, priority, markername):	#add an docking action to the given mission (guid), needs the marker guid
        data = '{"action_type": "docking", "mission_id": "'+guid_mission+'", "priority": '+priority+', "parameters": [ {"id": "marker","input_name": null,"value": "'+markername+'"} ]}'
        response = requests.post(self.mission_action_url, headers=headers, data=data)

    def moveToAction(self, map ,guid_mission, priority, distance, orientation, retries, x,y): #add an move to action to the given mission (guid)
        data = '{"action_type": "move_to_position", "mission_id": "'+guid_mission+'", "priority": '+priority+', "parameters": [ {"id": "distance_threshold","input_name": null,"value": '+distance+'}, {"id": "orientation","input_name": null,"value": '+orientation+'}, {"id": "retries","input_name": null,"value": '+retries+'}, {"id": "x","input_name": null,"value": '+x+'}, {"id": "y","input_name": null,"value": '+y+'} ]}'
        response = requests.post(self.moveto_action_url, headers=headers, data=data)
        #print(response.text)

    def moveRelativeAction(self, map ,guid_mission, priority,collision,angular, linear,orientation, x,y): #add an move relative action to the given mission (guid)
        data = '{"action_type": "relative_move", "mission_id": "'+guid_mission+'","priority": '+priority+', "parameters": [ {"id": "collision_detection","input_name": null,"value": '+collision+'}, {"id": "max_angular_speed","input_name": null,"value": '+angular+'},  {"id": "max_linear_speed","input_name": null,"value": '+linear+'},{"id": "orientation","input_name": null,"value": '+orientation+'},{"id": "x","input_name": null,"value": '+x+'},{"id": "y","input_name": null,"value": '+y+'} ]}'
        response = requests.post(self.relativemove_url, headers=headers, data=data)
        #print(response.text)

    def programmDockingMission(self,map,group,name, priority, marker):		#way of programming a new docking mission
        guid_mission = self.newMission(name,group)
        response = requests.get(self.maps_url, headers=headers)
        guid = self.findName(map, response)
        response = requests.get(self.url+'maps/'+guid+'/positions', headers=headers)
        guid_marker = self.findName(marker, response)
        self.dockingAction(guid_mission, priority, guid_marker)

    def getMission(self, mission):										#return guid of specific mission
        response = requests.get(self.missions_url, headers=headers)
        guid = self.findName(mission, response)
        print(guid)
        return guid

    def getActionsOfMission(self, mission):								#prints actions of specific mission
        guid = self.getMission(mission)
        response = requests.get(self.url+'missions/'+guid+'/actions', headers=headers)
        cnt = 0
        for x in response.json():
            if(response.json()[cnt].get('action_type')):
                print("Action %d: " %(cnt+1)  + response.json()[cnt].get('action_type') )
            cnt = cnt + 1

    def getSettings(self):												#prints settings of robot
        response = requests.get(self.url+'settings', headers=headers)
        cnt = 0
        for x in response.json():
            if(response.json()[cnt].get('name') != response.json()[cnt-1].get('name')):
                print(response.json()[cnt].get('name'),":",response.json()[cnt].get('value'))
            cnt = cnt + 1
