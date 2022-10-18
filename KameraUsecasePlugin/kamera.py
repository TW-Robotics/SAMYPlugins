#!/usr/bin/env python3
import cv2 as cv
import numpy as np
from pubsub import pub
from samyplugin.CRCL_DataTypes import *
import pyrealsense2 as rs
import logging

#camera class
class Kamera:

    def __init__(self,robot_settings):
        self.logger = logging.getLogger(__name__)
        self.logger.setLevel(logging.INFO)
        log_handler = logging.StreamHandler()
        log_handler.setFormatter(logging.Formatter("%(levelname)s %(filename)s - %(message)s"))
        self.logger.addHandler(log_handler)

        #workpiece coordinates in robot-coordinatesystem
        self.X_robot = 0
        self.Y_robot = 0

        self.origin = (630,38) # origin of the camera coordinate system in pixels
        self.mm = 3.84286 # one pixel corsponds to mm 
        self.center = None # center of the workpiece
        #Workpiece is either yellow or grey
        self.yellow = False
        self.grey = False
        
        #kernel for bluring
        self.kernelsize = 7
        self.kernel = np.ones((self.kernelsize, self.kernelsize), np.uint8)

        #custom camera matrix for the RGB module of the Intel Realsense D435
        self.kmat = [[1352.06955467799, 0, 973.803727404974] , [0, 1366.87675727465, 533.592541104211] , [0, 0, 1]]
        self.dstmat = [[0.127509519023764, -0.306871810182364, -0.000864384921776296, 0.00145553767677557, 0.125094379310040]]
        self.kmat = np.array(self.kmat)
        self.dstmat = np.array(self.dstmat)
        self.kmat.reshape((3, 3))

        # connect to camera
        self.logger.info("Searching Devices..")
        self.selected_devices = []                     # Store connected device(s)

        for d in rs.context().devices:
            self.selected_devices.append(d)
            self.logger.info(d.get_info(rs.camera_info.name))
        if not self.selected_devices:
            priself.logger.infont("No RealSense device is connected!")

        self.rgb_sensor = None

        for device in self.selected_devices:
            self.logger.info(f"Required sensors for device: {device.get_info(rs.camera_info.name)}")
            for s in device.sensors:                   # Show available sensors in each device
                if s.get_info(rs.camera_info.name) == 'RGB Camera':
                    self.logger.info(" - RGB sensor found")
                    self.rgb_sensor = s                                # Set RGB sensor
                #if s.get_info(rs.camera_info.name) == 'Stereo Module':
                #   self.depth_sensor = s                              # Set Depth sensor
                #  self.logger.info(" - Depth sensor found")

        self.rgb_sensor.set_option(rs.option.gain, 0) #Set the digital gain of the camera to zero
        self.rgb_sensor.set_option(rs.option.enable_auto_white_balance, 1) #set autowhitebalance to automatic

        self.pipe = rs.pipeline()
        cfg = rs.config() 
        cfg.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 6) # set resolution and std rgb format 
        profile = self.pipe.start(cfg)
        self.frame = None

        #mask for grey object
        self.lth_grey = np.array([50, 50, 20]) 
        self.hth_grey = np.array([110, 190, 130])

        #mask for yellow object
        self.lth_yellow = np.array([20, 80, 20]) 
        self.hth_yellow = np.array([35, 255, 255])

        pub.subscribe(self.get_status, "GetStatus") # callback method for GetStatus Skill


    def __del__(self):
        self.pipe.stop()
    #getting frame form Intel Realsense D435 camera
    def get_frame_from_camera(self):
        for _ in range(10): # Skip first frames to give syncer and auto-exposure time to adjust
            frameset = self.pipe.wait_for_frames()
        
        frameset = self.pipe.wait_for_frames()
        color_frame = frameset.get_color_frame()
        self.frame = np.asanyarray(color_frame.get_data())

        self.hsv = cv.cvtColor(self.frame, cv.COLOR_BGR2HSV)
        self.h,  self.w = self.hsv.shape[:2]
        self.logger.info("Writing image to file.")
        cv.imwrite("./configFiles/image.jpg", self.frame)
        
    #searching for workpieces
    def getCircles(self, mask):
        #bluring
        blured = cv.medianBlur(mask, 5)
        blured = cv.erode(blured, self.kernel)
        #searching for circles
        self.circles = cv.HoughCircles(blured, cv.HOUGH_GRADIENT, 1, 1000,
                                    param1=20, param2=10,
                                    minRadius=50, maxRadius=150)

    #publishing pose to the information source in the SAMY Core
    def get_status(self, data):
        self.get_frame_from_camera()
        self.detect()
        if self.X_robot != 0:
            parameters = MoveToParametersSetDataType()
            parameters.MoveStraight = False
            parameters.EndPosition.point.x = self.X_robot / 1000 # pose has to be in m
            parameters.EndPosition.point.y = self.Y_robot / 1000 # pose has to be in m
            parameters.EndPosition.point.z = 0.035
            parameters.EndPosition.xAxis.i = 0.707
            parameters.EndPosition.xAxis.j = 0.707
            parameters.EndPosition.xAxis.k = 0.0
            parameters.EndPosition.zAxis.i = 0.0
            parameters.EndPosition.zAxis.j = 0.0
            parameters.EndPosition.zAxis.k = -1.0
            pub.sendMessage("write_information_source", name="CameraPose", data=parameters)
        pub.sendMessage("write_information_source", name="YellowPartDetected", data=self.yellow)
        pub.sendMessage("write_information_source", name="GreyPartDetected", data=self.grey)        

    #detecting workpieces
    def detect(self):
        self.yellow = False
        self.grey = False

        #undistort
        self.undist = cv.undistort(self.frame, self.kmat, self.dstmat, None, None)
        self.undist = self.undist[100:1000, 100:800]
        self.hsv = cv.cvtColor(self.undist, cv.COLOR_BGR2HSV)
        #drawing coordinate system
        cv.line(self.undist,self.origin,(self.origin[0] - 100,self.origin[1]),(255,0,0),5)
        cv.line(self.undist,self.origin,(self.origin[0] ,self.origin[1]+ 100),(255,0,0),5)
        #creating masks
        self.mask_grey = cv.inRange(self.hsv, self.lth_grey, self.hth_grey)
        self.mask_yellow = cv.inRange(self.hsv, self.lth_yellow, self.hth_yellow)
        #cv.imwrite("./configFiles/mask_grey.png", self.mask_grey)
        #cv.imwrite("./configFiles/mask_yellow.png", self.mask_yellow)
        self.circles = 0
        self.getCircles(self.mask_grey)
        #self.logger.info(f"Found {self.circles} circles with grey mask.")

        if self.circles is not None:
            self.grey = True
            self.circles = np.uint16(np.around(self.circles))
            for i in self.circles[0, :]:
                self.center = (i[0], i[1])
                cv.circle(self.undist, self.center, 1, (0, 100, 100), 3)
                self.radius = i[2]
                cv.circle(self.undist, self.center, self.radius, (255, 0, 255), 3)
        else:
            self.getCircles(self.mask_yellow)
            self.logger.debug(f"Found {self.circles} circles with yellow mask.")
            if self.circles is not None:
                self.yellow = True
                self.circles = np.uint16(np.around(self.circles))
                for i in self.circles[0, :]:
                    self.center = (i[0], i[1])
                    cv.circle(self.undist, self.center, 1, (0, 100, 100), 3)
                    self.radius = i[2]
                    cv.circle(self.undist, self.center, self.radius, (255, 0, 255), 3)


        #printing results
        if self.circles is not None:
            self.distx_px = abs(self.center[0] - self.origin[0])
            self.distx_py = abs(self.center[1] - self.origin[1])
            self.X = np.round(self.distx_px/self.mm , 1)
            self.Y = np.round(self.distx_py/self.mm , 1)
            self.logger.info(f"Camera coordinates: {self.X} mm in X and {self.Y} mm in Y")

            self.X_robot = np.round(self.X - 75.0 , 1)
            self.Y_robot = np.round(self.Y + 220.0 , 1)

            self.logger.info(f"Robot coordinates: {self.X_robot} mm in X and {self.Y_robot} mm in Y")
        else:
            self.logger.error("No workpiece found!")



    
