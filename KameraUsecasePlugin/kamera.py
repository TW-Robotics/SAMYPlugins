#!/usr/bin/env python3
import cv2 as cv
import numpy as np
from pubsub import pub
from samyplugin.CRCL_DataTypes import *
#import pyrealsense2 as rs


class Kamera:

    def __init__(self,robot_settings):

        #workpiece coordinates in robot-coordinatesystem
        self.X_robot = 0
        self.Y_robot = 0

        self.origin = (630,38)
        self.mm = 3.84286
        #Workpiece is either yellow or grey
        self.yellow = False

        self.kernelsize = 7
        self.kernel = np.ones((self.kernelsize, self.kernelsize), np.uint8)

        self.frame = cv.imread("Beispielbilder/4.jpg")
        self.hsv = cv.cvtColor(self.frame, cv.COLOR_BGR2HSV)
        self.h,  self.w = self.frame.shape[:2]

        #camera matrix
        self.kmat = [[1352.06955467799, 0, 973.803727404974] , [0, 1366.87675727465, 533.592541104211] , [0, 0, 1]]
        self.dstmat = [[0.127509519023764, -0.306871810182364, -0.000864384921776296, 0.00145553767677557, 0.125094379310040]]
        self.kmat = np.array(self.kmat)
        self.dstmat = np.array(self.dstmat)
        self.kmat.reshape((3, 3))

        #mask_grey
        self.lth_grey = np.array([90, 60, 30]) 
        self.hth_grey = np.array([110, 160, 150])

        #mask_yellow
        self.lth_yellow = np.array([23, 165, 0]) 
        self.hth_yellow = np.array([27, 191, 220])

        cv.namedWindow("img", 0)

        pub.subscribe(self.get_status, "GetStatus")


    def __del__(self):
        cv.destroyAllWindows()

    def getCircles(self,mask):
        #bluring
        blured = cv.medianBlur(mask, 5)
        blured = cv.erode(blured, self.kernel)
        #searching for circles
        self.circles = cv.HoughCircles(blured, cv.HOUGH_GRADIENT, 1, 1000,
                                    param1=20, param2=10,
                                    minRadius=50, maxRadius=150)


    def get_status(self):
        self.detect()
        if self.X_robot != 0:
            parameters = CRCL_MoveToParametersDataType()
            parameters.EndPosition.pos.x = self.X_robot
            parameters.EndPosition.pos.y = self.Y_robot
            parameters.EndPosition.pos.z = 0.0
            parameters.EndPosition.xAxis.i = 0.707
            parameters.EndPosition.xAxis.j = 0.707
            parameters.EndPosition.xAxis.k = 0.0
            parameters.EndPosition.zAxis.i = 0.0
            parameters.EndPosition.zAxis.j = 0.0
            parameters.EndPosition.zAxis.k = -1.0
            pub.sendMessage("write_information_source", name="CameraPose", data=parameters)
            pub.sendMessage("write_information_source", name="Yellow", data=self.yellow)
            pub.sendMessage("write_information_source", name="partDetected", data=True)
        else:
            pub.sendMessage("write_information_source", name="partDetected", data=False)


    def detect(self):

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
        
        self.circles = 0
        self.getCircles(mask_grey)

        if self.circles is not None:
            self.yellow = False
            self.circles = np.uint16(np.around(self.circles))
            for i in self.circles[0, :]:
                self.center = (i[0], i[1])
                cv.circle(undist, self.center, 1, (0, 100, 100), 3)
                self.radius = i[2]
                cv.circle(undist, self.center, self.radius, (255, 0, 255), 3)
        else:
            self.getCircles(mask_yellow)
            self.yellow = True
            if self.circles is not None:
                self.circles = np.uint16(np.around(self.circles))
                for i in self.circles[0, :]:
                    self.center = (i[0], i[1])
                    cv.circle(self.undist, self.center, 1, (0, 100, 100), 3)
                    self.radius = i[2]
                    cv.circle(self.undist, self.center, self.radius, (255, 0, 255), 3)


        #printing results
        try:
            self.distx_px = abs(self.center[0] - self.origin[0])
            self.distx_py = abs(self.center[1] - self.origin[1])
            self.X = np.round(self.distx_px/self.mm , 1)
            self.Y = np.round(self.distx_py/self.mm , 1)
            print("Kamerakoordinaten: " ,X,"mm in X und",Y,"mm in Y")

            self.X_robot = np.round(self.X - 87.5 , 1)
            self.Y_robot = np.round(self.Y + 210.0 , 1)

            print("Kooordinaten im Roboter Koordiantensystem: " ,self.X_robot,"mm in X und",self.Y_robot,"mm in Y")


        except:
            print("No workpiece found!")

        #display img
        cv.imshow("img" , self.undist)
        cv.waitKey(1000)
        #writing img
        cv.imwrite("undist.png" ,self.undist)



    