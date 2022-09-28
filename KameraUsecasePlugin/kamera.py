#!/usr/bin/env python3
import cv2 as cv
import numpy as np
#import pyrealsense2 as rs

origin = (630,38)
mm = 3.84286


frame = cv.imread("Beispielbilder/3.jpg")
hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
h,  w = frame.shape[:2]

kernelsize = 7
kernel = np.ones((kernelsize, kernelsize), np.uint8)

#cv.namedWindow("img", 0)
#cv.imshow("img" , frame)
#cv.waitKey(1000)

#camera matrix
kmat = [[1352.06955467799, 0, 973.803727404974] , [0, 1366.87675727465, 533.592541104211] , [0, 0, 1]]
dstmat = [[0.127509519023764, -0.306871810182364, -0.000864384921776296, 0.00145553767677557, 0.125094379310040]]
kmat = np.array(kmat)
dstmat = np.array(dstmat)
kmat.reshape((3, 3))

#newkmat, roi = cv.getOptimalNewCameraMatrix(kmat, dstmat, (w,h), 1, (w,h))
undist = cv.undistort(frame, kmat, dstmat, None, None)
undist = undist[100:1000, 100:800]
hsv = cv.cvtColor(undist, cv.COLOR_BGR2HSV)

#mask
lth_gray = np.array([90, 60, 30]) 
hth_gray = np.array([110, 160, 150])
mask_grey = cv.inRange(hsv, lth_gray, hth_gray)

blured = cv.medianBlur(mask_grey, 5)
blured = cv.erode(blured, kernel)

circles = cv.HoughCircles(blured, cv.HOUGH_GRADIENT, 1, 1000,
                               param1=20, param2=10,
                               minRadius=50, maxRadius=150)

cv.line(undist,origin,(origin[0] - 100,origin[1]),(255,0,0),5)
cv.line(undist,origin,(origin[0] ,origin[1]+ 100),(255,0,0),5)
    
if circles is not None:
    circles = np.uint16(np.around(circles))
    for i in circles[0, :]:
        center = (i[0], i[1])
        #print(center)
        # circle center
        cv.circle(undist, center, 1, (0, 100, 100), 3)
        # circle outline
        radius = i[2]
        cv.circle(undist, center, radius, (255, 0, 255), 3)



try:
    distx_px = abs(center[0] - origin[0])
    distx_py = abs(center[1] - origin[1])
    X = np.round(distx_px/mm , 1)
    Y = np.round(distx_py/mm , 1)
    print("Kamerakoordinaten: " ,X,"mm in X und",Y,"mm in Y")

    X_robot = np.round(X - 87.5 , 1)
    Y_robot = np.round(Y + 210 , 1)

    print("Kooordinaten im Roboter Koordiantensystem: " ,X_robot,"mm in X und",Y_robot,"mm in Y")


except:
    print("No workpiece found!")






cv.imshow("img" , undist)
cv.waitKey(1000)


cv.imwrite("undist.png" ,undist)



cv.destroyAllWindows()