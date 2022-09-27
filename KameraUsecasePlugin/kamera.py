#!/usr/bin/env python3
import cv2 as cv
import numpy as np
import pyrealsense2 as rs

frame = cv.imread("Beispielbilder/1.jpg")

cv.namedWindow("img", 0)
cv.imshow("img" , frame)
cv.waitKey(1000)



kmat = [[1352.06955467799, 0, 973.803727404974] , [0, 1366.87675727465, 533.592541104211] , [0, 0, 1]]
dstmat = [[0.127509519023764, -0.306871810182364, -0.000864384921776296, 0.00145553767677557, 0.125094379310040]]
kmat = np.array(kmat)
dstmat = np.array(dstmat)
kmat.reshape((3, 3))

undist = cv.undistort(frame, kmat, dstmat, None, None)

cv.imshow("img" , undist)
cv.waitKey(1000)

cv.imwrite("undist.png" ,undist)



cv.destroyAllWindows()