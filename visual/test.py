import pybullet as p
import time
import pybullet_data
import numpy as np
import math
import sys
from camera import Camera
import cv2

dt = 1/240
T = 1000
t = 0

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
boxId = p.loadURDF("./simple.urdf.xml", useFixedBase=True)

# p.changeVisualShape(boxId, 3, rgbaColor=[0,0,0,1])

camera = Camera()
data = camera.get_frame()
img = cv2.UMat(np.asarray(data[:,:,[2,1,0]]))

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
parameters =  cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(dictionary, parameters)
corners, markerIds, rejectedCandidates = detector.detectMarkers(img)
print("img:",img)
print("markerIds:",detector.detectMarkers(img))
cv2.aruco.drawDetectedMarkers(img, corners, markerIds)

# # markerLength = 1
# # distCoeffs = np.zeros([0,0,0,0])
# # cameraMatrix = np.array([[1,0,100],[0,1,100],[0,0,1]])
# #rvecs, tvecs, _objPoints = cv2.aruco.estimatePoseSingleMarkers(	corners, markerLength, cameraMatrix, distCoeffs)

cv2.imshow('test', img)

cv2.waitKey(0) 
cv2.destroyAllWindows()

while t <= T:
    p.stepSimulation()
    t += dt
    time.sleep(dt)

p.disconnect()
