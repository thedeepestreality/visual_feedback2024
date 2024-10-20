import pybullet as p
import numpy as np
import time
from camera import Camera
import cv2

IMG_SIDE = 300
IMG_HALF = IMG_SIDE/2
MARKER_LENGTH = 0.1
MARKER_CORNERS_WORLD = np.array(
    [
        [-MARKER_LENGTH/2.0, MARKER_LENGTH/2.0, 0.0, 1.0],
        [ MARKER_LENGTH/2.0, MARKER_LENGTH/2.0, 0.0, 1.0],
        [ MARKER_LENGTH/2.0,-MARKER_LENGTH/2.0, 0.0, 1.0],
        [-MARKER_LENGTH/2.0,-MARKER_LENGTH/2.0, 0.0, 1.0]
    ]
)
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters()
parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
detector = cv2.aruco.ArucoDetector(dictionary, parameters)

# def computeInterMatrix(Z, sd0):
#     L = np.zeros((8,3))
#     for idx in range(4):
#         x = sd0[2*idx, 0]
#         y = sd0[2*idx+1, 0]
#         L[2*idx] = np.array([-1/Z,0,y])
#         L[2*idx+1] = np.array([0,-1/Z,-x])
#     return L

def computeInterMatrix(Z, sd0):
    L = np.zeros((8,2))
    for idx in range(4):
        x = sd0[2*idx, 0]
        y = sd0[2*idx+1, 0]
        L[2*idx] = np.array([x/Z,y])
        L[2*idx+1] = np.array([y/Z,-x])
    return L

def img_rp(t, s, w):
    Z0 = s[-1]
    L = computeInterMatrix(Z0, s[:-1,0])
    return L @ w

def prediction(s0, w):
    

def updateCamPos(cam, linkIdx):
    linkState = p.getLinkState(boxId, linkIndex=linkIdx)
    # pos
    xyz = linkState[0]
    # orientation
    quat = linkState[1]
    rotMat = p.getMatrixFromQuaternion(quat)
    rotMat = np.reshape(np.array(rotMat),(3,3))
    camera.set_new_position(xyz, rotMat)
    

camera = Camera(imgSize = [IMG_SIDE, IMG_SIDE])

dt = 1/240 # pybullet simulation step
Z0 = 0.5
maxTime = 5
logTime = np.arange(0.0, maxTime, dt)
sz = logTime.size

jointIndices = [1,2,3,4] # x, y, z, Oz
eefLinkIdx = 5

#or p.DIRECT for non-graphical version
physicsClient = p.connect(p.GUI, options="--background_color_red=1 --background_color_blue=1 --background_color_green=1")
p.resetDebugVisualizerCamera(
    cameraDistance=0.5,
    cameraYaw=-90,
    cameraPitch=-89.999,
    cameraTargetPosition=[0.5, 0.5, 0.6]
)
p.setGravity(0,0,-10)
boxId = p.loadURDF("./simple.urdf.xml", useFixedBase=True)

# add aruco cube and aruco texture
c = p.loadURDF('aruco.urdf', (0.5, 0.5, 0.0), useFixedBase=True)
x = p.loadTexture('aruco_cube.png')
p.changeVisualShape(c, -1, textureUniqueId=x)

numJoints = p.getNumJoints(boxId)
for idx in range(numJoints):
    print(f"{idx} {p.getJointInfo(boxId, idx)[1]} {p.getJointInfo(boxId, idx)[12]}")

# go to the desired position
p.setJointMotorControlArray(bodyIndex=boxId, jointIndices=jointIndices, targetPositions=[0.5,0.5,0.5, 0.0], controlMode=p.POSITION_CONTROL)
for _ in range(100):
    p.stepSimulation()

updateCamPos(camera, eefLinkIdx)
img = camera.get_frame()
corners, markerIds, rejectedCandidates = detector.detectMarkers(img)
sd0 = np.reshape(np.array(corners[0][0]),(8,1))
sd0 = np.array([(s-IMG_HALF)/IMG_HALF for s in sd0])
sd = np.reshape(np.array(corners[0][0]),(8,1)).astype(int)

# cv2.imshow("desired",img)
# cv2.waitKey(0)
# cv2.destroyAllWindows()

# go to the starting position
p.setJointMotorControlArray(bodyIndex=boxId, jointIndices=jointIndices, targetPositions=[0.5,0.5,0.5,0.5], controlMode=p.POSITION_CONTROL)
for _ in range(100):
    p.stepSimulation()

# updateCamPos(camera, eefLinkIdx)
# img = camera.get_frame()
# cv2.imshow("init",img)
# cv2.waitKey(0)
# cv2.destroyAllWindows()

linkState = p.getLinkState(boxId, linkIndex=eefLinkIdx)
# pos
Z0 = linkState[0][2]

camCount = 0
w = np.zeros((2,1))
logZ = [Z0]
start = time.time()
for t in logTime[1:]:
    p.stepSimulation()

    camCount += 1
    if (camCount == 10):
        camCount = 0
        updateCamPos(camera, eefLinkIdx)
        camera.get_frame()
        img = camera.get_frame()
        corners, markerIds, rejectedCandidates = detector.detectMarkers(img)
        s = corners[0][0,0]
        s0 = np.reshape(np.array(corners[0][0]),(8,1))
        s0 = np.array([(ss-IMG_HALF)/IMG_HALF for ss in s0])
        # logImg = np.append(logImg, s0.T)
        # logImg = np.vstack([logImg, s0.flatten()])
        linkState = p.getLinkState(boxId, linkIndex=eefLinkIdx)
        # pos
        Z0 = linkState[0][2]
        L0 = computeInterMatrix(Z0, s0)
        L0T = np.linalg.inv(L0.T@L0)@L0.T
        e = s0 - sd0
        coef = 2#1/2
        w = coef * L0T @ e

    p.setJointMotorControlArray(bodyIndex=boxId, jointIndices=[3,4], targetVelocities=w, controlMode=p.VELOCITY_CONTROL)
    linkState = p.getLinkState(boxId, linkIndex=eefLinkIdx)
    # pos
    xyz = linkState[0]
    # orientation
    quat = linkState[1]
    logZ.append(xyz[2])

print(time.time()-start)
    # time.sleep(dt)
p.disconnect()

import matplotlib.pyplot as plt
plt.plot(logTime, logZ)

plt.show()