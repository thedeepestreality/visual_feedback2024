import pybullet as p
import pybullet_data
import numpy as np
from camera import Camera
import cv2

dt = 1/240
T = 1
t = 0

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")

# add aruco cube and aruco texture
c = p.loadURDF('cube.urdf', (0.5, 0.5, 0.5), useFixedBase=True)
x = p.loadTexture('aruco.png')
p.changeVisualShape(c, -1, textureUniqueId=x)

#init aruco detector
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(dictionary, parameters)

camera = Camera()

data = camera.get_frame()
img = data[:,:,[2,1,0]]

corners, markerIds, rejectedCandidates = detector.detectMarkers(img)
print(corners[0][0])
sd0 = np.reshape(np.array(corners[0][0]),(8,1))
sd0 = np.array([(s-125)/125 for s in sd0])
sd = np.reshape(np.array(corners[0][0]),(8,1)).astype(int)

Z = 2.0
Ld0 = np.array([[-1/Z, 0,sd0[0,0]/Z, sd0[1,0]],
                [0, -1/Z,sd0[1,0]/Z,-sd0[0,0]],
                [-1/Z, 0,sd0[2,0]/Z, sd0[3,0]],
                [0, -1/Z,sd0[3,0]/Z,-sd0[2,0]],
                [-1/Z, 0,sd0[4,0]/Z, sd0[5,0]],
                [0, -1/Z,sd0[5,0]/Z,-sd0[4,0]],
                [-1/Z, 0,sd0[6,0]/Z, sd0[7,0]],
                [0, -1/Z,sd0[7,0]/Z,-sd0[6,0]]])

# pos = np.array([1.0, 0.0, 3.0])
pos = np.array([0.5, 0.5, 3.0])

al = np.pi/3
# al = 0

camera.set_new_position(pos, [np.cos(al), np.sin(al), 0])
Z = 2.0
data = camera.get_frame()
img = data[:,:,[2,1,0]]
corners, markerIds, rejectedCandidates = detector.detectMarkers(img)
si = np.reshape(np.array(corners[0][0]),(8,1)).astype(int)
path = [corners[0][0,0].astype(int)]

while t <= T:
    data = camera.get_frame()
    img = data[:,:,[2,1,0]]

    corners, markerIds, rejectedCandidates = detector.detectMarkers(img)
    
    # for corner debug plot
    prev = path[-1]
    curr = corners[0][0,0].astype(int)
    if (curr[0] != prev[0] or curr[1] != prev[1]):
        path.append(curr)

    s = corners[0][0,0]
    s0 = np.reshape(np.array(corners[0][0]),(8,1))
    s0 = np.array([(ss-125)/125 for ss in s0])

    Z = pos[2] - 1.0
    L0 = np.array([ [-1/Z, 0,s0[0,0]/Z, s0[1,0]],
                    [0, -1/Z,s0[1,0]/Z,-s0[0,0]],
                    [-1/Z, 0,s0[2,0]/Z, s0[3,0]],
                    [0, -1/Z,s0[3,0]/Z,-s0[2,0]],
                    [-1/Z, 0,s0[4,0]/Z, s0[5,0]],
                    [0, -1/Z,s0[5,0]/Z,-s0[4,0]],
                    [-1/Z, 0,s0[6,0]/Z, s0[7,0]],
                    [0, -1/Z,s0[7,0]/Z,-s0[6,0]]])
    
    # L0 = Ld0
    L0 = (L0+Ld0)/2
    
    L0T = np.linalg.inv(L0.T@L0)@L0.T
    e = s0 - sd0
    w = -L0T @ e

    # pos[1] += -10*w[0,0]*dt
    # pos[0] += -10*w[1,0]*dt
    pos[2] += -w[2,0]/10
    al += -w[3,0]/10
    upVector = [np.cos(al), np.sin(al), 0]
    camera.set_new_position(pos, upVector)

    p.stepSimulation()
    t += dt

# draw resulting image
data = camera.get_frame()
img = data[:,:,[2,1,0]]

img = cv2.UMat(img)
for i in range(4):
    cv2.circle(img, (sd[2*i,0],sd[2*i+1,0]), 5, (255,0,0),2)
    cv2.circle(img, (si[2*i,0],si[2*i+1,0]), 5, (0,0,255),2)
    ii = (i+1)%4
    cv2.line(img,(sd[2*i,0],sd[2*i+1,0]),(sd[2*ii,0],sd[2*ii+1,0]),(0,0,255),2)
    cv2.line(img,(si[2*i,0],si[2*i+1,0]),(si[2*ii,0],si[2*ii+1,0]),(255,0,0),2)
for pt in path:
    cv2.circle(img, pt, 3, (0,255,0))

cv2.imshow('test', img)
cv2.waitKey(0)

p.disconnect()
cv2.destroyAllWindows()