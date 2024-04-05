import pybullet as p
import pybullet_data
import numpy as np
from camera import Camera
import cv2
from scipy.spatial.transform import Rotation as R

IMG_SIDE = 500
IMG_HALF = IMG_SIDE/2

def computeInterMatrix(Z, sd0):
    L = np.zeros((8,6))
    for idx in range(4):
        x = sd0[2*idx, 0]
        y = sd0[2*idx+1, 0]
        L[2*idx] = np.array([-1/Z,0,x/Z,x*y,-(1+x**2),y])
        L[2*idx+1] = np.array([0,-1/Z,y/Z,1+y**2,-x*y,-x])
    return L

def rearrangeCorners(corners):
    idx = 0
    center = [0,0]
    for i in range(4):
        center[0] += corners[i,0]/4
        center[1] += corners[i,1]/4
    for i in range(4):
        p = corners[i]
        if p[0] <= center[0] and p[1] >= center[1]:
            idx = i
            break
    rearranged = np.array([corners[idx], corners[idx-1], corners[idx-2], corners[idx-3]])
    return rearranged

def cornersSubpix(img, corners):
    img = data[:,:,[2,1,0]]
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.001)
    corners_subpix = cv2.cornerSubPix(gray, corners,(5,5), (-1,-1), criteria)
    return corners_subpix

def skew(vec):
    return np.array([[0,-vec[2], vec[1]],
                     [vec[2], 0, -vec[0]],
                     [-vec[1],vec[0], 0]])

dt = 1/240
T = 0.5
t = 0

physicsClient = p.connect(p.GUI, options="--background_color_red=1 --background_color_blue=1 --background_color_green=1")#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
# planeId = p.loadURDF("plane.urdf")

# add aruco cube and aruco texture
c = p.loadURDF('aruco.urdf', (0.5, 0.5, 0.5), useFixedBase=True)
x = p.loadTexture('aruco_cube.png')
# c = p.loadURDF('cube.urdf', (0.5, 0.5, 0.5), useFixedBase=True)
# x = p.loadTexture('aruco.png')
p.changeVisualShape(c, -1, textureUniqueId=x)

#init aruco detector
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters()
# parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE
parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_CONTOUR
parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
detector = cv2.aruco.ArucoDetector(dictionary, parameters)

camera = Camera(imgSize = [IMG_SIDE, IMG_SIDE])

euler = [0.2,0.,0]
quat = p.getQuaternionFromEuler(euler)
rotMat = p.getMatrixFromQuaternion(quat)
rotMat = np.reshape(np.array(rotMat),(3,3))
camera.set_new_position([0.5, 0.5, 3.0], rotMat)

data = camera.get_frame()
img = data[:,:,[2,1,0]]

# img = cv2.blur(img,(3,3))

# img = cv2.UMat(img)
# cv2.imshow('init', img)
# cv2.waitKey(0)

corners, markerIds, rejectedCandidates = detector.detectMarkers(img)
sd0 = np.reshape(np.array(corners[0][0]),(8,1))
sd0 = np.array([(s-IMG_HALF)/IMG_HALF for s in sd0])
sd = np.reshape(np.array(corners[0][0]),(8,1)).astype(int)

Z = 2.0
Ld0 = computeInterMatrix(Z, sd0)

markerLength = 1
f = IMG_HALF/np.tan(np.pi/6)
distCoeffs = np.array([])
cameraMatrix = np.array([[f, 0, IMG_HALF],
                         [0, f, IMG_HALF],
                         [0, 0, 1]])
objPoints = np.array([
    [-markerLength/2, markerLength/2, 0],
    [ markerLength/2, markerLength/2, 0],
    [ markerLength/2,-markerLength/2, 0],
    [-markerLength/2,-markerLength/2, 0]
])
print(objPoints)
print(corners[0][0])
cornersRearranged = rearrangeCorners(corners[0][0])
#cornersRearranged = cornersSubpix(img, cornersRearranged)
retval, rvec, tvec = cv2.solvePnP(objPoints, cornersRearranged, cameraMatrix, distCoeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE)
print(tvec)
print(rvec)
r = R.from_rotvec(rvec.flatten())
print(r.as_euler('xyz'))

img = cv2.UMat(img)
cv2.aruco.drawDetectedMarkers(img, corners)
cv2.imshow('init', img)
cv2.waitKey(0)
cv2.destroyAllWindows()


# cv2.drawFrameAxes(img, cameraMatrix, distCoeffs, rvec, tvec, markerLength, 2)
# cv2.imshow('init', img)
# cv2.waitKey(0)

# pos = np.array([1.0, 0.0, 3.0])
pos = np.array([0.7, 0.5, 2.5])
# pos = np.array([1.0, 1.0, 3.5])

# al = 0.0
# bt = 0.0
# gm = np.pi/3

al = 0.2
bt = 0.0
gm = 0.0

euler = [al,bt,gm]
quat = p.getQuaternionFromEuler(euler)
rotMat = p.getMatrixFromQuaternion(quat)
rotMat = np.reshape(np.array(rotMat),(3,3))
camera.set_new_position(pos, rotMat)

Z = 2.0
data = camera.get_frame()
img = data[:,:,[2,1,0]]
corners, markerIds, rejectedCandidates = detector.detectMarkers(img)
si = np.reshape(np.array(corners[0][0]),(8,1)).astype(int)
path = [corners[0][0,0].astype(int)]

# T0 = np.array([[1,0,0,]])
v = []
while t <= T:
    data = camera.get_frame()
    img = data[:,:,[2,1,0]]

    corners, markerIds, rejectedCandidates = detector.detectMarkers(img)
    img = cv2.UMat(img)
    if (markerIds != None):
    
        cv2.aruco.drawDetectedMarkers(img, corners)
        # for corner debug plot
        prev = path[-1]
        curr = corners[0][0,0].astype(int)
        if (curr[0] != prev[0] or curr[1] != prev[1]):
            path.append(curr)

        s = corners[0][0,0]
        s0 = np.reshape(np.array(corners[0][0]),(8,1))
        s0 = np.array([(ss-IMG_HALF)/IMG_HALF for ss in s0])

        # s0i = np.reshape(np.array(corners[0][0]),(8,1)).astype(int)
        # depth = camera.get_depth()
        # dd = np.zeros((4,1))
        # for idx in range(4):
        #     dd[idx,0] = depth[s0i[2*idx],s0i[2*idx+1]]
        # print(dd)        

        Z = pos[2] - 1.0
        L0 = computeInterMatrix(Z, s0)
        
        # L0 = Ld0
        # L0 = (L0+Ld0)/2
        
        L0T = np.linalg.inv(L0.T@L0)@L0.T
        e = s0 - sd0
        coef = 1/10
        # w = -coef * L0T @ e

        cornersRearranged = rearrangeCorners(corners[0][0])
        retval, rvec, tvec = cv2.solvePnP(objPoints, cornersRearranged, cameraMatrix, distCoeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        print('curr',tvec, rvec)
        if (not np.isnan(tvec[0])):
            r = R.from_rotvec(rvec.flatten())
            w = np.zeros((6,1))
            # w[4,0] = 0.1*r.as_euler('xyz')[1] + 0.01*tvec[0,0]
            # w[0,0] = 0.1*tvec[0,0]
            # w[1,0] = 0.1*tvec[1,0]
            # w[2,0] = 0.1*(tvec[2,0]-2)

            # pos[1] -= w[0,0]
            # pos[0] -= w[1,0]
            # pos[2] -= w[2,0]
            # al -= w[4,0]
            # bt -= w[3,0]
            # gm -= w[5,0]
            # tvec = tvec
            tvec_d = [0,0,1.5]
            tvec = tvec.flatten()
            v = -0.1*((tvec_d-tvec)+skew(tvec)@rvec.flatten())
            w = 0.1*rvec.flatten()
        

    cv2.imshow('test', img)
    cv2.waitKey(int(dt*1000))

    pos[1] -= v[0]
    pos[0] -= v[1]
    pos[2] -= v[2]
    al -= w[1]
    bt -= w[0]
    gm -= w[2]

    euler = [al,bt,gm]
    quat = p.getQuaternionFromEuler(euler)
    rotMat = p.getMatrixFromQuaternion(quat)
    rotMat = np.reshape(np.array(rotMat),(3,3))
    camera.set_new_position(pos, rotMat)

    p.stepSimulation()
    t += dt

print('fin: ', tvec, r.as_euler('xyz'))
# draw resulting image
data = camera.get_frame()
img = data[:,:,[2,1,0]]
corners, markerIds, rejectedCandidates = detector.detectMarkers(img)

img = cv2.UMat(img)
# for i in range(4):
#     cv2.circle(img, (sd[2*i,0],sd[2*i+1,0]), 5, (255,0,0),2)
#     cv2.circle(img, (si[2*i,0],si[2*i+1,0]), 5, (0,0,255),2)
#     ii = (i+1)%4
#     cv2.line(img,(sd[2*i,0],sd[2*i+1,0]),(sd[2*ii,0],sd[2*ii+1,0]),(0,0,255),2)
#     cv2.line(img,(si[2*i,0],si[2*i+1,0]),(si[2*ii,0],si[2*ii+1,0]),(255,0,0),2)
# for pt in path:
#     cv2.circle(img, pt, 3, (0,255,0))

cv2.aruco.drawDetectedMarkers(img, corners)
cv2.imshow('fin', img)
cv2.waitKey(0)

p.disconnect()
cv2.destroyAllWindows()