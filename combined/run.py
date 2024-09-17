# Импорт библиотек. Задание констант
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
        [-MARKER_LENGTH/2,MARKER_LENGTH/2,0.0,1],
        [MARKER_LENGTH/2,MARKER_LENGTH/2,0.0,1],
        [MARKER_LENGTH/2,-MARKER_LENGTH/2.0,0.0,1],
        [-MARKER_LENGTH/2,-MARKER_LENGTH/2,0.0,1]
    ]
)

# Инициализация детектора ARUCO и камеры
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters()
parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
detector = cv2.aruco.ArucoDetector(dictionary, parameters)

# Определение вспомогательных функций
# def computeInterMatrix(Z, sd0):
#     L = np.zeros((8,3))
#     for idx in range(4):
#         x = sd0[2*idx, 0]
#         y = sd0[2*idx+1, 0]
#         L[2*idx] = np.array([-1/Z,0,y])
#         L[2*idx+1] = np.array([0,-1/Z,-x])
#     return L

def computeInterMatrix2(Z, sd0):
    L = np.zeros((8,4))
    for idx in range(4):
        x = sd0[2*idx, 0]
        y = sd0[2*idx+1, 0]
        L[2*idx] = np.array([-1/Z, 0, x/Z, y])
        L[2*idx+1] = np.array([0, -1/Z, y/Z, -x])
    return L


def updateCamPos(cam):
    linkState = p.getLinkState(boxId, linkIndex=8)
    # pos
    xyz = linkState[0]
    # orientation
    quat = linkState[1]
    rotMat = p.getMatrixFromQuaternion(quat)
    rotMat = np.reshape(np.array(rotMat),(3,3))  
    camera.set_new_position(xyz, rotMat)
    
# Создание объекта Camera
camera = Camera(imgSize = [IMG_SIDE, IMG_SIDE])

# Инициализация параметров симуляции и подключение к pybullet
dt = 1/240 # pybullet simulation step
q0 = 0.5  # starting position (radian)
qd = 0.5

xd = 0.5
yd = 0.5

L = 0.5
pos = q0
maxTime = 10
logTime = np.arange(0.0, maxTime, dt)
sz = logTime.size
logPos = np.zeros(sz)
logPos[0] = q0
logVel = np.zeros(sz)

jointIndices = [1,3,5,7]
eefLinkIdx = 8

#or p.DIRECT for non-graphical version
physicsClient = p.connect(p.GUI, options="--background_color_red=1 --background_color_blue=1 --background_color_green=1")
p.resetDebugVisualizerCamera(
    cameraDistance=0.5,
    cameraYaw=-90,
    cameraPitch=-89.999,
    cameraTargetPosition=[0.5, 0.5, 0.6]
)
p.setGravity(0,0,-10)

#Загрузка моделей и настройка симуляции:
boxId = p.loadURDF("simple1.urdf.xml", useFixedBase=True)

# add aruco cube and aruco texture
c = p.loadURDF('aruco.urdf', (0.5, 0.5, 0.0), useFixedBase=True)
x = p.loadTexture('aruco_cube.png')
p.changeVisualShape(c, -1, textureUniqueId=x)

numJoints = p.getNumJoints(boxId)
for idx in range(numJoints):
    print(f"{idx} {p.getJointInfo(boxId, idx)[1]} {p.getJointInfo(boxId, idx)[12]}")


print(p.isNumpyEnabled())

# go to the desired position
p.setJointMotorControlArray(bodyIndex=boxId, jointIndices=jointIndices, targetPositions=[0.0, 1.5708, 0.01, 0.0], controlMode=p.POSITION_CONTROL)
for _ in range(100):
    p.stepSimulation()

# while True:
#     p.stepSimulation()

updateCamPos(camera)
img = camera.get_frame()
corners, markerIds, rejectedCandidates = detector.detectMarkers(img)
if corners and len(corners) > 0 and len(corners[0]) > 0:
    sd0 = np.reshape(np.array(corners[0][0]), (8, 1))
    sd0 = np.array([(s - IMG_HALF) / IMG_HALF for s in sd0])
else:
    print("No corners detected or empty corners data.")
# sd0 = np.reshape(np.array(corners[0][0]),(8,1))
# sd0 = np.array([(s-IMG_HALF)/IMG_HALF for s in sd0])
sd = np.reshape(np.array(corners[0][0]),(8,1)).astype(int)

# go to the starting position
# p.setJointMotorControlArray(bodyIndex=boxId, jointIndices=jointIndices, targetPositions=[0.1, 1.4708, 0.15, 0.1], controlMode=p.POSITION_CONTROL)  # [0.1, 1.4708, 1, 0.1]
p.setJointMotorControlArray(bodyIndex=boxId, jointIndices=jointIndices, targetPositions=[0.0, 1.5708, 0.01, 0.5], controlMode=p.POSITION_CONTROL)  # [0.1, 1.4708, 1, 0.1]
for _ in range(100):
    p.stepSimulation()

idx = 1
camCount = 0
w = np.zeros((4,1)) 

# Создаем списки для данных
positions = []
orientations = []

# Цикл симуляции и обработки изображений
for t in logTime[1:]:
    p.stepSimulation()

    # Получаем состояние фланца робота
    linkState = p.getLinkState(boxId, linkIndex=8, computeLinkVelocity=1)
    # Позиция фланца
    pos = linkState[0]
    # Ориентация фланца в формате кватерниона
    quat = linkState[1]

    # Конвертируем кватернион в углы Эйлера
    euler = p.getEulerFromQuaternion(quat)
    # Преобразуем углы из радиан в градусы
    euler_deg = np.degrees(euler)

    # Сохраняем данные
    positions.append(pos)
    orientations.append(euler_deg)

    camCount += 1
    if (camCount == 5):
        camCount = 0
        updateCamPos(camera)
        img = camera.get_frame()
        corners, markerIds, rejectedCandidates = detector.detectMarkers(img)
        if corners is not None and len(corners) > 0 and len(corners[0]) > 0:
            s = corners[0][0,0]
            s0 = np.reshape(np.array(corners[0][0]),(8,1))  
            s0 = np.array([(ss-IMG_HALF)/IMG_HALF for ss in s0])
            Z0 = camera.eyePosition[2]
            L0 = computeInterMatrix2(Z0, s0) 
            L0T = np.linalg.inv(L0.T@L0)@L0.T
            # print("L0", L0)
            # print("L0T", L0T)
            e = s0 - sd0
            # print(len(e))
            coef = 1/2
            w = -coef * L0T @ e
            # print("w", w)

    jStates = p.getJointStates(boxId, jointIndices=jointIndices)
    jPos = [state[0] for state in jStates]
    jVel = [state[1] for state in jStates]
    (linJac,angJac) = p.calculateJacobian(
        bodyUniqueId = boxId, 
        linkIndex = eefLinkIdx, 
        localPosition = [0,0,0],
        objPositions = jPos,
        objVelocities = [0,0,0,0],
        objAccelerations = [0,0,0,0]
    )
    # print("linJac", linJac) 
    # print("angJac", angJac)

    # J = np.block([
    #     [np.array(linJac)[:2,:2], np.zeros((2,1))],
    #     [np.array(angJac)[2,:]]
    # ])

    J = np.block([
        [np.array(linJac)], 
        [np.array(angJac)[2,:]]  
    ])
    # print("J", J)
    
    dq = (np.linalg.inv(J) @ w).flatten()[[1, 0, 2, 3]] # dq = (np.linalg.inv(J) @ w).flatten()[[1,0,2]]
    dq[2] = -dq[2]
    dq[3] = -dq[3]

    # Убеждаемся, что dq содержит четыре компоненты
    assert len(dq) == 4, "Number of target velocities should be 4"
    
    p.setJointMotorControlArray(bodyIndex=boxId, jointIndices=jointIndices, targetVelocities=dq, controlMode=p.VELOCITY_CONTROL)
    # time.sleep(0.01)

# Завершение симуляции и отключение от pybullet
p.disconnect()

import matplotlib.pyplot as plt

# # Преобразуем списки в массивы numpy для удобства работы
# positions = np.array(positions)
# orientations = np.array(orientations)

# # Построение графиков
# plt.figure(figsize=(12, 8))

# # Графики положения
# plt.subplot(2, 1, 1)
# plt.plot(logTime[1:], positions[:, 0], label='X Position')
# plt.plot(logTime[1:], positions[:, 1], label='Y Position')
# plt.plot(logTime[1:], positions[:, 2], label='Z Position')
# plt.xlabel('Time')
# plt.ylabel('Position')
# plt.title('XYZ Position of Robot End Effector')
# plt.legend()

# # График угла фланца
# plt.subplot(2, 1, 2)
# plt.plot(logTime[1:], orientations[:, 0], label='Roll')
# plt.plot(logTime[1:], orientations[:, 1], label='Pitch')
# plt.plot(logTime[1:], orientations[:, 2], label='Yaw')
# plt.xlabel('Time')
# plt.ylabel('Angle (degrees)')
# plt.title('Orientation of Robot End Effector')
# plt.legend()

# plt.tight_layout()
# plt.show()

import matplotlib.pyplot as plt
import numpy as np

# Преобразуем список позиций в массив numpy для удобства работы
positions = np.array(positions)

# Построение графиков для позиции
plt.figure(figsize=(10, 6))
plt.plot(logTime[1:], positions[:, 0], label='X Position')
plt.plot(logTime[1:], positions[:, 1], label='Y Position')
plt.plot(logTime[1:], positions[:, 2], label='Z Position')
plt.xlabel('Time (s)')
plt.ylabel('Position')
plt.title('XYZ Position of End Effector link_eef3')
plt.legend()
plt.grid(True)
plt.show()

# Преобразуем список углов в массив numpy для удобства работы
orientations = np.array(orientations)

# Построение графиков для угла
plt.figure(figsize=(10, 6))
plt.plot(logTime[1:], orientations[:, 0], label='Roll')
# print(orientations[:, 0])
plt.plot(logTime[1:], orientations[:, 1], label='Pitch')
plt.plot(logTime[1:], orientations[:, 2], label='Yaw')
plt.xlabel('Time (s)')
plt.ylabel('Angle (degrees)')
plt.title('Orientation of End Effector link_eef3')
plt.legend()
plt.grid(True)
plt.show()
