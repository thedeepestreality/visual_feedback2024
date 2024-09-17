import pybullet as p
import time
import pybullet_data 


pybullet_data_path = pybullet_data.getDataPath()
print("PyBullet data path:", pybullet_data_path)


# TASKS FROM 01.03.2024:
# 1  set maxTime - done
# 2  plots (pos, vel)
# 3  position control based on p.VELOCITY_CONTROL (proportional regulator)
# 4  position control based on p.TORQUE_CONTROL (PI-regulator)
# 5* compare plots of pybullet and our own odeint and figure out the source of errors and fix it
# 6* figure out how to add control to our own integration script

dt = 1/240 # pybullet simulation step
q0 = 0.5   # starting position (radian)
max_time = 50

"""
def rp(x, t):
    return [x[1], 
            -g/L*math.sin(x[0])]

theta = odeint(rp, [q0, 0], logTime)
logTheta = theta[:,0]
"""


physicsClient = p.connect(p.DIRECT) # or p.DIRECT for non-graphical version,  p.GUI
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
boxId = p.loadURDF("./simple.urdf", useFixedBase=True)

# get rid of all the default damping forces
p.changeDynamics(boxId, 1, linearDamping=0, angularDamping=0)
p.changeDynamics(boxId, 2, linearDamping=0, angularDamping=0)

# go to the starting position
p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetPosition=q0, controlMode=p.POSITION_CONTROL)
for _ in range(1000):
    p.stepSimulation()

# turn off the motor for the free motion
p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetVelocity=0, controlMode=p.VELOCITY_CONTROL, force=0)

# velocity and torque controllers
# p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, targetVelocity=0.0, controlMode=p.VELOCITY_CONTROL)
# p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, force=0.0, controlMode=p.TORQUE_CONTROL)
while True:
    if time.time() > max_time:
        break
    p.stepSimulation()
    p.getJointState(boxId, jointIndex=1)
    """
    Я данные добывал так - после каждого шага симуляции вызывал p.getJointState(boxId, jointIndex=1). Она возвращает кортеж из четырёх элементов, первый - позиция, второй - скорость, остальные нам не нужны.
Ну а дальше стандартно - накопить их в массиве и после завершения симуляции отрисовать этот массив на графике.
    """
    time.sleep(dt)
p.disconnect()