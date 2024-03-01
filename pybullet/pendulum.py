import pybullet as p
import time
import pybullet_data

# TASKS FROM 01.03.2024:
# 1  set maxTime
# 2  plots (pos, vel)
# 3  position control based on p.VELOCITY_CONTROL (proportional regulator)
# 4  position control based on p.TORQUE_CONTROL (PI-regulator)
# 5* compare plots of pybullet and our own odeint and figure out the source of errors and fix it
# 6* figure out how to add control to our own integration script

dt = 1/240 # pybullet simulation step
q0 = 0.5   # starting position (radian)

physicsClient = p.connect(p.GUI) # or p.DIRECT for non-graphical version
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
    p.stepSimulation()
    time.sleep(dt)
p.disconnect()