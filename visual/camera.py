import pybullet as pb
import numpy as np

class Camera:

    default_size = {
        'width': 250,
        'height': 250
    }

    def __init__(self, cameraEyePosition=[0.5, 0.5, 3.0], cameraTargetPosition=[0.5, 0.5, 2.0]):
        self.size = self.default_size
        self.targetVec = np.array(cameraTargetPosition) - np.array(cameraEyePosition)
        self.viewMatrix = pb.computeViewMatrix(
            cameraEyePosition=cameraEyePosition,
            cameraTargetPosition = cameraTargetPosition,
            cameraUpVector=[1, 0, 0])
        self.projectionMatrix = pb.computeProjectionMatrixFOV(
            fov=60,
            aspect=1.0,
            nearVal=0.1,
            farVal=100)
        self.cam_image_kwargs = {
            **self.size,
            'viewMatrix': self.viewMatrix,
            'projectionMatrix': self.projectionMatrix,
            'renderer': pb.ER_TINY_RENDERER
        }

    def set_new_height(self, h):
        self.__init__(size=self.size, height=h)

    def set_new_position(self, pos, upVector=[1,0,0]):
        self.viewMatrix = pb.computeViewMatrix(
            cameraEyePosition = pos,
            cameraTargetPosition = pos + self.targetVec,
            cameraUpVector = upVector)
        self.cam_image_kwargs = {
            **self.size,
            'viewMatrix': self.viewMatrix,
            'projectionMatrix': self.projectionMatrix,
        }

    def get_frame(self):
        """
        returns RGBA array of size (x, y, 4)
        """
        return pb.getCameraImage(**self.cam_image_kwargs)[2]
    
    def get_depth(self):
        """
        returns RGBA array of size (x, y, 4)
        """
        return pb.getCameraImage(**self.cam_image_kwargs)[3]
