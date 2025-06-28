import time
import os

import numpy as np
import pybullet as p
import pybullet_data

from packages.load_gripper import Gripper, Generic
from packages import camera

class Environment():
    def __init__(self, gui):
        self.camera = None
        self.pix_size = 0.003125
        self.obj_ids = {'fixed': [], 'rigid': [], 'deformable': []}
        self.homej = np.array([
            -1, 
            -0.5, 
            0.5, 
            -0.5, 
            -0.5, 
            0]) * np.pi

        p.connect(p.GUI if gui else p.DIRECT)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        script_dir = os.path.dirname(os.path.abspath(__file__))
        p.setAdditionalSearchPath(script_dir)
        p.setTimeStep(1. / 240)

        if gui:
            target = p.getDebugVisualizerCamera()[11]
            p.resetDebugVisualizerCamera(
                cameraDistance = 1.1,
                cameraYaw=90,
                cameraPitch=-25,
                cameraTargetPosition = target
            )

    def reset(self, task=None):
        self.task = task
        self.obj_ids = {'fixed': [], 'rigid': [], 'deformable': []}
        p.setGravity(0, 0, -9.8)

        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)

        p.loadURDF('assets/plane/plane.urdf', (0, 0, -0.001)) # prevent z-fighting
        p.loadURDF('assets/ur5/workspace.urdf', (0.5, 0, 0))

        self.ur5 = p.loadURDF('assets/ur5/ur5.urdf')
        # self.ee = self.task.ee(self.ur5, 9, self.obj_ids)
        # self.ee_tip = 10

        self.ee = 9  # Use correct link index for 'ee_link' in your URDF
        self.gripper = Generic(self.ur5, self.ee, self.obj_ids)
        self.ee_tip = 10

        # get joint indices
        n_joints = p.getNumJoints(self.ur5)
        joints = [p.getJointInfo(self.ur5, i) for i in range(n_joints)]
        self.joints = [j[0] for j in joints if j[2] == p.JOINT_REVOLUTE]

        # Move robot to home joint configuration
        for i in range(len(self.joints)):
            p.resetJointState(self.ur5, self.joints[i], self.homej[i])

        # Reset end effector
        # self.ee.release()

        # Reset task
        # self.task.reset(self)
        
        # Re-enable rendering.
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

        #return self.step()

    def step(self, act=None):
        if act:
            timeout = self.task.primitive(self.movej, self.movep, self.ee, **act)

            if timeout:
                return {}, 0, True, self.info
        
        while not self.is_static:
            p.stepSimulation()
        
        reward, info = self.task.reward() if act else (0, {})
        done = self.task.done()

        info.update(self.info)

        return obs, reward, done, info