"""
plugin that is loaded one time only at the beginning
It is meant to be for you to upload your environment
"""

import rospy
import pybullet_ros.sdf.sdf_parser as sdf_parser

class Environment:
    def __init__(self, pybullet, **kargs):
        # get "import pybullet as pb" and store in self.pb
        self.pb = pybullet
        # enable soft body simulation if needed
        if rospy.get_param('~use_deformable_world', False):
            rospy.loginfo('Using deformable world (soft body simulation)')
            self.pb.resetSimulation(self.pb.RESET_USE_DEFORMABLE_WORLD)

    def load_environment(self):
        """
        set gravity, ground plane and load URDF or SDF models as required
        """
        # set gravity
        gravity = rospy.get_param('~gravity', -9.81) # get gravity from param server
        self.pb.setGravity(0, 0, gravity)
        # give control to the user to upload custom world via code
        self.load_environment_via_code()

    def load_environment_via_code(self):
        """
        This method provides the possibility for the user to define an environment via python code
        example:
        self.pb.loadURDF(...)
        self.pb.loadSDF(...)
        self.pb.loadSoftBody(...)
        self.pb.setTimeStep(0.001)
        self.pb.setPhysicsEngineParameter(sparseSdfVoxelSize=0.25) # ? related to soft bodies
        etc...
        NOTE: is not advised to directly write code below, instead make new class and inherit from this one
              see example: environment_template.py
        """
        # set floor
        self.pb.loadURDF('plane.urdf', basePosition=[0., 0., -0.2])
        self.pb.configureDebugVisualizer(self.pb.COV_ENABLE_GUI, 0)
        self.pb.resetDebugVisualizerCamera(cameraDistance=3, cameraYaw=0, cameraPitch=-40,
                                           cameraTargetPosition=(1.7, 0.5, 0.5))
