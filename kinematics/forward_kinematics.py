'''In this exercise you need to implement forward kinematics for NAO robot

* Tasks:
    1. complete the kinematics chain definition (self.chains in class ForwardKinematicsAgent)
       The documentation from Aldebaran is here:
       http://doc.aldebaran.com/2-1/family/robots/bodyparts.html#effector-chain
    2. implement the calculation of local transformation for one joint in function
       ForwardKinematicsAgent.local_trans. The necessary documentation are:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    3. complete function ForwardKinematicsAgent.forward_kinematics, save the transforms of all body parts in torso
       coordinate into self.transforms of class ForwardKinematicsAgent

* Hints:
    the local_trans has to consider different joint axes and link parameters for differnt joints
'''

# add PYTHONPATH
import os
import sys
import numpy as np
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'joint_control'))

from numpy.matlib import matrix, identity

from angle_interpolation import AngleInterpolationAgent
from joint_data_provider import OFFSET, CHAINS, JOINTS
                                      
class ForwardKinematicsAgent(AngleInterpolationAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(ForwardKinematicsAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.transforms = {n: identity(4) for n in self.joint_names}

    def think(self, perception):
        self.forward_kinematics(perception.joint)
        return super(ForwardKinematicsAgent, self).think(perception)

    def local_trans(self, joint_name, joint_angle):
        '''calculate local transformation of one joint
        :param str joint_name: the name of joint
        :param float joint_angle: the angle of joint in radians
        :return: transformation
        :rtype: 4x4 matrix
        '''
        T = identity(4)
        s = np.sin(joint_angle)
        c = np.cos(joint_angle)
        
        # transformation
        for trafo in (matrix for matrix, joints in JOINTS.iteritems() if joint_name in joints):
            T = np.dot(T, trafo(s, c))   
        
        # offset       
        x, y, z = OFFSET[joint_name]     
        T[3,0] = x
        T[3,1] = y
        T[3,2] = z
    
        return T

    def forward_kinematics(self, joints):
        '''forward kinematics
        :param joints: {joint_name: joint_angle}
        '''
        T = identity(4)
        for joint in joints.keys():
            angle = joints[joint]
            Tl = self.local_trans(joint, angle)
            self.transforms[joint] = np.dot(T, Tl)

if __name__ == '__main__':
    agent = ForwardKinematicsAgent()
    agent.run()
