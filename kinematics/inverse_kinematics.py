'''In this exercise you need to implemente inverse kinematics for NAO's legs

* Tasks:
    1. solve inverse kinemtatics for NAO's legs by using analytical or numerical method.
       You may need documentation of NAO's leg:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    2. use the results of inverse kinemtatics to control NAO's legs (in InverseKinematicsAgent.set_transforms)
       and test your inverse kinemtatics implementation.
'''


from forward_kinematics import ForwardKinematicsAgent
import numpy as np
from joint_data_provider import CHAINS
from math import atan2, sqrt, pi

EPSILON = 1e-6

class InverseKinematicsAgent(ForwardKinematicsAgent):
    
    def inverse_kinematics(self, effector_name, transform):
        '''solve the inverse kinematics

        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''
        joint_angles = []
        fitting_joints = {key : self.perception.joint[key] for key in CHAINS[effector_name]} #get joint->angle for all effector joints
        end_effector = CHAINS[effector_name][-1] # end effector for the given effectr chain
            
        while True: #break loop if angles are satisfiable
        
            self.forward_kinematics(fitting_joints) #execute forward kinematics
            T = self.transforms #result of forward kinematics for effector joints
            error = transform - T[end_effector] #calculate distance between target and actual result
            
            J = calculate_jacobian_matrix(error)
            
            
        # YOUR CODE HERE
        return joint_angles
        
    def calculate_alpha(error, jacobi):
        """
        :param error: vector which contains the error
        :param jacobi: jacobian matrix
        """
        JJTe = np.dot(np.dot(jacobi, np.transpose(jacobi)), error)
        return float(np.inner(error, JJTe)) / float(np.inner(JJTe, JJTe))

    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        
        self.inverse_kinematics(effector_name, transform)
        self.keyframes = ([], [], [])  # the result joint angles have to fill in
        
    def calculate_jacobian_matrix(error):
        for transform in error.T:
            J.append(extract_values(transform))
        return J
            
    def extract_values(transform):
        """
        :param transform: transformation matrix
        :return array with coordinates and angles
        """
        x = transform[0,-1]
        y = transform[1,-1]
        z = transform[2,-1]
        if(transform[3,1] > EPSILON):
            omega_x = atan2(transform[3,2], transform[3,3])
            omega_y = atan2(-transform[3,1], sqrt(transform[3,2]**2 + transform[3,3]**2))
            omega_z = atan2(transform[2,1], transform[1,1])
        else:
            omega_z = 0
            if(abs(transform[3,1] + 1) < EPSILON):
                omega_x = atan2(transform[1,2], transform[1,3])
                omega_y = pi / 2
            else:
                omega_x = atan2(-transform[1,2], -transform[1,3])
                omega_y = -pi / 2
        return np.array(x, y, z, omega_x, omega_y, omega_z)

	
if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = identity(4)
    T[-1, 1] = 0.05
    T[-1, 2] = 0.26
    agent.set_transforms('LLeg', T)
    agent.run()
