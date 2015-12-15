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
from joint_data_provider import CHAINS, ROT_X, ROT_Y, ROT_Z, JOINTS
from math import atan2, sqrt, pi

EPSILON = 1e-3

class InverseKinematicsAgent(ForwardKinematicsAgent):
    
    def inverse_kinematics(self, effector_name, transform):
        '''solve the inverse kinematics

        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''
        
        fitting_joints = {key : self.perception.joint[key] for key in CHAINS[effector_name]} #get (joint->angle ) for all effector joints
        
        effector_chain = CHAINS[effector_name] #end effector for the given effectr chain
        end_effector = effector_chain[-1] #end_effector
        
        Te = self.extract_values(transform)
        theta = np.zeros(len(fitting_joints)) #initial return value
        
                    
        while True: #break loop if angles are satisfiable
        
            self.forward_kinematics(fitting_joints) #execute forward kinematics
            T = self.transforms #result of forward kinematics for effector joints
            error = Te - self.extract_values(T[end_effector]) #calculate distance between target and actual result
            
            jacobi = self.calculate_jacobian_matrix(Te, T, effector_chain) #get jacobian matrix
            
            alpha = self.calculate_alpha(error, jacobi) #get scalar for jacobian tranpose method
            
            d_theta = alpha * np.dot(jacobi.T, error)#jacobian transpose method
            theta += d_theta #update theta value
            self.update_joint_angles(fitting_joints, theta) #update joint angles for next iteration
            print np.inner(error, error)
            if np.inner(error, error) < EPSILON: #break if scalar product of error is lower than tolerated error
                break
            
        return theta
        
    def calculate_alpha(self, error, jacobi):
        """
        :param error: vector which contains the error
        :param jacobi: jacobian matrix
        """
        JJTe = np.dot(np.dot(jacobi, np.transpose(jacobi)), error)
        return float(np.inner(error, JJTe)) / float(np.inner(JJTe, JJTe))

    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        theta = self.inverse_kinematics(effector_name, transform)
        self.keyframes = (CHAINS[effector_name], [], theta)  # the result joint angles have to fill in
        
    def calculate_jacobian_matrix(self, Te, T, effector_chain):
        jacobi = np.zeros((6, len(effector_chain)))
        for i, (x, t) in enumerate(T.iteritems()):
            x, y, z = self.extract_coordinate_values(t)
            jacobi[:i] = (Te - np.array([x, y, z, 0, 0, 0]))
        return self.set_joint_axis(jacobi, effector_chain)
        
    def extract_coordinate_values(self, transform):
        x = transform[-1,0]
        y = transform[-1,1]
        z = transform[-1,2]
        
        return x ,y ,z
            
    def extract_values(self, transform):
        """
        :param transform: transformation matrix
        :return array with coordinates and angles
        """
        
        x, y, z = self.extract_coordinate_values(transform)
        
        if(transform[2,0] > EPSILON):
            omega_x = atan2(transform[2,1], transform[2,2])
            omega_y = atan2(-transform[2,0], sqrt(transform[2,1]**2 + transform[2,2]**2))
            omega_z = atan2(transform[1,0], transform[0,0])
        else:
            omega_z = 0
            if(abs(transform[2,0] + 1) < EPSILON):
                omega_x = atan2(transform[0,1], transform[0,2])
                omega_y = pi / 2
            else:
                omega_x = atan2(-transform[0,1], -transform[0,2])
                omega_y = -pi / 2
        return np.array([x, y, z, omega_x, omega_y, omega_z])

    
    def set_joint_axis(self, jacobi, effector_chain):
        for i, joint in enumerate(effector_chain):
            for trafo in (matrix for matrix, joints in JOINTS.iteritems() if joint in joints):
                jacobi[self.get_angle_index(trafo), i] = 1
        return jacobi

    def get_angle_index(self, rotation):
        return 3 if rotation is ROT_X else 4 if rotation is ROT_Y else 5
        
    def update_joint_angles(self, joints, theta):
        for i, key in enumerate(joints.keys()):
            joints[key] = theta[i]
        return joints
	
if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = np.identity(4)
    T[-1, 1] = 0.05
    T[-1, 2] = 0.26
    agent.set_transforms('LLeg', T)
    agent.run()
