'''In this exercise you need to implement an angle interploation function which makes NAO executes keyframe motion

* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interploation or Bezier interploation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interploation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder

* Keyframe data format:
    keyframe := (names, times, keys)
    names := [str, ...]  # list of joint names
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes the curve following the point.
'''


from pid import PIDAgent
from keyframes import hello, leftBackToStand, leftBellyToStand, rightBackToStand, rightBellyToStand, wipe_forehead

class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])
        self.start_time = self.perception.time

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        target_joints = {}
        
        start_time = perception.time - self.start_time
        
        names, times, keys = keyframes
        
        #Iterate through joints
        for i in range(len(names)):
            joint = names[i]
            # Only update known joints
            if joint in self.joint_names:
                # Each line of keys corresponds to a time
                for j in range(len(times[i]) - 1):
                    # Now is before interpolation
                    if start_time < times[i][0]:
                        target_joints[joint] = self.calculate_first_bezier_angle(times, keys, i, joint, start_time)
                    # Now is between interpolation
                    elif times[i][j] < start_time < times[i][j + 1] and j+1 < len(times[i]):
                        target_joints[joint] = self.calculate_bezier_angle(times, keys, i, j, joint, start_time)
        
        return target_joints
        
    def calculate_first_bezier_angle(self, times, keys, j_index, joint, start_time):
        '''
        @param ...
        @param j_index: index of joint
        @return bezier(t)
        '''
        # Time values
        t_0 = 0.0
        t_3 = times[j_index][0]
        
# ??? Rocket science ???
        # Angles
        a_0 = self.perception.joint[joint]
        a_3 = keys[j_index][0][0]
        # Control angles
        a_1 = keys[j_index][0][1][2] + a_0
        a_2 = keys[j_index][0][2][2] + a_3
        
#        a_0 = self.perception.joint[joint]
#        a_2 = keys[j_index][0][0]
#        # Control angles
#        a_1 = keys[j_index][0][1][2] + a_0
#        a_3 = keys[j_index][0][2][2] + a_2
        
        dt = (start_time) / t_3
        return self.calculate_bezier_interpolation(a_0, a_1, a_2, a_3, dt)
        
    def calculate_bezier_angle(self, times, keys, j_index, t_index, joint, start_time):
        '''
        @param ...
        @param j_index: index of joint
        @param t_index: time index
        @return bezier(t)
        '''
        # Time values
        t_0 = times[j_index][t_index]
        t_1 = keys[j_index][t_index][1][1] + t_0
        t_2 = times[j_index][t_index + 1]
        t_3 = keys[j_index][t_index][2][1] + t_2
        
# ??? Rocket science ???
        # Angles
        a_0 = keys[j_index][t_index][0]
        a_3 = keys[j_index][t_index + 1][0]
        # Control angles
        a_1 = keys[j_index][t_index][1][2] + a_0
        a_2 = keys[j_index][t_index][2][2] + a_3
        
#        # Angles
#        a_0 = keys[j_index][t_index][0]
#        a_2 = keys[j_index][t_index + 1][0]
#        # Control angles
#        a_1 = keys[j_index][t_index][1][2] + a_0
#        a_3 = keys[j_index][t_index][2][2] + a_2
#        
        dt = (start_time - t_0) / (t_3 - t_0)
        
        return self.calculate_bezier_interpolation(a_0, a_1, a_2, a_3, dt)
        
    @staticmethod
    def calculate_bezier_interpolation(a_0, a_1, a_2, a_3, dt):
        c_0 = (1 - dt)**3
        c_1 = 3 * (1 - dt)**2
        c_2 = 3 * (1 - dt)
        return c_0 * a_0 + c_1 * a_1 * dt + c_2 * a_2 * dt**2 + a_3 * dt**3
        

if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = leftBackToStand()  #hello, leftBackToStand, leftBellyToStand, rightBackToStand, rightBellyToStand, wipe_forehead
    agent.run()
    

