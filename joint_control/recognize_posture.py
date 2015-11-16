'''In this exercise you need to use the learned classifier to recognize current posture of robot

* Tasks:
    1. load learned classifier in `PostureRecognitionAgent.__init__`
    2. recognize current posture in `PostureRecognitionAgent.recognize_posture`

* Hints:
    Let the robot execute different keyframes, and recognize these postures.

'''


from angle_interpolation import AngleInterpolationAgent
from keyframes import hello
import pickle as pickle
from os import listdir, path

ROBOT_POSE_DATA_DIR = 'robot_pose_data'
ROBOT_POSE_CLR = 'robot_pose.pkl'

class PostureRecognitionAgent(AngleInterpolationAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(PostureRecognitionAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.posture = 'unknown'
        self.posture_classifier = ROBOT_POSE_CLR

    def think(self, perception):
        self.posture = self.recognize_posture(perception)
        return super(PostureRecognitionAgent, self).think(perception)

    def recognize_posture(self, perception):        
        classifier = pickle.load(open(self.posture_classifier))
        pose_index = classifier.predict(self.get_actual_posture_data(perception))
        return listdir(ROBOT_POSE_DATA_DIR)[pose_index]
        
    def get_actual_posture_data(self, perception):
        posture_data = []
        
        
        posture_data.append(perception.joint['LHipYawPitch'])
        posture_data.append(perception.joint['LHipRoll'])
        posture_data.append(perception.joint['LHipPitch'])
        posture_data.append(perception.joint['LKneePitch'])
        posture_data.append(perception.joint['RHipYawPitch'])
        posture_data.append(perception.joint['RHipRoll'])
        posture_data.append(perception.joint['RHipPitch'])
        posture_data.append(perception.joint['RKneePitch'])
	posture_data.append(perception.imu[0])
        posture_data.append(perception.imu[1])
        
        return posture_data

if __name__ == '__main__':
    agent = PostureRecognitionAgent()
    agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
