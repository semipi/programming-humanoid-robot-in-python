# -*- coding: utf-8 -*-
"""
Created on Mon Nov 16 17:39:12 2015

@author: kai & maik
"""

OFFSET = {  'HeadYaw'       : (   0.00,   0.00, 126.50), 
            'HeadPitch'     : (   0.00,   0.00,   0.00),
            'LShoulderPitch': (   0.00,  98.00, 100.00),
            'LShoulderRoll' : (   0.00,   0.00,   0.00),
            'LElbowYaw'     : ( 105.00,  15.00,   0.00),
            'LElbowRoll'    : (   0.00,   0.00,   0.00),
            'RShoulderPitch': (   0.00, -98.00, 100.00),
            'RShoulderRoll' : (   0.00,   0.00,   0.00),
            'RElbowYaw'     : ( 105.00, -15.00,   0.00),
            'RElbowRoll'    : (   0.00,   0.00,   0.00),
            'LHipYawPitch'  : (   0.00,  50.00, -85.00),
            'LHipRoll'      : (   0.00,   0.00,   0.00),
            'LHipPitch'     : (   0.00,   0.00,   0.00),
            'LKneePitch'    : (   0.00,   0.00,-100.00),
            'LAnklePitch'   : (   0.00,   0.00,-102.90),
            'LAnkleRoll'    : (   0.00,   0.00,   0.00),
            'RHipYawPitch'  : (   0.00, -50.00, -85.00),
            'RHipRoll'      : (   0.00,   0.00,   0.00),
            'RHipPitch'     : (   0.00,   0.00,   0.00),
            'RKneePitch'    : (   0.00,   0.00,-100.00),
            'RAnklePitch'   : (   0.00,   0.00,-102.90),
            'RAnkleRoll'    : (   0.00,   0.00,   0.00)
         }
         
CHAINS = { 'Head' : ['HeadYaw', 'HeadPitch'],
           'LArm' : ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll'],
		'LLeg' : ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll'],
		'RLeg' : ['RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll'],
		'RArm' : ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll']
          }