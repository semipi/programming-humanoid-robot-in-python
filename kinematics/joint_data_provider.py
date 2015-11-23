# -*- coding: utf-8 -*-
"""
Created on Mon Nov 16 17:39:12 2015

@author: kai & maik
"""

HEADYAW         = 'HeadYaw'
HEADPITCH       = 'HeadPitch'
LSHOULDERPITCH  = 'LShoulderPitch'
LSHOULDERROLL   = 'LShoulderRoll'
LELBOWYAW       = 'LElbowYaw'
LELBOWROLL      = 'LElbowRoll'
RSHOULDERPITCH  = 'RShoulderPitch'
RSHOULDERROLL   = 'RShoulderRoll'
RELBOWYAW       = 'RElbowYaw'
RELBOWROLL      = 'RElbowRoll'
LHIPYAWPITCH    = 'LHipYawPitch'
LHIPROLL        = 'LHipRoll'
LHIPPITCH       = 'LHipPitch'
LKNEEPITCH      = 'LKneePitch'
LANKLEPITCH     = 'LAnklePitch'
LANKLEROLL      = 'LAnkleRoll'
RHIPYAWPITCH    = 'RHipYawPitch'
RHIPROLL        = 'RHipRoll'
RHIPPITCH       = 'RHipPitch'
RKNEEPITCH      = 'RKneePitch'
RANKLEPITCH     = 'RAnklePitch'
RANKLEROLL      = 'RAnkleRoll'

OFFSET = {  HEADYAW       : (   0.00,   0.00, 126.50), 
            HEADPITCH     : (   0.00,   0.00,   0.00),
            LSHOULDERPITCH: (   0.00,  98.00, 100.00),
            LSHOULDERROLL : (   0.00,   0.00,   0.00),
            LELBOWYAW     : ( 105.00,  15.00,   0.00),
            LELBOWROLL    : (   0.00,   0.00,   0.00),
            RSHOULDERPITCH: (   0.00, -98.00, 100.00),
            RSHOULDERROLL: (   0.00,   0.00,   0.00),
            RELBOWYAW     : ( 105.00, -15.00,   0.00),
            RELBOWROLL    : (   0.00,   0.00,   0.00),
            LHIPYAWPITCH  : (   0.00,  50.00, -85.00),
            LHIPROLL      : (   0.00,   0.00,   0.00),
            LHIPPITCH     : (   0.00,   0.00,   0.00),
            LKNEEPITCH    : (   0.00,   0.00,-100.00),
            LANKLEPITCH   : (   0.00,   0.00,-102.90),
            LANKLEROLL    : (   0.00,   0.00,   0.00),
            RHIPYAWPITCH  : (   0.00, -50.00, -85.00),
            RHIPROLL      : (   0.00,   0.00,   0.00),
            RHIPPITCH     : (   0.00,   0.00,   0.00),
            RKNEEPITCH    : (   0.00,   0.00,-100.00),
            RANKLEPITCH   : (   0.00,   0.00,-102.90),
            RANKLEROLL    : (   0.00,   0.00,   0.00)
         }
         
CHAINS = { 'Head' : [HEADYAW, HEADPITCH],
           'LArm' : [LSHOULDERPITCH, LSHOULDERROLL, LELBOWYAW, LELBOWROLL],
		'LLeg' : [LHIPYAWPITCH, LHIPROLL, LHIPPITCH, LKNEEPITCH, LANKLEPITCH, LANKLEROLL],
		'RLeg' : [RHIPYAWPITCH, RHIPROLL, RHIPPITCH, RKNEEPITCH, RANKLEPITCH, RANKLEROLL],
		'RArm' : [RSHOULDERPITCH, RSHOULDERROLL, RELBOWYAW, RELBOWROLL],
          }
 
