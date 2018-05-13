import random
from copy import deepcopy
#import math
from copy import copy
import time
from utility_function_2017 import *
from math import pow, sqrt, sin, cos, radians, exp
import numpy as np
import pickle
import sys
import os.path
scriptpath = os.path.dirname(__file__)

def check_score_posture(posture_type, posture_value,score_weight):
    elbow_score_dict = np.load(str(posture_type) +'_elbow_score_array_reduce.npy').item()
    wrist_score_dict = np.load(str(posture_type) +'_wrist_score_array_reduce.npy').item()
    q_score_dict = np.load(str(posture_type) +'_quaternion_score_array_reduce.npy').item()

    max_key = [max(elbow_score_dict, key=lambda i: elbow_score_dict[i]),
                    max(wrist_score_dict, key=lambda i: wrist_score_dict[i]),
                    max(q_score_dict, key=lambda i: q_score_dict[i])]   
    max_value = [elbow_score_dict[(max_key[0][0], max_key[0][1], max_key[0][2])],
                        wrist_score_dict[(max_key[1][0], max_key[1][1], max_key[1][2])],
                        q_score_dict[(max_key[2][0], max_key[2][1], max_key[2][2], max_key[2][3])]] 

    T_matrix = cal_kinematics_namo_numpy(posture_value, 'R')

    Q = cal_quaternion(T_matrix[7])

    elbow_position = [np.round(T_matrix[3][0, 3],0), np.round(T_matrix[3][1,3],0), np.round(T_matrix[3][2,3],0)]
    wrist_position = [np.round(T_matrix[4][0, 3],0), np.round(T_matrix[4][1,3],0), np.round(T_matrix[4][2,3],0)]
    Q_position = [np.round(Q[0]*100,0), np.round(Q[1]*100,0), np.round(Q[2]*100,0), np.round(Q[3]*100,0)]

    try:
        score_elbow = elbow_score_dict[(elbow_position[0], elbow_position[1], elbow_position[2])]
    except:
        score_elbow = 0

    try:
        score_wrist = wrist_score_dict[(wrist_position[0], wrist_position[1], wrist_position[2])]
    except:
        score_wrist = 0

    try:
        score_Q = q_score_dict[(Q_position[0], Q_position[1], Q_position[2], Q_position[3])]
    except:
        score_Q = 0

    sum_socre = (score_weight[0] - (score_elbow*score_weight[0]/max_value[0])) + (score_weight[1] - (score_wrist*score_weight[1]/max_value[1])) + (score_weight[2] -(score_Q*score_weight[2]/max_value[2]))

    
    print("posture >>>>>>>", str(posture_type))
    print("elbow", elbow_position, score_elbow, score_elbow/max_value[0]*score_weight[0])
    print("wrist", wrist_position, score_wrist, score_wrist/max_value[1]*score_weight[1])
    print("Q", Q_position, score_Q, score_Q/max_value[2]*score_weight[2])

    print("sum score=", sum_socre)


if __name__ == "__main__":

    posture_weight = [1, 1, 1]

    # ############### joint 0 = 68 #############

    # check_score_posture("bye",[68,-20,-16,105,-55,-18,-35],posture_weight)

    # check_score_posture("salute",[68,-17,-45,93,-16,16,45],posture_weight)

    # check_score_posture("sinvite",[68,-44,26,57,19,1,-10],posture_weight)

    # check_score_posture("wai",[68,-3,-23,127,3,4,-7],posture_weight)

    # ############### joint 1 = -19 #############

    # check_score_posture("bye",[70,-19,10,101,-46,-3,-14],posture_weight)

    # check_score_posture("salute",[103,-19,-17,107,7,6,-26],posture_weight)

    # check_score_posture("sinvite",[29,-19,20,25,79,38,15],posture_weight)

    # check_score_posture("wai",[72,-19,-28,112,15,-5,12],posture_weight)

    # ############### joint 2 = -7 #############

    # check_score_posture("bye",[79,-3,-7,50,-52,-48,0],posture_weight)

    # check_score_posture("salute",[100,-6,-7,107,41,45,-19],posture_weight)

    # check_score_posture("sinvite",[30,-17,-7,104,69,-41,31],posture_weight)

    # check_score_posture("wai",[68,-11,-7,115,16,16,5],posture_weight)

    # ############### joint 3 = 101 #############

    # check_score_posture("bye",[55,-9,28,101,-90,-23,36],posture_weight)

    # check_score_posture("salute",[113,-44,-20,101,62,45,-2],posture_weight)

    # check_score_posture("sinvite",[54,-40,39,101,18,-6,-43],posture_weight)

    # check_score_posture("wai",[49,0,-41,101,-7,-15,33],posture_weight)

    # ############### joint 4 = 2 #############

    # check_score_posture("bye",[54,-9,-21,94,2,-21,-8],posture_weight)

    # check_score_posture("salute",[114,-37,-38,107,2,-17,0],posture_weight)

    # check_score_posture("sinvite",[54,-39,36,30,2,33,6],posture_weight)

    # check_score_posture("wai",[88,-13,2,61,2,35,40],posture_weight)

    # ############### joint 5 = 1 #############

    # check_score_posture("bye",[54,-10,14,105,-63,1,16],posture_weight)

    # check_score_posture("salute",[114,-37,-38,107,-24,1,36],posture_weight)

    # check_score_posture("sinvite",[30,-17,44,66,68,1,28],posture_weight)

    # check_score_posture("wai",[88,-19,-7,130,32,1,-25],posture_weight)

    # ############### joint 6 = 24 #############

    # check_score_posture("bye",[54,-9,15,95,-52,-20,24],posture_weight)

    # check_score_posture("salute",[114,-37,-38,107,-67,1,24],posture_weight)

    # check_score_posture("sinvite",[36,-29,0,74,40,-32,24],posture_weight)

    # check_score_posture("wai",[88,-14,-45,89,20,-15,24],posture_weight)


    posture_weight = [0.5, 2, 1]
    check_score_posture("wai",[72,-19,29,107,45,45,-11],posture_weight)

    
    