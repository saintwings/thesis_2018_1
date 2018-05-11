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

def check_score_posture("posture_type")


if __name__ == "__main__":

    # ### bye score ###
    # bye_elbow_score_dict = np.load('bye_elbow_score_array_reduce.npy').item()
    # bye_wrist_score_dict = np.load('bye_wrist_score_array_reduce.npy').item()
    # bye_quaternion_score_dict = np.load('bye_quaternion_score_array_reduce.npy').item()
    # print(len(bye_elbow_score_dict), len(bye_wrist_score_dict), len(bye_quaternion_score_dict))
    # bye_score = [bye_elbow_score_dict, bye_wrist_score_dict, bye_quaternion_score_dict]

    # bye_max_key = [max(bye_elbow_score_dict, key=lambda i: bye_elbow_score_dict[i]),
    #                 max(bye_wrist_score_dict, key=lambda i: bye_wrist_score_dict[i]),
    #                 max(bye_quaternion_score_dict, key=lambda i: bye_quaternion_score_dict[i])]   
    # bye_max_value = [bye_elbow_score_dict[(bye_max_key[0][0], bye_max_key[0][1], bye_max_key[0][2])],
    #                     bye_wrist_score_dict[(bye_max_key[1][0], bye_max_key[1][1], bye_max_key[1][2])],
    #                     bye_quaternion_score_dict[(bye_max_key[2][0], bye_max_key[2][1], bye_max_key[2][2], bye_max_key[2][3])]]    
    # print("bye_max_key", bye_max_key)
    # print("bye_max_value", bye_max_value)

    # ### salute score ###
    # salute_elbow_score_dict = np.load('salute_elbow_score_array_reduce.npy').item()
    # salute_wrist_score_dict = np.load('salute_wrist_score_array_reduce.npy').item()
    # salute_quaternion_score_dict = np.load('salute_quaternion_score_array_reduce.npy').item()
    # print(len(salute_elbow_score_dict), len(salute_wrist_score_dict), len(salute_quaternion_score_dict))
    # salute_score = [salute_elbow_score_dict, salute_wrist_score_dict, salute_quaternion_score_dict]

    # salute_max_key = [max(salute_elbow_score_dict, key=lambda i: salute_elbow_score_dict[i]),
    #                 max(salute_wrist_score_dict, key=lambda i: salute_wrist_score_dict[i]),
    #                 max(salute_quaternion_score_dict, key=lambda i: salute_quaternion_score_dict[i])]   
    # salute_max_value = [salute_elbow_score_dict[(salute_max_key[0][0], salute_max_key[0][1], salute_max_key[0][2])],
    #                     salute_wrist_score_dict[(salute_max_key[1][0], salute_max_key[1][1], salute_max_key[1][2])],
    #                     salute_quaternion_score_dict[(salute_max_key[2][0], salute_max_key[2][1], salute_max_key[2][2], salute_max_key[2][3])]]    
    # print("salute_max_key", salute_max_key)
    # print("salute_max_value", salute_max_value)

    ### sinvite score ###
    sinvite_elbow_score_dict = np.load('sinvite_elbow_score_array_reduce.npy').item()
    sinvite_wrist_score_dict = np.load('sinvite_wrist_score_array_reduce.npy').item()
    sinvite_quaternion_score_dict = np.load('sinvite_quaternion_score_array_reduce.npy').item()
    print(len(sinvite_elbow_score_dict), len(sinvite_wrist_score_dict), len(sinvite_quaternion_score_dict))
    sinvite_score = [sinvite_elbow_score_dict, sinvite_wrist_score_dict, sinvite_quaternion_score_dict]

    sinvite_max_key = [max(sinvite_elbow_score_dict, key=lambda i: sinvite_elbow_score_dict[i]),
                    max(sinvite_wrist_score_dict, key=lambda i: sinvite_wrist_score_dict[i]),
                    max(sinvite_quaternion_score_dict, key=lambda i: sinvite_quaternion_score_dict[i])]   
    sinvite_max_value = [sinvite_elbow_score_dict[(sinvite_max_key[0][0], sinvite_max_key[0][1], sinvite_max_key[0][2])],
                        sinvite_wrist_score_dict[(sinvite_max_key[1][0], sinvite_max_key[1][1], sinvite_max_key[1][2])],
                        sinvite_quaternion_score_dict[(sinvite_max_key[2][0], sinvite_max_key[2][1], sinvite_max_key[2][2], sinvite_max_key[2][3])]]    
    print("sinvite_max_key", sinvite_max_key)
    print("sinvite_max_value", sinvite_max_value)

    # ### wai score ###
    # wai_elbow_score_dict = np.load('wai_elbow_score_array_reduce.npy').item()
    # wai_wrist_score_dict = np.load('wai_wrist_score_array_reduce.npy').item()
    # wai_quaternion_score_dict = np.load('wai_quaternion_score_array_reduce.npy').item()
    # print(len(wai_elbow_score_dict), len(wai_wrist_score_dict), len(wai_quaternion_score_dict))
    # wai_score = [wai_elbow_score_dict, wai_wrist_score_dict, wai_quaternion_score_dict]

    # wai_max_key = [max(wai_elbow_score_dict, key=lambda i: wai_elbow_score_dict[i]),
    #                 max(wai_wrist_score_dict, key=lambda i: wai_wrist_score_dict[i]),
    #                 max(wai_quaternion_score_dict, key=lambda i: wai_quaternion_score_dict[i])]   
    # wai_max_value = [wai_elbow_score_dict[(wai_max_key[0][0], wai_max_key[0][1], wai_max_key[0][2])],
    #                     wai_wrist_score_dict[(wai_max_key[1][0], wai_max_key[1][1], wai_max_key[1][2])],
    #                     wai_quaternion_score_dict[(wai_max_key[2][0], wai_max_key[2][1], wai_max_key[2][2], wai_max_key[2][3])]]    
    # print("wai_max_key", wai_max_key)
    # print("wai_max_value", wai_max_value)


