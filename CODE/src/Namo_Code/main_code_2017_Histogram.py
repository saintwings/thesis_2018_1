from configobj import ConfigObj
import glob
import sys
import os
import os.path
scriptpath = os.path.dirname(__file__)
import numpy as np
from math import pow, sqrt, sin, cos, radians
from scipy.stats import norm
from sklearn.svm import SVR
from sklearn.metrics import mean_squared_error, explained_variance_score

import matplotlib.pyplot as plt
from sklearn.metrics import classification_report, mean_absolute_error
from sklearn import cross_validation, preprocessing
from sklearn.ensemble import ExtraTreesRegressor
from sklearn.metrics import classification_report

from sklearn.neural_network import MLPRegressor

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from tempfile import TemporaryFile



import time
#import pickle

from sys import getsizeof
from sys import exit, getsizeof

from utility_function_2017 import *

def create_3dim_normalize_score_array(radius):

    normal_dis_array_size = radius + radius + 1
    normal_dis_sigma_width = 3

    array = np.zeros((normal_dis_array_size,normal_dis_array_size,normal_dis_array_size))
    for z in range(0,normal_dis_array_size):
        for y in range(0,normal_dis_array_size):
            for x in range(0, normal_dis_array_size):
                distance = sqrt(pow(x-radius,2) + pow(y-radius,2) + pow(z-radius,2))
                distance = distance*normal_dis_sigma_width/radius
                array[x, y, z] = round(norm.pdf(distance), 3)

    #print(array)
    return array

def create_4dim_normalize_score_array(radius):

    normal_dis_array_size = radius + radius + 1
    normal_dis_sigma_width = 3

    array = np.zeros((normal_dis_array_size,normal_dis_array_size,normal_dis_array_size,normal_dis_array_size))
    for z in range(0,normal_dis_array_size):
        for y in range(0,normal_dis_array_size):
            for x in range(0, normal_dis_array_size):
                for w in range(0, normal_dis_array_size):
                    distance = sqrt(pow(w-radius,2) + pow(x-radius,2) + pow(y-radius,2) + pow(z-radius,2))
                    distance = distance*normal_dis_sigma_width/radius
                    array[w, x, y, z] = round(norm.pdf(distance), 3)

    #print(array)
    return array
def collect_data(postureName, postureType = 'main'):
    posture_dataset = []
    fileName_load = os.path.join(scriptpath,'Postures\posture_set_' + str(postureName))
    for i, filename in enumerate(glob.glob(os.path.join(fileName_load, '*.ini'))):
    #path = './Postures/posture_set_' + str(postureName)
    #for i, filename in enumerate(glob.glob(os.path.join(path, '*.ini'))):
        config = ConfigObj(filename)
        try:
            main_index = [index for index, x in enumerate(config['Keyframe_Posture_Type']) if x == postureType]
            #print("main Index",main_index)
        except:
            print("no posture type >>> exit")
            sys.exit()
        for index in main_index:
            posture_dataset.append(list(map(int, config['Keyframe_Value']['Keyframe_' + str(index)])))

    data_set = convert_motorValue_to_cartesianSpace(posture_dataset)
    return data_set

def convert_motorValue_to_cartesianSpace(posture_dataSet):
    int_motorDirection_and_ratio = [0.5, -1, 1, 1, 1, -1, 1,-0.5, -1, 1, -1, 1, -1, -1, 1, -1, 1]

    ### read motor center value ###
    fileName_load = os.path.join(scriptpath,'Postures\motor_center.txt')
    #file_center = open("./Postures/motor_center.txt", 'r')
    file_center = open(fileName_load, 'r')
    int_motorCenterValue = file_center.read()
    file_center.close()
    int_motorCenterValue = int_motorCenterValue.split('\n')
    for x in range(17):
        int_motorCenterValue[x] = int(int_motorCenterValue[x])

    ### cal diff ###
    diff_set = []
    for i, item in enumerate(posture_dataSet):
        diff = []
        for j,value in enumerate(item):
            diff.append(round((value - int_motorCenterValue[j])*int_motorDirection_and_ratio[j]*359/4095,0))

        diff_set.append(diff)

    #print("diff_set=",diff_set)

    ### convert to degree ###
    motorDegree_set = []
    for i, item in enumerate(diff_set):
        motorDegree = []
        for j, value in enumerate(item):
            #motorDegree.append(round(value * 359 / 4095.0,0))
            motorDegree.append(value)

        motorDegree_set.append(motorDegree)

   #print("motor_degree=", motorDegree_set)

    return motorDegree_set

def reduce_score_data(score, offset, scale):

    # print(getsizeof(bye_elbow_score_array))
    # print(bye_elbow_score_array.shape)
    # print(np.amax(bye_elbow_score_array))
    # print(np.amin(bye_elbow_score_array))
    # print(np.mean(bye_elbow_score_array))
    # print(np.average(bye_elbow_score_array))
    # print(np.std(bye_elbow_score_array))
    # print(bye_elbow_score_array.ndim)

    score_mean = np.mean(score)
    score_std = np.std(score)
    
    lower_score = score_mean + (4)*score_std
    #lower_score = 3

    # print("score_mean = " , score_mean)
    # print("score_std = "  ,score_std)
    # print("lower_score = "  ,lower_score)

    new_score_data = {}
    for d1_i, d1_value in enumerate(score):
        for d2_i, d2_value in enumerate(d1_value):
            for d3_i, d3_value in enumerate(d2_value):
                if score.ndim == 3:
                    if d3_value >= lower_score:
                        new_score_data[(d1_i + offset[0], d2_i + offset[1], d3_i + offset[2])] = d3_value
                        #print(d1_i,d2_i,d3_i,d3_value)
                    
                elif score.ndim == 4:
                    for d4_i, d4_value in enumerate(d3_value):
                        if d4_value >= lower_score:
                            new_score_data[(d1_i + offset[0], d2_i + offset[1], d3_i + offset[2], d4_i + offset[3])] = d4_value
                            #print(d1_i,d2_i,d3_i,d4_i,d4_value)
    print(getsizeof(new_score_data))
    print(len(new_score_data))
    return new_score_data

def build_scoring_histogram_model(postureName, postureType, save_name):
    base_score_3dim = create_3dim_normalize_score_array(20) ### @param(sphere_radius)

    

    ### load data ###
    print("load data" + str(postureName)+str(postureType))
    jointAngle_degree_set = collect_data(postureName, postureType) ### @param(postureName)
    ### extract right arm data ###
    print("extract right arm data")
    right_side_set = extract_arm_data( jointAngle_degree_set,'right')

    # ### calculate each posture stat ###
    # right_side_bye_stat = calculate_stat_all_joint(right_side_bye_set)

    ### calculate kinematics ###
    kinematics_set = collect_kinematics_data(right_side_set)

    # ### collect sinvite catesian position and quaternion ###
    elbow_position = np.asarray(collect_cartesian_position_data(kinematics_set, 3))  ### 3 = elbow position
    wrist_position = np.asarray(collect_cartesian_position_data(kinematics_set, 4))  ### 4 = wrist position

    
    
    elbow_score_array, elbow_index_offset = build_score_array_and_offset_3D(elbow_position, 42, base_score_3dim)
    wrist_score_array, wrist_index_offset = build_score_array_and_offset_3D(wrist_position, 42, base_score_3dim)

    

    elbow_score_array_reduce = reduce_score_data(elbow_score_array, elbow_index_offset, 100)
    np.save(str(postureName)+'_'+str(postureType)+ "_elbow_score_array",elbow_score_array_reduce)

    wrist_score_array_reduce = reduce_score_data(wrist_score_array, wrist_index_offset, 100)
    np.save(str(postureName)+'_'+str(postureType)+ "_wrist_score_array",wrist_score_array_reduce)

    if(postureType == 'main'):
        base_score_4dim = create_4dim_normalize_score_array(5)
        quaternion = np.asarray(collect_quaternion_data(kinematics_set))
        quaternion_upScale = np.copy(upScale_quaternion_value(quaternion, 100)) 
        quaternion_score_array, quaternion_index_offset = build_score_array_and_offset_4D(quaternion_upScale, 42, base_score_4dim)

        quaternion_score_array_reduce = reduce_score_data(quaternion_score_array, quaternion_index_offset, 100)
        np.save(str(postureName)+'_'+str(postureType)+ "_quaternion_score_array",quaternion_score_array_reduce)


##################################################################################################
if __name__ == "__main__":
    build_scoring_histogram_model("side_invite", "pre", "001")

    # base_score_3dim = create_3dim_normalize_score_array(20) ### @param(sphere_radius)

    # base_score_4dim = create_4dim_normalize_score_array(5)

    # print(scriptpath)
    # ############# Start prepair data #############

    #  ### load data ###
    # jointAngle_degree_bye_set = collect_data('bye') ### @param(postureName)
    # jointAngle_degree_salute_set = collect_data('salute')  ### @param(postureName)
    # jointAngle_degree_sinvite_set = collect_data('side_invite')  ### @param(postureName)
    # jointAngle_degree_wai_set = collect_data('wai')  ### @param(postureName)

    # ### extract right arm data ###
    # right_side_bye_set = extract_arm_data( jointAngle_degree_bye_set,'right')
    # right_side_salute_set = extract_arm_data(jointAngle_degree_salute_set, 'right')
    # right_side_sinvite_set = extract_arm_data(jointAngle_degree_sinvite_set, 'right')
    # right_side_wai_set = extract_arm_data(jointAngle_degree_wai_set, 'right')

    # np.save("right_side_bye_set",right_side_bye_set)
    # np.save("right_side_salute_set",right_side_salute_set)
    # np.save("right_side_sinvite_set",right_side_sinvite_set)
    # np.save("right_side_wai_set",right_side_wai_set)

    # sys.exit()

    # ### calculate each posture stat ###
    # right_side_bye_stat = calculate_stat_all_joint(right_side_bye_set)
    # right_side_salute_stat = calculate_stat_all_joint(right_side_salute_set)
    # right_side_sinvite_stat = calculate_stat_all_joint(right_side_sinvite_set)
    # right_side_wai_stat = calculate_stat_all_joint(right_side_wai_set)

    # ### calculate average join angle from all posture ###
    # all_posture_stat_list = [right_side_bye_stat, right_side_salute_stat, right_side_sinvite_stat, right_side_wai_stat]
    # ### type :: 'std' = standard_deviation, 'equl' = all weight equal
    # avg_joint_angle_std = find_avg_joint_angle(all_posture_stat_list, 'std')
    # avg_joint_angle_equl = find_avg_joint_angle(all_posture_stat_list, 'equl')
    # print(avg_joint_angle_std)
    # print(avg_joint_angle_equl)

    # ### calculate kinematics ###
    # bye_kinematics_set = collect_kinematics_data(right_side_bye_set)
    # salute_kinematics_set = collect_kinematics_data(right_side_salute_set)
    # sinvite_kinematics_set = collect_kinematics_data(right_side_sinvite_set)
    # wai_kinematics_set = collect_kinematics_data(right_side_wai_set)

    # # # ##### bye posture #####
    # # # ### collect bye catesian position and quaternion ###
    # # bye_elbow_position = np.asarray(collect_cartesian_position_data(bye_kinematics_set, 3))  ### 3 = elbow position
    # # bye_wrist_position = np.asarray(collect_cartesian_position_data(bye_kinematics_set, 4))  ### 4 = wrist position
    
    # # bye_quaternion = np.asarray(collect_quaternion_data(bye_kinematics_set))
    # # bye_quaternion_upScale = np.copy(upScale_quaternion_value(bye_quaternion, 100))
    
    # # bye_elbow_score_array, bye_elbow_index_offset = build_score_array_and_offset_3D(bye_elbow_position, 42, base_score_3dim)
    # # bye_wrist_score_array, bye_wrist_index_offset = build_score_array_and_offset_3D(bye_wrist_position, 42, base_score_3dim)
    # # bye_quaternion_score_array, bye_quatenion_index_offset = build_score_array_and_offset_4D(bye_quaternion_upScale, 42, base_score_4dim)

    # # bye_elbow_score_array_reduce = reduce_score_data(bye_elbow_score_array, bye_elbow_index_offset, 100)
    # # np.save("bye_elbow_score_array_reduce",bye_elbow_score_array_reduce)

    # # bye_wrist_score_array_reduce = reduce_score_data(bye_wrist_score_array, bye_wrist_index_offset, 100)
    # # np.save("bye_wrist_score_array_reduce",bye_wrist_score_array_reduce)

    # # bye_quaternion_score_array_reduce = reduce_score_data(bye_quaternion_score_array, bye_quatenion_index_offset, 100)
    # # np.save("bye_quaternion_score_array_reduce",bye_quaternion_score_array_reduce)


    # # # ##### salute posture #####
    # # # ### collect salute catesian position and quaternion ###
    # # salute_elbow_position = np.asarray(collect_cartesian_position_data(salute_kinematics_set, 3))  ### 3 = elbow position
    # # salute_wrist_position = np.asarray(collect_cartesian_position_data(salute_kinematics_set, 4))  ### 4 = wrist position

    # # salute_quaternion = np.asarray(collect_quaternion_data(salute_kinematics_set))
    # # salute_quaternion_upScale = np.copy(upScale_quaternion_value(salute_quaternion, 100))

    # # salute_elbow_score_array, salute_elbow_index_offset = build_score_array_and_offset_3D(salute_elbow_position, 42, base_score_3dim)
    # # salute_wrist_score_array, salute_wrist_index_offset = build_score_array_and_offset_3D(salute_wrist_position, 42, base_score_3dim)
    # # salute_quaternion_score_array, salute_quaternion_index_offset = build_score_array_and_offset_4D(salute_quaternion_upScale, 42, base_score_4dim)

    # # salute_elbow_score_array_reduce = reduce_score_data(salute_elbow_score_array, salute_elbow_index_offset, 100)
    # # np.save("salute_elbow_score_array_reduce",salute_elbow_score_array_reduce)

    # # salute_wrist_score_array_reduce = reduce_score_data(salute_wrist_score_array, salute_wrist_index_offset, 100)
    # # np.save("salute_wrist_score_array_reduce",salute_wrist_score_array_reduce)

    # # salute_quaternion_score_array_reduce = reduce_score_data(salute_quaternion_score_array, salute_quaternion_index_offset, 100)
    # # np.save("salute_quaternion_score_array_reduce",salute_quaternion_score_array_reduce)

    # # ##### sinvite posture #####
    # # ### collect sinvite catesian position and quaternion ###
    # sinvite_elbow_position = np.asarray(collect_cartesian_position_data(sinvite_kinematics_set, 3))  ### 3 = elbow position
    # sinvite_wrist_position = np.asarray(collect_cartesian_position_data(sinvite_kinematics_set, 4))  ### 4 = wrist position

    # sinvite_quaternion = np.asarray(collect_quaternion_data(sinvite_kinematics_set))
    # sinvite_quaternion_upScale = np.copy(upScale_quaternion_value(sinvite_quaternion, 100)) 

    # sinvite_elbow_score_array, sinvite_elbow_index_offset = build_score_array_and_offset_3D(sinvite_elbow_position, 42, base_score_3dim)
    # sinvite_wrist_score_array, sinvite_wrist_index_offset = build_score_array_and_offset_3D(sinvite_wrist_position, 42, base_score_3dim)
    # sinvite_quaternion_score_array, sinvite_quaternion_index_offset = build_score_array_and_offset_4D(sinvite_quaternion_upScale, 42, base_score_4dim)

    # sinvite_elbow_score_array_reduce = reduce_score_data(sinvite_elbow_score_array, sinvite_elbow_index_offset, 100)
    # np.save("sinvite_elbow_score_array_reduce",sinvite_elbow_score_array_reduce)

    # sinvite_wrist_score_array_reduce = reduce_score_data(sinvite_wrist_score_array, sinvite_wrist_index_offset, 100)
    # np.save("sinvite_wrist_score_array_reduce",sinvite_wrist_score_array_reduce)

    # sinvite_quaternion_score_array_reduce = reduce_score_data(sinvite_quaternion_score_array, sinvite_quaternion_index_offset, 100)
    # np.save("sinvite_quaternion_score_array_reduce",sinvite_quaternion_score_array_reduce)

    # ##### wai posture #####
    # ### collect wai catesian position and quaternion ###
    # wai_elbow_position = np.asarray(collect_cartesian_position_data(wai_kinematics_set, 3))  ### 3 = elbow position
    # wai_wrist_position = np.asarray(collect_cartesian_position_data(wai_kinematics_set, 4))  ### 4 = wrist position

    # wai_quaternion = np.asarray(collect_quaternion_data(wai_kinematics_set))
    # wai_quaternion_upScale = np.copy(upScale_quaternion_value(wai_quaternion, 100)) 

    # wai_elbow_score_array, wai_elbow_index_offset = build_score_array_and_offset_3D(wai_elbow_position, 42, base_score_3dim)
    # wai_wrist_score_array, wai_wrist_index_offset = build_score_array_and_offset_3D(wai_wrist_position, 42, base_score_3dim)
    # wai_quaternion_score_array, wai_quaternion_index_offset = build_score_array_and_offset_4D(wai_quaternion_upScale, 42, base_score_4dim)

    # wai_elbow_score_array_reduce = reduce_score_data(wai_elbow_score_array, wai_elbow_index_offset, 100)
    # print(type(wai_elbow_score_array_reduce))
    # np.save("wai_elbow_score_array_reduce",wai_elbow_score_array_reduce)

    # wai_wrist_score_array_reduce = reduce_score_data(wai_wrist_score_array, wai_wrist_index_offset, 100)
    # np.save("wai_wrist_score_array_reduce",wai_wrist_score_array_reduce)

    # wai_quaternion_score_array_reduce = reduce_score_data(wai_quaternion_score_array, wai_quaternion_index_offset, 100)
    # np.save("wai_quaternion_score_array_reduce",wai_quaternion_score_array_reduce)

    # # wai_dataSet = np.concatenate((wai_elbow_position[0:100,:], wai_wrist_position[0:100,:], wai_quaternion[0:100, :-1]), axis=1)
    # # wai_dataSet = np.insert(wai_dataSet, 10, 4, axis=1)

    # # print(wai_quaternion[0])
    # # print(wai_quaternion_upScale[0])

    # # all_dataSet = np.concatenate((bye_dataSet, salute_dataSet, sinvite_dataSet, wai_dataSet), axis=0)

    # # np.savetxt("all_dataSet",all_dataSet)

    # # #print(all_dataSet[0])

    # # ############# finished prepair data #############
