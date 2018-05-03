from configobj import ConfigObj
import glob
import os
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



import time
import pickle

from sys import getsizeof

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

def add_value_to_index(n):
    array = []
    if n == 0:
        array.append(0)
    else:
        array.append(n-1)
        add_value_to_index(n-1)

    print("aaa=",array)

def create_ndim_normalize_score_array(radius, ndim):

    normal_dis_array_size = radius + radius + 1
    normal_dis_sigma_width = 3

    array_dim = []
    for i in range(ndim):
        array_dim.append(normal_dis_array_size)
    print("array_dim=",array_dim)
    sphere_array = np.zeros(array_dim)
    print(sphere_array)
    # sphere_array[0, 0, 0, 0, 0, 0] = 1
    #
    # print("value",sphere_array[0,0,0,0,0,0])

    index = []

    for dim in range(ndim):
        sub_index = []
        n = normal_dis_array_size

def convert_cartesianSpace_to_motorValue_rightSide(posture_dataSet):
    int_motorDirection_and_ratio = [-0.5, -1, 1, -1, 1, -1, -1]
    #print("posture_data=",posture_dataSet)
    ### read motor center value ###
    file_center = open('./Postures/motor_center.txt', 'r')
    int_motorCenterValue = file_center.read()
    file_center.close()
    int_motorCenterValue = int_motorCenterValue.split('\n')
    for x in range(17):
        int_motorCenterValue[x] = int(int_motorCenterValue[x])

    index_range = [7, 14]
    int_motorCenterValue = int_motorCenterValue[index_range[0]:index_range[1]]

    #print("center =",int_motorCenterValue)
    motorValue_set = []
    for i, value in enumerate(posture_dataSet):
        motorValue_set.append((value/int_motorDirection_and_ratio[i]*4095/359))

    for i, value in enumerate(motorValue_set):
        motorValue_set[i] = int(round(motorValue_set[i] + int_motorCenterValue[i],0))


    print("motor_value=",motorValue_set)
    #
    # ### cal diff ###
    # diff_set = []
    # for i, item in enumerate(posture_dataSet):
    #     diff = []
    #     for j,value in enumerate(item):
    #         diff.append((value - int_motorCenterValue[j])*int_motorDirection_and_ratio[j])
    #
    #     diff_set.append(diff)
    #
    # #print("diff_set=",diff_set)
    #
    # ### convert to degree ###
    # motorDegree_set = []
    # for i, item in enumerate(diff_set):
    #     motorDegree = []
    #     for j, value in enumerate(item):
    #         motorDegree.append(round(value * 359 / 4095.0,0))
    #
    #     motorDegree_set.append(motorDegree)
    #
    # #print("motor_degree=", motorDegree_set)

    return motorValue_set

def convert_motorValue_to_cartesianSpace(posture_dataSet):
    int_motorDirection_and_ratio = [0.5, -1, 1, 1, 1, -1, 1,-0.5, -1, 1, -1, 1, -1, -1, 1, -1, 1]

    ### read motor center value ###
    file_center = open('./Postures/motor_center.txt', 'r')
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

def calculate_stat_all_joint(posture_dataSet):

    mean = np.mean(posture_dataSet, axis = 0)
    std = np.std(posture_dataSet, axis = 0)
    var = np.var(posture_dataSet, axis = 0)

    return [mean, std, var]

def collect_data(postureName):

    posture_dataset = []
    path = './Postures/posture_set_' + str(postureName)
    for i, filename in enumerate(glob.glob(os.path.join(path, '*.ini'))):
        config = ConfigObj(filename)
        main_index = [index for index, x in enumerate(config['Keyframe_Posture_Type']) if x == 'main']
        for index in main_index:
            posture_dataset.append(list(map(int, config['Keyframe_Value']['Keyframe_' + str(index)])))

    data_set = convert_motorValue_to_cartesianSpace(posture_dataset)
    return data_set

def extract_arm_data(posture_dataSet,armSide):

    if armSide == 'right':
        index_range = [7, 14]
    elif armSide == 'left':
        index_range = [0, 7]
    else:
        index_range = [14, 17]
    data_set = np.asarray(posture_dataSet)

    new_data_set = data_set[:,index_range[0]:index_range[1]]

    return new_data_set

def collect_cartesian_position_data(kinematics_dataSet, position_index):

    cartesian_dataSet = []
    #print("len=",len(kinematics_data_set))
    for kinematics_data in kinematics_dataSet:
         cartesian_dataSet.append([round(kinematics_data[position_index][0][3],0), round(kinematics_data[position_index][1][3],0), round(kinematics_data[position_index][2][3],0)])
    #print("len=", len(cartesian_dataSet))
    #print(cartesian_dataSet)

    return cartesian_dataSet



def upScale_quaternion_value(quaternion_set,scale):
    return quaternion_set*scale

def find_avg_joint_angle(all_posture_stat_list, weight_type):
    posture_amount = len(all_posture_stat_list)
    joint_amount = len(all_posture_stat_list[0][0])

    all_posture_mean = []

    for i in range(posture_amount):
        all_posture_mean.append(all_posture_stat_list[i][0])


    #print("all_posture_mean=",all_posture_mean)

    if weight_type == 'std':
        all_joint_std = []
        all_joint_std_inv = []
        all_joint_weight = []

        for joint_num in range(joint_amount):
            joint_std = []
            joint_std_inv = []

            #print("joint_num=",joint_num)
            for posture_num in range(posture_amount):
                joint_std.append(all_posture_stat_list[posture_num][1][joint_num])
                joint_std_inv.append(1/all_posture_stat_list[posture_num][1][joint_num])

            all_joint_std.append(joint_std)

            all_joint_std_inv.append(joint_std_inv)
            all_joint_weight.append([float(i)/sum(joint_std_inv) for i in joint_std_inv])

        print("all_joint_std=",all_joint_std)
        print("all_joint_std_inv=", all_joint_std_inv)
        print("all_joint_weight=", all_joint_weight)
        #
        # print("transpose_mean", np.transpose(all_posture_mean))

        all_posture_mean_T = np.transpose(all_posture_mean)

        joint_avg = []
        for joint_num in range(joint_amount):
            joint_avg.append(float(round(np.average(all_posture_mean_T[joint_num], weights=all_joint_weight[joint_num]),1)))

    elif weight_type == 'equl':
        #print("all_posture_mean=", all_posture_mean)
        joint_avg = np.round(np.mean(all_posture_mean, axis = 0),2)


    return joint_avg

def add_score(main_score_array, score_array, index_to_add_score_set, offset_index):
    accumulate_score_array = np.copy(main_score_array)
    # print("accumulate_score_array_shape",np.shape(accumulate_score_array))
    #
    # print("score_array_shape", np.shape(score_array))
    lower_bound = int((np.shape(score_array)[0] - 1)/2)
    upper_bound = int((np.shape(score_array)[0] + 1) / 2)

    # print("lower_bound",lower_bound)
    # print("upper_bound", upper_bound)


    for index in index_to_add_score_set:
        index_after_offset = index - offset_index
        accumulate_score_array[int((round(index_after_offset[0],0) - lower_bound)):int((round(index_after_offset[0],0) + upper_bound)),
                                int((round(index_after_offset[1],0) - lower_bound)):int((round(index_after_offset[1],0) + upper_bound)),
                                int((round(index_after_offset[2],0) - lower_bound)):int((round(index_after_offset[2],0) + upper_bound))] += score_array

    #print(accumulate_score_array)
    return accumulate_score_array

def prepare_data_for_fit_model(filename, score_array, index_offset):

    posture_score_array = np.copy(score_array)
    print("posture score array size",filename , getsizeof(posture_score_array))
    np.save(str(filename) + ".npy", posture_score_array)

    posture_score_array_shape = np.shape(posture_score_array)
    posture_index_offset = np.copy(index_offset)

    data = []
    ### convert to data ###
    for z in range(posture_score_array_shape[0]):
        for y in range(posture_score_array_shape[1]):
            for x in range(posture_score_array_shape[2]):
                if posture_score_array[z][y][x] != 0:
                    # if (z + posture_index_offset[0]) == 121 and (y + posture_index_offset[1]) == -178:
                    # print([(z + posture_index_offset[0]), (y + posture_index_offset[1]), (x + posture_index_offset[2]), posture_score_array[z][y][x]])
                    data.append(
                        [(z + posture_index_offset[0]), (y + posture_index_offset[1]), (x + posture_index_offset[2]),
                         posture_score_array[z][y][x]])
    print("data amount", len(data))
    data = np.asarray(data)
    X, y = data[:, :-1], data[:, -1]

    return X, y

def build_extraTreeRegressor(filename, print_status, X, y, test_size, n_estimators, max_depth):
    ### ExtraTree ###
    # Split data into training and testing datasets
    if test_size == 0:
        X_train = X
        y_train = y
    else:
        X_train, X_test, y_train, y_test = cross_validation.train_test_split(
            X, y, test_size=test_size, random_state=5)

    print("time start", time.ctime())

    # Extremely Random Forests regressor
    params = {'n_estimators': n_estimators, 'max_depth': max_depth, 'random_state': 0}
    regressor = ExtraTreesRegressor(**params)
    regressor.fit(X_train, y_train)

    # Compute the regressor performance on test data
    if test_size != 0:
        y_pred = regressor.predict(X_test)
        print("Mean absolute error:", round(mean_absolute_error(y_test, y_pred), 2))

    print("time stop",time.ctime())

    # Predict the output for the test datapoint
    if print_status == "print_train":
        for i, dataTrain in enumerate(X_train):
            print("Data:", dataTrain, "Trained score:", y_train[i], "Predicted score:", regressor.predict([dataTrain])[0])
    elif print_status == "print_test":
        for i, dataTest in enumerate(X_test):
            print("Data:", dataTest, "Trained score:", y_test[i], "Predicted score:", regressor.predict([dataTest])[0])

    print("time stop (Predict)", time.ctime())
    pickle.dump(regressor, open(filename, 'wb'))

def build_MLPRegressor(filename, print_status, X, y, test_size, hidden_layer_sizes, alpha, max_iter):
    ### ExtraTree ###
    # Split data into training and testing datasets
    if test_size == 0:
        X_train = X
        y_train = y
    else:
        X_train, X_test, y_train, y_test = cross_validation.train_test_split(
            X, y, test_size=test_size, random_state=5)

    print("time start", time.ctime())

    param = {'hidden_layer_sizes':hidden_layer_sizes,'alpha': alpha, 'max_iter': max_iter}
    regressor = MLPRegressor(**param)
    regressor.fit(X_train, y_train)

    # Compute the regressor performance on test data
    if test_size != 0:
        y_pred = regressor.predict(X_test)
        print("Mean absolute error:", round(mean_absolute_error(y_test, y_pred), 2))

    print("time stop", time.ctime())

    # Predict the output for the test datapoint
    if print_status == "print_train":
        for i, dataTrain in enumerate(X_train):
            print("Data:", dataTrain, "Trained score:",y_train[i], "Predicted score:", regressor.predict([dataTrain])[0])
    elif print_status == "print_test":
        for i, dataTest in enumerate(X_test):
            print("Data:", dataTest, "Trained score:", y_test[i], "Predicted score:", regressor.predict([dataTest])[0])

    print("time stop (Predict)", time.ctime())
    pickle.dump(regressor, open(filename, 'wb'))

def build_score_array_and_offset(posture_position_set, add_boundary):
    min_value = np.min(posture_position_set, axis=0)
    max_value = np.max(posture_position_set, axis=0)
    diff_value = max_value - min_value

    index_offset = [int(min_value[0] - (add_boundary / 2)), int(min_value[1] - (add_boundary / 2)),
                          int(min_value[2] - (add_boundary / 2))]

    print("min", min_value)
    print("max", max_value)
    print(diff_value)
    print("index_offset", index_offset)

    score_array = np.zeros(
        [int(diff_value[0] + add_boundary), int(diff_value[1] + add_boundary), int(diff_value[2] + add_boundary)])

    score_array_shape = np.shape(score_array)
    print("score_array_shape", score_array_shape)

    #print("posture_position_set", posture_position_set)
    score_array = np.copy(add_score(score_array, base_score_3dim, posture_position_set, index_offset))
    score_array_shape = np.shape(score_array)
    print("score_array_shape",score_array_shape)

    return score_array, index_offset


##################################################################################################
if __name__ == "__main__":

    base_score_3dim = create_3dim_normalize_score_array(3) ### @param(sphere_radius)
    #base_score_4dim = create_4dim_normalize_score_array(3)  ### @param(sphere_radius)
    #print(base_score_3dim)


    ### load data ###
    jointAngle_degree_bye_set = collect_data('bye') ### @param(postureName)
    jointAngle_degree_salute_set = collect_data('salute')  ### @param(postureName)
    jointAngle_degree_sinvite_set = collect_data('side_invite')  ### @param(postureName)
    jointAngle_degree_wai_set = collect_data('wai')  ### @param(postureName)

    ### extract right arm data ###
    right_side_bye_set = extract_arm_data( jointAngle_degree_bye_set,'right')
    right_side_salute_set = extract_arm_data(jointAngle_degree_salute_set, 'right')
    right_side_sinvite_set = extract_arm_data(jointAngle_degree_sinvite_set, 'right')
    right_side_wai_set = extract_arm_data(jointAngle_degree_wai_set, 'right')

    ### calculate each posture stat ###
    right_side_bye_stat = calculate_stat_all_joint(right_side_bye_set)
    right_side_salute_stat = calculate_stat_all_joint(right_side_salute_set)
    right_side_sinvite_stat = calculate_stat_all_joint(right_side_sinvite_set)
    right_side_wai_stat = calculate_stat_all_joint(right_side_wai_set)

    ### calculate average join angle from all posture ###
    all_posture_stat_list = [right_side_bye_stat, right_side_salute_stat, right_side_sinvite_stat, right_side_wai_stat]
    ### type :: 'std' = standard_deviation, 'equl' = all weight equal
    avg_joint_angle_std = find_avg_joint_angle(all_posture_stat_list, 'std')
    avg_joint_angle_equl = find_avg_joint_angle(all_posture_stat_list, 'equl')


    print("right_side_bye_stat",right_side_bye_stat)
    print("right_side_salute_stat",right_side_salute_stat)
    print("right_side_sinvite_stat",right_side_sinvite_stat)
    print("right_side_wai_stat",right_side_wai_stat)
    print("avg M:std", avg_joint_angle_std)
    print("avg M:equl", avg_joint_angle_equl)

    # ### calculate kinematics ###
    # bye_kinematics_set = collect_kinematics_data(right_side_bye_set)
    # salute_kinematics_set = collect_kinematics_data(right_side_salute_set)
    # sinvite_kinematics_set = collect_kinematics_data(right_side_sinvite_set)
    # wai_kinematics_set = collect_kinematics_data(right_side_wai_set)
    #
    # ##### bye posture #####
    # ### collect bye catesian position and quaternion ###
    # y_max_min = []
    # bye_elbow_position = np.asarray(collect_cartesian_position_data(bye_kinematics_set, 3))  ### 3 = elbow position
    # bye_wrist_position = np.asarray(collect_cartesian_position_data(bye_kinematics_set, 4))  ### 4 = wrist position
    # bye_quaternion = np.asarray(collect_quaternion_data(bye_kinematics_set))
    # bye_quaternion_upScale = np.copy(upScale_quaternion_value(bye_quaternion, 100))    ### ExtraTree ###
    # bye_elbow_score_array, bye_elbow_index_offset = build_score_array_and_offset(bye_elbow_position, 20)
    # X,y = prepare_data_for_fit_model("bye_elbow", bye_elbow_score_array, bye_elbow_index_offset)
    # build_extraTreeRegressor("bye_elbow_ex", "print_none", X, y, 0, 1, None)
    # y_max_min.append([round(np.amax(y),3),round(np.amin(y),3)])
    #
    # bye_wrist_score_array, bye_wrist_index_offset = build_score_array_and_offset(bye_wrist_position, 20)
    # X, y = prepare_data_for_fit_model("bye_wrist", bye_wrist_score_array, bye_wrist_index_offset)
    # build_extraTreeRegressor("bye_wrist_ex", "print_none", X, y, 0, 1, None)
    # y_max_min.append([round(np.amax(y),3),round(np.amin(y),3)])
    #
    # bye_qua_score_array, bye_qua_index_offset = build_score_array_and_offset(bye_quaternion_upScale[:,0:3], 20)
    # X, y = prepare_data_for_fit_model("bye_Q", bye_qua_score_array, bye_qua_index_offset)
    # build_extraTreeRegressor("bye_Q_ex", "print_none", X, y, 0, 1, None)
    # y_max_min.append([round(np.amax(y),3),round(np.amin(y),3)])
    # np.savetxt("bye_ex_max_min", y_max_min)
    #
    # ##### salute posture #####
    # ### collect salute catesian position and quaternion ###
    # y_max_min = []
    # salute_elbow_position = np.asarray(collect_cartesian_position_data(salute_kinematics_set, 3))  ### 3 = elbow position
    # salute_wrist_position = np.asarray(collect_cartesian_position_data(salute_kinematics_set, 4))  ### 4 = wrist position
    # salute_quaternion = np.asarray(collect_quaternion_data(salute_kinematics_set))
    # salute_quaternion_upScale = np.copy(upScale_quaternion_value(salute_quaternion, 100))    ### ExtraTree ###
    # salute_elbow_score_array, salute_elbow_index_offset = build_score_array_and_offset(salute_elbow_position, 20)
    # X,y = prepare_data_for_fit_model("salute_elbow", salute_elbow_score_array, salute_elbow_index_offset)
    # build_extraTreeRegressor("salute_elbow_ex", "print_none", X, y, 0, 1, None)
    # y_max_min.append([round(np.amax(y),3),round(np.amin(y),3)])
    #
    # salute_wrist_score_array, salute_wrist_index_offset = build_score_array_and_offset(salute_wrist_position, 20)
    # X, y = prepare_data_for_fit_model("salute_wrist", salute_wrist_score_array, salute_wrist_index_offset)
    # build_extraTreeRegressor("salute_wrist_ex", "print_none", X, y, 0, 1, None)
    # y_max_min.append([round(np.amax(y),3),round(np.amin(y),3)])
    #
    # salute_qua_score_array, salute_qua_index_offset = build_score_array_and_offset(salute_quaternion_upScale[:,0:3], 20)
    # X, y = prepare_data_for_fit_model("salute_Q", salute_qua_score_array, salute_qua_index_offset)
    # build_extraTreeRegressor("salute_Q_ex", "print_none", X, y, 0, 1, None)
    # y_max_min.append([round(np.amax(y),3),round(np.amin(y),3)])
    # np.savetxt("salute_ex_max_min", y_max_min)
    #
    # ##### sinvite posture #####
    # ### collect sinvite catesian position and quaternion ###
    # y_max_min = []
    # sinvite_elbow_position = np.asarray(collect_cartesian_position_data(sinvite_kinematics_set, 3))  ### 3 = elbow position
    # sinvite_wrist_position = np.asarray(collect_cartesian_position_data(sinvite_kinematics_set, 4))  ### 4 = wrist position
    # sinvite_quaternion = np.asarray(collect_quaternion_data(sinvite_kinematics_set))
    # sinvite_quaternion_upScale = np.copy(upScale_quaternion_value(sinvite_quaternion, 100))    ### ExtraTree ###
    # sinvite_elbow_score_array, sinvite_elbow_index_offset = build_score_array_and_offset(sinvite_elbow_position, 20)
    # X,y = prepare_data_for_fit_model("sinvite_elbow", sinvite_elbow_score_array, sinvite_elbow_index_offset)
    # build_extraTreeRegressor("sinvite_elbow_ex", "print_none", X, y, 0, 1, None)
    # y_max_min.append([round(np.amax(y),3),round(np.amin(y),3)])
    #
    # sinvite_wrist_score_array, sinvite_wrist_index_offset = build_score_array_and_offset(sinvite_wrist_position, 20)
    # X, y = prepare_data_for_fit_model("sinvite_wrist", sinvite_wrist_score_array, sinvite_wrist_index_offset)
    # build_extraTreeRegressor("sinvite_wrist_ex", "print_none", X, y, 0, 1, None)
    # y_max_min.append([round(np.amax(y),3),round(np.amin(y),3)])
    #
    # sinvite_qua_score_array, sinvite_qua_index_offset = build_score_array_and_offset(sinvite_quaternion_upScale[:,0:3], 20)
    # X, y = prepare_data_for_fit_model("sinvite_Q", sinvite_qua_score_array, sinvite_qua_index_offset)
    # build_extraTreeRegressor("sinvite_Q_ex", "print_none", X, y, 0, 1, None)
    # y_max_min.append([round(np.amax(y),3),round(np.amin(y),3)])
    # np.savetxt("sinvite_ex_max_min", y_max_min)
    #
    # ##### wai posture #####
    # ### collect wai catesian position and quaternion ###
    # y_max_min = []
    # wai_elbow_position = np.asarray(collect_cartesian_position_data(wai_kinematics_set, 3))  ### 3 = elbow position
    # wai_wrist_position = np.asarray(collect_cartesian_position_data(wai_kinematics_set, 4))  ### 4 = wrist position
    # wai_quaternion = np.asarray(collect_quaternion_data(wai_kinematics_set))
    # wai_quaternion_upScale = np.copy(upScale_quaternion_value(wai_quaternion, 100))    ### ExtraTree ###
    # wai_elbow_score_array, wai_elbow_index_offset = build_score_array_and_offset(wai_elbow_position, 20)
    # X,y = prepare_data_for_fit_model("wai_elbow", wai_elbow_score_array, wai_elbow_index_offset)
    # build_extraTreeRegressor("wai_elbow_ex", "print_none", X, y, 0, 1, None)
    # y_max_min.append([round(np.amax(y),3),round(np.amin(y),3)])
    #
    # wai_wrist_score_array, wai_wrist_index_offset = build_score_array_and_offset(wai_wrist_position, 20)
    # X, y = prepare_data_for_fit_model("wai_wrist", wai_wrist_score_array, wai_wrist_index_offset)
    # build_extraTreeRegressor("wai_wrist_ex", "print_none", X, y, 0, 1, None)
    # y_max_min.append([round(np.amax(y), 3), round(np.amin(y), 3)])
    #
    # wai_qua_score_array, wai_qua_index_offset = build_score_array_and_offset(wai_quaternion_upScale[:,0:3], 20)
    # X, y = prepare_data_for_fit_model("wai_Q", wai_qua_score_array, wai_qua_index_offset)
    # build_extraTreeRegressor("wai_Q_ex", "print_none", X, y, 0, 1, None)
    # y_max_min.append([round(np.amax(y), 3), round(np.amin(y), 3)])
    # np.savetxt("wai_ex_max_min", y_max_min)
    
    # ### MLP ###
    # bye_elbow_score_array, bye_elbow_index_offset = build_score_array_and_offset(bye_elbow_position, 20)
    # X, y = prepare_data_for_fit_model("bye_elbow_MPL", bye_elbow_score_array, bye_elbow_index_offset)
    # build_MLPRegressor("bye_elbow_MLP", "print_test", X, y, 0.01, 300,0.1, 5000)

    # fig = plt.figure()
    # ax = fig.add_subplot(111, projection='3d')

    # a = [[250, 3309, 2303, 1801, 2379, 2320, 1725, 3467, 774, 2681, 1136, 1696, 2342, 1479, 598, 533, 372],
    #      [250, 3309, 2303, 1801, 2379, 2320, 1725, 3467, 774, 2681, 1136, 1696, 2342, 1479, 598, 533, 372]]
    #
    # b = convert_motorValue_to_cartesianSpace(a)
    # print(b)
    #
    # c = [20.0, 0.0, 19.0, 101.0, -58.0, -34.0, 36.0]

    # #3467, 774, 2681, 1136, 1696, 2342, 1479
    a1 = [46, 0, -11, 101.0, 33, 16, 44]
    a2 = [135, -71, -2, 101.0, 90, 39, -9]
    a3 = [0, -90, -45, 101.0, 37, 45, 45]
    a4 = [25, 0, -45, 101.0, -52, -16, 45]

    convert_cartesianSpace_to_motorValue_rightSide(a1)
    convert_cartesianSpace_to_motorValue_rightSide(a2)
    convert_cartesianSpace_to_motorValue_rightSide(a3)
    convert_cartesianSpace_to_motorValue_rightSide(a4)
    # #[69, -3, -38, 101, -31, -29, -10]
    #
    #
    #
    #
###