from math import pow, sqrt, sin, cos, radians
import numpy as np


def collect_kinematics_data(joint7dof_dataSet):
    kinematics_dataSet = []

    for i, jointSet in enumerate(joint7dof_dataSet):
        kinematics_dataSet.append(cal_kinematics_namo_numpy(jointSet,'right'))

    return kinematics_dataSet

def cal_kinematics_namo_numpy(degree7Joint,armSide):
    """ calculate Namo robot kinematic 7DOF Arm
    :param degree7Joint: input [degree0,1,2,3,4,5,6]
    :param armSide: input arm side 'L' for Left side, 'R' for Right side
    :return: Transformation Matrix List [T01,T02,T03,T04,T05,T06,T07,T0E]
    """
    ## DH parameter >> alpha[0,1,...6,end effector]
    alpha = [radians(90),radians(90),radians(-90),radians(-90),radians(90),radians(90),
             radians(-90),radians(0)]
    ## DH parameter >> a[0,1,...6,end effector]
    a = [0,0,0,0,0,0,0,-140]
    ## DH parameter >> d[1,2,...7,end effector]
    d = [182,0,206.5,0,206,0,0,0]
    if armSide == 'left':
        d[0] = d[0]*(-1)
    elif armSide == 'right':
        d[0] = d[0]*1
    #print("7dof ="+str(degree7Joint))
    ## DH parameter >> theta[1,2,...7,end effector]
    theta = [radians(degree7Joint[0] + 90),radians(degree7Joint[1] + 90),radians(degree7Joint[2] - 90),
             radians(degree7Joint[3]),radians(degree7Joint[4] + 90),radians(degree7Joint[5] - 90),
             radians(degree7Joint[6]),radians(0)]
    T = {}
    for i in range(0,8):
        #print i

        T[i] = np.array([[(cos(theta[i])), (-sin(theta[i])), 0, a[i]],
                       [(sin(theta[i]) * (cos(alpha[i]))), (cos(theta[i])) * (cos(alpha[i])), (-sin(alpha[i])),
                        (-sin(alpha[i])) * d[i]],
                       [(sin(theta[i]) * (sin(alpha[i]))), (cos(theta[i])) * (sin(alpha[i])), (cos(alpha[i])),
                        (cos(alpha[i])) * d[i]],
                       [0, 0, 0, 1]])


    T01 = T[0]
    T02 = np.dot(T01,T[1])
    T03 = np.dot(T02,T[2])
    T04 = np.dot(T03,T[3])
    T05 = np.dot(T04,T[4])
    T06 = np.dot(T05,T[5])
    T07 = np.dot(T06,T[6])
    T0E = np.dot(T07,T[7])
    return [T01,T02,T03,T04,T05,T06,T07,T0E]

def collect_quaternion_data(kinematics_dataSet):
    quaternion_dataSet = []

    for i, kinetics in enumerate(kinematics_dataSet):
        quaternion_dataSet.append(cal_quaternion(kinetics[7]))

    return quaternion_dataSet

def cal_quaternion(Tmatrix):
    #print Tmatrix
    tr = Tmatrix[0,0] + Tmatrix[1,1] + Tmatrix[2,2]
    if(tr>0):
        S = sqrt(tr+1.0)*2 ## S=4*qw
        qw = 0.25*S
        qx = (Tmatrix[2,1]-Tmatrix[1,2])/S
        qy = (Tmatrix[0,2]-Tmatrix[2,0])/S
        qz = (Tmatrix[1,0]-Tmatrix[0,1])/S
        #print "case1"
    elif((Tmatrix[0,0]>Tmatrix[1,1]) and (Tmatrix[0,0]>Tmatrix[2,2])):
        S = sqrt(1.0 + Tmatrix[0,0] - Tmatrix[1,1] - Tmatrix[2,2])*2 ## S=4*qx
        qw = (Tmatrix[2,1]-Tmatrix[1,2])/S
        qx = 0.25*S
        qy = (Tmatrix[0,1]+Tmatrix[1,0])/S
        qz = (Tmatrix[0,2]+Tmatrix[2,0])/S
        #print "case2"
    elif(Tmatrix[1,1]>Tmatrix[2,2]):
        S = sqrt(1.0 + Tmatrix[1,1] - Tmatrix[0,0] - Tmatrix[2,2])*2 ## S=4*qy
        qw = (Tmatrix[0,2]-Tmatrix[2,0])/S
        qx = (Tmatrix[0,1]+Tmatrix[1,0])/S
        qy = 0.25*S
        qz = (Tmatrix[1,2]+Tmatrix[2,1])/S
        #print "case3"
    else:
        S = sqrt(1.0 + Tmatrix[2,2] - Tmatrix[0,0] - Tmatrix[1,1])*2 ## S=4*qz
        qw = (Tmatrix[1,0]-Tmatrix[0,1])/S
        qx = (Tmatrix[0,2]+Tmatrix[2,0])/S
        qy = (Tmatrix[1,2]+Tmatrix[2,1])/S
        qz = 0.25*S
        #print "case4"

    qw = round(qw,2)
    qx = round(qx, 2)
    qy = round(qy, 2)
    qz = round(qz, 2)

    norm = round(sqrt((qw*qw) + (qx*qx) + (qy*qy) + (qz*qz)),1)
    return [qw,qx,qy,qz]

def cal_Single_Posture_Score(T_matrix_ref, Q_ref, T_matrix, Q, score_weight):
    score = []

    Q_diff = [Q_ref[0] - Q[0], Q_ref[1] - Q[1], Q_ref[2] - Q[2], Q_ref[3] - Q[3]]

    norm_Q_diff = sqrt(
        (Q_diff[0] * Q_diff[0]) + (Q_diff[1] * Q_diff[1]) + (Q_diff[2] * Q_diff[2]) + (Q_diff[3] * Q_diff[3]))
    # elbow_diff =[TMatrix_Target_Posture[3][0,3] - T_Matrix[3][0,3], TMatrix_Target_Posture[3][1,3] - T_Matrix[3][1,3], TMatrix_Target_Posture[3][2,3] - T_Matrix[3][2,3]]
    norm_elbow_diff = sqrt(pow(T_matrix_ref[3][0, 3] - T_matrix[3][0, 3], 2) + pow(
        T_matrix_ref[3][1, 3] - T_matrix[3][1, 3], 2) + pow(
        T_matrix_ref[3][2, 3] - T_matrix[3][2, 3], 2))
    # wrist_diff =[TMatrix_Target_Posture[4][0,3] - T_Matrix[4][0,3], TMatrix_Target_Posture[4][1,3] - T_Matrix[4][1,3], TMatrix_Target_Posture[4][2,3] - T_Matrix[4][2,3]]
    norm_wrist_diff = sqrt(pow(T_matrix_ref[4][0, 3] - T_matrix[4][0, 3], 2) + pow(
        T_matrix_ref[4][1, 3] - T_matrix[4][1, 3], 2) + pow(
        T_matrix_ref[4][2, 3] - T_matrix[4][2, 3], 2))
    sum_diff = norm_Q_diff * score_weight[0] + norm_elbow_diff * score_weight[1] + norm_wrist_diff * score_weight[2]

    #score.append(Q_diff)
    score.append(norm_Q_diff)
    score.append(norm_elbow_diff)
    score.append(norm_wrist_diff)
    score.append(sum_diff)

    return score

def extract_arm_data(posture_dataSet, armSide):
    if armSide == 'right':
        index_range = [7, 14]
    elif armSide == 'left':
        index_range = [0, 7]
    else:
        index_range = [14, 17]
    data_set = np.asarray(posture_dataSet)
    print(data_set.shape)

    new_data_set = data_set[:, index_range[0]:index_range[1]]

    return new_data_set

def calculate_stat_all_joint(posture_dataSet):
    mean = np.mean(posture_dataSet, axis=0)
    std = np.std(posture_dataSet, axis=0)
    var = np.var(posture_dataSet, axis=0)

    return [mean, std, var]


def find_avg_joint_angle(all_posture_stat_list, weight_type):
    posture_amount = len(all_posture_stat_list)
    joint_amount = len(all_posture_stat_list[0][0])

    all_posture_mean = []

    for i in range(posture_amount):
        all_posture_mean.append(all_posture_stat_list[i][0])

    # print("all_posture_mean=",all_posture_mean)

    if weight_type == 'std':
        all_joint_std = []
        all_joint_std_inv = []
        all_joint_weight = []

        for joint_num in range(joint_amount):
            joint_std = []
            joint_std_inv = []

            # print("joint_num=",joint_num)
            for posture_num in range(posture_amount):
                joint_std.append(all_posture_stat_list[posture_num][1][joint_num])
                joint_std_inv.append(1 / all_posture_stat_list[posture_num][1][joint_num])

            all_joint_std.append(joint_std)

            all_joint_std_inv.append(joint_std_inv)
            all_joint_weight.append([float(i) / sum(joint_std_inv) for i in joint_std_inv])

        # print("all_joint_std=",all_joint_std)
        # print("all_joint_std_inv=", all_joint_std_inv)
        # print("all_joint_weight=", all_joint_weight)
        #
        # print("transpose_mean", np.transpose(all_posture_mean))

        all_posture_mean_T = np.transpose(all_posture_mean)

        joint_avg = []
        for joint_num in range(joint_amount):
            joint_avg.append(
                float(round(np.average(all_posture_mean_T[joint_num], weights=all_joint_weight[joint_num]), 1)))

    elif weight_type == 'equl':
        # print("all_posture_mean=", all_posture_mean)
        joint_avg = np.round(np.mean(all_posture_mean, axis=0), 2)

    return joint_avg


def collect_cartesian_position_data(kinematics_dataSet, position_index):
    cartesian_dataSet = []
    # print("len=",len(kinematics_data_set))
    for kinematics_data in kinematics_dataSet:
        cartesian_dataSet.append(
            [round(kinematics_data[position_index][0][3], 0), round(kinematics_data[position_index][1][3], 0),
             round(kinematics_data[position_index][2][3], 0)])
    # print("len=", len(cartesian_dataSet))
    # print(cartesian_dataSet)

    return cartesian_dataSet


def upScale_quaternion_value(quaternion_set,scale):
    return quaternion_set*scale

def build_score_array_and_offset_4D(posture_position_set, add_boundary, base_score_4dim):
    min_value = np.min(posture_position_set, axis=0)
    max_value = np.max(posture_position_set, axis=0)
    diff_value = max_value - min_value

    index_offset = [int(min_value[0] - (add_boundary / 2)), int(min_value[1] - (add_boundary / 2)),
                          int(min_value[2] - (add_boundary / 2)), int(min_value[3] - (add_boundary / 2))]

    print("min", min_value)
    print("max", max_value)
    print(diff_value)
    print("index_offset", index_offset)

    score_array = np.zeros(
        [int(diff_value[0] + add_boundary), int(diff_value[1] + add_boundary), int(diff_value[2] + add_boundary), int(diff_value[3] + add_boundary)])

    score_array_shape = np.shape(score_array)
    print("score_array_shape", score_array_shape)

    #print("posture_position_set", posture_position_set)
    score_array = np.copy(add_score(score_array, base_score_4dim, posture_position_set, index_offset))
    score_array_shape = np.shape(score_array)
    print("score_array_shape",score_array_shape)

    return score_array, index_offset


def build_score_array_and_offset_3D(posture_position_set, add_boundary, base_score_3dim):
    min_value = np.min(posture_position_set, axis=0)
    max_value = np.max(posture_position_set, axis=0)
    diff_value = max_value - min_value;''

    index_offset = [int(min_value[0] - (add_boundary / 2)), int(min_value[1] - (add_boundary / 2)),
                          int(min_value[2] - (add_boundary / 2))]

    

    score_array = np.zeros(
        [int(diff_value[0] + add_boundary), int(diff_value[1] + add_boundary), int(diff_value[2] + add_boundary)])

    score_array_shape = np.shape(score_array)
    print("score_array_shape", score_array_shape)

    #print("posture_position_set", posture_position_set)
    score_array = np.copy(add_score(score_array, base_score_3dim, posture_position_set, index_offset))
    score_array_shape = np.shape(score_array)
    #print("score_array_shape",score_array_shape)

    return score_array, index_offset

def build_score_array_and_offset_4D(posture_position_set, add_boundary, base_score_4dim):
    min_value = np.min(posture_position_set, axis=0)
    max_value = np.max(posture_position_set, axis=0)
    diff_value = max_value - min_value;''

    index_offset = [int(min_value[0] - (add_boundary / 2)), int(min_value[1] - (add_boundary / 2)),
                          int(min_value[2] - (add_boundary / 2)), int(min_value[3] - (add_boundary / 3))]

    print("index_offset",index_offset)

    score_array = np.zeros(
        [int(diff_value[0] + add_boundary), int(diff_value[1] + add_boundary), int(diff_value[2] + add_boundary),int(diff_value[3] + add_boundary)])

    score_array_shape = np.shape(score_array)
    print("score_array_shape", score_array_shape)

    
    score_array = np.copy(add_score4d(score_array, base_score_4dim, posture_position_set, index_offset))
    score_array_shape = np.shape(score_array)
    # #print("score_array_shape",score_array_shape)

    return score_array, index_offset

def add_score(main_score_array, score_array, index_to_add_score_set, offset_index):
    accumulate_score_array = np.copy(main_score_array)
    #print("accumulate_score_array_shape",np.shape(accumulate_score_array))
    #
    #print("score_array_shape", np.shape(score_array))
    lower_bound = int((np.shape(score_array)[0] - 1) / 2)
    upper_bound = int((np.shape(score_array)[0] + 1) / 2)



    for index in index_to_add_score_set:
        index_after_offset = index - offset_index
        
        try:
            accumulate_score_array[
            int((round(index_after_offset[0], 0) - lower_bound)):int((round(index_after_offset[0], 0) + upper_bound)),
            int((round(index_after_offset[1], 0) - lower_bound)):int((round(index_after_offset[1], 0) + upper_bound)),
            int((round(index_after_offset[2], 0) - lower_bound)):int((round(index_after_offset[2], 0) + upper_bound))] += score_array
        except:
            print("index 0 upper - lower",int((round(index_after_offset[0], 0) - lower_bound)),int((round(index_after_offset[0], 0) + upper_bound)))
            print("diff index 0",int((round(index_after_offset[0], 0) - lower_bound)) - int((round(index_after_offset[0], 0) + upper_bound)))
            print("index 1 upper - lower",int((round(index_after_offset[1], 0) - lower_bound)),int((round(index_after_offset[1], 0) + upper_bound)))
            print("diff index 1",int((round(index_after_offset[1], 0) - lower_bound)) - int((round(index_after_offset[1], 0) + upper_bound)))
            print("index 2 upper - lower",int((round(index_after_offset[2], 0) - lower_bound)),int((round(index_after_offset[2], 0) + upper_bound)))
            print("diff index 2",int((round(index_after_offset[2], 0) - lower_bound)) - int((round(index_after_offset[2], 0) + upper_bound)))
            print('index',index)
            print("index after",index_after_offset)

            a = accumulate_score_array[157:199,165:206,123:164]

            print(a[40][0])
            print(a.shape)
            print(type(accumulate_score_array))

            b = np.array([1,2,3,4,5,6,7,8,9])
            print(b[2:5].shape)
            exit()
            #print(accumulate_score_array)

    # print(accumulate_score_array)
    return accumulate_score_array

def add_score4d(main_score_array, score_array, index_to_add_score_set, offset_index):
    accumulate_score_array = np.copy(main_score_array)
    #print("accumulate_score_array_shape",np.shape(accumulate_score_array))
    #
    #print("score_array_shape", np.shape(score_array))
    lower_bound = int((np.shape(score_array)[0] - 1) / 2)
    upper_bound = int((np.shape(score_array)[0] + 1) / 2)



    for index in index_to_add_score_set:
        #print("index",index)
        index_after_offset = index - offset_index
        print("index_after_offset",index_after_offset)
        
        try:
            accumulate_score_array[
            int((round(index_after_offset[0], 0) - lower_bound)):int((round(index_after_offset[0], 0) + upper_bound)),
            int((round(index_after_offset[1], 0) - lower_bound)):int((round(index_after_offset[1], 0) + upper_bound)),
            int((round(index_after_offset[2], 0) - lower_bound)):int((round(index_after_offset[2], 0) + upper_bound)),
            int((round(index_after_offset[3], 0) - lower_bound)):int((round(index_after_offset[3], 0) + upper_bound))] += score_array
        except:
            print("index 0 upper - lower",int((round(index_after_offset[0], 0) - lower_bound)),int((round(index_after_offset[0], 0) + upper_bound)))
            print("diff index 0",int((round(index_after_offset[0], 0) - lower_bound)) - int((round(index_after_offset[0], 0) + upper_bound)))
            print("index 1 upper - lower",int((round(index_after_offset[1], 0) - lower_bound)),int((round(index_after_offset[1], 0) + upper_bound)))
            print("diff index 1",int((round(index_after_offset[1], 0) - lower_bound)) - int((round(index_after_offset[1], 0) + upper_bound)))
            print("index 2 upper - lower",int((round(index_after_offset[2], 0) - lower_bound)),int((round(index_after_offset[2], 0) + upper_bound)))
            print("diff index 2",int((round(index_after_offset[2], 0) - lower_bound)) - int((round(index_after_offset[2], 0) + upper_bound)))
            print('index',index)
            print("index after",index_after_offset)

            a = accumulate_score_array[157:199,165:206,123:164]

            print(a[40][0])
            print(a.shape)
            print(type(accumulate_score_array))

            b = np.array([1,2,3,4,5,6,7,8,9])
            print(b[2:5].shape)
            exit()
            #print(accumulate_score_array)
        #print(accumulate_score_array[index_after_offset[0],index_after_offset[1],index_after_offset[2],index_after_offset[3]])

    # print(accumulate_score_array)
    return accumulate_score_array