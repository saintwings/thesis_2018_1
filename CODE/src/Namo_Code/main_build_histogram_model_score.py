from configobj import ConfigObj
import sys
from utility_function_2017 import *


def collect_data(postureName, postureType):

    posture_dataset = []
    fileName_load = os.path.join(scriptpath,'Postures\posture_set_' + str(postureName))
    for i, filename in enumerate(glob.glob(os.path.join(fileName_load, '*.ini'))):
    #path = './Postures/posture_set_' + str(postureName)
    #for i, filename in enumerate(glob.glob(os.path.join(path, '*.ini'))):
        config = ConfigObj(filename)
        try:
            main_index = [index for index, x in enumerate(config['Keyframe_Posture_Type']) if x == postureType]
        except:
            print("no posture type")
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
