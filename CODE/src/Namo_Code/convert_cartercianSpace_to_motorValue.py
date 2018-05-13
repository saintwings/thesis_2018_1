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

def convert_cartersianSpace_to_motorValue(single_data):
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
        #print("motor centor_ x",x," = ",int_motorCenterValue[x])

    #if armSide == 'right':
    index_range = [7, 14]
    motor_center = int_motorCenterValue[7: 14]
    motor_direction = int_motorDirection_and_ratio[7: 14]
    #print("motor degree = ",single_data)
    #print("motor center =", motor_center)
    #print("motor direction =",motor_direction)

    motor_value = []
    for i in range(len(single_data)):
        motor_value.append(  round(((single_data[i]*4095/359)/motor_direction[i]) + motor_center[i],0)   )
    
    print("data = ", single_data)
    print("motor value = ",motor_value)


if __name__ == "__main__":
    #data = [54,-40,39,101,18,-6,-43]
    #convert_cartersianSpace_to_motorValue(data)
    data = [70,-19,10,101,-46,-3,-14]
    convert_cartersianSpace_to_motorValue(data)
    # data = [114,-37,-38,107,-67,1,24]
    # convert_cartersianSpace_to_motorValue(data)
    # data = [55,-40,45,43,-82,4,24]
    # convert_cartersianSpace_to_motorValue(data)
    # data = [56,-20,-33,115,6,-9,24]
    # convert_cartersianSpace_to_motorValue(data)

    # data = [36,-29,0,74,40,-32,24]
    # convert_cartersianSpace_to_motorValue(data)
    # data = [88,-14,-45,89,20,-15,24]
    # convert_cartersianSpace_to_motorValue(data)

    