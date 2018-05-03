from configobj import ConfigObj
import glob
import os
import numpy as np
from math import pow, sqrt
from scipy.stats import norm

def create_sphere_normaldis_score_array(sphere_radius):

    normal_dis_array_size = sphere_radius + sphere_radius + 1
    normal_dis_sigma_width = 3

    sphere_array = np.zeros((normal_dis_array_size,normal_dis_array_size,normal_dis_array_size))
    for z in range(0,normal_dis_array_size):
        for y in range(0,normal_dis_array_size):
            for x in range(0, normal_dis_array_size):
                distance = sqrt(pow(x-sphere_radius,2) + pow(y-sphere_radius,2) + pow(z-sphere_radius,2))
                distance = distance*normal_dis_sigma_width/sphere_radius
                sphere_array[x, y, z] = round(norm.pdf(distance), 3)

    print(sphere_array)
    return sphere_array



def collect_data(path):

    for filename in glob.glob(os.path.join(path, '*.ini')):
        print(filename)


        config = ConfigObj(filename)
        print(config['Keyframe_Time'])
        # self.int_numberOfKeyframe = int(config['Keyframe_Amount'])
        # self.ui.numOfKeyframeStatus_label.setText(str(self.int_numberOfKeyframe))
        #
        # for i in range(int(self.int_numberOfKeyframe)):
        #     self.bool_activeKeyframe[i] = True
        #     self.int_motorValue[i] = list(map(int, config['Keyframe_Value']['Keyframe_' + str(i)]))
        #
        #     self.int_time[i] = int(config['Keyframe_Time'][i])
        #
        # for i in range(int(self.int_numberOfKeyframe), 30):
        #     self.bool_activeKeyframe[i] = False




create_sphere_normaldis_score_array(3)

collect_data('./Postures/posture_set_bye')

