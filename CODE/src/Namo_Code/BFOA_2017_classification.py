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


class Bacteria:
    def __init__(self, problem_size, search_space, joint_fixed, joint_fixed_value, seed = 0,verbal = None):
        self.rnd = random.Random(seed)
        self.joint_fixed = joint_fixed
        self.joint_fixed_value = joint_fixed_value
        self.vector = None
        self.cost = None
        self.inter = None
        self.fitness = None
        self.sum_nutrients = None
        self.transformation_matrix = None
        self.quaternion = None
        self.T_matrix_cell = None
        self.Q_cell = None

        if verbal == 'r':
            ## random only ##
            self.vector = [random.randint(search_space[i][0], search_space[i][1]) for i in range(problem_size)]
            self.cal_T_matrix_cell('c')
            self.cal_Q_cell('c')

        elif verbal == 'rf':
            ## random and fix ##
            self.vector = [random.randint(search_space[i][0], search_space[i][1]) for i in range(problem_size)]
            self.fixed_vector_some_value()

            self.cal_T_matrix_cell('c')
            self.cal_Q_cell('c')

    def print_vector(self):
        print(self.vector)


    def fixed_vector_some_value(self):
        self.vector[self.joint_fixed] = self.joint_fixed_value

    def cal_T_matrix_cell(self, verbal = 'c'):
        if verbal == 'c':
            self.T_matrix_cell = cal_kinematics_namo_numpy(self.vector, 'R')

        elif verbal == 'cp':
            self.T_matrix_cell = cal_kinematics_namo_numpy(self.vector, 'R')
            print("T_matric_cell = " + str(self.T_matrix_cell))

        elif verbal == 'p':
            print("T_matric_cell = " + str(self.T_matrix_cell))


    def cal_Q_cell(self, verbal = 'c'):
        if verbal == 'c':
            self.Q_cell = cal_quaternion(self.T_matrix_cell[7])

        elif verbal == 'cp':
            self.Q_cell = cal_quaternion(self.T_matrix_cell[7])
            print("Q_cell = " + str(self.Q_cell))

        elif verbal == 'p':
            print("Q_cell = " + str(self.Q_cell))



def Search_New_Postures_by_BFOA(ref_regressor, all_y_max, joint_fixed, joint_fixed_value,weight,
            problem_size, search_space, pop_size_S,
            elim_disp_steps_Ned, repro_steps_Nre, chem_steps_Nc, swim_length_Ns, step_size_Ci, p_eliminate_Ped,
            d_attr, w_attr, h_rep, w_rep):

    rnd = random.Random(0)
    best_set = []

    ## create n random particles ##
    random_cells = [Bacteria(problem_size, search_space, joint_fixed, joint_fixed_value, rnd, 'rf') for i in range(pop_size_S)]
    # for cell in random_cells:
    #     cell.print_vector()

    ## for each posture ##
    for posture_count in range(4):


        cells = deepcopy(random_cells)
        best = Bacteria(problem_size, search_space, joint_fixed, joint_fixed_value, rnd, 'None')

        ref_reg = ref_regressor
        ref_y_max = posture_count


        for l in range(0, elim_disp_steps_Ned):
            for k in range(0, repro_steps_Nre):
                print("posture count =" + str(posture_count) )
                print("l = " + str(l) + " k = " + str(k)+">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
                c_best, cells = ChemotaxisAndSwim(ref_reg, ref_y_max, weight, search_space,
                                                  cells, chem_steps_Nc,swim_length_Ns, step_size_Ci, d_attr, w_attr, h_rep, w_rep)

                if (best.cost == None) or (c_best.cost < best.cost):
                    best = c_best

                print("best fitness = " + str(best.fitness) + ", cost = " + str(best.cost))
                cells = sorted(cells, key=lambda Bacteria: Bacteria.sum_nutrients)
                ## reproduction step ##

                print("BBBBBBB",cells)
                print(len(cells)/2)
                cells = cells[:int(len(cells)/2)]
                cells = cells + cells

            ## eliminate step ##
            print("eliminate step")
            for i in range(len(cells)):
                if (random.random() <= p_eliminate_Ped):
                    cells.pop(i)
                    new_cell = Bacteria(problem_size, search_space, joint_fixed, joint_fixed_value, rnd, 'rf')
                    cells.insert(i,new_cell)
                    #print("generated new cell =" + str(new_cell.vector))

        best_set.append(best)

    return best_set

def evaluate(ref_reg, ref_y_max, weight, cell, cells, d_attr, w_attr, h_rep, w_rep):
    #t1 = time.clock()
    cell.cost = objective_function(ref_reg, ref_y_max, cell.T_matrix_cell,cell.Q_cell, weight)
    cell.inter = attract_repel(cell, cells, d_attr, w_attr, h_rep, w_rep)
    cell.fitness = cell.cost + cell.inter
    #t2 = time.clock()
    #t_diff = t2 - t1
    #print("eval t= " + str(t_diff))
    #print("cost = " + str(cell.cost) + " inter" + str(cell.inter) + " fitness = " + str(cell.fitness))

def attract_repel(cell, cells, d_attr, w_attr, h_rep, w_rep):
    attract = compute_cell_interaction(cell, cells, -d_attr, -w_attr)
    repel = compute_cell_interaction(cell, cells, h_rep, -w_rep)
    return attract + repel

def compute_cell_interaction(cell, cells, d, w):
    sum = 0
    for other in cells:
        diff = 0

        for i in range(len(cell.vector)):
            diff += (cell.vector[i] - other.vector[i])**2

        sum += d * exp(w*diff)
        #print("sum = " + str(sum))

    return sum

def objective_function(ref_reg, ref_y_max, T_matrix, Q, weight):
    all_score = cal_Single_Posture_Score(ref_reg, ref_y_max, T_matrix, Q, weight)
    #print("all score = " + str(all_score))
    score = all_score
    return score

def cal_Single_Posture_Score(ref_reg, ref_y_max, T_matrix, Q, score_weight):




    datapoint = [T_matrix[3][0, 3], T_matrix[3][1,3], T_matrix[3][2,3],T_matrix[4][0, 3], T_matrix[4][1,3], T_matrix[4][2, 3],Q[0], Q[1], Q[2], Q[3]]

    probabilities = ref_reg.predict_proba([datapoint])[0]
    score = 1 - probabilities[ref_y_max]


    return score


def generate_random_direction(problem_size):
    random_vector = [round(random.uniform(-1,1),2) for i in range(problem_size)]

    #print("random_vector="+str(random_vector))

    return random_vector

def tumble_cell(search_space, cell, step_size_Ci):
    #t1 = time.clock()
    new_tumble_cell = copy(cell)

    step = generate_random_direction(len(search_space))
    vector = [None for i in range(len(search_space))]
    #
    for i in range(len(vector)):
        vector[i] = cell.vector[i] + step_size_Ci * step[i]
        vector[i] = int(round(vector[i],0))
        if vector[i] < search_space[i][0]: vector[i] = search_space[i][0]
        if vector[i] > search_space[i][1]: vector[i] = search_space[i][1]

    ## set lock joint ##
    vector[joint_fixed] = joint_fixed_value

    new_tumble_cell.vector = vector
    new_tumble_cell.cal_T_matrix_cell('c')
    new_tumble_cell.cal_Q_cell('c')

    #print("old cell = " + str(cell.vector) + " Old Q = " + str(cell.Q_cell))
    #print("tumble cell = " + str(new_tumble_cell.vector) + " tumble Q = " + str(new_tumble_cell.Q_cell))
    #t2 = time.clock()
    #t_diff = t2 - t1
    #print("tumble t= " + str(t_diff))
    return new_tumble_cell


def ChemotaxisAndSwim(ref_reg, ref_y_max, weight, search_space,
                        cells, chem_steps_Nc,swim_length_Ns, step_size_Ci, d_attr, w_attr, h_rep, w_rep):
    global operating_count

    best = None

    for j in range(0, chem_steps_Nc):

        moved_cells = []

        for i, cell in enumerate(cells):

            sum_nutrients = 0

            evaluate(ref_reg, ref_y_max, weight, cell, cells, d_attr, w_attr, h_rep, w_rep)

            if (best == None) or (cell.cost < best.cost):
                best = cell

            sum_nutrients += cell.fitness

            for m in range(swim_length_Ns):
                #t1 = time.clock()
                #print("swim length m = " + str(m))
                new_cell = tumble_cell(search_space, cell, step_size_Ci)
                evaluate(ref_reg, ref_y_max, weight, new_cell, cells, d_attr, w_attr, h_rep, w_rep)

                if (cell.cost < best.cost):
                   best = cell
                   print("chemo j = " + str(j) + " f = " + str(best.fitness) + " cost" + str(best.cost))

                if new_cell.fitness > cell.fitness:
                    break

                cell = new_cell
                sum_nutrients += cell.fitness

                #t2 = time.clock()
                #diff_t = t2 - t1
                #print("t cell" + str(diff_t))

            cell.sum_nutrients = sum_nutrients
            moved_cells.append(cell)

            #print("chemo j = " + str(j) + " f = " + str(best.fitness) + " cost" + str(best.cost))
            cells = moved_cells

    return best, cells

def loop(popsize):

    ### BFOA for 2017 ###
    #initial
    time_start = time.time()
    print(time_start)

    joint_angle_limit = [[0, 135], [-90, 0], [-45, 45], [0, 135], [-90, 90], [-50, 45], [-45, 45]]

    search_space = copy(joint_angle_limit)
    #posture_weight = copy.copy(score_weight)

    problem_size = 7 #Dimension of the search space

    pop_size_S = popsize#50 #Total number of bacteria in the population
    chem_steps_Nc = 100#100 #The number of chemo tactic steps
    swim_length_Ns = 4#4  #The swimming length
    repro_steps_Nre = 4#4 #Number of reproduction steps
    elim_disp_steps_Ned = 2 #Number of elimination-dispersal event
    p_eliminate_Ped = 0.25 #Elimination-dispersal probability
    step_size_Ci = 5 #The size of the step taken in the random direction specified by the tumble

    d_attr = 0.1 #attraction coefficient
    w_attr = 0.2 #attraction coefficient
    h_rep = d_attr #repulsion coefficient
    w_rep = 10 #repulsion coefficient

    best_set = Search_New_Postures_by_BFOA(ref_regressor, all_y_max, joint_fixed, joint_fixed_value,posture_weight,
            problem_size, search_space, pop_size_S,
            elim_disp_steps_Ned, repro_steps_Nre, chem_steps_Nc, swim_length_Ns, step_size_Ci, p_eliminate_Ped,
            d_attr, w_attr, h_rep, w_rep)

    for i in range(len(best_set)):

        print("set = "+str(i)+"\tvector = "+str(best_set[i].vector)+"\tcost = "+str(best_set[i].cost))

    time_stop = time.time()
    time_diff = time_stop - time_start
    print("time = " + str(time_diff))



if __name__ == "__main__":

    bye_y_max_min = np.loadtxt("bye_ex_max_min")
    bye_y_max_min = np.round(bye_y_max_min,3)
    bye_y_max = bye_y_max_min[:,0]
    print(bye_y_max)


    salute_y_max_min = np.loadtxt("salute_ex_max_min")
    salute_y_max_min = np.round(salute_y_max_min, 3)
    salute_y_max = salute_y_max_min[:, 0]
    print(salute_y_max)

    sinvite_y_max_min = np.loadtxt("sinvite_ex_max_min")
    sinvite_y_max_min = np.round(sinvite_y_max_min, 3)
    sinvite_y_max = sinvite_y_max_min[:, 0]
    print(sinvite_y_max)

    wai_y_max_min = np.loadtxt("wai_ex_max_min")
    wai_y_max_min = np.round(wai_y_max_min, 3)
    wai_y_max = wai_y_max_min[:, 0]
    print(wai_y_max)

    all_y_max = [bye_y_max, salute_y_max, sinvite_y_max, wai_y_max]

    # print(bye_y_max_min)
    # print(salute_y_max_min)
    # print(sinvite_y_max_min)
    # print(wai_y_max_min)

    ### load regressor ###
    clf_reg = pickle.load(open("ANN_Classification", 'rb'))

    ref_regressor = clf_reg

    # ###avg M: std[68.1, -18.9, -6.9, 101.0, 1.5, 0.6, 23.5]
    # ###avg M: equl[66.6 - 19.09 - 5.67 88.22 3.65 - 3.39 21.44]


    posture_weight = [1, 1, 1]
    joint_fixed_value_set = [68.1, -18.9, -6.9, 101.0, 1.5, 0.6, 23.5]
    joint_fixed_value_set = np.round(joint_fixed_value_set,0)
    print(joint_fixed_value_set)


    joint_fixed = 3
    joint_fixed_value = joint_fixed_value_set[joint_fixed]


    num_particle = 200
    loop(num_particle)
    print("fixed joint =", joint_fixed, "value =", joint_fixed_value)
    print("num_particle =", num_particle)
    #
    # for i in range(5):
    #     loop(10)
    # for i in range(5):
    #     loop(5)
    #

######### 26s

