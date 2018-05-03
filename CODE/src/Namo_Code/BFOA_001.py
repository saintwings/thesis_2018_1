import random
import math    # cos() for Rastrigin
import copy    # array-copying convenience
import sys     # max float
import time
from copy import deepcopy
### local import ###
from utility_function import*
from posture_files import*


operating_count = 0

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
            self.T_matrix_cell = calKinematicNamo(self.vector, 'R')

        elif verbal == 'cp':
            self.T_matrix_cell = calKinematicNamo(self.vector, 'R')
            print("T_matric_cell = " + str(self.T_matrix_cell))

        elif verbal == 'p':
            print("T_matric_cell = " + str(self.T_matrix_cell))


    def cal_Q_cell(self, verbal = 'c'):
        if verbal == 'c':
            self.Q_cell = calQuaternion(self.T_matrix_cell[7])

        elif verbal == 'cp':
            self.Q_cell = calQuaternion(self.T_matrix_cell[7])
            print("Q_cell = " + str(self.Q_cell))

        elif verbal == 'p':
            print("Q_cell = " + str(self.Q_cell))


def Search_New_Postures_by_BFOA(ref_postures, joint_fixed, joint_fixed_value,weight,
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
    for posture_count in range(len(ref_postures)):


        cells = deepcopy(random_cells)
        best = Bacteria(problem_size, search_space, joint_fixed, joint_fixed_value, rnd, 'None')

        ## cal reference cell posture value ##
        ref_cell = Bacteria(problem_size, search_space, joint_fixed, joint_fixed_value, rnd, 'None')
        ref_cell.vector = [ref_postures[posture_count][i] for i in range(problem_size)]
        ref_cell.cal_T_matrix_cell('c')
        ref_cell.cal_Q_cell('c')


        for l in range(0, elim_disp_steps_Ned):
            for k in range(0, repro_steps_Nre):
                print("posture count =" + str(posture_count) + " value=" + str(ref_postures[posture_count]))
                print("l = " + str(l) + " k = " + str(k)+">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
                c_best, cells = ChemotaxisAndSwim(ref_cell, weight, search_space,
                                                  cells, chem_steps_Nc,swim_length_Ns, step_size_Ci, d_attr, w_attr, h_rep, w_rep)

                if (best.cost == None) or (c_best.cost < best.cost):
                    best = c_best

                print("best fitness = " + str(best.fitness) + ", cost = " + str(best.cost))
                cells = sorted(cells, key=lambda Bacteria: Bacteria.sum_nutrients)
                ## reproduction step ##
                cells = cells[:len(cells)/2]
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


def cal_Single_Posture_Score(T_matrix_ref, Q_ref, T_matrix, Q, score_weight):
    score = []

    Q_diff = [Q_ref[0] - Q[0], Q_ref[1] - Q[1], Q_ref[2] - Q[2], Q_ref[3] - Q[3]]

    norm_Q_diff = math.sqrt(
        (Q_diff[0] * Q_diff[0]) + (Q_diff[1] * Q_diff[1]) + (Q_diff[2] * Q_diff[2]) + (Q_diff[3] * Q_diff[3]))
    # elbow_diff =[TMatrix_Target_Posture[3][0,3] - T_Matrix[3][0,3], TMatrix_Target_Posture[3][1,3] - T_Matrix[3][1,3], TMatrix_Target_Posture[3][2,3] - T_Matrix[3][2,3]]
    norm_elbow_diff = math.sqrt(pow(T_matrix_ref[3][0, 3] - T_matrix[3][0, 3], 2) + pow(
        T_matrix_ref[3][1, 3] - T_matrix[3][1, 3], 2) + pow(
        T_matrix_ref[3][2, 3] - T_matrix[3][2, 3], 2))
    # wrist_diff =[TMatrix_Target_Posture[4][0,3] - T_Matrix[4][0,3], TMatrix_Target_Posture[4][1,3] - T_Matrix[4][1,3], TMatrix_Target_Posture[4][2,3] - T_Matrix[4][2,3]]
    norm_wrist_diff = math.sqrt(pow(T_matrix_ref[4][0, 3] - T_matrix[4][0, 3], 2) + pow(
        T_matrix_ref[4][1, 3] - T_matrix[4][1, 3], 2) + pow(
        T_matrix_ref[4][2, 3] - T_matrix[4][2, 3], 2))
    sum_diff = norm_Q_diff * score_weight[0] + norm_elbow_diff * score_weight[1] + norm_wrist_diff * score_weight[2]

    #score.append(Q_diff)
    score.append(norm_Q_diff)
    score.append(norm_elbow_diff)
    score.append(norm_wrist_diff)
    score.append(sum_diff)

    return score



def evaluate(ref_cell, weight, cell, cells, d_attr, w_attr, h_rep, w_rep):
    #t1 = time.clock()
    cell.cost = objective_function(ref_cell.T_matrix_cell,ref_cell.Q_cell,cell.T_matrix_cell,cell.Q_cell, weight)
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

        sum += d * math.exp(w*diff)
        #print("sum = " + str(sum))

    return sum

def objective_function(T_matrix_ref,Q_ref, T_matrix, Q, weight):
    all_score = cal_Single_Posture_Score(T_matrix_ref, Q_ref, T_matrix, Q, weight)
    #print("all score = " + str(all_score))
    score = all_score[3]
    return score

def generate_random_direction(problem_size):
    random_vector = [round(random.uniform(-1,1),2) for i in range(problem_size)]

    #print("random_vector="+str(random_vector))

    return random_vector

def tumble_cell(search_space, cell, step_size_Ci):
    #t1 = time.clock()
    new_tumble_cell = copy.copy(cell)

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


def ChemotaxisAndSwim(ref_cell, weight, search_space,
                        cells, chem_steps_Nc,swim_length_Ns, step_size_Ci, d_attr, w_attr, h_rep, w_rep):
    global operating_count

    best = None

    for j in range(0, chem_steps_Nc):

        moved_cells = []

        for i, cell in enumerate(cells):

            sum_nutrients = 0

            evaluate(ref_cell, weight, cell, cells, d_attr, w_attr, h_rep, w_rep)

            if (best == None) or (cell.cost < best.cost):
                best = cell

            sum_nutrients += cell.fitness

            for m in range(swim_length_Ns):
                #t1 = time.clock()
                #print("swim length m = " + str(m))
                new_cell = tumble_cell(search_space, cell, step_size_Ci)
                evaluate(ref_cell, weight, new_cell, cells, d_attr, w_attr, h_rep, w_rep)

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

def loop( popsize):
    #initial
    #problem configuration
    time_start = time.time()
    print time_start

    ref_postures = copy.copy(original_postures_config)
    search_space = copy.copy(joint_angle_limit)
    posture_weight = copy.copy(score_weight)

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

    best_set = Search_New_Postures_by_BFOA(ref_postures, joint_fixed, joint_fixed_value,posture_weight,
            problem_size, search_space, pop_size_S,
            elim_disp_steps_Ned, repro_steps_Nre, chem_steps_Nc, swim_length_Ns, step_size_Ci, p_eliminate_Ped,
            d_attr, w_attr, h_rep, w_rep)

    for i in range(len(best_set)):

        print("set = "+str(i)+"\tvector = "+str(best_set[i].vector)+"\tcost = "+str(best_set[i].cost))

    time_stop = time.time()
    time_diff = time_stop - time_start
    print("time = " + str(time_diff))



    ##save file##
    fileName = "./BFOA_data/FixedIndex = "+str(joint_fixed) + "\t popsize = " +str(pop_size_S) + "\t Date " + str(time.asctime()) + ".txt"
    file_data = open(fileName, 'w')
    file_data.write("ref pos\n")
    for item in ref_postures:
        file_data.write("%s\n"% item)
    file_data.write("\nsearch space\n")
    for item in search_space:
        file_data.write("%s\n" % item)
    file_data.write("\nbest set\n")
    for item in best_set:
        file_data.write(str(item.vector) + "\tcost = " + str(item.cost) + "\n")
        #file_data.write("%s\n" % item)
    file_data.write("\njoint fixed index = " + str(joint_fixed) + "\tvalue = " + str(joint_fixed_value) + "\n")
    file_data.write("\ntime = "+str(time_diff)+"\n")

    file_data.write("\nParameter\n")
    file_data.write("\npop_size_S = "+str(pop_size_S))
    file_data.write("\nchem_steps_Nc = "+str(chem_steps_Nc))
    file_data.write("\nswim_length_Ns = "+str(swim_length_Ns))
    file_data.write("\nrepro_steps_Nre = "+str(repro_steps_Nre))
    file_data.write("\np_eliminate_Ped = "+str(p_eliminate_Ped))
    file_data.write("\nstep_size_Ci = "+str(step_size_Ci))
    file_data.write("\nd_attr = "+str(d_attr))
    file_data.write("\nw_attr = "+str(w_attr))
    file_data.write("\nh_rep = "+str(h_rep))
    file_data.write("\nw_rep = "+str(w_rep))
    file_data.close()


if __name__ == "__main__":
    for i in range(5):
        loop(10)
    for i in range(5):
        loop(5)


######### 26s
