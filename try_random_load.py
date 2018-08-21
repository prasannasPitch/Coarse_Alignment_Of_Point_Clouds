# -*- coding: utf-8 -*-
"""
Created on Thu Jul 26 23:58:03 2018

@author: prasa
"""

import time
import itertools
import numpy as np
import math as calc
import tensorflow as tf
import pickle 
import pandas as pd
from sklearn.tree import DecisionTreeRegressor
import scipy.spatial as spatial
from sklearn.ensemble import RandomForestRegressor


#----------------------------------------------------------------------loading data----------------------------------------------------------
#target_path = 'E:\\python_projects\\forrest dataset\\load_save\\keypoints_detected.pcd'
#target_path = 'E:\\python_projects\\forrest dataset\\load_save\\last_sort_centroid_cam.pcd'
target_path = 'C:\\Users\\prasa\\Desktop\\dataset\\master_stl\\reading_2\\centroid_cam.pcd'
#target_path = 'C:\\Users\\prasa\\Desktop\\dataset\\master_stl\\reading_1\\last_sort_centroid_cam.pcd'

load_pcd_data = 1
decision_tree_pkl_filename = 'decision_tree_split_train.pkl'
decision_tree_split_train = open(decision_tree_pkl_filename, 'rb')

#loading points from pcd
if(load_pcd_data == 1 ):
    cnt = 0
    linar = []
    strlin = []
    #target_dt = open("tr_pcd_tar.pcd", "r") 
    target_dt = open(target_path, "r") 
    for lin in target_dt:
            cnt = cnt + 1
            if(cnt<12):
                strlin.append(lin)
            if(cnt>11):
                linar.append(lin)
    final = []
    for eachl in linar: 
        final.append(list(eachl.split()))
    big_vertical_tr_list = list(np.array(final).astype(np.float))

#global variables
big_vertical_tr = np.array(big_vertical_tr_list)
tr_pt_count = len(big_vertical_tr)
tr_pt_range = range(0,3)
tr_pt_range_comb = range(0,tr_pt_count)
combination_pt_master = pickle.load(decision_tree_split_train) 
sc_sorted = pickle.load(decision_tree_split_train)
sc_pt_count = len(sc_sorted)
train_data = pickle.load(decision_tree_split_train)
tr_sorted = np.asarray(big_vertical_tr)
tr_ptCloud = list(tr_sorted)
#-----------------------------------------------------------functions---------------------------------------------------------------------

def rigid_transform_3D(A, B):
    
    if ((len(A) == len(B)) and len(A) >=3):

        N = A.shape[0]; # total points
        centroid_A = np.mean(A, axis=0)
        centroid_B = np.mean(B, axis=0)
        
        # centre the points
        AA = A - np.tile(centroid_A, (N, 1))
        BB = B - np.tile(centroid_B, (N, 1))
    
        # dot is matrix multiplication for array
        H = np.transpose(AA) * BB
        U, S, Vt = np.linalg.svd(H)
        R = Vt.T * U.T
    
        # special reflection case
        if np.linalg.det(R) < 0:
           #print ('reflection detected')
           Vt[2,:] *= -1
           R = Vt.T * U.T
    
        t = -R*centroid_A.T + centroid_B.T
        
        #rounding to 5 decimal
        mat_r = np.matrix(R).reshape(9,1)
        list_r = [float(s) for s in mat_r]
        round_list = [ round(elem, 5) for elem in list_r ]
        rot_mat = np.matrix(round_list).reshape(3,3)
        
        list_t = [float(s) for s in t]
        round_list = [ round(elem, 5) for elem in list_t ]
        trans_mat = np.matrix(round_list)
        return rot_mat, trans_mat
    else:
        print('Not enough data to find transformation matrix. The matrix below is an approximate estimate.')

def calculate_transformed_points(ans_class):
    
        predicted_combination = combination_pt_master[ans_class]
        sc_ptCloud =[]
        for k in range(0,len(predicted_combination)):
            sc_ptCloud.append(sc_sorted[predicted_combination[k],:]) 
        sc_mat = np.mat(sc_ptCloud)
        tr_mat = np.mat(tr_ptCloud)
    # recover the transformation
        mat_rot, mat_trans = rigid_transform_3D(sc_mat, tr_mat)
    
    #error metrics
        mat_transform = (mat_rot*sc_mat.T) + np.tile(mat_trans.T, (1, len(sc_ptCloud)))
        mat_transform = mat_transform.T
        trnp = np.array(tr_mat)
        matnp = np.array(mat_transform)
        return matnp, trnp



def find_confident_combination(possible_indices):
    deviation_in_mat = []
    for error_check in possible_indices:
        matnp, trnp = calculate_transformed_points(error_check)
        deviation = 0
        for each_pt in range(0,len(matnp)):
            deviation = deviation + compute_eucl_dist(matnp[each_pt],trnp[each_pt])
        deviation_in_mat.append(deviation)
        predicted_class_confident = possible_indices[deviation_in_mat.index(min(deviation_in_mat))]
    return predicted_class_confident, deviation_in_mat

def analyze_result(predicted_class_confident):
    matnp, trnp = calculate_transformed_points(predicted_class_confident)
#find deviation from each point
#    deviation_pt = []
    for each_pt in range(0,len(matnp)):
        pq = compute_eucl_dist(matnp[each_pt],trnp[each_pt])
#        deviation_pt.append(pq)
#        deviated_pt = deviation_pt.index(max(deviation_pt))
        print('Point : ' + str(each_pt) + ' in Camera measurement is deviated from the CAD model by ' + str(round(pq,3)) + 'mm')
#
def compute_result_list(sc_ptCloud,tr_ptCloud):
    pt_deviation = np.array(sc_ptCloud) - np.array(tr_ptCloud)
    result_list = []
    for k in range(0, len(pt_deviation)):
        result_list.append(calc.sqrt(calc.pow(pt_deviation[k,0],2) + calc.pow(pt_deviation[k,1],2) + calc.pow(pt_deviation[k,2],2)))
        result_list_pre_idx = sum(result_list)
    return(result_list_pre_idx)

def computeClassNumber(binary_num): 
    ans_n = int(str(binary_num).replace("\n","").replace(" ","").replace(".","").replace("[","").replace("]",""),2)
    return(ans_n)
    
#    
def compute_eucl_dist(dist1,dist2):
    ANS = abs(dist1 - dist2)
    euclDist = calc.sqrt((calc.pow(ANS[0], 2)) + (calc.pow(ANS[1], 2)) + (calc.pow(ANS[2], 2)))
    return(euclDist)    

#saving in text file
def save_transformation(ans_noise_filter):
    predicted_combination = combination_pt_master[ans_noise_filter]
    print('Class : ',ans_noise_filter)
    print('Best combination possible : ',predicted_combination)
    sc_ptCloud =[]
    for k in range(0,len(predicted_combination)):
        sc_ptCloud.append(sc_sorted[predicted_combination[k],:]) 
    sc_mat = np.mat(sc_ptCloud)
    tr_mat = np.mat(tr_ptCloud)
    
    mat_rot, mat_trans = rigid_transform_3D(sc_mat, tr_mat)
    #write in txt
    file_trans_matwo= open("trans_exchange.txt","w")
    low_val1 = np.mat(np.array([0, 0, 0, 1]).reshape(1,4))
    up_val1 = np.hstack((mat_rot,mat_trans.T))
    fin_4x41 = np.vstack((up_val1,low_val1)).tolist()
    for each_mat_elem in fin_4x41:
        each_mat_elem = str(list(each_mat_elem)).replace("[","").replace("]","").replace(",","")
        file_trans_matwo.write( str(each_mat_elem) + '\n')
    file_trans_matwo.close()


def check_all(eval_data):
    t1=time.time()
    deviation_in_mat = []
    for error_check in range(0,len(combination_pt_master)):
            final_Y = dt.predict(eval_data)
            final_Y = np.around(final_Y)
            #ans_class = computeClassNumber(final_Y)
            ans_class = error_check
            predicted_combination = combination_pt_master[ans_class]
            sc_ptCloud =[]
            for k in range(0,len(predicted_combination)):
                sc_ptCloud.append(sc_sorted[predicted_combination[k],:]) 
            
    
    
    #without outlier
            sc_mat = np.mat(sc_ptCloud)
            tr_mat = np.mat(tr_ptCloud)
                    # recover the transformation
            mat_rot, mat_trans = rigid_transform_3D(sc_mat, tr_mat)
    
    #error metrics
            mat_transform = (mat_rot*sc_mat.T) + np.tile(mat_trans.T, (1, len(sc_ptCloud)))
            mat_transform = mat_transform.T
            trnp = np.array(tr_mat)
            matnp = np.array(mat_transform)
            dev = 0
            for each_ptcom in range(0,len(mat_transform)):
                dev = dev + compute_eucl_dist(matnp[each_ptcom],trnp[each_ptcom])
            
            deviation_in_mat.append(dev)
#            if(dev < 49):
#                break
    print ('Time taken to check all possibilities:', round(time.time()-t1, 3), 's')
    return deviation_in_mat.index(min(deviation_in_mat)), min(deviation_in_mat)

#--------------------------------------------------------feature generation------------------------------------------------------------------------------

#reducing features
distance_combination = np.asarray(list(itertools.combinations(tr_pt_range_comb,2)))
unknown_value_count = len(distance_combination)

eval_data_list = list(tr_sorted)
eval_distance_list = []
for each_distance_combination in range(0,unknown_value_count):
    eval_distance_list.append(compute_eucl_dist(eval_data_list[distance_combination[each_distance_combination][0]],eval_data_list[distance_combination[each_distance_combination][1]]))

#form evaluation data
eval_data = np.reshape(eval_distance_list,(unknown_value_count,1)).T

#--------------------------------------------------------Model Analysis------------------------------------------------------------------------------

print('Source points taken : ' + str(len(sc_sorted)))
print('Target points taken : ' + str(tr_pt_count))
#print('Number of trees to be formed :' + str(combination_idx_limit))
print('Features taken : ' + str(unknown_value_count))
t0=time.time()
train_data_temp = train_data
each_tree_limit = len(train_data)/sc_pt_count
each_tree_limit = int(each_tree_limit)
completed_train_data = 0
possible_indices = []

for each_tree in range(0,sc_pt_count):
    each_tree_down = completed_train_data
    each_tree_up = completed_train_data + each_tree_limit - 1
    train_data = train_data_temp[each_tree_down:each_tree_up]
    dt = pickle.load(decision_tree_split_train)
    testval = np.reshape(np.array(eval_data),(1,unknown_value_count))
    ans_class = dt.predict(testval)
    final_Y = int(np.around(ans_class) + completed_train_data)
    completed_train_data = each_tree_up +1
    #test_result = np.sum((final_Y-y)**2)
    #print(final_Y)
    fine_tune= 20
    #making more confidence
    for offset_val in range(-fine_tune,fine_tune):
        if (final_Y + offset_val >= 0 and final_Y + offset_val <= completed_train_data):
            predicted_indices = final_Y + offset_val
            possible_indices.append(predicted_indices)
    break

predicted_class_confident,deviation_in_mat = find_confident_combination(possible_indices)
save_transformation(predicted_class_confident)
print('RMSE : ', min(deviation_in_mat))
print ('Evaluation time:', round(time.time()-t0, 3), 's')
analyze_result(predicted_class_confident)