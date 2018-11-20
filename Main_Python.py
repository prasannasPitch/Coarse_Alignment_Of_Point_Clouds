<<<<<<< HEAD
# -*- coding: utf-8 -*-
"""
Created on Thu Jul 26 23:58:03 2018
@author: prasa
"""

import time
import itertools
import numpy as np
import math as calc
import pickle 
import pandas as pd
from sklearn.tree import DecisionTreeRegressor
from collections import Counter
from itertools import cycle, islice
#loading data
#----------------------------------------------------------------------loading data CAD----------------------------------------------------------

#source_path = 'E:\\python_projects\\forrest dataset\\load_save\\keypoints_detected_cad.pcd'
#source_path = 'E:\\python_projects\\forrest dataset\\load_save\\last_sort_centroid_cad.pcd'
source_path = 'C:\\Users\\prasa\\Desktop\\dataset\\master_stl\\reading_2\\centroid_cad.pcd'


#loading points from pcd
cnt = 0
linar = []
#source_dt = open("tr_pcd.pcd", "r") 
source_dt = open(source_path, "r") 
for lin in source_dt:
		cnt = cnt + 1
		if(cnt>11):
			linar.append(lin)
final = []
for eachl in linar:
	final.append(list(eachl.split()))
big_vertical_sc_list = list(np.array(final).astype(np.float))



#global variables
big_vertical_sc = np.array(big_vertical_sc_list)
sc_pt_count = len(big_vertical_sc)
log_in_csv = 0

#----------------------------------------------------------------------loading data CAM----------------------------------------------------------
#target_path = 'E:\\python_projects\\forrest dataset\\load_save\\keypoints_detected.pcd'
#target_path = 'E:\\python_projects\\forrest dataset\\load_save\\last_sort_centroid_cam.pcd'
target_path = 'C:\\Users\\prasa\\Desktop\\dataset\\master_stl\\reading_9\\centroid_cam.pcd'

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

tr_sorted = np.asarray(big_vertical_tr)
tr_ptCloud = list(tr_sorted)
    
distance_combination = np.asarray(list(itertools.combinations(tr_pt_range_comb,2)))
unknown_value_count = len(distance_combination)



#-----------------------------------------------------------functions---------------------------------------------------------------------
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


#estimate transformation between two sets of points
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

def compute_result_list(sc_ptCloud,tr_ptCloud):
    pt_deviation = np.array(sc_ptCloud) - np.array(tr_ptCloud)
    result_list = []
    for k in range(0, len(pt_deviation)):
        result_list.append(calc.sqrt(calc.pow(pt_deviation[k,0],2) + calc.pow(pt_deviation[k,1],2) + calc.pow(pt_deviation[k,2],2)))
        result_list_pre_idx = sum(result_list)
    return(result_list_pre_idx)

#binary to int conversion of class number
def computeClassNumber(binary_num): 
    ans_n = int(str(binary_num).replace(" ","").replace(".","").replace("[","").replace("]",""),2)
    return(ans_n)

#computing distance matrix
def compute_dist_mat(list2mat):
    R_mat = np.asarray(list2mat, dtype=np.float32)
    trans_mat = np.zeros((tr_pt_count,tr_pt_count))
    trans_mat[np.triu_indices(tr_pt_count, 1)] = R_mat
    trans_mat[np.tril_indices(tr_pt_count, -1)] = trans_mat.T[np.tril_indices(tr_pt_count, -1)]
    final_mat = np.array(trans_mat,dtype=np.float32)
    print(final_mat)
    return(final_mat)
    
#computing eucledian distance
def compute_eucl_dist(dist1,dist2):
    ANS =abs( dist1 - dist2)
    euclDist = calc.sqrt((calc.pow(ANS[0], 2)) + (calc.pow(ANS[1], 2)) + (calc.pow(ANS[2], 2)))
    return(euclDist)     
    
#get the best class from all possible classes
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

#transform set of points by applying rigid transformation
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

def addnoise(eval_data):
    class_collected = []
    temp_eval = eval_data
    #for each_feature in range(0,10):
    eval_data = temp_eval
    noise = -20
    while(noise < 20):
        eval_data = temp_eval + noise
        final_Y = rf.predict(eval_data)
        final_Y = np.around(final_Y)
        ans_class = computeClassNumber(final_Y)
        class_collected.append(ans_class)
        noise += noise_rate
    deviation_in_mat = []
    for error_check in class_collected:
         
        ans_class =error_check
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
    
    predicted_cl_after_filter = class_collected[deviation_in_mat.index(min(deviation_in_mat))]
    
    return predicted_cl_after_filter,min(deviation_in_mat)

#find top ranked classes
def select_top_classes(flatten_indices):
    df = pd.DataFrame(flatten_indices)
    first_axis_df = pd.value_counts(df[0])
    list_needed = first_axis_df.head(15)
    list_selected = list(list_needed.index.values)
    return list_selected


#save histogram for class number
def save_plot(list_selected): 
    my_colors1 = list(islice(cycle(['b']), None, len(list_needed)))
    if(predicted_class_confident in list_selected):
        my_colors1[list_selected.index(predicted_class_confident)] = 'g'
    ax1 = list_needed.plot.barh(color=my_colors1,figsize = [7,4])
    ax1.set_title(str(feature_selected)+' Feature Histogram')
    ax1.set_xlabel("Votes")
    ax1.set_ylabel("Class")
    fig1 = ax1.get_figure()
    fig1.savefig('C:\\Users\\prasa\\Desktop\\dataset\\master_stl\\reading_9\\'+'feature_'+str(feature_selected)+'_histogram.jpg')
    print('Plot saved')
    print(my_colors1)
    my_colors1 = list(islice(cycle(['b']), None, len(list_needed)))


#------------------------------------------------------combinations generated------------------------------------------------------------------------------
#creating combinations
lst = range(0,sc_pt_count)
combination_pt_master = np.asarray(list(itertools.permutations(lst, tr_pt_count)))
print('combinations created') 
sc_sorted = big_vertical_sc
master_count = len(combination_pt_master)

#--------------------------------------------------------feature generation------------------------------------------------------------------------------

distance_combination = np.asarray(list(itertools.combinations(tr_pt_range_comb,2)))
unknown_value_count = len(distance_combination)

#preprocessing
start_idx = 0
resultant_idxs = []
ans_combinations = []
combination_idx_limit = sc_pt_count - tr_pt_count + 1
print('Source points taken : ' + str(sc_pt_count))
print('Target points taken : ' + str(tr_pt_count))
#print('Number of trees to be formed :' + str(combination_idx_limit))
print('Features taken : ' + str(unknown_value_count))
print('Forming Train Data...')
t0=time.time()

eval_data_list = list(tr_sorted)
eval_distance_list = []
for each_distance_combination in range(0,unknown_value_count):
    eval_distance_list.append(compute_eucl_dist(eval_data_list[distance_combination[each_distance_combination][0]],eval_data_list[distance_combination[each_distance_combination][1]]))

eval_data_temp = np.reshape(eval_distance_list,(unknown_value_count,1)).T
#eval_data = eval_data_temp[:,0:4]

temp_arr = []
train_data_mat2list = []
for predict_index in range(0, master_count):
        pt_distance_list = []
        seprate_pt = []
        for col_idx in combination_pt_master[predict_index,:]:
                seprate_pt.append(sc_sorted[col_idx])
        for each_distance_combination in range(0,unknown_value_count):
            pt_distance_list.append(compute_eucl_dist(seprate_pt[distance_combination[each_distance_combination][0]],seprate_pt[distance_combination[each_distance_combination][1]]))
        train_data_mat2list.append(pt_distance_list)

#form training data 
train_data =  np.reshape(np.asarray(train_data_mat2list),(master_count,unknown_value_count))
train_data_temp = train_data

rmse_list = []
evalTime_list = []
features_considered = []
final_result = []
treescount_list = []
lst = range(0,unknown_value_count)
for feature_selected in range(7,unknown_value_count+1):
    feature_selection =  np.asarray(list(itertools.combinations(lst,feature_selected)))
    feature_row = feature_selection.shape[0]
    feature_col = feature_selection.shape[1]
    train_data_temp = train_data
    each_tree_limit = len(train_data)/sc_pt_count
    each_tree_limit = int(each_tree_limit)
    completed_train_data = 0
    possible_indices = []
    for each_tree in range(0,sc_pt_count):
        each_tree_down = completed_train_data
        each_tree_up = completed_train_data + each_tree_limit
        train_data_red = train_data_temp[each_tree_down:each_tree_up]
        possible_indices_local = []
        for each_fs in range(0, len(feature_selection)):
            formed_traindata = []
            formed_evaldata = []
            for each_fsi in feature_selection[each_fs]:
                formed_traindata.append(train_data_red[:,each_fsi])
                formed_evaldata.append(eval_data_temp[:,each_fsi])
            train_chk = np.reshape(formed_traindata,(feature_col, each_tree_limit)).T
            eval_check = np.reshape(formed_evaldata,(feature_col,1)).T
            y = np.arange(0,train_chk.shape[0],1)
	     #Decision tree model initialized 
            dt = DecisionTreeRegressor()
            dt = dt.fit( train_chk, y ) 
            testval = np.reshape(np.array(eval_check),(1,feature_col))
            ans_class = dt.predict(testval)
            final_Y_local = int(np.around(ans_class))
            possible_indices_local.append(final_Y_local)
            
        final_Y = list(map(lambda x: x + completed_train_data, possible_indices_local))
        completed_train_data = each_tree_up
        possible_indices.append(final_Y)
        print(str(each_tree) + ' out of ' + str(sc_pt_count))
        
    possible_indices_final = np.reshape(np.asarray(possible_indices),(sc_pt_count*feature_row,1)).tolist()
    flatten_indices = [i[0] for i in possible_indices_final]
    
    top_class_list = select_top_classes(list(flatten_indices))
    predicted_class_confident,deviation_in_mat = find_confident_combination(top_class_list)
    print('RMSE : ', min(deviation_in_mat))
    print('Class : ' ,predicted_class_confident)
    print ('Evaluation time:', round(time.time()-t0, 3), 's')
    print(combination_pt_master[predicted_class_confident])
    save_transformation(predicted_class_confident)
    save_plot(predicted_class_confident)
    rmse_list.append(round(min(deviation_in_mat),3))
    evalTime_list.append(round(time.time()-t0, 3))
    t0=time.time()
    treescount_list.append(feature_row)
    features_considered.append(feature_selected)
    final_result.append(predicted_class_confident)

#--------------------------------------------------------Log result in csv-----------------------------------------------------------------------------


if (log_in_csv == 1):    
    df = pd.DataFrame({'No. Of Features ': features_considered, 'Time Taken': evalTime_list, 'No. Of Trees':treescount_list, 'RMSE': rmse_list,'Class Result': final_result, })
    df.to_csv('C:\\Users\\prasa\\Desktop\\dataset\\master_stl\\reading_9\\data1.csv', encoding='utf-8', index=False)
