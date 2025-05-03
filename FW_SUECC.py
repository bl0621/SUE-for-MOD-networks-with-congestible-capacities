import numpy as np
import math
from gurobipy import *
import sympy as sp
import matplotlib.pyplot as plt
import time



def FW_SUECC(e,tolerance,tau,consecu_iter, alpha,t_link,t_path,A,B,G,r,s0,PRINT):
    
    whole_start_t = time.time()

    error_lst = []
    ALPHA_lst = []

    # Get a feasible initial solution
    m = Model()
    h = m.addVars(A.shape[1],lb=0, vtype=GRB.CONTINUOUS, name='h')
    m.update()
    for aa in range(A.shape[0]):
        m.addConstr(quicksum(G[aa,bb]*h[bb] for bb in range(A.shape[1])) >= -s0[aa], name = 'Capacity'+str(aa))
    for aa in range(B.shape[0]):
        m.addConstr(quicksum(B[aa,bb]*h[bb] for bb in range(B.shape[1])) == r[aa], name = 'OD Demand'+str(aa))
    C = np.eye(A.shape[1])
    for aa in range(C.shape[0]):
        m.addConstr(quicksum(C[aa,bb]*h[bb] for bb in range(C.shape[1])) >= tau, name = 'Non Neg'+str(aa))
    m.update()
    objective = 888
    m.setObjective(objective,GRB.MINIMIZE)
    m.params.outputflag=0
    m.optimize()

    if m.status == 3 or m.status == 4:
        if PRINT == 1:
            print("Model infeasible!")
            m.computeIIS()
            m.write("model_infea.ilp")
            a = read('model_infea.ilp')
#             a.display() 
        return [],[]
    else:
        if PRINT == 1:
            print("Model feasible!")
        h_new = np.zeros(len(t_path))
        for j in range(len(t_path)):
            h_new[j] = h[j].x
        if PRINT == 1:
            print("---- Initial h:", h_new)
        # print("RHS of capacity constrs:", np.dot(G,h_new)+s0) #FAh+s0>=Ah
        # print("RHS of demand constrs:", np.dot(B,h_new)-r)

        count_iter = 0

        for iii in range(10000000000):


            start = time.time()

            if PRINT == 1:
                print("############ Iter", iii, "############")

            h = h_new.copy()

            # Derivative
            Der = np.log(h) + alpha*np.dot(t_link,A)
    #         if PRINT == 1:
    #             print("Derivative:", Der)

            # Solve for y
            m = Model()
            y = m.addVars(len(h),lb=0,vtype=GRB.CONTINUOUS, name="y")
            m.update()
            for s in range(len(r)):
                m.addConstr(quicksum(B[s,j]*y[j] for j in range(len(h))) == r[s])
            m.update()
            for i in range(len(t_link)):
                m.addConstr(quicksum(G[i,j]*y[j] for j in range(len(h))) >= -s0[i])
            m.update()
            for j in range(len(t_path)):
                m.addConstr(y[j] >= tau)
            m.update()
            obj = quicksum(Der[j]*y[j] for j in range(len(h)))
            m.setObjective(obj,GRB.MINIMIZE)
            m.update()
            m.params.outputflag = 0
            m.optimize()
            y_ = np.zeros(len(h))
            for j in range(len(y)):
                y_[j] = y[j].x
            y = y_
    #         if PRINT == 1:
    #             print("---- y:", y)
        #     print("RHS of capacity constrs:", np.dot(G,y)+s0) #FAh+s0>=Ah
        #     print("RHS of demand constrs:", np.dot(B,y)-r)

            # line optimization
            ALPHA_opt = binary_search(e,alpha,h,y,t_link,A)

            if ALPHA_opt == []:
                X_array = list(np.arange(0,1,0.001))
                DER = []
                for X in X_array:
                    h_ = h+X*(y-h)
                    ddd = np.multiply((np.log(h_) + alpha*np.dot(t_link,A)),(y-h))
                    DER.append(sum(ddd))        
                plt.plot(X_array, DER)
                plt.show()
                if der_func(0,h,y,t_link,A,alpha)>0:
                    ALPHA_opt = 0
                else:
                    ALPHA_opt = 1

            if PRINT == 1:
                print("---- Alpha:", ALPHA_opt)
                if ALPHA_opt<10**(-8):
                    X_array = list(np.arange(0,1,0.001))
                    DER = []
                    for X in X_array:
                        h_ = h+X*(y-h)
                        ddd = np.multiply((np.log(h_) + alpha*np.dot(t_link,A)),(y-h))
                        DER.append(sum(ddd))        
                    plt.plot(X_array, DER)
                    plt.show()

            ALPHA_lst.append(ALPHA_opt)
            h_new = h+ALPHA_opt*(y-h)

    #         if PRINT == 1:
    #             print('---- Updated h:', h_new)
    #             print("RHS of capacity constrs:", np.dot(G,h_new)+s0)
    #             print("RHS of demand constrs:", np.dot(B,h_new)-r)

            if PRINT == 1: 
                print("ERROR:", abs(np.linalg.norm(h_new - h)))
                
            error_lst.append(abs(np.linalg.norm(h_new - h)))
    #             print(h)
    #             print(h_new)

            if abs(np.linalg.norm(h_new - h)) <= tolerance or ALPHA_opt <= tolerance:
                count_iter += 1
                if PRINT == 1:
                    print('## Consecu iteration = ', count_iter, "##")
                if count_iter >= consecu_iter:
                    if PRINT == 1:
                        print('## H converge !! ##')
                    h_opt = h_new.copy()
                    v_opt = np.dot(A,h_opt)
                    break
            else:
                count_iter = 0

            end = time.time()
            
            if PRINT == 1:
                print("iter time:", end-start, 'sec')

        if PRINT == 1:
            print("   ")
        def obj(h):
            return sum(np.multiply((np.log(h)-1),h))+alpha*np.dot(t_path,h)
        if PRINT == 1:
            print('---- FINAL h:', h_opt)
            print("Objective value:", obj(h_opt))
            print("RHS of capacity constrs:", np.dot(G,h_opt)+s0)
            print("RHS of demand constrs:", np.dot(B,h_opt)-r)
            iter_lst = []
            for i in range(len(error_lst)):
                iter_lst.append(i)
            plt.plot(iter_lst,error_lst)
            plt.title("L2 norm error of path flows")
            plt.show()
            plt.plot(iter_lst,ALPHA_lst)
            plt.title("Alpha value")
            plt.show()
        
        whole_end_t = time.time()
        
        whole_time = whole_end_t - whole_start_t
        if PRINT == 1:
            print("Total time:", whole_time, "sec")


        return h_opt, v_opt


######################################################################################
######################################################################################
######################################################################################


def find_nearest(array, value):
    array = np.asarray(array)
    idx = (np.abs(array-value)).argmin()
    return idx, array[idx]


######################################################################################
######################################################################################
######################################################################################


def binary_search(e,alpha,h,y,t_link,A):
    ALPHA_left = 0
    ALPHA_right = 1
    if der_func(ALPHA_left,h,y,t_link,A,alpha)*der_func(ALPHA_right,h,y,t_link,A,alpha) >0:
        return []
    else:   
        ALPHA = 0.5
        value = der_func(ALPHA,h,y,t_link,A,alpha)
        while abs(value) > e:
            if value > 0:
                ALPHA_right = ALPHA
                ALPHA = (ALPHA_left + ALPHA_right)/2
            else:
                ALPHA_left = ALPHA
                ALPHA = (ALPHA_left + ALPHA_right)/2
            value = der_func(ALPHA,h,y,t_link,A,alpha)
        return ALPHA
    

######################################################################################
######################################################################################
######################################################################################


def der_func(ALPHA,h,y,t_link,A,alpha):
    h_ = h+ALPHA*(y-h)
    ddd = np.multiply((np.log(h_) + alpha*np.dot(t_link,A)),(y-h))
    return sum(ddd)
    

######################################################################################
######################################################################################
######################################################################################

