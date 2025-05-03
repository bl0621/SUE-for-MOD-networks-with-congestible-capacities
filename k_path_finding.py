import numpy as np
import pandas as pd
import copy


def k_path_finding(od, done_OD, bounded_criterion, O,D, r,N,link_node,c_int,t_link,UBER_NODES):

    ##################################### Initialization #################################### 

    w_div_OD = {1: []}

    A_node_OD = {1: []}
    link_16_17_ind = []
    link_16_17_indB = []

    a_node_OD = {1: []}

    A_link_OD = {1: []}

    a_link_OD = {1: []}

    c_link_dijkstra = t_link

    feasible_sol    = 0

    objective       = []

    B_node_OD = {1: []}
    B_link_OD = {1: []}

    tAh_vector = []
    numpath_vector = []

    k = 0
    
    
    print ("----- OD ",od+1, " -----")
    print (bounded_criterion)

    #------------------------------------------------------------------------------------------------------------------------------

    for l in range(1000000000):


        if l < 7:
            maxitr = 10
            verbose = 0
        else:
            maxitr = 10000
            verbose = 0

        a = np.zeros((len(link_node)))
        b = np.zeros((len(O)))
        A = np.empty((a.shape[0], 0))# empty A
        B = np.empty((b.shape[0], 0))# empty B
        h_ = np.empty((1,0))


        num_path  = np.zeros((len(O))) # Number of Path you actually find per OD pair **this iteration
        num_path_ = np.zeros((len(O))) # Number of Path you actually find per OD pair **last iteration

        
        

        ################# OD loop ##################


        for i in range(len(O)):


            if done_OD == 1:
                print ("********* NO PATH CAN BE GENERATED (B EMPTY) *********")
#                 print ("Path :")
#                 for path in A_node_OD[i+1]:
#                     print (path)
                print ("--- OD ",od+1," finished ---")
                for v in range(len(A_node_OD[i+1])):
                #transfer the path to column a
                    a = np.zeros((len(link_node)))
                    b = np.zeros((len(O)))
                    for s in range(len(A_node_OD[i+1][v])-1):
                        for e in range(len(link_node)):
                            if (link_node[e,1:3]==[A_node_OD[i+1][v][s],A_node_OD[i+1][v][s+1]]).all():
                                link = link_node[e,0]
                        a[link-1] = 1
                    a = a.reshape(a.shape[0],1)

                    #add new path a to A
                    k = k+1
                    A = np.hstack((A,a))
                    b[i] = 1
                    b = b.reshape(b.shape[0],1)
                    B = np.hstack((B,b))
                continue

            #################################### K-shortest path finding ####################################

            # 1st path
            if k == 0: 
                c = c_int.copy()
                ######################### Dijkstra's Algorithm ########################### 
                #initialize [w p open/closed]
                w = np.ones((len(N)))*np.inf #w
                p_node = np.zeros((len(N)))#p_node
                p_link = np.zeros((len(N)))#p_link
                index_list = ["o"]*len(N)  #open/closed
                index = np.asarray(index_list)
                df = pd.DataFrame({"w": w,"p_node": p_node,"p_link": p_link,"Open/Closed": index})

                #Origin
                df['Open/Closed'][O[i] - 1] = 'c' 
                df['w'][O[i] - 1] = 0        
                current = O[i]

                #iterate
                for j in range(100000):
                    ds = N[current] #obtain the downstream nodes
                    for s in ds: # update w and p for all downstream nodes
                        if df['w'][current-1] + c[current-1,s-1] < df['w'][s-1]:
                            df['p_node'][s-1] = current

                            for e in range(len(link_node)):
                                if (link_node[e,1:3]==[current,s]).all():
#                                     if (current == 16) & (s == 17): 
#                                         print("c=",c)
#                                     link = link_node[e,0]
                                    if   (current == 16) & (s == 17) & (c[current-1,s-1]== 8.5 ):
                                        link = 37
                                    elif (current == 16) & (s == 17) & (c[current-1,s-1]== 11  ):
                                        link = 38
                                    elif (current == 16) & (s == 17) & (c[current-1,s-1]== 12.5):
                                        link = 39
                                    else:
                                        link = link_node[e,0]

                            df['p_link'][s-1] = link
                            

                        df['w'][s-1] = min(df['w'][s-1], df['w'][current-1] + c[current-1,s-1])

                    #find the next starting point
                    df_open = df[df['Open/Closed'] == 'o']
                    current = df_open.loc[df_open['w']== min(df_open['w'])].index[0]+1

                    #close the current node
                    df['Open/Closed'][current-1] = 'c'

                    #see if we reached the Destination
                    if df['Open/Closed'][D[i]-1] == 'c':
                        break

                #Get the nodes of the path
                N_a = []
                Link_a = []
                for s in range(10000000):
                    if s == 0:
                        n = D[i]
                        n_link = df['p_link'][n-1]
                    else:
                        n = df['p_node'][n-1]
                        if (n != 0):
                            if (df['p_link'][n-1]!=0):
                                n_link = df['p_link'][n-1]
                    if n == 0:
                        break
                    N_a.append(n)
                    Link_a.append(n_link)
                Link_a = Link_a[0:-1]
#                 print(Link_a)
#                 print(N_a)


                # Add path to A_node
                a_node_OD[i+1] = np.asarray(N_a[::-1])
                A_node_OD[i+1].append(a_node_OD[i+1])
                a_link_OD[i+1] = np.asarray(Link_a[::-1])
                A_link_OD[i+1].append(a_link_OD[i+1])
                
                
                if 16 in a_node_OD[i+1] and 17 in a_node_OD[i+1]:
                    if c[16-1,17-1] == 8.5:
                        link_16_17_ind.append(1)
                    elif c[16-1,17-1] == 11:
                        link_16_17_ind.append(2)
                    elif c[16-1,17-1] == 12.5:
                        link_16_17_ind.append(3)
                    else:
                        pass
#                         print("Something wrong!!! (16,17)length = ",c[16-1,17-1])
                else:
                    link_16_17_ind.append(0)
                
#                 print ("Node path:")
#                 for path in A_node_OD[i+1]:
#                     print (path)
#                 print ("Link path:")
#                 for path in A_link_OD[i+1]:
#                     print (path)
#                 print ("--- OD ",od+1," finished ---")
#                 print ("k = ",k)
                k = k+1




        #-------------------------------------------------------------------------------------------------------------------    

            # kth path
            if k != 0:  

                for v in range(100000):

                    if v == len(a_node_OD[i+1])-1:
                        break                


                    ################################## INITIALIZE ####################################
                    c = c_int.copy()

                    ########################## Compare with existing paths ###########################
#                     print("-----")
#                     print("compared path:",a_node_OD)
                    
                    for s in range(k):
#                         print("s = ",s)
#                         print("link_16_17_ind = ",link_16_17_ind, len(link_16_17_ind))
                        if len(a_node_OD[i+1][:(v+1)]) == len(A_node_OD[i+1][s][:(v+1)]):
                            if (a_node_OD[i+1][:(v+1)] == A_node_OD[i+1][s][:(v+1)]).all():
                                if (int(A_node_OD[i+1][s][v]) == 16)&(int(A_node_OD[i+1][s][v+1]) == 17)&(c[int(A_node_OD[i+1][s][v]-1),int(A_node_OD[i+1][s][v+1]-1)] == 8.5)&link_16_17_ind[s] == 1:
                                    c[int(A_node_OD[i+1][s][v]-1),int(A_node_OD[i+1][s][v+1]-1)] = 11
#                                     print("***link(16,17) adjusted to", 11)
                                    
                                elif (int(A_node_OD[i+1][s][v]) == 16)&(int(A_node_OD[i+1][s][v+1]) == 17)&(c[int(A_node_OD[i+1][s][v]-1),int(A_node_OD[i+1][s][v+1]-1)] == 11)&link_16_17_ind[s] == 2:
                                    c[int(A_node_OD[i+1][s][v]-1),int(A_node_OD[i+1][s][v+1]-1)] = 12.5
#                                     print("***link(16,17) adjusted to", 12.5)
                                    
                                elif (int(A_node_OD[i+1][s][v]) == 16)&(int(A_node_OD[i+1][s][v+1]) == 17)&(c[int(A_node_OD[i+1][s][v]-1),int(A_node_OD[i+1][s][v+1]-1)] == 12.5)&link_16_17_ind[s] == 3:
                                    c[int(A_node_OD[i+1][s][v]-1),int(A_node_OD[i+1][s][v+1]-1)] = np.inf
#                                     print("***link(16,17) adjusted to", np.inf)
                                    
                                else:
                                    c[int(A_node_OD[i+1][s][v]-1),int(A_node_OD[i+1][s][v+1]-1)] = np.inf


                    ################################## GET R ######################################
                    R = a_node_OD[i+1][:(v+1)]
                    R_link = a_link_OD[i+1][:v]
                    
#                     print("R(node) = ",R)

#                     for s in range(len(R)-1):
#                         for e in range(len(link_node)):
#                             if (link_node[e,1:3]==[R[s],R[s+1]]).all():
#                                 link = link_node[e,0]


                    num_UBER_NODES_a = len(set(UBER_NODES) & set(list(a_node_OD[i+1])))
                    num_UBER_NODES_R = len(set(UBER_NODES) & set(R))


                    ########################### Change the Network #########################
                    N_S = copy.deepcopy(N)

                    for r_in_R in R[:-1]:
                        for nodes in N_S[r_in_R]:
                            c[int(r_in_R)-1, nodes-1] = np.inf
                        del N_S[r_in_R]

                    for key in N_S:
                        for r_in_R in R[:-1]:
                            next_nodes = N_S[key]
                            if r_in_R in next_nodes:
                                N_S[key].remove(r_in_R)
                                c[key-1,int(r_in_R)-1] = np.inf

                    if num_UBER_NODES_a == 2:
                        if num_UBER_NODES_R == 2:
                            for uber_node in UBER_NODES:
                                for uber_node_1 in UBER_NODES:
                                    if uber_node != uber_node_1:
                                        c[uber_node-1,uber_node_1-1] = np.inf

                        elif num_UBER_NODES_R == 1:
                            for uber_node in UBER_NODES:
                                for uber_node_1 in UBER_NODES:
                                    if c[uber_node-1,uber_node_1-1] > 99999999:
                                        for uber_node_2 in UBER_NODES:
                                            c[uber_node_2-1,uber_node_1-1] = np.inf                       

                            latter_uber_node = a_node_OD[i+1][v+1]
                            for uber_node in UBER_NODES:
                                if uber_node != latter_uber_node:
                                    c[uber_node-1,int(latter_uber_node)-1] = np.inf


                    ######################### Get S (Dijkstra's Algorithm) ########################
                    #initialize [w p open/closed]
                    node_id = list(N_S.keys())
                    w = np.ones((len(N_S)))*np.inf #w
                    p_node = np.zeros((len(N_S))) #p_node
                    p_link = np.zeros((len(N_S))) #p_node
                    index_list = ["o"]*len(N_S) #open/closed
                    index = np.asarray(index_list)
                    df = pd.DataFrame({"node_id":node_id,"w": w,"p_node": p_node,"p_link": p_link,"Open/Closed": index})

                    #Origin
                    ori = R[-1]
                    df['Open/Closed'][node_id.index(R[-1])] = 'c' 
                    df['w'][node_id.index(R[-1])] = 0  
                    df['p_node'][node_id.index(R[-1])] = 0
                    df['p_link'][node_id.index(R[-1])] = 0
                    current = int(R[-1])


                    #iterate
                    for j in range(1000000):

                        ds = N_S[current] # obtain the downstream nodes
                        for s in ds:      # update w and p for all downstream nodes
                            if df['w'][node_id.index(current)] + c[current-1,s-1] < df['w'][node_id.index(s)]:
                                df['p_node'][node_id.index(s)] = current
                                
#                                 for e in range(len(link_node)):
#                                     if (link_node[e,1:3]==[current,s]).all():
#                                         if (current == 16) & (s == 17): 
#                                             print("c=",c)
#                                         link = link_node[e,0]

                                for e in range(len(link_node)):
                                    if (link_node[e,1:3]==[current,s]).all():
                                        if (current == 16) & (s == 17) & (c[current-1,s-1]== 8.5):
                                            link = 37
                                        elif (current == 16) & (s == 17) & (c[current-1,s-1]== 11):
                                            link = 38
                                        elif (current == 16) & (s == 17) & (c[current-1,s-1]== 12.5):
                                            link = 39
                                        else:
                                            link = link_node[e,0]                           
                                df['p_link'][node_id.index(s)] = link

                            df['w'][node_id.index(s)] = min(df['w'][node_id.index(s)], df['w'][node_id.index(current)] + c[current-1,s-1])

                        #find the next starting point
                        df_open = df[df['Open/Closed'] == 'o']
                        current_index = df_open.loc[df_open['w']== min(df_open['w'])].index[0]
                        current =  node_id[current_index]

                        #close the current node
                        df['Open/Closed'][node_id.index(current)] = 'c'

                        #see if we reached the Destination
                        if df['Open/Closed'][node_id.index(D[i])] == 'c':
                            break

                    #Get the nodes of the path
                    N_a = []
                    Link_a = []
                    for s in range(10000000):
                        if s == 0:
                            n = D[i]
                            n_link = df['p_link'][node_id.index(n)]
                        else:
                            n = df['p_node'][node_id.index(n)]
                            if (n != 0):
                                if (df['p_link'][node_id.index(n)]!=0):
                                    n_link = df['p_link'][node_id.index(n)]
                        if n == 0:
                            break
                        N_a.append(n)
                        Link_a.append(n_link)
                    Link_a = Link_a[0:-1]  

                    if N_a == [D[i]]:
                        aaa = 1
                        S = N_a[::-1]
#                         print("S(node) = ",S)
#                         print("c[16,17] = ",c[15,16])
                    else:
                        S = N_a[::-1]
                        S_link = Link_a[::-1]
                        
                        for sss in S:
                            sss = int(sss)
                    
#                         print("S(node) = ",S)
                        
#                         print("c[16,17] = ",c[15,16])


                        ################## COMBINE R AND S , see if it's already in B ########################
                        b_node = np.append(R,S[1:])
                        b_link = np.append(R_link,S_link)
                        B_node_list = [list(item) for item in B_node_OD[i+1]]
                        B_link_list = [list(item) for item in B_link_OD[i+1]]
                        b_node_list = list(b_node)
                        b_link_list = list(b_link)
                        if (b_node_list in B_node_list) & (b_link_list in B_link_list):
                            aaa = 1
#                             print("-Rep-")
                        else:
                            B_node_OD[i+1].append(b_node)
                            if 16 in b_node and 17 in b_node:
                                if 16 in R and 17 in R:
                                    link_16_17_indB.append(link_16_17_ind[-1])
                                else:
                                    if c[16-1,17-1] == 8.5:
                                        link_16_17_indB.append(1)
                                    elif c[16-1,17-1] == 11:
                                        link_16_17_indB.append(2)
                                    elif c[16-1,17-1] == 12.5:
                                        link_16_17_indB.append(3)
                                    else:
                                        pass
#                                         print("SOMETHING WRONG! LINK",16-1,17-1,"LENGTH = ",c[16-1,17-1])
                            else:
                                link_16_17_indB.append(0)
                                
                            B_link_OD[i+1].append(b_link)
                            
                            
                            w = 0
                            for s in range(len(b_node)-1):
                                if b_node[s] in R and b_node[s+1] in R:
                                    if b_node[s] == 16 and b_node[s+1] == 17:
                                        if link_16_17_ind[-1] == 1:
                                            w = w + 8.5
                                        elif link_16_17_ind[-1] == 2:
                                            w = w + 11
                                        elif link_16_17_ind[-1] == 3:
                                            w = w + 12.5
                                    else:
                                        w = w + c_int[int(b_node[s]-1),int(b_node[s+1]-1)]
                                else:
                                    w = w + c[int(b_node[s]-1),int(b_node[s+1]-1)]
                            w_div_OD[i+1].append(w)
                            
#                             print("New B path:",b_node,w)
#                             print("B node set:",B_node_OD)
#                             print("B link set:",B_link_OD)


            ####################### find the shortest in B_node and add that to A_node #######################
            if w_div_OD[i+1] == []:
                num_path[i] = k
#                 print ("********* NO PATH CAN BE GENERATED (B EMPTY) *********")
                done_OD = 1
#                 print ("Node Path :")
#                 for path in A_node_OD[i+1]:
#                     print (path)
#                 print ("Link Path :")
#                 for path in A_link_OD[i+1]:
#                     print (path)
#                 print ("--- OD ",od+1," finished ---")
#                 print (" ")
    #                 break
            else:
                ind = w_div_OD[i+1].index(min(w_div_OD[i+1]))
#                 print("w = ",w_div_OD, ind)
                A_node_OD[i+1].append(B_node_OD[i+1][ind])
                
                link_16_17_ind.append(link_16_17_indB[ind])
                    
                A_link_OD[i+1].append(B_link_OD[i+1][ind])
                B_node_OD[i+1].pop(ind)
                B_link_OD[i+1].pop(ind)
                w_div_OD[i+1].pop(ind)
                
                link_16_17_indB.pop(ind)
                
#                 print("w = ",w_div_OD, ind)
                
#                 print("NEW PATH ADDED:",A_node_OD[i+1][-1])
#                 print ("Node Path :")
#                 for path in A_node_OD[i+1]:
#                     print (path)
#                 print ("Link Path :")
#                 for path in A_link_OD[i+1]:
#                     print (path)
#                 print ("--- OD ",od+1," finished ---")


                ####################### update a_node (last generated path) #################
                a_node_OD[i+1] = A_node_OD[i+1][k]
                a_link_OD[i+1] = A_link_OD[i+1][k]
                k = k+1


            ##################### Tansfer A_node into A matrix #########################
#             print(A_link_OD)
            for v in range(len(A_link_OD[i+1])):
                #transfer the path to column a
                a = np.zeros((len(link_node)))
                b = np.zeros((len(O)))

                for s in range(len(A_link_OD[i+1][v])):
                    path_path = A_link_OD[i+1][v]
                    link = int(path_path[s])
                    a[link-1] = 1

                a = a.reshape(a.shape[0],1)

                #add new path a to A
                A = np.hstack((A,a))
                b[i] = 1
                b = b.reshape(b.shape[0],1)
                B = np.hstack((B,b))

#         print('TOTAL NUMBER OF PATHS = ',A.shape[1])


        if l > 0:
            if A.shape[1] == A_.shape[1]:
                
#                 print ("***************** ALL PATH GENERATED *********************")
                
                t_path = np.dot(A.T,t_link)
#                 print("Longest/Bound: ",max(t_path),"/",min(t_path)*(1+bounded_criterion), "   ", bounded_criterion)
                if min(t_path)*(1+bounded_criterion) < max(t_path):
#                     print("***************** FINAL PATH OUT OF BOUND *****************")
                    A = A[:,0:A.shape[1]-1]

#                     print ("Node Path :")
#                     j = 0
#                     for path in A_node_OD[i+1]:
#                         print (path, t_path[j])
#                         j += 1
#                     print ("Link Path :")
#                     j = 0
#                     for path in A_link_OD[i+1]:
#                         print (path, t_path[j])
#                         j += 1
#                     print ("--- OD ",od+1," finished ---")
                    
                else:
                    pass
#                     print("***************** ALL PATHS IN BOUND *****************")
                    
#                     print ("Node Path :")
#                     j = 0
#                     for path in A_node_OD[i+1]:
#                         print (path, t_path[j])
#                         j += 1
#                     print ("Link Path :")
#                     j = 0
#                     for path in A_link_OD[i+1]:
#                         print (path, t_path[j])
#                         j += 1
                    print ("--- OD ",od+1," finished ---")
                    
                break
                
        A_ = A.copy()

        if l > 0:
            t_path = np.dot(A.T,t_link)
#             print("Longest/Bound: ",max(t_path),"/",min(t_path)*(bounded_criterion), "   ", bounded_criterion, min(t_path))
            if min(t_path)*(bounded_criterion) < max(t_path):
#                 print("***************** FINAL PATH OUT OF BOUND *****************")
                A = A[:,0:A.shape[1]-1]
                t_path = np.dot(A.T,t_link)
                A_node_OD[i+1] = A_node_OD[i+1][0:-1]
                A_link_OD[i+1] = A_link_OD[i+1][0:-1]
            
            
#                 print ("Node Path :")
#                 j = 0
#                 for path in A_node_OD[i+1]:
#                     print (path, t_path[j])
#                     j += 1
#                 print ("Link Path :")
#                 j = 0
#                 for path in A_link_OD[i+1]:
#                     print (path, t_path[j])
#                     j += 1
                print ("--- OD ",od+1," finished ---")
                
                break


    return A, t_path, A_node_OD