import numpy as np
import rps.robotarium as robotarium
from rps.utilities.transformations import *
from rps.utilities.graph import *
from rps.utilities.barrier_certificates import *
from rps.utilities.misc import *
from rps.utilities.controllers import *
import time
from scipy.spatial import Voronoi, voronoi_plot_2d
from matplotlib.artist import Artist
import csv
import pandas as pd
from copy import deepcopy
from time import sleep

#this function is used to normalize the weight array

# def normalize(arr,t_min,t_max):
#     s = sum(l)
#     return [i/s for i in l]

break_out_flag=False

# c_v is value for grid cell c_v=np.zeros(2,N)
# w = np.zeros(N,1)

def get_sensor(j,q):
     return 1


#weights = [1,1,1,1,1,1,1,1,1,1]
#weights = [2,1,1,1,1,2,1,1,1,1]



#while reading the paper I assume sensor health is equivalent to 

#Scenario 0

# sensors=[1]
# N=10
# sensors=[1]
# robot_by_sensor=[1,1,1,1,1,1,1,1,1,1]
# binary_robot_by_sensor=[1,1,1,1,1,1,1,1,1,1]
# hi1=[[1,1,1,1,1],[1,1,1,1,1]]
# wi1=[[1,1,1,1,1],[1,1,1,1,1]]
# numbots1=10
# numbots2=0
# initial_conditions = np.asarray([[1.25, 0.25, 0],[1, 0.5, 0],[1, -0.5, 0],[-1, -0.75, 0],[0.1, 0.2, 0],[0.2, -0.6, 0],[-0.75, -0.1, 0],[-1, 0, 0],[-0.8, -0.25, 0],[1.3, -0.4, 0]])

scenario_num = 1

#Scenario 1
if scenario_num == 1:
    N=10
    sensors=[1,2]
    robot_by_sensor=[1,1,1,1,1,2,2,2,2,2]
    numbots1=5
    numbots2=5
    binary_robot_by_sensor=[[1,1,1,1,1,0,0,0,0,0],[0,0,0,0,0,1,1,1,1,1]]
    n1=[1,2,3,4,5]
    n2=[6,7,8,9,10]
    hi1=[[1,1,1,1,1,0,0,0,0,0],[0,0,0,0,0,1,1,1,1,1]]
    wi1=[[1,1,1,1,1,0,0,0,0,0],[0,0,0,0,0,1,1,1,1,1]]
    initial_conditions = np.asarray([[1.25, 0.25, 0],[1, 0.5, 0],[1, -0.5, 0],[-1, -0.75, 0],[0.1, 0.2, 0],[0.2, -0.6, 0],[-0.75, -0.1, 0],[-1, 0, 0],[-0.8, -0.25, 0],[1.3, -0.4, 0]])
    gain=.2 #.1 gain causes HG to get pretty stable to be ~400

#scenario 2

if scenario_num == 2:
    N=10
    gain=.3
    sensors=[1,2]
    robot_by_sensor=[1,1,1,1,1,2,2,2,2,2]
    numbots1=5
    numbots2=5
    binary_robot_by_sensor=[[1,1,1,1,1,0,0,0,0,0],[0,0,0,0,0,1,1,1,1,1]]
    n1=[1,2,3,4,5]
    n2=[6,7,8,9,10]
    hi1=[[1,1,2,1,1,0,0,0,0,0],[0,0,0,0,0,2,1,1,1,2]]
    wi1=[[1,1,1,1,1,0,0,0,0,0],[0,0,0,0,0,1,1,1,1,1]]
    initial_conditions = np.asarray([[1.25, 0.25, 0],[1, 0.5, 0],[1, -0.5, 0],[-1, -0.75, 0],[0.1, 0.2, 0],[0.2, -0.6, 0],[-0.75, -0.1, 0],[-1, 0, 0],[-0.8, -0.25, 0],[1.3, -0.4, 0]])    



# #Scenario 3
############################Scenario 3 as written in the paper is not working for my implementation. Something about the starting point for robot number 8 with a y starting location of 0 causes it to never get the minimum voronoi centroid distance. I changed the starting location to .4 as it was the first positive tenth of a decimal point causing my implementation to work.################################################################
if scenario_num == 3:
    N=10
    gain=.1
    sensors=[1,2,3,4,5]
    robot_by_sensor=[1,1,2,2,3,3,4,4,5,5]
    numbots1=5
    numbots2=5
    binary_robot_by_sensor=[[1,1,0,0,0,0,0,0,0,0],[0,0,1,1,0,0,0,0,0,0],[0,0,0,0,1,1,0,0,0,0],[0,0,0,0,0,0,1,1,0,0],[0,0,0,0,0,0,0,0,1,1]]
    n1=[1,2,3,4,5]
    n2=[6,7,8,9,10]
    hi1=[[1,2,0,0,0,0,0,0,0,0],[0,0,2,1,0,0,0,0,0,0],[0,0,0,0,1,2,0,0,0,0],[0,0,0,0,0,0,2,1,0,0],[0,0,0,0,0,0,0,0,1,2]]
    wi1=[[1,2,0,0,0,0,0,0,0,0],[0,0,2,1,0,0,0,0,0,0],[0,0,0,0,1,2,0,0,0,0],[0,0,0,0,0,0,2,1,0,0],[0,0,0,0,0,0,0,0,1,2]]
    initial_conditions = np.asarray([[1.25, 0.25, 0],[1, 0.5, 0],[1, -0.5, 0],[-1, -0.75, 0],[0.1, 0.2, 0],[0.2, -0.6, 0],[-0.75, -0.1, 0],[-1,.4, 0],[-0.8, -0.25, 0],[1.3, -0.4, 0]])


#scenario 4
if scenario_num == 4:
    N=5
    gain=.5
    sensors=[1,2]
    robot_by_sensor=[1,1,1,2,2]
    numbots1=5
    numbots2=5
    binary_robot_by_sensor=[[1,1,1,0,0],[0,0,1,1,1]]
    n1=[1,2,3]
    n2=[3,2,1]
    hi1=[[1,1,1,0,0],[0,0,1,1,1]]
    wi1=[[1,1,2,0,0],[0,0,1,2,2]]
    initial_conditions = np.asarray([[1.25, 0.25, 0],[1, 0.5, 0],[1, -0.5, 0],[-1, -0.75, 0],[0.1, 0.2, 0]])


# #Scenario 5
if scenario_num == 5:
    N=5
    gain=.5
    sensors=[1,2]
    robot_by_sensor=[1,1,1,2,2]
    numbots1=5
    numbots2=5    
    binary_robot_by_sensor=[[1,1,1,0,0],[0,0,1,1,1]]
    n1=[1,2,3]
    n2=[3,4,5]
    hi1=[[1,1,2,0,0],[0,0,1,2,1]]
    wi1=[[1,1,1,0,0],[0,0,1,1,1]]
    initial_conditions = np.asarray([[1.25, 0.25, 0],[1, 0.5, 0],[1, -0.5, 0],[-1, -0.75, 0],[0.1, 0.2, 0]])

#r = robotarium.Robotarium(number_of_robots=N, show_figure=True, initial_conditions=initial_conditions, sim_in_real_time=False)

r = robotarium.Robotarium(number_of_robots=N,sim_in_real_time=True,initial_conditions=initial_conditions[0:N].T)

single_integrator_position_controller = create_si_position_controller()

# Create barrier certificates to avoid collision
#si_barrier_cert = create_single_integrator_barrier_certificate()
si_barrier_cert = create_single_integrator_barrier_certificate_with_boundary()

_, uni_to_si_states = create_si_to_uni_mapping()

# Create mapping from single integrator velocity commands to unicycle velocity commands
si_to_uni_dyn = create_si_to_uni_dynamics_with_backwards_motion()

# define x initially
x = r.get_poses()
x_si = uni_to_si_states(x)

#python is nx2

w = [0]*N
hg=0
c_v= [[0 for col in range(3)] for row in range(N)]

#print(c_v)

counting = 0

x_min_robotarium = -1.6
y_min_robotarium = -1
x_max_robotarium = 1.6
y_max_robotarium = 1
res=.2

#print(r.boundaries)

#current_x and current_y 
#si_to_uni mapping to dxu getting velocities
font_height_meters = 0.1
font_height_points = determine_font_size(r,font_height_meters)
count=0


robot_marker_size_m = 0.1

wijminushij= [[0 for col in range(N)] for row in range(len(sensors))]

marker_size_robot = determine_marker_size(r, robot_marker_size_m)
font_size = determine_font_size(r,0.05)
line_width = 5
robot_sensor_labels=[]
for ii in range(x.shape[1]):
    if robot_by_sensor[ii] == 1:
        robot_sensor_labels.append(r.axes.scatter(x[0,ii], x[1,ii], s=marker_size_robot, marker='o', facecolors='none',edgecolors='#ca2a2a',linewidth=line_width))
    if robot_by_sensor[ii] == 2:
        robot_sensor_labels.append(r.axes.scatter(x[0,ii], x[1,ii], s=marker_size_robot, marker='o', facecolors='none',edgecolors='#b6ca2a',linewidth=line_width))
    if robot_by_sensor[ii] == 3:
        robot_sensor_labels.append(r.axes.scatter(x[0,ii], x[1,ii], s=marker_size_robot, marker='o', facecolors='none',edgecolors='#2aca88',linewidth=line_width))
    if robot_by_sensor[ii] == 4:
        robot_sensor_labels.append(r.axes.scatter(x[0,ii], x[1,ii], s=marker_size_robot, marker='o', facecolors='none',edgecolors='#2a40ca',linewidth=line_width))
    if robot_by_sensor[ii] == 5:
        robot_sensor_labels.append(r.axes.scatter(x[0,ii], x[1,ii], s=marker_size_robot, marker='o', facecolors='none',edgecolors='#a82aca',linewidth=line_width))

robot_number_labels=[]
for xx in range(x.shape[1]):
    robot_number_labels.append(r.axes.text(x[0,0],x[1,xx]+0.25,str(int(xx+1)),fontsize=font_size, color='r',fontweight='bold',horizontalalignment='center',verticalalignment='center',zorder=0))

r.step()
weighted_mass= [[0 for col in range(3)] for row in range(N)]

all_values=[]
distance_traveled=[0]*N
old_traveled=[0]*N
previous_x_si=x_si
wijminushij1=[]
wijminushij2=[]
wijminushij3=[]
wijminushij4=[]
wijminushij5=[]

iterations = 1000

for iteration in range(iterations):
    values_to_write=[]
    values_to_write.append(iteration)
    dxi = np.zeros((2, N))
    # Get poses of agents
    x = r.get_poses()
    x_si = uni_to_si_states(x)
    values_to_write.append(x_si)

    for i in range(x.shape[1]):
        robot_sensor_labels[i].set_offsets(x[:2,i].T)
        robot_number_labels[i].set_position([x_si[0,i],x_si[1,i]+0.15])
        robot_sensor_labels[i].set_sizes([determine_marker_size(r, robot_marker_size_m)])

    # if len(sensors) == 1: 
    #     w = [0]*N
    #     c_v= [[0 for col in range(3)] for row in range(N)]            
    #     for xi in np.arange(x_min_robotarium,x_max_robotarium,res):
    #             for yi in np.arange(y_min_robotarium,y_max_robotarium,res):
    #                 sensor_value = 1
    #  #               distances = np.zeros((len(sensors),N))
    #                 for k in range(len(sensors)):
    #                     distances = np.zeros((len(sensors),N))
    #                     distances2=[0]*N
    #                     for robot in range(N):
    #                         if binary_robot_by_sensor[k][robot]==1:
    #                             distances[k][robot] =  (np.square(xi - x_si[0,robot]) + np.square(yi - x_si[1,robot])) 
    #                             distances2[robot] = (np.square(xi - x_si[0,robot]) + np.square(yi - x_si[1,robot])) 

    #                     min_index = np.argmin(distances[k])
    #                     # if min_index == 1:
    #                     #     print(min_index)
    #                     c_v[min_index][0] += (xi * sensor_value)
    #                     c_v[min_index][1] += (yi * sensor_value)
    #                     k2=np.argmin(distances2[distances2 != 0])
    #                     density=get_sensor(1,[x_sample,y_sample])
    #                     hg += .5*distances2[k2]*density
    #                     w[min_index] += sensor_value
    

    a=1

    x_sample=0
    y_sample=0
 
    hg=0

    if len(sensors) >0:
        #for types in sensors:
        w = [0]*N
        c_v= [[0 for col in range(3)] for row in range(N)]            
        for xi in np.arange(x_min_robotarium,x_max_robotarium,res):
                for yi in np.arange(y_min_robotarium,y_max_robotarium,res):
                    sensor_value = 1
     #               distances = np.zeros((len(sensors),N))
                    for k in range(len(sensors)):
                        distances = np.zeros((len(sensors),N))
                        distances2=[0]*N
                        for robot in range(N):
                            if binary_robot_by_sensor[k][robot]==1:
                                distances[k][robot] =  (np.square(xi - x_si[0,robot]) + np.square(yi - x_si[1,robot])) - wi1[k][robot]
                                distances2[robot] = (np.square(xi - x_si[0,robot]) + np.square(yi - x_si[1,robot])) - wi1[k][robot]

                        min_index = np.argmin(distances[k])
                        # if min_index == 1:
                        #     print(min_index)
                        c_v[min_index][0] += (xi * sensor_value)
                        c_v[min_index][1] += (yi * sensor_value)
                        k2=np.argmin(distances2[distances2 != 0])
                        density=get_sensor(1,[x_sample,y_sample])
                        hg += .5*distances2[k2]*density
                        w[min_index] += sensor_value


    L=completeGL(N)
    #Equation 8 paper 2 is below, c_v is centroid of weighted voronoi
    for robot in range(N):
        current_x=x_si[0,robot]
        current_y=x_si[1,robot]
        if not w[robot] == 0:
            c_x = c_v[robot][0]/(w[robot])
            c_y = c_v[robot][1]/(w[robot])
        
        else:
            c_x = c_y = 0

        #Below is equation 8 from paper 2
        dxi[:,robot] = [a*(c_x - current_x), a*(c_y - current_y )] 
    brake_check = np.around(dxi,2)
    brake_check = not np.any(brake_check)

    #Equation 14
    sum_of_neighbors_matrix= [[0 for col in range(len(sensors))] for row in range(N)]

    


    #this weight array is for the commented method at line 337

    # #EQ 14 from PAPER 2

    ### THIS IS NOT WORKING CORRECTLY. The weights do not converge to health they converge in an ever-increasing way and eventually go to a linear path suggesting they converge but not parallel to x axis but 

    #this is the second method described in the paper I wrote this based off of the code you put in slack on march 16



    ###METHOD ONE BELOW ALSO UNCOMMENT LINE 373
    # ######this block performed better in terms of where the robots went but did not converge correctly
    # weight_array =[0]*N
    # for robot in range(N):
    #     z=topological_neighbors(L,robot) 
    #     for k in range(len(sensors)):
    #         for neighbors in z:
    #             sum_of_neighbors_matrix[robot][k]+=hi1[k][robot]-hi1[k][neighbors]
    #     sum_of_neighbors_array=np.array(sum_of_neighbors_matrix, dtype=int)
    #     if w[robot] == 0:
    #         weight_array[robot]=np.array([0]*len(sensors),dtype=int)
    #     if not w[robot] == 0:  
    #         weight_array[robot]=(gain/(2*w[robot]))*sum_of_neighbors_array[robot]
    #         #print(weight_array)
    # #print(weight_array)
    # weight_array=np.vstack(weight_array) #to make array not a list of arrays

    ##METHOD 2 FROM REPORT
    weight_array=[[0 for col in range(N)] for row in range(len(sensors))]
    for robot in range(N):
        z=topological_neighbors(L,robot) 
        for k in range(len(sensors)):
            for neighbors in z:
                #print(k)
                #print(robot)
                #print(neighbors)
                weight_array[k][robot]=weight_array[k][robot]-((wi1[k][robot]-wi1[k][neighbors])-(hi1[k][robot]-hi1[k][neighbors])) ##The error in scenario 2 is something to do with wi1[k][robot]
        #sum_of_neighbors_array=np.array(sum_of_neighbors_matrix, dtype=int)

        for k in range(len(sensors)):
            if w[robot] == 0:
                weight_array[robot]=np.array([0]*len(sensors),dtype=int)            
            if not w[robot] == 0:  
                wi1[k][robot]+=(gain/(w[robot]))*weight_array[k][robot]
    # #         #print(weight_array)


    #print(weight_array)
    # print("w8",w[8])
    # print("sum of nuym array 8",sum_of_neighbors_array[8])
    #print(weight_array)
    #print("weight array ",weight_array)
    # weight_array=gain/(2*density)*sum_of_neighbors_array
    #weight_array=normalize(weight_array,0,1)
    #wi1=np.array(wi1)
    #print(wi1)
    #print('wi1',wi1)
    
    for sensors_num in range(len(sensors)):
        for robot in range(N):
                #wi1[sensors_num][robot] = wi1[sensors_num][robot]+ weight_array[robot][sensors_num]
                wijminushij[sensors_num][robot]=wi1[sensors_num][robot]-hi1[sensors_num][robot]
    #print(sum_of_neighbors_array)
    # print("w8",w[7])
    # print("sum of nuym array 8",sum_of_neighbors_array[7])
    #print(wi1)
  
    cwij=[0]*N
    #print(wijminushij)
    #print('wi1 after changing',wi1)
    # for i in range(len(sensors)):
    values_to_write.append(deepcopy(wi1))
    #for i in range(len(sensors)):
    values_to_write.append(deepcopy(wijminushij))

    # wijminushij1.append(deepcopy(wijminushij[0]))
    # wijminushij2.append(deepcopy(wijminushij[1]))
    # wijminushij3.append(deepcopy(wijminushij[2]))
    # wijminushij4.append(deepcopy(wijminushij[3]))
    # wijminushij5.append(deepcopy(wijminushij[4]))
    for i in range(N):
        if not w[i] == 0:
            cwij[i]=[c_v[i][0]/(w[i]),c_v[i][1]/(w[i])]
    values_to_write.append(cwij)
    #print(values_to_write)
    
    # if brake_check:#and convergence of wij-hij:
    #     print("Simulation has ended due to brake checking movement of robots at iteration: ",iteration)
    #     #For some reason this does not add the voronoi to the graph though if i do it outside this loop it will overlay the voronoi on the robotarium graph.
    #     #Do not have enough time to get voronoi to display properly.
    #     # v=Voronoi(x_si.T)
    #     # voronoi_plot_2d(v,ax=r.axes)
    #     sleep(5)
    #     break
    # Create safe control inputs (i.e., no collisions)
    #dxi = si_barrier_cert(dxi, x_si)
    if iterations == 999:
         print("Simulation has ended due to hitting 1,000 iterations")
         sleep(5)
    # Transform single integrator velocity commands to unicycle
    dxu = si_to_uni_dyn(dxi, x)
    for i in range(N):
        distance_traveled[i]+=np.linalg.norm((x_si[:,i])-(previous_x_si[:,i]))

    values_to_write.append(distance_traveled[:])

    total_distance_among_all_bots=np.sum(distance_traveled)

    values_to_write.append(total_distance_among_all_bots)
    values_to_write.append(hg)
    all_values.append(values_to_write)

    previous_x_si = x_si
    hg=0
    # Set the velocities by mapping the single-integrator inputs to unciycle inputs
    r.set_velocities(np.arange(N), dxu)
    # Iterate the simulation
    r.step()

#below is the code i used to create the CSV and graph what i could. I did not have enough time to get all of the graphs for all of the robots.

firstrow=["Iteration","Pose_x_y_Theta","Wij_sen1","Wij_minus_Hij_sen1","Cwij","Distance_traveled_by_bot","Total_distance_traveled_by_all","H_Global_locational_Cost"]
with open('project_schneider.csv','w') as f1:
    writer=csv.writer(f1, delimiter=',',lineterminator='\n',)
    writer.writerow(firstrow)
    writer.writerows(all_values)
# #print(wijminushij1)
# # print(wijminushij2)
# # print(all_values)
# # with open('wij_minus_hij1.csv','w') as f2:
# #     writer=csv.writer(f2, delimiter=',',lineterminator='\n',)
# #     writer.writerow(['bot_1','bot_2','bot_3','bot_4','bot_5'])
# #     writer.writerows(wijminushij1)

# # with open('wij_minus_hij2.csv','w') as f2:
# #     writer=csv.writer(f2, delimiter=',',lineterminator='\n',)
# #     writer.writerow(['bot_6','bot_7','bot_8','bot_9','bot_10'])
# #     writer.writerows(wijminushij2)

with open('wij_minus_hij1.csv','w') as f2:
    writer=csv.writer(f2, delimiter=',',lineterminator='\n',)
    writer.writerow(['bot_1','bot_2'])
    writer.writerows(wijminushij1)

with open('wij_minus_hij2.csv','w') as f2:
    writer=csv.writer(f2, delimiter=',',lineterminator='\n',)
    writer.writerow(['bot_3','bot_4'])
    writer.writerows(wijminushij2)

# with open('wij_minus_hij3.csv','w') as f2:
#     writer=csv.writer(f2, delimiter=',',lineterminator='\n',)
#     writer.writerow(['bot_5','bot_6'])
#     writer.writerows(wijminushij3)

# with open('wij_minus_hij4.csv','w') as f2:
#     writer=csv.writer(f2, delimiter=',',lineterminator='\n',)
#     writer.writerow(['bot_7','bot_8'])
#     writer.writerows(wijminushij4)

# with open('wij_minus_hij5.csv','w') as f2:
#     writer=csv.writer(f2, delimiter=',',lineterminator='\n',)
#     writer.writerow(['bot_9','bot_10'])
#     writer.writerows(wijminushij5)


