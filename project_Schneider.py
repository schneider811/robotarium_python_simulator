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


#Scenario 1
# N=10
# sensors=[1,2]
# robot_by_sensor=[1,1,1,1,1,2,2,2,2,2]
# numbots1=5
# numbots2=5
# binary_robot_by_sensor=[[1,1,1,1,1,0,0,0,0,0],[0,0,0,0,0,1,1,1,1,1]]
# n1=[1,2,3,4,5]
# n2=[6,7,8,9,10]
# hi1=[[1,1,1,1,1,0,0,0,0,0],[0,0,0,0,0,1,1,1,1,1]]
# wi1=[[1,1,1,1,1,0,0,0,0,0],[0,0,0,0,0,1,1,1,1,1]]
# initial_conditions = np.asarray([[1.25, 0.25, 0],[1, 0.5, 0],[1, -0.5, 0],[-1, -0.75, 0],[0.1, 0.2, 0],[0.2, -0.6, 0],[-0.75, -0.1, 0],[-1, 0, 0],[-0.8, -0.25, 0],[1.3, -0.4, 0]])
# gain=.2 #.1 gain causes HG to get pretty stable to be ~400

#scenario 2

# N=10
# gain=.3
# sensors=[1,2]
# robot_by_sensor=[1,1,1,1,1,2,2,2,2,2]
# numbots1=5
# numbots2=5
# binary_robot_by_sensor=[[1,1,1,1,1,0,0,0,0,0],[0,0,0,0,0,1,1,1,1,1]]
# n1=[1,2,3,4,5]
# n2=[6,7,8,9,10]
# hi1=[[1,1,2,1,1,0,0,0,0,0],[0,0,0,0,0,2,1,1,1,2]]
# wi1=[[2,1,1,1,1,0,0,0,0,0],[0,0,0,0,0,2,1,1,1,1]]
# initial_conditions = np.asarray([[1.25, 0.25, 0],[1, 0.5, 0],[1, -0.5, 0],[-1, -0.75, 0],[0.1, 0.2, 0],[0.2, -0.6, 0],[-0.75, -0.1, 0],[-1, 0, 0],[-0.8, -0.25, 0],[1.3, -0.4, 0]])



# #Scenario 3
# N=10
# gain=.1
# sensors=[1,2,3,4,5]
# robot_by_sensor=[1,1,2,2,3,3,4,4,5,5]
# numbots1=5
# numbots2=5
# binary_robot_by_sensor=[[1,1,0,0,0,0,0,0,0,0],[0,0,1,1,0,0,0,0,0,0],[0,0,0,0,1,1,0,0,0,0],[0,0,0,0,0,0,1,1,0,0],[0,0,0,0,0,0,0,0,1,1]]
# n1=[1,2,3,4,5]
# n2=[6,7,8,9,10]
# hi1=[[1,2,0,0,0,0,0,0,0,0],[0,0,2,1,0,0,0,0,0,0],[0,0,0,0,1,2,0,0,0,0],[0,0,0,0,0,0,2,1,0,0],[0,0,0,0,0,0,0,0,1,2]]
# wi1=[[1,2,0,0,0,0,0,0,0,0],[0,0,2,1,0,0,0,0,0,0],[0,0,0,0,1,2,0,0,0,0],[0,0,0,0,0,0,2,1,0,0],[0,0,0,0,0,0,0,0,1,2]]
# initial_conditions = np.asarray([[1.25, 0.25, 0],[1, 0.5, 0],[1, -0.5, 0],[-1, -0.75, 0],[0.1, 0.2, 0],[0.2, -0.6, 0],[-0.75, -0.1, 0],[-1, 0, 0],[-0.8, -0.25, 0],[1.3, -0.4, 0]])


#scenario 4
N=5
gain=.5
sensors=[1,2]
robot_by_sensor=[1,1,1,2,2]
numbots1=5
numbots2=5
binary_robot_by_sensor=[[1,1,1,0,0],[0,0,1,1,1]]
n1=[1,2,3]
n2=[3,2,1]
hi1=[[1,1,1,0,0],[0,0,1,1,2]]
wi1=[[1,1,1,0,0],[0,0,1,2,2]]
initial_conditions = np.asarray([[1.25, 0.25, 0],[1, 0.5, 0],[1, -0.5, 0],[-1, -0.75, 0],[0.1, 0.2, 0]])


# #Scenario 5
# N=5
# gain=.5
# sensors=[1,2]
# robot_by_sensor=[1,1,1,2,2]
# numbots1=5
# numbots2=5
# binary_robot_by_sensor=[[1,1,1,0,0],[0,0,1,1,1]]
# n1=[1,2,3]
# n2=[3,4,5]
# hi1=[[1,1,2,0,0],[0,0,1,1,1]]
# wi1=[[1,2,1,0,0],[0,0,1,1,1]]
# initial_conditions = np.asarray([[1.25, 0.25, 0],[1, 0.5, 0],[1, -0.5, 0],[-1, -0.75, 0],[0.1, 0.2, 0]])

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


# for label_by_sensor in robot_by_sensor:
#         count+=1
#         sensor_label = r.axes.text(x[0,0],x[1,0]+0.25,label_by_sensor,fontsize=font_height_points, color='r',fontweight='bold',horizontalalignment='center',verticalalignment='center',zorder=0)
#         x.append(sensor_label)

robot_marker_size_m = 0.1

wijminushij= [[0 for col in range(N)] for row in range(len(sensors))]

marker_size_robot = determine_marker_size(r, robot_marker_size_m)
font_size = determine_font_size(r,0.05)
line_width = 5
robot_sensor_labels=[]
for ii in range(x.shape[1]):
    #sensor_label = r.axes.scatter(x[0,ii],x[1,ii]+0.25,robot_by_sensor[ii],fontsize=.1, color='r',fontweight='bold',horizontalalignment='center',verticalalignment='center',zorder=0)
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


#robot_markers = [r.axes.scatter(x[0,ii], x[1,ii], s=marker_size_robot, marker='o', facecolors='none',edgecolors=CM[ii,:],linewidth=line_width) for ii in range(goal_points.shape[1])]

r.step()
weighted_mass= [[0 for col in range(3)] for row in range(N)]

all_values=[]
distance_traveled=[0]*N
old_traveled=[0]*N
previous_x_si=x_si

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

    if len(sensors) == 1: 
        for xi in np.arange(x_min_robotarium,x_max_robotarium,res):
                for yi in np.arange(y_min_robotarium,y_max_robotarium,res):
                    sensor_value = 1
                    distances = np.zeros((N))
                    for robot in range(N):  
                        distances[robot] =  (np.square(xi - x_si[0,robot]) + np.square(yi - x_si[1,robot]))-wi1[robot]
                        # if not w[robot] == 0:
                        #     hg += np.sqrt(np.square(x_si[0,robot]-(c_v[robot][0]/(w[robot]))) + np.square(x_si[1,i]-(c_v[robot][1]/(w[robot]) ) )) 
                    min_index = np.argmin(distances)
                    #print(min_index)
                    #c_v is value for grid cell c_v=np.zeros(2,N)
                    #w = np.zeros(N,1)
                    c_v[min_index][0] += (xi * sensor_value)
                    c_v[min_index][1] += (yi * sensor_value)
                    
                    w[min_index] += sensor_value
                    # for robot in range(N):
                    #     if not w[robot] == 0:
                    #         hg += np.sqrt(np.square(x_si[0,robot]-(c_v[robot][0]/(w[robot]))) + np.square(x_si[1,i]-(c_v[robot][1]/(w[robot]) ) ))
                    #         print(hg)
                    #print(w)
#     #print(w)
    #print(w)
    a=1
    #L=completeGL(N)
    #print(L)
    #dxi = single_integrator_position_controller(x_si, c_v_array[:2][:])


    if len(sensors) >1:
        #for types in sensors:
        w = [0]*N
        c_v= [[0 for col in range(3)] for row in range(N)]            
        for xi in np.arange(x_min_robotarium,x_max_robotarium,res):
                for yi in np.arange(y_min_robotarium,y_max_robotarium,res):
                    sensor_value = 1
                    distances = np.zeros((len(sensors),N))
                    for k in range(len(sensors)):
                        for robot in range(N):
                            if binary_robot_by_sensor[k][robot]==1:
                                distances[k][robot] =  (np.square(xi - x_si[0,robot]) + np.square(yi - x_si[1,robot])) - wi1[k][robot]
                        min_index = np.argmin(distances[k])
                        c_v[min_index][0] += (xi * sensor_value)
                        c_v[min_index][1] += (yi * sensor_value)
                        w[min_index] += sensor_value
        x_sample=0
        y_sample=0

        hg=0
        #Equation 4 from paper 1 while calculating over weighted voronoi cell (lines 277-278)
        for xi in np.arange(x_min_robotarium,x_max_robotarium,res):
            for yi in np.arange(y_min_robotarium,y_max_robotarium,res):
                    for k in range(len(sensors)):
                        distances3=[0]*N
                        for robot in range(N):
                                    if binary_robot_by_sensor[k][robot] == 1:
                                        distances3[robot] = (np.square(xi - x_si[0,robot]) + np.square(yi - x_si[1,robot])) - wi1[k][robot]
                                #    print(distances3)
                        # mask = (distances3[:] > 0)
                        k2=np.argmin(distances3[distances3 != 0])
                        density=get_sensor(1,[x_sample,y_sample])
                        #hg += ((np.square(x_si[0,k2]-xi) + np.square(x_si[1,k2]-yi ))-wi1[k][k2])*density
                        #hg += ((np.square(x_si[0,k2]-(c_v[k2][0]/w[k2])) + np.square(x_si[1,k2]-(c_v[k2][1]/w[k2]) ))-wi1[k][k2])*density
                        hg += distances3[k2]*density
#LINES 279 swapped to 280
#
#
#
#
    #print(hg)


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
        # print("c_x",c_x)
        # print("current_x",current_x)
        #Below is equation 8 from paper 2
        dxi[:,robot] = [a*(c_x - current_x), a*(c_y - current_y )] 
    brake_check = np.around(dxi,2)
    brake_check = not np.any(brake_check)

    #Equation 14
    sum_of_neighbors_matrix= [[0 for col in range(len(sensors))] for row in range(N)]

    weight_array =[0]*N
    # #EQ 14 from PAPER 2
    #updating the weights they do not converge to health

    for robots in range(N):
        #sum_of_neighbors=0
        z=topological_neighbors(L,robot)
        for k in range(len(sensors)):
            for neighbors in z:
                #print(robots)
                #print(k)
                #print(neighbors)
                sum_of_neighbors_matrix[robots][k]+=hi1[k][robots]-hi1[k][neighbors]
    #print(sum_of_neighbors)
        sum_of_neighbors_array=np.array(sum_of_neighbors_matrix, dtype=int)
        if not w[robots] == 0:  
            weight_array[robots]=(gain/(2*w[robots]))*sum_of_neighbors_array[robots]
    #print("weight array ",weight_array)
    # weight_array=gain/(2*density)*sum_of_neighbors_array
    

    #print('wi1',wi1)
    for sensors_num in range(len(sensors)):
        for robot in range(N):
            # if weight_array[robot][sensors_num]<0:
            #     wi1[sensors_num][robot] = 0
            # else:
                wi1[sensors_num][robot] = wi1[sensors_num][robot]+ weight_array[robot][sensors_num]
                wijminushij[sensors_num][robot]=wi1[sensors_num][robot]-hi1[sensors_num][robot]

    #normalize weight array

    print('wi1 after changing',wi1)
    values_to_write.append(deepcopy(wi1))
    values_to_write.append(deepcopy(wijminushij))
    cwij=[c_v[robot][0]/(w[robot]),c_v[robot][1]/(w[robot])]
    values_to_write.append(cwij)
    #print(values_to_write)
    
    #print(wi1)
    
    if brake_check:#and convergence of wij-hij:
         print("Simulation has ended due to brake checking movement of robots at iteration: ",iteration)
         break
    # Create safe control inputs (i.e., no collisions)
    #dxi = si_barrier_cert(dxi, x_si)
    if iterations == 999:
         print("Simulation has ended due to hitting 1,000 iterations")
    # Transform single integrator velocity commands to unicycle
    dxu = si_to_uni_dyn(dxi, x)
    for i in range(N):
        distance_traveled[i]+=np.linalg.norm((x_si[:,i])-(previous_x_si[:,i]))

    #Why is this distance traveled working so poorly?
    #
    #
    #
    #

    #print(distance_traveled)
    #old_traveled=distance_traveled
    

    #this print debug statement is working as I would hope it would work but writing the distance traveled to values_to_write only is writing the total distance traveled for each
    # print(distance_traveled)

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


print(x_si)

firstrow=["Iteration","Pose_x_y_Theta","Wij","Wij_minus_Hij","Cwij","Distance_traveled_by_bot","Total_distance_traveled_by_all","H_Global_locational_Cost"]
with open('project_schneider.csv','w') as f1:
    writer=csv.writer(f1, delimiter=',',lineterminator='\n',)
    writer.writerow(firstrow)
    writer.writerows(all_values)

# df=pd.read_csv('project_schneider.csv')

# df_2 = df.groupby(['Iteration'])
# df_2.plot(y="H:Global locational Cost")

#r.call_at_scripts_end()    
