import numpy as np
import rps.robotarium as robotarium
from rps.utilities.transformations import *
from rps.utilities.graph import *
from rps.utilities.barrier_certificates import *
from rps.utilities.misc import *
from rps.utilities.controllers import *
import time
from scipy.spatial import Voronoi, voronoi_plot_2d


# c_v is value for grid cell c_v=np.zeros(2,N)
# w = np.zeros(N,1)

def get_sensor(j,q):
     x=1

N=10
weights = [2,1,1,1,1,2,1,1,1,1]

initial_conditions = np.asarray([[1.25, 0.25, 0],[1, 0.5, 0],[1, -0.5, 0],[-1, -0.75, 0],[0.1, 0.2, 0],[0.2, -0.6, 0],[-0.75, -0.1, 0],[-1, 0, 0],[-0.8, -0.25, 0],[1.3, -0.4, 0]])
r = robotarium.Robotarium(number_of_robots=N,sim_in_real_time=True,initial_conditions=initial_conditions[0:N].T)

#while reading the paper I assume sensor health is equivalent to 

sensor_health = []

#Scenario 0

# sensors=[1]
# robot_by_sensor=[1,1,1,1,1,1,1,1,1,1]
# hi1=[[1,1,1,1,1],[1,1,1,1,1]]
# wi1=[[1,1,1,1,1],[1,1,1,1,1]]

#Scenario 1

sensors=[1,2]
robot_by_sensor=[1,1,1,1,1,2,2,2,2,2]
numbots1=5
numbots2=5
n1=[1,2,3,4,5]
n2=[6,7,8,9,10]
hi1=[[1,1,1,1,1],[1,1,1,1,1]]
wi1=[[1,1,1,1,1],[1,1,1,1,1]]

#scenario 2

# s=[1,2]
# n1=[1,2,3,4,5]
# n2=[6,7,8,9,10]
# hi1=[1,1,1,1,1]
# hi2=[1,1,1,1,1]
# wi1=[1,1,1,1,1,1,1,1,1,1]

# #Scenario 3

# sensors=[1,2,3,4,5]
# robot_by_sensor=[1,1,2,2,3,3,4,4,5,5]
# hi1=[1,2,2,1,1,2,2,1,1,2]
# wi1=[1,2,2,1,1,2,2,1,1,2]

#scenario 4

#Scenario 5

#r = robotarium.Robotarium(number_of_robots=N, show_figure=True, initial_conditions=initial_conditions, sim_in_real_time=False)

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
h=[0]*N
c_v= [[0 for col in range(3)] for row in range(N)]

print(c_v)

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

print(range(5))

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
    robot_number_labels.append(r.axes.text(x[0,0],x[1,xx]+0.25,str(xx),fontsize=font_size, color='r',fontweight='bold',horizontalalignment='center',verticalalignment='center',zorder=0))


#robot_markers = [r.axes.scatter(x[0,ii], x[1,ii], s=marker_size_robot, marker='o', facecolors='none',edgecolors=CM[ii,:],linewidth=line_width) for ii in range(goal_points.shape[1])]

r.step()

L=completeGL(N)

iterations = 1000
#current_x and current_y = getposes
#while True:
for k in range(iterations):
    L=completeGL(N)

    dxi = np.zeros((2, N))
    # Get poses of agents
    x = r.get_poses()
    x_si = uni_to_si_states(x)
    for i in range(x.shape[1]):
        #print(i)
        robot_sensor_labels[i].set_offsets(x[:2,i].T)
        robot_number_labels[i].set_position([x_si[0,i],x_si[1,i]+0.15])
        # This updates the marker sizes if the figure window size is changed. 
        # This should be removed when submitting to the Robotarium.
        robot_sensor_labels[i].set_sizes([determine_marker_size(r, robot_marker_size_m)])

    # print(x)
    # print(current_x)
    # print(current_y)

    #This is what I initally tried as what i had hoped was a simple and elegant solution. Decided to scrap it after all robots were just constantly converging to center of graph
    # if len(sensors) != 1:
    #     for types in sensors:
    #         for xi in np.arange(x_min_robotarium,x_max_robotarium,res):
    #                 for yi in np.arange(y_min_robotarium,y_max_robotarium,res):
    #                     sensor_value = 1
    #                     distances = np.zeros((N))
    #                     for bot_s_by_sensor in robot_by_sensor:
    #                         robot=counting
    #                         if bot_s_by_sensor == types:
    #                             distances[robot] =  np.sqrt(np.square(xi - x_si[0,robot])*weights[robot] + np.square(yi - x_si[1,robot])*weights[robot] )
    #                         counting+=1 
    #                     counting=0
    #                     min_index = np.argmin(distances)

    #                     #c_v is value for grid cell c_v=np.zeros(2,N)
    #                     #w = np.zeros(N,1)
    #                     c_v[min_index][0] += (xi * sensor_value)
    #                     c_v[min_index][1] += (yi * sensor_value)
    #                     w[min_index] += sensor_value

    if len(sensors) == 2 and numbots1 == 5 and numbots2 == 5:
        for types in sensors:
            for xi in np.arange(x_min_robotarium,x_max_robotarium,res):
                    for yi in np.arange(y_min_robotarium,y_max_robotarium,res):
                        sensor_value = 1
                        distances1 = np.zeros(numbots1)
                        distances2 = np.zeros(numbots2)
                        if types == 1:
                            for robot in range(numbots1):
                                distances1[robot] =  np.sqrt(np.square(xi - x_si[0,robot])*weights[robot] + np.square(yi - x_si[1,robot])*weights[robot] ) 
                        if types == 2:
                            for robot in range(numbots2):
                                distances2[robot] =  np.sqrt(np.square(xi - x_si[0,robot+5])*weights[robot+5] + np.square(yi - x_si[1,robot+5])*weights[robot+5] ) 
                        counting=0
                        min_index1 = np.argmin(distances1)
                        min_index2 = np.argmin(distances2)
                        c_v[min_index1][0] += (xi * sensor_value)
                        c_v[min_index1][1] += (yi * sensor_value)
                        w[min_index1] += sensor_value

                        c_v[min_index2+5][0] += (xi * sensor_value)
                        c_v[min_index2+5][1] += (yi * sensor_value)
                        w[min_index2+5] += sensor_value
    # # was what i initially did to figure out how to separate out the robots based on sensor it did not work as well as above lines  this looks to work decently well
    # if len(sensors) == 2 and numbots1 == 5 and numbots2 == 5:
    #     for types in sensors:
    #         if types == 1: 
    #             for xi in np.arange(x_min_robotarium,x_max_robotarium,res):
    #                     for yi in np.arange(y_min_robotarium,y_max_robotarium,res):
    #                         sensor_value = .5
    #                         distances = np.zeros(numbots1)
    #                         for robot in range(numbots1):  
    #                                 distances[robot] =  np.sqrt(np.square(xi - x_si[0,robot])*weights[robot] + np.square(yi - x_si[1,robot])*weights[robot] ) 
    #                         min_index = np.argmin(distances)
    #                         #print(min_index)
    #                         #c_v is value for grid cell c_v=np.zeros(2,N)
    #                         #w = np.zeros(N,1)
    #                         c_v[min_index][0] += (xi * sensor_value)
    #                         c_v[min_index][1] += (yi * sensor_value)
    #                         w[min_index] += sensor_value
    #         if types == 2:
    #             for xi in np.arange(x_min_robotarium,x_max_robotarium,res):
    #                     for yi in np.arange(y_min_robotarium,y_max_robotarium,res):
    #                         sensor_value = .5
    #                         distances2 = np.zeros((numbots2))
    #                         for robot in range(numbots2):  
    #                                 distances2[robot] =  np.sqrt(np.square(xi - x_si[0,robot+5])*weights[robot+5] + np.square(yi - x_si[1,robot+5])*weights[robot+5] ) 
    #                         #print(distances2)
    #                         min_index = np.argmin(distances2)
    #                         # print(min_index)
    #                         #c_v is value for grid cell c_v=np.zeros(2,N)
    #                         #w = np.zeros(N,1)
    #                         c_v[min_index+5][0] += (xi * sensor_value)
    #                         c_v[min_index+5][1] += (yi * sensor_value)
    #                         w[min_index+5] += sensor_value  
    #minimum and maximum x and y points for robotarium, resolution.
    if len(sensors) == 1: 
        for xi in np.arange(x_min_robotarium,x_max_robotarium,res):
                for yi in np.arange(y_min_robotarium,y_max_robotarium,res):
                    sensor_value = .5
                    distances = np.zeros((N))
                    for robot in range(N):  
                            distances[robot] =  np.sqrt(np.square(xi - x_si[0,robot])*weights[robot] + np.square(yi - x_si[1,robot])*weights[robot] ) 
                    min_index = np.argmin(distances)
                    #print(min_index)
                    #c_v is value for grid cell c_v=np.zeros(2,N)
                    #w = np.zeros(N,1)
                    c_v[min_index][0] += (xi * sensor_value)
                    c_v[min_index][1] += (yi * sensor_value)
                    w[min_index] += sensor_value
                    #print(w)
#     #print(w)
    
    a=1
    #L=completeGL(N)
    #print(L)
    #dxi = single_integrator_position_controller(x_si, c_v_array[:2][:])
    
    for robot in range(N):
        current_x=x_si[0,robot]
        current_y=x_si[1,robot]
        if not w[robot] == 0:
            c_x = c_v[robot][0]/(w[robot])
            c_y = c_v[robot][1]/(w[robot])
            # print("c_x :",c_x)
            # print("c_y :",c_y)
        else:
            c_x = c_y = 0
        dxi[:,robot] = [a*(c_x - current_x), a*(c_y - current_y )] 

    #Equation 4 paper 1

    # for j in sensors:
    #      for i in range(robot_by_sensor):
    #           #degrade with the square of the distance (same as example in paper)
    #           if j == robot_by_sensor[i]:
    #                w[i] = 


    # Create safe control inputs (i.e., no collisions)
    #dxi = si_barrier_cert(dxi, x_si)

    # Transform single integrator velocity commands to unicycle
    dxu = si_to_uni_dyn(dxi, x)
    counting=0
    # Set the velocities by mapping the single-integrator inputs to unciycle inputs
    r.set_velocities(np.arange(N), dxu)
    # Iterate the simulation
    r.step()
