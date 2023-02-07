from sympy import *
import sympy as sp
import numpy as np
import matplotlib.pyplot as plt
# import random as rd

# First Part:: Get the Homogenous Transformation Matrices for Forward Kinematics:

#To print unicode characters for mathematical expressions:
init_printing(use_unicode=True, wrap_line=False)


# To define required variables and symbols for the variables:
a, alpha, d, theta, theta1, theta2, theta3, theta4, theta5, theta6, theta7 = symbols('a alpha d theta theta1 theta2 theta3 theta4 theta5 theta6 theta7')

#Creating a function to iterate the elements of the transformation matrix so that we can creat successive transformation matrix for each link:
def calculation_terms(theta, d, a, alpha):
    A_11 = sp.cos(theta)
    A_12 = -sp.sin(theta)*sp.cos(alpha)
    A_13 = sp.sin(theta)*sp.sin(alpha)
    A_14 = a*sp.cos(theta)
    A_21 = sp.sin(theta)
    A_22 = sp.cos(theta)*sp.cos(alpha)
    A_23 = -sp.cos(theta)*sp.sin(alpha)
    A_24 = a*sp.sin(theta)
    A_31 = 0
    A_32 = sp.sin(alpha)
    A_33 = sp.cos(alpha)
    A_34 = d

    return Matrix([[A_11, A_12, A_13, A_14], [A_21, A_22, A_23,A_24], [A_31, A_32, A_33, A_34], [0,0,0,1]])


#Defining the function to compute the final transformation matrix that maps each link with frame 0:
def final_transform(theta1, theta2, theta3, theta4, theta5, theta6, theta7):
    T1 = calculation_terms(theta1 - (sp.pi/2), 0.36, 0, -(sp.pi)/2)
    T2 = calculation_terms(theta2 , 0 , 0, (sp.pi)/2)
    T3 = calculation_terms(theta3 , 0.41978 , 0, (sp.pi)/2)
    T4 = calculation_terms(theta4 , 0 , 0, -(sp.pi)/2)
    T5 = calculation_terms(theta5 , 0.39975, 0, -(sp.pi)/2)
    T6 = calculation_terms(theta6, 0, 0, (sp.pi)/2)
    T7 = calculation_terms(theta7, 0.12276, 0, 0)

    return [T1, T1 * T2, T1 * T2 * T3,  T1 * T2 * T3 * T4, T1 * T2 * T3 * T4 * T5, T1 * T2 * T3 * T4 * T5 * T6, T1 * T2 * T3 * T4 * T5 * T6 *T7]


# Defining tranformation matrices

T_70 = final_transform(theta1, theta2, theta3, theta4, theta5, theta6, theta7)[6]

T_60 = final_transform(theta1, theta2, theta3, theta4, theta5, theta6, theta7)[5]

T_50 = final_transform(theta1, theta2, theta3, theta4, theta5, theta6, theta7)[4]

T_30 = final_transform(theta1, theta2, theta3, theta4, theta5, theta6, theta7)[3]

T_40 = final_transform(theta1, theta2, theta3, theta4, theta5, theta6, theta7)[2]

T_20 = final_transform(theta1, theta2, theta3, theta4, theta5, theta6, theta7)[1]

T_10 = final_transform(theta1, theta2, theta3, theta4, theta5, theta6, theta7)[0]


# NO of trial points while plotting the workspace
trial_points = 200

# Generating random valies of theta1 and theta2 to plot the workspace.
theta_1_arr = np.random.uniform(-(np.pi/180)* 170 ,(np.pi/180)* 170,trial_points)
theta_2_arr = np.random.uniform(-(np.pi/180)* 120 ,(np.pi/180)* 120,trial_points)

#To plot the trajectory of the end effector
plot_trajectory = plt.figure(figsize=(8, 8))
plot_3d = plot_trajectory.add_subplot(111, projection='3d')

X_plot_ws1 = []
Y_plot_ws1 = []
Z_plot_ws1 = []

X_plot_ws2 = []
Y_plot_ws2 = []
Z_plot_ws2 = []

X_plot_ws3 = []
Y_plot_ws3 = []
Z_plot_ws3 = []

X_plot_ws4 = []
Y_plot_ws4 = []
Z_plot_ws4 = []

X_plot_ws5 = []
Y_plot_ws5 = []
Z_plot_ws5 = []

for i in range(0,trial_points):

    T_7 = T_70.subs({theta1: 0, theta2: theta_2_arr[i], theta3: 0, theta4: 0, theta5: 0, theta6: 0, theta7: 0})
    X_plot_ws1.append(T_7[3])
    Y_plot_ws1.append(T_7[7])
    Z_plot_ws1.append(T_7[11])


for i in range(0,trial_points):

    T_7 = T_70.subs({theta1: np.pi/6, theta2: theta_2_arr[i], theta3: 0, theta4: 0, theta5: 0, theta6: 0, theta7: 0})
    X_plot_ws2.append(T_7[3])
    Y_plot_ws2.append(T_7[7])
    Z_plot_ws2.append(T_7[11])
   
for i in range(0,trial_points):

    T_7 = T_70.subs({theta1: np.pi/3, theta2: theta_2_arr[i], theta3: 0, theta4: 0, theta5: 0, theta6: 0, theta7: 0})
    X_plot_ws3.append(T_7[3])
    Y_plot_ws3.append(T_7[7])
    Z_plot_ws3.append(T_7[11])

for i in range(0,trial_points):

    T_7 = T_70.subs({theta1: -np.pi/6, theta2: theta_2_arr[i], theta3: 0, theta4: 0, theta5: 0, theta6: 0, theta7: 0})
    X_plot_ws4.append(T_7[3])
    Y_plot_ws4.append(T_7[7])
    Z_plot_ws4.append(T_7[11])

for i in range(0,trial_points):

    T_7 = T_70.subs({theta1: np.pi/2, theta2: theta_2_arr[i], theta3: 0, theta4: 0, theta5: 0, theta6: 0, theta7: 0})
    X_plot_ws4.append(T_7[3])
    Y_plot_ws4.append(T_7[7])
    Z_plot_ws4.append(T_7[11])

for i in range(0,trial_points):

    T_7 = T_70.subs({theta1: theta_1_arr[i] , theta2:-(np.pi/180)* 120, theta3: 0, theta4: 0, theta5: 0, theta6: 0, theta7: 0})
    X_plot_ws5.append(T_7[3])
    Y_plot_ws5.append(T_7[7])
    Z_plot_ws5.append(T_7[11])

#Plotting the trajectory:

i = 0

while (i < trial_points):
    plot_3d.scatter(X_plot_ws1, Y_plot_ws1, Z_plot_ws1, c="red")
    plot_3d.scatter(X_plot_ws2, Y_plot_ws2, Z_plot_ws2, c="orange")
    plot_3d.scatter(X_plot_ws3, Y_plot_ws3, Z_plot_ws3, c="green")
    plot_3d.scatter(X_plot_ws4, Y_plot_ws4, Z_plot_ws4, c="blue")
    plot_3d.scatter(X_plot_ws5, Y_plot_ws5, Z_plot_ws5, c="black")
    plt.pause(0.001)
    i += 1
plt.show()


























