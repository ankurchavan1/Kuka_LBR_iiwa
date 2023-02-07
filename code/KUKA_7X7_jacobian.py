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



# Second Part :: Inverse kinematics (Getting Jacobian)

T_7 = final_transform(theta1, theta2, theta3, theta4, theta5, theta6, theta7)[6]

T_6 = final_transform(theta1, theta2, theta3, theta4, theta5, theta6, theta7)[5]

T_5 = final_transform(theta1, theta2, theta3, theta4, theta5, theta6, theta7)[4]

T_3 = final_transform(theta1, theta2, theta3, theta4, theta5, theta6, theta7)[3]

T_4 = final_transform(theta1, theta2, theta3, theta4, theta5, theta6, theta7)[2]

T_2 = final_transform(theta1, theta2, theta3, theta4, theta5, theta6, theta7)[1]

T_1 = final_transform(theta1, theta2, theta3, theta4, theta5, theta6, theta7)[0]


# Extracting the last column i.e. translation of end effector wrt base frame
X_P = T_7[:3, 3]

# Taking partial derivative of the traslation wrt theta1 to theta7
X_P_diff1 = X_P.diff(theta1)
X_P_diff2 = X_P.diff(theta2)
X_P_diff3 = X_P.diff(theta3)
X_P_diff4 = X_P.diff(theta4)
X_P_diff5 = X_P.diff(theta5)
X_P_diff6 = X_P.diff(theta6)
X_P_diff7 = X_P.diff(theta7)

# Extracting the z column of all the transformation matrices wrt to base frame
Z_axis_1 = T_1[:3, 2]
Z_axis_2 = T_2[:3, 2]
Z_axis_3 = T_3[:3, 2]
Z_axis_4 = T_4[:3, 2]
Z_axis_5 = T_5[:3, 2]
Z_axis_6 = T_6[:3, 2]
Z_axis_7 = T_7[:3, 2]

# Defining the Jacobian Matrix
J = Matrix([[X_P_diff1, X_P_diff2, X_P_diff3, X_P_diff4, X_P_diff5, X_P_diff6, X_P_diff7],
    [Z_axis_1, Z_axis_2,Z_axis_3,  Z_axis_4, Z_axis_5, Z_axis_6, Z_axis_7]])

print("\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n")
#The final jacobian:
print('The final jacobian maatrix (7X6):')
pprint(J)


print("\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n")
# Defining the joint angles for home configuration:
theta_1_new = 3.14/2
theta_2_new = 0
theta_3_new = 0
theta_4_new = -3.14/2
theta_5_new = 0
theta_6_new = 3.14/2
theta_7_new = 0


#The jacobian for the above home position is:
print('The jacobian for the position before drawing is:')
pprint(J.subs([(theta1, theta_1_new), (theta2,theta_2_new),(theta3,theta_3_new), (theta4, theta_4_new), (theta5, theta_5_new), (theta6, theta_6_new), (theta7, theta_7_new)]))
print("\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n")


