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


print("\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n")
#T1
print("The Transfomration matrix T1(1-0): \n")
pprint(calculation_terms(theta1 - (sp.pi/2), 0.36, 0, -(sp.pi)/2))

print("\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n")
#T2
print("The Transfomration matrix T2(2-1): \n")
pprint(calculation_terms(theta2 , 0 , 0, (sp.pi)/2))

print("\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n")
#T3
print("The Transfomration matrix T3(3-2): \n")
pprint(calculation_terms(theta3 , 0.41978 , 0, (sp.pi)/2))

print("\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n")
#T4
print("The Transfomration matrix T4(4-3): \n")
pprint(calculation_terms(theta4 , 0 , 0, -(sp.pi)/2))

print("\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n")
#T5
print("The Transfomration matrix T5(5-4): \n")
pprint(calculation_terms(theta5 , 0.39975, 0, -(sp.pi)/2))

print("\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n")
#T6
print("The Transfomration matrix T6(6-5): \n")
pprint(calculation_terms(theta6, 0, 0, (sp.pi)/2))

print("\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n")
#T7
print("The Transfomration matrix T7(7-6): \n")
pprint(calculation_terms(theta7, 0.12276, 0, 0))

#Poses to varify the tranfomration matrix

print("\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n")
#Defining the Homogeneous transformations from frame i to frame 0: here i= 1 to 7:
print('The Homogeneous tranformation matrix from frame 7 to frame 0 with all thetas as zero:')
pprint(final_transform(0, 0, 0, 0, 0, 0, 0)[6])

print("\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n")
#First Pose:
print("The final transformation matrix T(7-0) Pose 1: \n")
pprint(final_transform(0, 0, sp.pi/2, sp.pi/2, 0, -sp.pi/2, 0)[6])
print("\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n")

print("\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n")
#Second Pose:
print("The final transformation matrix T(7-0) Pose 2: \n")
pprint(final_transform(sp.pi/2, sp.pi/2, 0, sp.pi/2, 0, sp.pi/2, 0)[6])
print("\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n")

print("\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n")
#Third Pose:
print("The final transformation matrix T(7-0) Pose 3: \n")
pprint(final_transform(sp.pi/2, 0, 0, sp.pi/2, 0, sp.pi/2, 0)[6])
print("\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n")

print("\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n")
#Defining the Homogeneous transformations from frame i to frame 0: here i= 1 to 7 with thetas as variables:
print('The Homogeneous tranformation matrix from frame 7 to frame 0 with the all the thetas as variables:')
pprint((final_transform(theta1, theta2, theta3, theta4, theta5, theta6, theta7)[6]))

