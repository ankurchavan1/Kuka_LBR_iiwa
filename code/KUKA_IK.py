import matplotlib.pyplot as plt
from sympy import *
from mpl_toolkits.mplot3d import *
from sympy import *
import sympy as sp
import numpy as np

# For the IK we are going to fix the third joint hence theta3 is zero in the code below:

# To define required variables and symbols for the variables:
a, alpha, d, theta, theta1, theta2, theta3, theta4, theta5, theta6, theta7 = symbols('a alpha d theta theta1 theta2 theta3 theta4 theta5 theta6 theta7')

#Creating a function to iterate the elements of the transformation matrix so that we can creat successive transformation matrix for each link
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
def final_transform(theta1, theta2, theta4, theta5, theta6, theta7):
    T1 = calculation_terms(theta1 - (sp.pi/2), 0.36, 0, -(sp.pi)/2)
    T2 = calculation_terms(theta2 , 0 , 0, (sp.pi)/2)
    T3 = calculation_terms(0 , 0.41978 , 0, (sp.pi)/2)
    T4 = calculation_terms(theta4 , 0 , 0, -(sp.pi)/2)
    T5 = calculation_terms(theta5 , 0.39975, 0, -(sp.pi)/2)
    T6 = calculation_terms(theta6, 0, 0, (sp.pi)/2)
    T7 = calculation_terms(theta7, 0.12276, 0, 0)
        
    return [T1, T1 * T2, T1 * T2 * T3,  T1 * T2 * T3 * T4, T1 * T2 * T3 * T4 * T5, T1 * T2 * T3 * T4 * T5 * T6, T1 * T2 * T3 * T4 * T5 * T6 *T7]

#Defining the transformations of every frame with respect to the base frame.

T_7 = final_transform(theta1, theta2, theta4, theta5, theta6, theta7)[6]

T_6 = final_transform(theta1, theta2, theta4, theta5, theta6, theta7)[5]

T_5 = final_transform(theta1, theta2, theta4, theta5, theta6, theta7)[4]

T_4 = final_transform(theta1, theta2, theta4, theta5, theta6, theta7)[2]

T_2 = final_transform(theta1, theta2, theta4, theta5, theta6, theta7)[1]

T_1 = final_transform(theta1, theta2, theta4, theta5, theta6, theta7)[0]


print("\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n")
#Third Pose:
print("The final transformation matrix T(7-0) before drawing the circle: \n")
pprint(final_transform(sp.pi/2, 0, -sp.pi/2, 0, sp.pi/2, 0)[6])
print("\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n")


# Extracting the last column i.e. translation of end effector wrt base frame
X_P = T_7[:3, 3]

# Taking partial derivative of the traslation wrt theta1 to theta7
X_P_diff1 = X_P.diff(theta1)
X_P_diff2 = X_P.diff(theta2)
X_P_diff4 = X_P.diff(theta4)
X_P_diff5 = X_P.diff(theta5)
X_P_diff6 = X_P.diff(theta6)
X_P_diff7 = X_P.diff(theta7)

# Extracting the z column of all the transformation matrices wrt to base frame
Z_axis_1 = T_1[:3, 2]
Z_axis_2 = T_2[:3, 2]
Z_axis_4 = T_4[:3, 2]
Z_axis_5 = T_5[:3, 2]
Z_axis_6 = T_6[:3, 2]
Z_axis_7 = T_7[:3, 2]

# Defining the Jacobian Matrix
J = Matrix([[X_P_diff1, X_P_diff2, X_P_diff4, X_P_diff5, X_P_diff6, X_P_diff7],
    [Z_axis_1, Z_axis_2,Z_axis_4, Z_axis_5, Z_axis_6, Z_axis_7]])


# Initializing the time with 0
t = 0

# Defining the joint angles for home configuration
theta_1_new = 3.14/2
theta_2_new = 0
theta_4_new = -3.14/2
theta_5_new = 0
theta_6_new = 3.14/2
theta_7_new = 0


# Creating a matrix to store values of the end effector so that we can plot them
Theta_new = []
X_PLOT = []
Y_PLOT = []
Z_PLOT = []

#Creating joint angles array of the inverse kinematics
theta1_arr = [3.14/2]
theta2_arr = [0]
theta3_arr = [0]
theta4_arr = [-3.14/2]
theta5_arr = [0]
theta6_arr = [3.14/2]
theta7_arr = [0]


print("\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n")
print("The 6X6 Jacobian is: ")
pprint(J.subs([(theta1, theta_1_new), (theta2,theta_2_new), (theta4, theta_4_new), (theta5, theta_5_new), (theta6, theta_6_new), (theta7, theta_7_new)]))

print("\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n")

# Creating loop for 5 seconds
theta = np.linspace(90, 450, num=50)
fig = plt.figure(figsize=(10, 10))
ax = fig.add_subplot(111, projection='3d')

for t in theta:
  
    # Defining the velocities for next iteration
    x_dot = -0.1*2*(pi/5)*sin(t*(pi/180))
    y_dot = 0.1*2*(pi/5)*cos(t*(pi/180))
    

    # Defining matrix X_dot
    X_dot = Matrix([[x_dot], [y_dot], [0], [0], [0], [0]])  

    # Substituting the values of new joint angles in the translation column of transformation matrix of end effector wrt base frame
    T = X_P.subs([(theta1, theta_1_new), (theta2,theta_2_new), (theta4, theta_4_new), (theta5, theta_5_new), (theta6, theta_6_new), (theta7, theta_7_new)])

    # Extracting the translation along X direction
    X_new = T[0]
    # Extracting the translation along Y direction
    Y_new = T[1]
    # Extracting the translation along Z direction
    Z_new = T[2]
    ax.scatter(T[0], T[1],T[2])
    plt.xlim(0, 0.5)
    plt.ylim(0, 0.5)
    ax.set_zlim3d(0, 0.7)
    # Appending the values in array
    X_PLOT.append(X_new)
    Y_PLOT.append(Y_new)
    Z_PLOT.append(Z_new)


    # Substituting new joint angles in the jacobian for next iteration
    J_sub = J.subs([(theta1, (theta_1_new + 0.0000001)), (theta2, (theta_2_new + 0.0000001)), (theta4, (theta_4_new+ 0.0000001)), (theta5, (theta_5_new+ 0.0000001)), (theta6, (theta_6_new+ 0.0000001)), (theta7, (theta_7_new+ 0.0000001))])

    # Calculating inverse of jacobian
    J_inverse = J_sub.evalf().pinv()

    # Calculating theta_dot by inverse kinematic equation
    Theta_dot = (J_inverse @ X_dot).evalf()

   # Numerical integration to find new joint angles
    Theta_new = Theta_dot * 0.05

    theta_1_new += Theta_new[0]
    theta1_arr.append(theta_1_new)

    theta_2_new += Theta_new[1]
    theta2_arr.append(theta_2_new)

    theta3_arr.append(0)

    theta_4_new += Theta_new[2]
    theta4_arr.append(theta_4_new)

    theta_5_new += Theta_new[3]
    theta5_arr.append(theta_5_new)

    theta_6_new += Theta_new[4]
    theta6_arr.append(theta_6_new)

    theta_7_new += Theta_new[5]
    theta7_arr.append(theta_7_new)

ax.set_xlabel('Translation of along X Direction')
ax.set_ylabel('Translation of along Y Direction')
ax.set_zlabel('Translation of along Z Direction')
plt.title("Circle drawn by the end effector")
plt.show()