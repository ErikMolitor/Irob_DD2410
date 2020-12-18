#! /usr/bin/env python3
import math 
import numpy as np
"""
    # Erik Molitor
    # emolitor@kth.se
"""

def scara_IK(point):
    x = point[0]
    y = point[1]
    z = point[2]
    q = [0.0, 0.0, 0.0]

    """
    #Fill in your IK solution here and return the three joint values in q
    """
    l0 = 0.07
    l1 = 0.3
    l2 = 0.35

    q2 = math.acos(((x-l0)**2+y**2-l1**2-l2**2)/(2*l1*l2))
    q1 = math.atan2(y,(x-l0))-math.atan2((l2*math.sin(q2)),(l1+l2*math.cos(q2)))
    
    q=[q1,q2,z]
    return q



def trans(q,alpha, r, d):
    T = np.array([[math.cos(q), -math.sin(q)*math.cos(alpha), math.sin(q)*math.sin(alpha), r*math.cos(q)],
        [math.sin(q), math.cos(q)*math.cos(alpha), -math.cos(q)*math.sin(alpha), r*math.sin(q)],
        [0, math.sin(alpha), math.cos(alpha), d],
        [0, 0, 0, 1]])
    return T 


def kuka_IK(point, R, joint_positions):
    x = point[0]
    y = point[1]
    z = point[2]
    q = joint_positions #it must contain 7 elements

    """
    #Fill in your IK solution here and return the seven joint values in q
    """
    # input values from assignment
    L = 0.4
    M = 0.39
    alpha = np.array([math.pi/2, -math.pi/2, -math.pi/2, math.pi/2, math.pi/2, -math.pi/2, 0])
    d = np.array([0.311, 0, L, 0, M, 0, 0.078])
    a = 0
    
    # base posistion 
    p0 = np.array([0,0,0])
    # z-axis 
    z0 = np.array([0,0,1])
    
    #transfer function for each joint
    T01 = trans(q[0], alpha[0], a, d[0])
    T12 = trans(q[1], alpha[1], a, d[1])
    T23 = trans(q[2], alpha[2], a, d[2])
    T34 = trans(q[3], alpha[3], a, d[3])
    T45 = trans(q[4], alpha[4], a, d[4])
    T56 = trans(q[5], alpha[5], a, d[5])
    T67 = trans(q[6], alpha[6], a, d[6])

    #transfer function from base to joint 
    T02 = np.dot(T01,T12)
    T03 = np.dot(T02,T23)
    T04 = np.dot(T03,T34)
    T05 = np.dot(T04,T45)
    T06 = np.dot(T05,T56)
    T07 = np.dot(T06,T67)

    #posistion of joint  r = Tp (p = vec1)
    vec1 = np.array([0,0,0,1])
 
    p1 = np.dot(T01,vec1)
    p2 = np.dot(T02,vec1)
    p3 = np.dot(T03,vec1)
    p4 = np.dot(T04,vec1)
    p5 = np.dot(T05,vec1)
    p6 = np.dot(T06,vec1)
    p7 = np.dot(T07,vec1)
    #remove last row
    p1 = p1[0:3]
    p2 = p2[0:3]
    p3 = p3[0:3]
    p4 = p4[0:3]
    p5 = p5[0:3]
    p6 = p6[0:3]
    p7 = p7[0:3] 

    #z-axis of rotation, just as with p 
    vec2 = np.array([0,0,1,0])

    z1 = np.dot(T01,vec2)
    z2 = np.dot(T02,vec2)
    z3 = np.dot(T03,vec2)
    z4 = np.dot(T04,vec2)
    z5 = np.dot(T05,vec2)
    z6 = np.dot(T06,vec2)
    z7 = np.dot(T07,vec2)

    #remove last row
    z1 = z1[0:3]
    z2 = z2[0:3]
    z3 = z3[0:3]
    z4 = z4[0:3]
    z5 = z5[0:3]
    z6 = z6[0:3]
    z7 = z7[0:3]

    #jacobi matrix colums then total matrix 
    J1 = np.concatenate((np.cross(z0,(p7-p0)),z0), axis=0)
    J2 = np.concatenate((np.cross(z1,(p7-p1)),z1), axis=0)
    J3 = np.concatenate((np.cross(z2,(p7-p2)),z2), axis=0)
    J4 = np.concatenate((np.cross(z3,(p7-p3)),z3), axis=0)
    J5 = np.concatenate((np.cross(z4,(p7-p4)),z4), axis=0)
    J6 = np.concatenate((np.cross(z5,(p7-p5)),z5), axis=0)
    J7 = np.concatenate((np.cross(z6,(p7-p6)),z6), axis=0)
    
    #total jacobi matix
    Jacobi_matrix = np.column_stack((J1,J2,J3,J4,J5,J6,J7))

    #pseudo inverse matrix by moore-penrose 
    inv_Jacobi = np.linalg.pinv(Jacobi_matrix)

    # R is the desired rotation of end effector frame
    Rd = np.array(R)
    nd = Rd[:,0]
    sd = Rd[:,1]
    ad = Rd[:,2]
    
    # acctual rotation of end effector frame
    T07_T = np.transpose(T07[0:3,0:3])
    ne = T07_T[:,0]
    se = T07_T[:,1]
    ae = T07_T[:,2]

    # eO = 1/2 (ne(q) x nd + se(q) x sd + ae(q) x ad)
    e_theta = 1/2*(np.cross(ne,nd) + np.cross(se,sd) + np.cross(ae,ad))
    angle = np.array([0,0,0])

    #make the desired vector and the end effector vector
    p_des = np.concatenate((point,e_theta))
    p_end_eff = np.concatenate((p7,angle))
    
    #error in X vector
    X_error = p_end_eff-p_des

    #error in q vector 
    q_error = np.dot(inv_Jacobi,X_error)

    #new q vector
    q = q-q_error

    #q = [q1,q2,q3,q4,q5,q6,q7]
    return q