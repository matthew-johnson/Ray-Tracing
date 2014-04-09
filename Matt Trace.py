__author__ = 'Matt'

import numpy as np
import scipy as sp

#Defining a wall with points
P = np.array([2,0,10])
Q = np.array([2,0,0])
R = np.array([2,5,10])

#Creating vectors from the points.
PQ = np.array(Q-P)
PR = np.array(R-P)

#Finding the normal vector of the plane.
normalv = np.cross(PQ, PR)
print 'normal vector = ' + str(normalv)
#Normalization of the normal vector.
n = normalv/np.linalg.norm(normalv)
print 'normalized = ' + str(n)

#Source position
source = np.array([3,3,3])
print 'Source position = ' + str(source)

#Mult-dimensional array of the points P, Q, and R multiplied element-wise by the normalized normal vector.
A = np.array([[(n[0]*P[0]),(n[1]*P[1]),(n[2]*P[2])],
            [(n[0]*Q[0]),(n[1]*Q[1]),(n[2]*Q[2])],
            [(n[0]*R[0]),(n[1]*R[1]),(n[2]*R[2])]])
B = np.array([-1,-1,-1])
print A
print B
#A least square estimation of the solution of the system of equations. Solving the form Ax = B where x = [1/D, 1/D, 1/D]
least_square = np.linalg.lstsq(A, B)
print least_square

#Parsing the result of the least square estimation to determine the plane constant D.
def find_D(lst_sq):
    for row in lst_sq:
        for entry in row:
            if entry != 0:
                D = 1/entry
                return D

D = find_D(least_square)

#Dot product of source position and normalized normal vector.
dot_product = np.dot(n, source)

#Calculation of
source_distance = dot_product + D
print 'Distance from source to plane = ' + str(source_distance)

source_image = (-1*2*source_distance*n)+source
print 'Image position = ' + str(source_image)
