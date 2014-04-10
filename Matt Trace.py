__author__ = 'Matt'

import numpy as np
import scipy as sp

#Defining a wall with points
P = np.array([2,0,10])
Q = np.array([2,0,0])
R = np.array([4,5,10])
Wall1 = [P, Q, R]


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
source = np.array([7,4,4])
receiver = np.array([-5,-5,20])
source_vector = np.array(source-Q)
print 'Source position = ' + str(source)

#Mult-dimensional array of the points P, Q, and R multiplied element-wise by the normalized normal vector.
'''
A = np.array([[(n[0]*P[0]),(n[1]*P[1]),(n[2]*P[2])],
            [(n[0]*Q[0]),(n[1]*Q[1]),(n[2]*Q[2])],
            [(n[0]*R[0]),(n[1]*R[1]),(n[2]*R[2])]])
B = np.array([-1,-1,-1])

print A
print B
'''

#A least square estimation of the solution of the system of equations. Solving the form Ax = B where x = [1/D, 1/D, 1/D]
'''
least_square = np.linalg.lstsq(A, B)
print least_square
'''
#Parsing the result of the least square estimation to determine the plane constant D.
'''
def find_D(lst_sq):
    for row in lst_sq:
        for entry in row:
            if entry != 0:
                D = 1/entry
                return D

D = find_D(least_square)
'''

#Dot product of source position and normalized normal vector.
source_distance = np.dot(n, source_vector)

print 'Distance from source to plane = ' + str(source_distance)

image = source - (2*source_distance*n)
print 'Image position = ' + str(image)

print receiver - image
print PQ
print PR

#If the determinant below is = 0, then the three vectors are linearly dependent meaning the line does not intersect the plane and the reflection is not valid.
check_reflection = np.linalg.det(np.array([receiver - image, PQ, PR]))
print 'Determinant = ' + str(check_reflection)
if check_reflection != 0:
    print 'Valid reflection.'
else:
    print 'Invalid reflection.'

#Solving for the intersection point of the line from image to reciever and the plane PQR. http://en.wikipedia.org/wiki/Line%E2%80%93plane_intersection
A = np.array([
    [image[0]-receiver[0], PQ[0], PR[0]],
    [image[1]-receiver[1], PQ[1], PR[1]],
    [image[2]-receiver[2], PQ[2], PR[2]]
])

B = np.array([
    image[0] - P[0], image[1] - P[1], image[2] - P[2]
])

#Using linalg.solve to solve for the solution vector x from the form Ax = B. x is in the form [t, u, v].
x = np.linalg.solve(A, B)
print x
intersect_mag = x[0]
print intersect_mag

#Intersection point in form Ia + (Ib - Ia)t where Ia is the image position and Ib is the receiver position.
intersect = image + (receiver - image)*intersect_mag
print 'Reflection point = ' + str(intersect)
