__author__ = 'Matt'

import numpy as np

#Defining a wall with points

P = np.array([0, 0, 0])
Q = np.array([0, 10, 0])
R = np.array([10, 0, 0])
Wall1 = [P, Q, R]

# Integer to define how accurate we want the calc to be. Might need to be adjusted based on units being used like ft vs cm
round_number = 4

# Check var for if reflection exists
valid_reflection = False

# List of valid reflection times
valid_time = []


#Creating vectors from the points.
PQ = np.array(Q - P)
PR = np.array(R - P)

#Finding the normal vector of the plane.
normalv = np.cross(PQ, PR)
print 'normal vector = ' + str(normalv)
#Normalization of the normal vector.
n = normalv / np.linalg.norm(normalv)
print 'normalized = ' + str(n)

#Source position
source = np.array([4, 2, 4])
receiver = np.array([4, 4, 4])
source_vector = np.array(source - Q)
print 'Source position = ' + str(source)

#Dot product of source position and normalized normal vector.
source_distance = np.dot(n, source_vector)

print 'Distance from source to plane = ' + str(source_distance)

image = source - (2 * source_distance * n)
print 'Image position = ' + str(image)

image_v = receiver - image
print 'Distance from image to receiver = ' + str(image_v)
print('Vector 1: ' + str(PQ))
print('Vector 2: ' + str(PR))


#Solving for the intersection point of the line from image to receiver and the plane PQR. http://en.wikipedia.org/wiki/Line%E2%80%93plane_intersection
A = np.array([
    [image_v[0], PQ[0], PR[0]],
    [image_v[1], PQ[1], PR[1]],
    [image_v[2], PQ[2], PR[2]]
])

B = np.array([
    image[0] - P[0], image[1] - P[1], image[2] - P[2]
])

#Using linalg.solve to solve for the solution vector x from the form Ax = B. x is in the form [t, u, v].
x = np.linalg.solve(A, B)
print x
intersect_mag = x[0]
print('Intersection_matt mag: ' + str(intersect_mag))


# Alternate solution to intersection point http://mathworld.wolfram.com/Line-PlaneIntersection.html
top = np.array([[1, 1, 1, 1], [P[0], P[1], P[2], image[0]], [Q[0], Q[1], Q[2], image[1]], [R[0], R[1], R[2], image[2]]])
bottom = np.array([[1, 1, 1, 0], [P[0], Q[0], R[0], receiver[0] - image[0]], [P[1], Q[1], R[1], receiver[1] - image[1]],
                   [P[2], Q[2], R[2], receiver[2] - image[2]]])
t = np.linalg.det(top) / np.linalg.det(bottom)
print t

intersect_nathan = np.zeros([1, 3])
print intersect_nathan
for i in [0, 1, 2]:
    intersect_nathan[0][i] = round(image[i] + (receiver[i] - image[i]) * t, round_number)
print('Nathan Intersection: ' + str(intersect_nathan))

#Intersection point in form Ia + (Ib - Ia)t where Ia is the image position and Ib is the receiver position.
intersect_matt = image + image_v * intersect_mag
print 'Matt Intersection: ' + str(intersect_matt)

#If the determinant below is = 0, then the three vectors are linearly dependent meaning the line does not intersect the plane and the reflection is not valid.
for intersect in [intersect_matt, intersect_nathan]:
    print(str(intersect) + ':')
    # This only checks if the point lies within the infinite plane PQR
    is_orthogonal = np.dot(intersect - Q, n)
    if is_orthogonal == 0:
        if np.linalg.norm(intersect - PR) <= np.linalg.norm(PR) and np.linalg.norm(intersect - PQ) <= np.linalg.norm(PQ):
            print 'Valid reflection.'
            valid_reflection = True
        else:
            print('Invalid Reflection. Not in Finite Plane')
    else:
        print('Invalid reflection. Not in Infinite Plane: ')

if valid_reflection == True:
    ray_1 = intersect_nathan - source
    ray_2 = receiver - intersect_nathan
    distance = np.linalg.norm(ray_1) + np.linalg.norm(ray_2)
    time = distance / 1130
    valid_time.append(time)
    print('Time: ' + str(time))