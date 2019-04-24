import numpy as np
from ransac import *


# Aux functions
def augment(xyzs):
    parametersNumber = len(xyzs[0])
    axyz = np.ones((len(xyzs), parametersNumber + 1))
    axyz[:, :(parametersNumber)] = xyzs
    return axyz

def estimate(xyzs):
    parametersNumber = len(xyzs[0])
    axyz = augment(xyzs[:parametersNumber])
    return np.linalg.svd(axyz)[-1][-1, :]

def is_inlier(coeffs, xyz, threshold):
    return np.abs(coeffs.dot(augment([xyz]).T)) < threshold


# RANSAC function
def bestHomogeneousTransformation(xyzs, final_xyzs, n):
    # Parameters
    max_iterations = 100
    goal_inliers = n * 0.8
    parametersDimension = 5

    # Reorganize the data
    for m in range(n):
        if m == 0:
            finalData1 = np.array([[xyzs[0][m], xyzs[1][m], xyzs[2][m], final_xyzs[0][m]]])
            finalData2 = np.array([[xyzs[0][m], xyzs[1][m], xyzs[2][m], final_xyzs[1][m]]])
            finalData3 = np.array([[xyzs[0][m], xyzs[1][m], xyzs[2][m], final_xyzs[2][m]]])
        else:
            finalData1 = np.append(finalData1, np.array([[xyzs[0][m], xyzs[1][m], xyzs[2][m], final_xyzs[0][m]]]), axis = 0)
            finalData2 = np.append(finalData2, np.array([[xyzs[0][m], xyzs[1][m], xyzs[2][m], final_xyzs[1][m]]]), axis = 0)
            finalData3 = np.append(finalData3, np.array([[xyzs[0][m], xyzs[1][m], xyzs[2][m], final_xyzs[2][m]]]), axis = 0)


    # run RANSAC 3 times, for each row of the homogenous transformation
    m1, b1 = run_ransac(finalData1, estimate, lambda x, y: is_inlier(x, y, 0.1), parametersDimension, goal_inliers, max_iterations)
    new_m1 = - m1 / m1[-2]

    m2, b2 = run_ransac(finalData2, estimate, lambda x, y: is_inlier(x, y, 0.1), parametersDimension, goal_inliers, max_iterations)
    new_m2 = - m2 / m2[-2]

    m3, b3 = run_ransac(finalData3, estimate, lambda x, y: is_inlier(x, y, 0.1), parametersDimension, goal_inliers, max_iterations)
    new_m3 = - m3 / m3[-2]


    # generate the Homogeneous Transformation Matrix
    homogeneousTransform = np.array([[new_m1[0], new_m1[1], new_m1[2], new_m1[4]],
                                     [new_m2[0], new_m2[1], new_m2[2], new_m2[4]],
                                     [new_m3[0], new_m3[1], new_m3[2], new_m3[4]],
                                     [        0,         0,         0,         1]])


    return homogeneousTransform


def bestOrthogonalHT(xyzs, final_xyzs, n):
    M = bestHomogeneousTransformation(xyzs, final_xyzs, n)
    R = M[0:3, 0:3]
    u, s, vh = np.linalg.svd(R, full_matrices=True)
    O = np.matmul(u, vh)
    #print(O)

    homogeneousTransform = np.array([[O[0][0], O[0][1], O[0][2], M[0][3]],
                                     [O[1][0], O[1][1], O[1][2], M[1][3]],
                                     [O[2][0], O[2][1], O[2][2], M[2][3]],
                                     [      0,       0,       0,       1]])

    return homogeneousTransform
