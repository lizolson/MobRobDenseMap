# Open3D: www.open3d.org
# The MIT License (MIT)
# See license file or visit www.open3d.org for details

# examples/Python/Basic/icp_registration.py

from ransac import *
from team3RANSAC import *
import cv2
import numpy as np
import os
import sys
from open3d import *
import csv
import copy
from math import sqrt



def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    # source_temp.paint_uniform_color([1, 0.706,   0.0])
    # target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    draw_geometries([source_temp, target_temp])

if __name__ == "__main__":
    idx = []
    for file in os.listdir("./output"):
        if file[-3:] != 'ply':
            continue
        idx = np.append(idx, file)
    idx.sort()

    for i in range(len(idx)):
    # for i in range(4):
        if i == 0:
            source = read_point_cloud("./output/" + idx[i])

            finalPoints = np.asarray(source.points)
            finalColors = np.asarray(source.colors)
        else:
            target = read_point_cloud("./output/" + idx[i])

            sourcePoints = np.asarray(source.points)
            targetPoints = np.asarray(target.points)

            sourceColors = np.asarray(source.colors)
            targetColors = np.asarray(target.colors)

            threshold = 2
            trans_init = [[1.0, 0.0, 0.0, 0.0],
                          [0.0, 1.0, 0.0, 0.0],
                          [0.0, 0.0, 1.0, 0.0],
                          [0.0, 0.0, 0.0, 1.0]]

            # draw_registration_result(source, target, trans_init)
            # print("Initial alignment")
            evaluation = evaluate_registration(source, target,
                    threshold, trans_init)
            # print(evaluation)

            # print("Apply point-to-point ICP")
            reg_p2p = registration_icp(source, target, threshold, trans_init,
                    TransformationEstimationPointToPoint())
            # print(reg_p2p)
            # print("Transformation is:")
            # print(reg_p2p.transformation)
            # print("")
            # draw_registration_result(source, target, reg_p2p.transformation)

            # print("Apply point-to-plane ICP")

            downsource = voxel_down_sample(source, voxel_size = 0.05)

            estimate_normals(downsource, search_param = KDTreeSearchParamHybrid(
                    radius = 0.1, max_nn = 30))

            downtarget = voxel_down_sample(target, voxel_size = 0.05)

            estimate_normals(downtarget, search_param = KDTreeSearchParamHybrid(
                    radius = 0.1, max_nn = 30))


            reg_p2l = registration_icp(downsource, downtarget, threshold, reg_p2p.transformation,
                    TransformationEstimationPointToPlane())
            # print(reg_p2l)
            # print("Transformation is:")
            # print(reg_p2l.transformation)
            # print("")
            # print(reg_p2l.transformation)
            # draw_registration_result(downsource, downtarget, reg_p2l.transformation)
            #
            augSourcePoints = np.ones((len(sourcePoints),4))
            augSourcePoints[:,:3] = sourcePoints

            augDisplacedSourcePoints = np.matmul(reg_p2l.transformation, augSourcePoints.T)
            displacedSourcePoints = augDisplacedSourcePoints[:3,:]
            print(displacedSourcePoints.shape)
            displacedSourcePoints = displacedSourcePoints.T

            finalPoints = np.append(displacedSourcePoints, targetPoints, axis = 0)
            finalColors = np.append(np.asarray(sourceColors), np.asarray(targetColors), axis = 0)

            source = PointCloud()
            source.points = Vector3dVector(finalPoints)
            source.colors = Vector3dVector(finalColors)

    # # print(finalPoints.shape)
    # # print(finalColors.shape)
    #
    # pcd = PointCloud()
    # pcd.points = Vector3dVector(finalPoints)
    # pcd.colors = Vector3dVector(finalColors)
    write_point_cloud('finalOutput/testCloud.ply', source)
