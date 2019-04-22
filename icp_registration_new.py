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
# from sklearn import linear_model, datasets
from matplotlib import pyplot as plt
from open3d import *

#
# def draw_registration_result(source, target, transformation):
#     source_temp = copy.deepcopy(source)
#     target_temp = copy.deepcopy(target)
#     # source_temp.paint_uniform_color([1, 0.706,   0.0])
#     # target_temp.paint_uniform_color([0, 0.651, 0.929])
#     source_temp.transform(transformation)
#     draw_geometries([source_temp, target_temp])

if __name__ == "__main__":
    idx = []
    for file in os.listdir("./output_mymap"):
        if file[-3:] != 'ply':
            continue
        idx = np.append(idx, file)
    idx.sort()

    with open('VF_pointcloud_expanded_full.csv') as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        next(csv_reader)
        data = [[] for i in range(4544)]  # 4544
        uniqueID = [[] for i in range(4544)]  # 4544
        for row in csv_reader:
            step = int(row[0])
            data[step].append(row[4:])
            uniqueID[step].append(row[3])
    distance = 0
    theta_best = 0
    transpose = np.zeros((1,3))
    reading_range = np.add(range(3), 55)
    # reading_range = range(4)
    # for i in range(len(idx)):
    for i in reading_range:
        if i == reading_range[0]:
        # if i == 0:
            output = read_point_cloud("./output_translated/" + idx[i])
        #     output = read_point_cloud("./output_mymap/" + idx[i])
            # draw_geometries([source])

        else:
            file_source = "./output_translated/" + idx[i]
            file_target = "./output_translated/" + idx[i-1]
            # file_source = "./output_mymap/" + idx[i]
            # file_target = "./output_mymap/" + idx[i - 1]
            source = read_point_cloud(file_source)
            target = read_point_cloud(file_target)
            # draw_geometries([target])
            sourcePoints = np.asarray(source.points)
            targetPoints = np.asarray(target.points)

            sourceColors = np.asarray(source.colors)
            targetColors = np.asarray(target.colors)

##################################################

            time_source = int(file_source[-8:-4])
            time_target = int(file_target[-8:-4])
            height = 376
            width = 1241
            # ulist = [int(float(x[0])) for x in data[time_source]]
            feature_source = uniqueID[time_source]
            feature_source = map(int, feature_source)
            feature_target = uniqueID[time_target]
            feature_target = map(int, feature_target)
            feature_target_set = set(feature_target)
            feature_source_set = set(feature_source)
            source_ID = [l for l, item in enumerate(feature_source) if item in feature_target_set]
            target_ID = [l for l, item in enumerate(feature_target) if item in feature_source_set]
            us = np.zeros((1, len(source_ID)))
            vs = np.zeros((1, len(source_ID)))
            # for j in range(len(ulist)):
            k = 0
            for j in source_ID:
                us[0, k] = int(float(data[time_source][j][0]))
                vs[0, k] = int(float(data[time_source][j][1]))
                k += 1

            ut = np.zeros((1, len(target_ID)))
            vt = np.zeros((1, len(target_ID)))
            k = 0
            for j in target_ID:
                ut[0, k] = int(float(data[time_target][j][0]))
                vt[0, k] = int(float(data[time_target][j][1]))
                k += 1

            # ransac = linear_model.RANSACRegressor()
            source_select = us * height + vs
            target_select = ut * height + vt
            source_xyz = sourcePoints[source_select.astype(int)][0]
            target_xyz = targetPoints[target_select.astype(int)][0]
            non_zero = np.copy(np.logical_and(~np.all(target_xyz == 0, axis=1), ~np.all(source_xyz == 0, axis=1)))
            source_xyz = source_xyz[non_zero]
            target_xyz = target_xyz[non_zero]
            source_color = sourceColors[source_select.astype(int)][0]
            target_color = targetColors[target_select.astype(int)][0]
            source_color = source_color[non_zero]
            target_color = target_color[non_zero]
            diff = source_xyz - target_xyz
            diff_color = source_color - target_color
            use = [np.linalg.norm(diff[l]) < 10 for l in range(len(diff))]
            use_color = [np.linalg.norm(diff_color[l]) < 0.5 for l in range(len(diff_color))]
            use = np.logical_and(use, use_color)
            source_xyz = source_xyz[use]
            target_xyz = target_xyz[use]

            source_ICP = PointCloud()
            source_ICP.points = Vector3dVector(source_xyz)
            source_ICP.colors = Vector3dVector(source_color)

            target_ICP = PointCloud()
            target_ICP.points = Vector3dVector(source_xyz)
            target_ICP.colors = Vector3dVector(source_color)
#########################

            threshold = 100
            trans_init = [[1.0, 0.0, 0.0, 0.0],
                          [0.0, 1.0, 0.0, 0.0],
                          [0.0, 0.0, 1.0, 0.0],
                          [0.0, 0.0, 0.0, 1.0]]

            reg_p2p = registration_icp(source_ICP, target_ICP, threshold, trans_init,
                                       TransformationEstimationPointToPoint())

            reg_p2p = registration_icp(source, target, threshold, trans_init,
                                       TransformationEstimationPointToPoint())

            downsource = voxel_down_sample(source, voxel_size=0.05)

            estimate_normals(downsource, search_param=KDTreeSearchParamHybrid(
                radius=0.1, max_nn=30))

            downtarget = voxel_down_sample(target, voxel_size=0.05)

            estimate_normals(downtarget, search_param=KDTreeSearchParamHybrid(
                radius=0.1, max_nn=30))

            reg_p2l = registration_icp(downsource, downtarget, threshold, reg_p2p.transformation,
                                       TransformationEstimationPointToPlane())
##########################
            # radius = 0.05
            # max_iter = 250
            # print("3-1. Downsample with a voxel size %.2f" % radius)
            # source_down = voxel_down_sample(source, radius)
            # target_down = voxel_down_sample(target, radius)
            #
            # print("3-2. Estimate normal.")
            # estimate_normals(source_down, KDTreeSearchParamHybrid(
            #     radius=radius * 2, max_nn=30))
            # estimate_normals(target_down, KDTreeSearchParamHybrid(
            #     radius=radius * 2, max_nn=30))
            #
            # print("3-3. Applying colored point cloud registration")
            # result_icp = registration_colored_icp(source_down, target_down,
            #                                       radius, trans_init,
            #                                       ICPConvergenceCriteria(relative_fitness = 1,
            #                                         relative_rmse = 10, max_iteration = max_iter))
            # current_transformation = result_icp.transformation
            # print(current_transformation)
#######################
            augSourcePoints = np.ones((len(sourcePoints), 4))
            augSourcePoints[:, :3] = sourcePoints

            # augDisplacedSourcePoints = np.matmul(reg_p2p.transformation, augSourcePoints.T)
            augDisplacedSourcePoints = np.matmul(current_transformation, augSourcePoints.T)
            displacedSourcePoints = augDisplacedSourcePoints[:3, :]
            print(displacedSourcePoints.shape)
            displacedSourcePoints = displacedSourcePoints.T

###################################
            stepout = PointCloud()
            stepout.points = Vector3dVector(displacedSourcePoints)
            stepout.colors = Vector3dVector(sourceColors)
            write_point_cloud('ICPOutput/' + idx[i], stepout)

            outputPoints = np.asarray(output.points)
            outputColors = np.asarray(output.colors)
            finalPoints = np.append(outputPoints, displacedSourcePoints, axis=0)
            finalColors = np.append(outputColors, sourceColors, axis=0)

            output = PointCloud()
            output.points = Vector3dVector(finalPoints)
            output.colors = Vector3dVector(finalColors)


    outputPoints = np.asarray(output.points)
    outputColors = np.asarray(output.colors)
    output_notzeros = np.copy(~np.all(outputPoints == 0, axis=1))
    outputPoints = outputPoints[output_notzeros]
    outputColors = outputColors[output_notzeros]
    output = PointCloud()
    output.points = Vector3dVector(outputPoints)
    output.colors = Vector3dVector(outputColors)
    write_point_cloud('ICPOutput/ICPCloud.ply', output)
