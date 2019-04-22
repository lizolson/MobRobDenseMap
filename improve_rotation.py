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
    for file in os.listdir("./output_blank"):
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
    reading_range = range(len(idx))
    # for i in range(len(idx)):
    for i in reading_range:
        if i == reading_range[0]:
        # if i == 0:
            output = read_point_cloud("./output_blank/" + idx[i])
            # draw_geometries([source])

        else:
            file_source = "./output_blank/" + idx[i]
            file_target = "./output_blank/" + idx[i-1]
            source = read_point_cloud(file_source)
            target = read_point_cloud(file_target)
            # draw_geometries([target])
            sourcePoints = np.asarray(source.points)
            targetPoints = np.asarray(target.points)

            sourceColors = np.asarray(source.colors)
            targetColors = np.asarray(target.colors)

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
            us = np.zeros((1,len(source_ID)))
            vs = np.zeros((1, len(source_ID)))
            # for j in range(len(ulist)):
            k = 0
            for j in source_ID:
                us[0,k] = int(float(data[time_source][j][0]))
                vs[0,k] = int(float(data[time_source][j][1]))
                k += 1

            ut = np.zeros((1,len(target_ID)))
            vt = np.zeros((1, len(target_ID)))
            k = 0
            for j in target_ID:
                ut[0,k] = int(float(data[time_target][j][0]))
                vt[0,k] = int(float(data[time_target][j][1]))
                k += 1


            # ransac = linear_model.RANSACRegressor()
            source_select = us * height + vs
            target_select = ut * height + vt
            source_xyz = sourcePoints[source_select.astype(int)][0]
            target_xyz = targetPoints[target_select.astype(int)][0]
            non_zero = np.copy(np.logical_and(~np.all(target_xyz == 0, axis=1),~np.all(source_xyz == 0, axis=1)))
            source_xyz = source_xyz[non_zero]
            target_xyz = target_xyz[non_zero]
            source_color = sourceColors[source_select.astype(int)][0]
            target_color = targetColors[target_select.astype(int)][0]
            source_color = source_color[non_zero]
            target_color = target_color[non_zero]
            diff = source_xyz - target_xyz
            diff_color = source_color - target_color
            use = [np.linalg.norm(diff[l])<10 for l in range(len(diff))]
            use_color = [np.linalg.norm(diff_color[l]) < 0.2 for l in range(len(diff_color))]
            use = np.logical_and(use,use_color)
            source_xyz = source_xyz[use]
            target_xyz = target_xyz[use]
            # source_xyz = source_xyz.transpose()
            # target_xyz = target_xyz.transpose()

            if len(source_xyz) > 10:
                # angles = np.multiply(np.subtract(range(30),15),2*np.pi/180)
                angles = np.multiply(np.subtract(range(30), 15), np.pi / 180)
                m = 0
                total_distance = np.zeros((1,len(angles)))
                for theta in angles:
                    rotation_matrix = [[np.cos(theta), -np.sin(theta)],[np.sin(theta), np.cos(theta)]]
                    input_xz = source_xyz.transpose()[[0, 2], :]
                    target_xz = target_xyz.transpose()[[0, 2], :]
                    trans = np.reshape(-np.mean(input_xz, axis= 1),[2,1])
                    input_xz = np.add(input_xz, trans)
                    target_xz = np.add(target_xz, trans)
                    new_xz = np.matmul(rotation_matrix, input_xz)
                    distance = np.sum(np.sqrt(np.sum(np.square(target_xz-new_xz), axis = 0)))
                    total_distance[0,m] = np.copy(distance)
                    m = m + 1
                theta_best_iter = angles[np.argmin(total_distance)]
                print(theta_best_iter)
                theta_best += theta_best_iter
                transition = np.zeros([3,1])
                transition[0,0] = trans[0]
                transition[2,0] = trans[1]
            else:
                theta_best += 0
                theta_best_iter = 0
                transition = np.zeros([3,1])

            # rotation_best = [[np.cos(theta_best), 0, -np.sin(theta_best)], [0, 1, 0],
            #                  [np.sin(theta_best), 0, np.cos(theta_best)]]

            rotation_iter = [[np.cos(theta_best_iter), 0, -np.sin(theta_best_iter)], [0, 1, 0],
                             [np.sin(theta_best_iter), 0, np.cos(theta_best_iter)]]
            # rotated_xyz = np.matmul(rotation_iter,source_xyz.transpose())
            # transpose_iter = np.mean(target_xyz - rotated_xyz.transpose(), axis=0)
            # distance += np.linalg.norm(np.mean(target_xyz-source_xyz, axis = 1))
            # transpose = [0,0,distance]
            # transpose = np.add(transpose, transpose_iter)
            displacedSourcePoints = np.add(sourcePoints.transpose(), transition)
            displacedSourcePoints = np.matmul(rotation_iter, displacedSourcePoints)
            displacedSourcePoints = np.add(displacedSourcePoints, -transition)

            displacedSourcePoints = displacedSourcePoints.transpose()
            # displacedSourcePoints = np.add(displacedSourcePoints, transpose)
            # myH = bestOrthogonalHT(source_xyz, target_xyz, len(target_xyz[0]))
            # homg = np.vstack([sourcePoints[:,0], sourcePoints[:,1], sourcePoints[:,2], np.ones((1,len(sourcePoints[:,0])))])
            # displacedSourcePoints = np.matmul(myH, homg)
            # displacedSourcePoints = np.transpose(displacedSourcePoints)[:, :-1]
            stepout = PointCloud()
            stepout.points = Vector3dVector(displacedSourcePoints)
            stepout.colors = Vector3dVector(sourceColors)
            write_point_cloud('RotationOutput/' + idx[i], stepout)

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
    write_point_cloud('RotationOutput/testCloud.ply', output)

