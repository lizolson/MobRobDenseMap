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
    for file in os.listdir("./output"):
        if file[-3:] != 'ply':
            continue
        idx = np.append(idx, file)
    idx.sort()

    with open('vins_100.csv') as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        next(csv_reader)
        data = [[] for i in range(100)]#4544
        for row in csv_reader:
            step = int(row[0])
            data[step].append(row[4:])
    # for i in range(len(idx)):
    for i in range(2):
        if i == 0:
            source = read_point_cloud("./output/" + idx[i])
            draw_geometries([source])

        else:
            file_source = "./output/" + idx[i]
            file_target = "./output/" + idx[i-1]
            target = read_point_cloud("./output/" + idx[i])
            draw_geometries([target])
            sourcePoints = np.asarray(source.points)
            targetPoints = np.asarray(target.points)

            sourceColors = np.asarray(source.colors)
            targetColors = np.asarray(target.colors)

            height = 376
            width = 1241
            time_source = int(file_source[-8:-4])
            ulist = [int(float(x[0])) for x in data[time_source]]
            us = np.zeros((1,len(ulist)))
            vs = np.zeros((1, len(ulist)))
            for j in range(len(ulist)):
                us[0,j] = int(float(data[time_source][j][0]))
                vs[0,j] = int(float(data[time_source][j][1]))

            time_target = int(file_target[-8:-4])
            ulist = [int(float(x[0])) for x in data[time_target]]
            ut = np.zeros((1,len(ulist)))
            vt = np.zeros((1, len(ulist)))
            for j in range(len(ulist)):
                ut[0,j] = int(float(data[time_target][j][0]))
                vt[0,j] = int(float(data[time_target][j][1]))


            # ransac = linear_model.RANSACRegressor()
            source_select = us * height + vs
            target_select = us*height + vs
            source_xyz = sourcePoints[source_select.astype(int)][0]
            target_xyz = targetPoints[target_select.astype(int)][0]
            source_xyz = source_xyz.transpose()
            target_xyz = target_xyz.transpose()
            myH = bestOrthogonalHT(source_xyz, target_xyz , len(targetPoints[target_select.astype(int)][0]))
            # ransac.fit(sourcePoints[source_select.astype(int)][0], targetPoints[target_select.astype(int)][0])
            # inlier_mask = ransac.inlier_mask_
            # outlier_mask = np.logical_not(inlier_mask)
            #
            # # Predict data of estimated models
            homg = np.vstack([sourcePoints[:,0], sourcePoints[:,1], sourcePoints[:,2], np.ones((1,len(sourcePoints[:,0])))])
            displacedSourcePoints = np.matmul(myH, homg)
            displacedSourcePoints = np.transpose(displacedSourcePoints)[:, :-1]


            finalPoints = np.append(displacedSourcePoints, targetPoints, axis=0)
            finalColors = np.append(np.asarray(sourceColors), np.asarray(targetColors), axis=0)

            source = PointCloud()
            source.points = Vector3dVector(finalPoints)
            source.colors = Vector3dVector(finalColors)

    write_point_cloud('finalOutput/testCloud.ply', source)

