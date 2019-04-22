from ransac import *
from team3RANSAC import *
import cv2
import numpy as np
import os
import sys
from open3d import *
import csv 

def main():
  leftdir = sys.argv[1]
  rightdir = sys.argv[2]
  #Reading in Sprites points in the global frame. Creating an array data, where each row is for a timestep, and contains a 2d array of the
  #data for that time step.
  with open('VF_pointcloud_expanded.csv') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')
    next(csv_reader)
    data = [[] for i in range(4544)]
    for row in csv_reader:
        step = int(row[0])
        data[step].append(row[4:])
  for file in os.listdir(leftdir):
    if file[-3:] != 'png':
      continue
    time = int(file[-8:-4])
    #Temporary time sequences to process
    if time > 120:
        continue
    if time <11:
        continue
    ulist = [int(float(x[0])) for x in data[time]]
    xyz = np.zeros((100, 3))
    xyzp = np.zeros((100, 3))
    # We need there to be at least 8 points, chose to have more, as it better removes outliers
    if len(ulist) <= 9:
    #if len(ulist) <= 7:
        continue
    print(file)
    print(len(ulist))
    #Computing the disparity map with SGM
    leftimg = cv2.imread(leftdir + file)
    rightimg = cv2.imread(rightdir + file)
    bl = 0.537
    focal = 718.856
    height, width, chan = leftimg.shape
    gray0 = cv2.cvtColor(leftimg, cv2.COLOR_BGR2GRAY)
    gray1 = cv2.cvtColor(rightimg, cv2.COLOR_BGR2GRAY)
    window_size = 9#3 #5
    min_disp = 0#48 #32
    num_disp = 320-min_disp
    # stereo = cv2.StereoSGBM(minDisparity = min_disp,
    #         numDisparities = 192,#num_disp,
    #         SADWindowSize = window_size,
    #         uniquenessRatio = 5,#10,
    #         speckleWindowSize = 100,
    #         speckleRange = 32,
    #         disp12MaxDiff = 1,
    #         P1 = 8*3*window_size**2,
    #         P2 = 32*3*window_size**2,
    #         fullDP = 1)#False)
    stereo = cv2.StereoSGBM_create(minDisparity = min_disp,
            numDisparities = 192,#num_disp,
            # SADWindowSize = window_size,
            uniquenessRatio = 5,#10,
            speckleWindowSize = 100,
            speckleRange = 32,
            disp12MaxDiff = 1,
            P1 = 8*3*window_size**2,
            P2 = 32*3*window_size**2)

    disp = stereo.compute(gray0, gray1).astype(np.float32) / 16.0
    #Cloud: will contain the x, y, z points
    #colors: the color of the points
    cloud = np.zeros((width * height, 3))
    colors = np.zeros((width * height, 3))
    #U and V are the coordinates of the feature. I convert them to the x' y' z' in my frame, store them in xyzp. 
    #Sprite's points in the global frame are in xyz
    for i in range(len(ulist)):
        u = int(float(data[time][i][0]))
        v = int(float(data[time][i][1]))
        d = disp[v, u]
        if d < 10:
            continue
        zp = focal * bl /d
        xp = u * zp/ focal
        yp = v * zp / focal
        
        xyzp[i] = [xp, yp, zp]
        x = float(data[time][i][2])
        y = float(data[time][i][3])
        z = float(data[time][i][4])
        xyz[i] = [x, y, z]
    #print('val check')
    #print(np.linalg.norm(np.array((xyz[0, 0], xyz[0, 1], xyz[0, 2]))-np.array((xyz[1, 0], xyz[1, 1], xyz[1,2]))))
    #print(np.linalg.norm(np.array((xyzp[0, 0], xyzp[0, 1], xyzp[0, 2]))-np.array((xyzp[1, 0], xyzp[1, 1], xyzp[1,2]))))
    #Removing the empy rows, that didn't have any points
    xyz = xyz[~np.all(xyz==0, axis = 1)]
    xyz = xyz.T
    xyzp = xyzp[~np.all(xyzp==0, axis = 1)]
    xyzp = xyzp.T
    n = len(xyz[0])
    
    #myH = bestHomogeneousTransformation(xyzp, xyz, len(xyz[0]))
    myH = bestOrthogonalHT(xyzp, xyz, len(xyz[0]))
    print(np.linalg.det(myH))
    #Threshold of which points to add to final cloud. higher d threshold, closer up points have to be for consideration
    for c in range(width):
        for r in range(height):
            u = c - (width/2)
            v = r - (height/2)
            d = disp[r,c]
            if d > 20:
                z = focal * bl /d
                #z = 1
                x = u * z/ focal
                y = v *z / focal
                homg = np.array([[x],[y],[z],[1.0]])
                out = myH.dot(homg)
                cloud[c * height + r] = [out[0,0], out[1,0], out[2,0]]
                color = leftimg[r, c] / 255.0
                colors[c * height + r] = [color[2],color[1] ,color[0]]
    #Clear out array rows that are empty
    cloud = cloud[~np.all(cloud==0, axis = 1)]
    colors = colors[~np.all(colors==0, axis = 1)]
    pcd = PointCloud()
    pcd.points = Vector3dVector(cloud)
    pcd.colors = Vector3dVector(colors)
    write_point_cloud('output/' + file[:-3] + 'ply', pcd)

if __name__ == '__main__':
  main()










