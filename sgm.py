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
  with open('globpts.csv') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')
    next(csv_reader)
    data = [[] for i in range(4544)]
    for row in csv_reader:
        step = int(row[0])
        data[step].append(row[4:])
  cloudfull = np.array([])
  for file in os.listdir(leftdir):
    print(file)
    if file[-3:] != 'png':
      continue
    time = int(file[-8:-4])
    if time > 60:
        continue
    if time <11:
        continue
    #print(data[time])
    ulist = [int(float(x[0])) for x in data[time]]
    xyz = np.zeros((100, 3))
    xyzp = np.zeros((100, 3))
    print(len(ulist))
    if len(ulist) <= 7:
        continue
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
    stereo = cv2.StereoSGBM(minDisparity = min_disp,
            numDisparities = 192,#num_disp,
            SADWindowSize = window_size,
            uniquenessRatio = 5,#10,
            speckleWindowSize = 100,
            speckleRange = 32,
            disp12MaxDiff = 1,
            P1 = 8*3*window_size**2,
            P2 = 32*3*window_size**2,
            fullDP = 1)#False)
    disp = stereo.compute(gray0, gray1).astype(np.float32) / 16.0
    #cv2.imwrite('temp.png', disp)
    #print(disp.shape)
    #print(width, height)
    cloud = np.zeros((width * height, 3))
    #cloudperf = np.zeros((width * height, 3))
    colors = np.zeros((width * height, 3))
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
    #print('xyz', xyz)
    #print('xyzp', xyzp)
    print('val check')
    #print(np.linalg.norm(np.array((xyz[0, 0], xyz[0, 1], xyz[0, 2]))-np.array((xyz[1, 0], xyz[1, 1], xyz[1,2]))))
    #print(np.linalg.norm(np.array((xyzp[0, 0], xyzp[0, 1], xyzp[0, 2]))-np.array((xyzp[1, 0], xyzp[1, 1], xyzp[1,2]))))
    xyz = xyz[~np.all(xyz==0, axis = 1)]
    xyz = xyz.T
    #print('xyz', xyz)
    xyzp = xyzp[~np.all(xyzp==0, axis = 1)]
    xyzp = xyzp.T
    #print('xyz', xyz)
    #print('xyzp', xyzp)
    n = len(xyz[0])
    #print(n)
    
    #myH = bestHomogeneousTransformation(xyz, xyz, len(xyz[0]))
    myH = bestHomogeneousTransformation(xyzp, xyz, len(xyz[0]))
    print(np.linalg.det(myH))
    #print(myH) 
    for c in range(width):
        for r in range(height):
            u = c - (width/2)
            v = r - (height/2)
            #print(r)
            d = disp[r,c]
            #print(d)
            if d > 50:
                z = focal * bl /d
                #z = 1
                x = u * z/ focal
                y = v *z / focal
                homg = np.array([[x],[y],[z],[1.0]])
                #out = np.array(myH) @ homg
                out = myH.dot(homg)
                #print('test')
                #print(homg)
                #print(out)
                #print([out[0,0], out[1,0], out[2,0]])
                #print(out)
                #x = u * z / focal
                #y = v * z / focal
                cloud[c * height + r] = [out[0,0], out[1,0], out[2,0]]
                #cloudperf[c * height + r] = [x, y, z]
                #print(out[0])
                color = leftimg[r, c] / 255.0
                colors[c * height + r] = [color[2],color[1] ,color[0]]
    cloud = cloud[~np.all(cloud==0, axis = 1)]
    #cloudperf = cloudperf[~np.all(cloud==0, axis = 1)]
    colors = colors[~np.all(colors==0, axis = 1)]
    pcd = PointCloud()
    pcd.points = Vector3dVector(cloud)
    pcd.colors = Vector3dVector(colors)
    write_point_cloud('output/' + file[:-3] + 'ply', pcd)

if __name__ == '__main__':
  main()










