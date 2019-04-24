from team3RANSAC import *
from open3d import *
from pfmfun import *
import cv2
import numpy as np
import os
import sys
import csv 

left = sys.argv[1] 
plys = sys.argv[2]
num = int(sys.argv[3])
output = sys.argv[4]
with open(sys.argv[5]) as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')
    next(csv_reader)
    data = []
    for row in csv_reader:
        step = int(row[0])
        if step == 10:
            data.append(row[4:])
    xyz = np.zeros((100, 3))
    xyzp = np.zeros((100, 3))
    ulist = [int(float(x[0])) for x in data]

    xyz = np.zeros((100, 3))
    xyzp = np.zeros((100, 3))
    disp, scale = readPFM(plys + '0000000010.pfm')
    focal = 718.856
    bl = 0.537
    for i in range(len(ulist)):
        u = int(float(data[i][0]))
        v = int(float(data[i][1]))
        #print(u)
        #print(v)
        d = disp[v, u]
        if d < 25:
            continue
        zp = focal * bl /d
        xp = u * zp/ focal
        yp = v * zp / focal
        
        xyzp[i] = [xp, yp, zp]
        x = float(data[i][2])
        y = float(data[i][3])
        z = float(data[i][4])
        xyz[i] = [x, y, z]
    #Removing the empy rows, that didn't have any points
    xyz = xyz[~np.all(xyz==0, axis = 1)]
    xyz = xyz.T
    xyzp = xyzp[~np.all(xyzp==0, axis = 1)]
    xyzp = xyzp.T
    #print(xyz)
    #print(xyzp)
    n = len(xyz[0])
    myH = bestOrthogonalHT(xyzp, xyz, n)
    np.save('transform.npy', myH)
    #Writes a file for the transform to the VINS global frame
    for i in range(num):
      leftimg = cv2.imread(left + "/000000" + "{:04d}".format(i) + ".png")
      image, scale = readPFM( plys + "000000"+ "{:04d}".format(i) + ".pfm")
      bl = 0.537
      focal = 718.856
      height, width = image.shape
      disp = image
      #Cloud: will contain the x, y, z points
      #colors: the color of the points
      cloud = np.zeros((width * height, 3))
      colors = np.zeros((width * height, 3))
      for c in range(width):
	  for r in range(height):
	      u = c - (width/2)
	      v = r - (height/2)
	      d = disp[r,c]
	      if d > 25:
		  z = focal * bl /d
		  #z = 1
		  x = u * z/ focal
		  y = v *z / focal
		  cloud[c * height + r] = [x, y, z]
		  color = leftimg[r, c] / 255.0
		  colors[c * height + r] = [color[2],color[1] ,color[0]]
      #Clear out array rows that are empty
      cloud = cloud[~np.all(cloud==0, axis = 1)]
      colors = colors[~np.all(colors==0, axis = 1)]
      pcd = PointCloud()
      pcd.points = Vector3dVector(cloud)
      pcd.colors = Vector3dVector(colors)
      #print(type(pcd))
      write_point_cloud(output + "000000" + "{:04d}".format(i) + ".ply", pcd)
if __name__ == '__main__':
  main()










