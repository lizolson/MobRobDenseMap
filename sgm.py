from pfmfun import *
import cv2
import numpy as np
import os
import sys
from open3d import *
import csv 

def main():
  leftdir = sys.argv[1]
  rightdir = sys.argv[2]
  for i in range(500):
      print("{:04d}".format(i))
      leftimg = cv2.imread(leftdir + "000000" +"{:04d}".format(i) + ".png")
      rightimg = cv2.imread(rightdir + "000000" +"{:04d}".format(i) + ".png")
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
      writePFM(sys.argv[3] + "000000" + "{:04d}".format(i) + ".pfm", disp)


if __name__ == '__main__':
  main()










