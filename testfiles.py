import sys
import os

num = sys.argv[1]
num = int(num) + 1
right = open('right.txt', "w+")
left = open('left.txt', "w+")
disp = open('disp.txt', "w+")
for i in range(num):
    filenm = "000000" + "{:04d}".format(i) + ".png"
    l = '/input-output/left/' + filenm
    r = '/input-output/right/' + filenm
    d = '/input-output/' + filenm[:-3] + "pfm"
    left.write(l + "\n")
    right.write(r + "\n")
    disp.write(d + "\n")








