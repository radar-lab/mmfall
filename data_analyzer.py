#!/usr/bin/env python
# Author: Feng Jin
# Comments: Convert the ros bag files to bin files for training purpose

import argparse
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
import matplotlib.pyplot as plt
import os

class data_analyzer:
    def __init__(self, testfile=None, predictfile=None, rawfile=None):
        if testfile is not None:
            self.test_data = list(np.load(testfile, allow_pickle=True))
        if predictfile is not None:
            self.prediction_data = list(np.load(predictfile, allow_pickle=True))
        if rawfile is not None:
            self.raw_data = list(np.load(rawfile, allow_pickle=True))

    def print_data(self):
        batch_idx = 0
        frame_idx = 3
        point_idx = 5
        test_slice = self.test_data[batch_idx][frame_idx][point_idx]
        prediction_slice = self.prediction_data[batch_idx][frame_idx][point_idx]
        print(test_slice)
        print(prediction_slice)
        print(np.mean(np.abs(test_slice - prediction_slice)))

    def print_rawdata(self):
        frame_idx = 3
        point_idx = 5
        for point in self.raw_data[frame_idx]:
            # Get the original point information.
            pointR      = point[9]
            pointAZ     = point[10]
            pointEL     = point[11]
            pointD      = point[12]
            pointSNR    = point[13]
            pointNoise  = point[14]
            # Get the point's position in the Cartesian coord.
            pointX      = pointR*np.cos(pointEL)*np.sin(pointAZ)
            pointY      = pointR*np.cos(pointEL)*np.cos(pointAZ)
            pointZ      = pointR*np.sin(pointEL)
            # Get the centorid information in Cartesian coord.
            centoridX    = point[3]
            centoridY    = point[4]
            centoridZ    = point[5]
            centoridVx   = point[6]
            centoridVy   = point[7]
            centoridVz   = point[8]
            # Get the point feature vector
            delta_x     = pointX - centoridX
            # print("INFO: PointX and centoridX: % s and %s" %(pointX, centoridX))
            delta_y     = pointY - centoridY
            # print("INFO: PointY and centoridY: % s and %s" %(pointY, centoridY))
            # delta_z     = pointZ - centoridZ
            delta_z     = pointZ + 0.3
            delta_D     = pointD - (pointX*centoridVx+pointY*centoridVy+pointZ*centoridVz)/pointR
            pointRCS    = 4*10*np.log10(pointR) + pointSNR*0.1 + pointNoise*0.1 # in dBsm
            print([delta_x, delta_y, delta_z, pointD, pointRCS])  

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--test', type=str, default=None, help='Load which file. Default: None.')
    parser.add_argument('--prediction', type=str, default=None, help='Load which file. Default: None.')
    parser.add_argument('--raw', type=str, default=None, help='Load which file. Default: None.')
    args = parser.parse_args()

    data_analyzer(testfile=args.test, predictfile=args.prediction, rawfile=args.raw).print_rawdata()
    data_analyzer(testfile=args.test, predictfile=args.prediction, rawfile=args.raw).print_data()