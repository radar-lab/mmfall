#!/usr/bin/env python
# Author: Feng Jin
# Comments: Convert the ros bag files to bin files for training purpose

import argparse
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
import matplotlib.pyplot as plt
import os

class data_visualizer:
    def __init__(self, gtfile_path = None, binfile_path = None, GT_accum = False, pattern_accum = False, plot_accum = False):
        if binfile_path is not None:
            if pattern_accum is False:
                self.total_frame = list(np.load(binfile_path, allow_pickle=True))
            else:
                total_pattern = list(np.load(binfile_path, allow_pickle=True))
                self.total_frame = []
                if plot_accum is True:
                    for pattern in total_pattern:
                        self.total_frame.extend(pattern)
                else:
                    for pattern in total_pattern:
                        self.total_frame.append(pattern[0])

        if gtfile_path is not None:
            if GT_accum is False:
                self.groundtruth = list(np.load(gtfile_path, allow_pickle=True))
            else:
                total_pattern = list(np.load(gtfile_path, allow_pickle=True))
                self.groundtruth = []
                if plot_accum is True:
                    for pattern in total_pattern:
                        self.groundtruth.extend(pattern)
                else:
                    for pattern in total_pattern:
                        self.groundtruth.append(pattern[0])

        print("INFO: Input data shape: " + str(np.array(self.total_frame).shape))
        # for pattern in self.total_frame:
        #     print("INFO: Input data shape: " + str(np.array(pattern).shape))
        self.bagsrcdir  = None  

        # Data range for denormalization
        self.def_x_min, self.def_x_max      = -1.5, 1.5
        self.def_y_min, self.def_y_max      = -1.5, 1.5
        self.def_z_min, self.def_z_max      = -1.5, 1.5
        self.def_D_min, self.def_D_max      = -2.6, 2.6
        self.def_RCS_min, self.def_RCS_max  =   40, 130

        self.def_x_min, self.def_x_max      = -3, 3
        self.def_y_min, self.def_y_max      = 0, 6.1
        self.def_z_min, self.def_z_max      = -3, 3
        self.def_D_min, self.def_D_max      = -2.6, 2.6
        self.def_RCS_min, self.def_RCS_max  =   40, 130

        self.sec_min         = 0.0
        self.sec_max         = 1.0
        self.data_scale      = self.sec_max-self.sec_min
        self.data_shift      = self.sec_min 

        # Rotation matrix due to tilt angle
        tilt_angle  = -10 # degrees
        self.height = 1.80 # meters
        self.rotation_matrix = np.array([[1.0, 0.0, 0.0],\
                                        [0.0, np.cos(np.deg2rad(tilt_angle)), -np.sin(np.deg2rad(tilt_angle))],\
                                        [0.0, np.sin(np.deg2rad(tilt_angle)), np.cos(np.deg2rad(tilt_angle))]])

        # Coordiante limits
        self.xlim = -2.0, 2.0
        self.ylim = 0.0, 6.0
        self.zlim = 0.0, 2.0

        self.nor_xlim = -1.5, 1.5
        self.nor_ylim = -1.5, 1.5
        self.nor_zlim = -1.5, 1.5

    def RawAnimate(self, i):
        # Clear previous frame
        self.ax.clear()
        # Reset the scale according to the scene
        self.ax.set_xlim(self.xlim)
        self.ax.set_ylim(self.ylim)
        self.ax.set_zlim(self.zlim)
        # Set the labels
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        # Update the new frame
        for point in self.total_frame[i]:
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
            # Coordinate transformation
            results     = np.matmul(self.rotation_matrix, np.array([pointX, pointY, pointZ]))
            pointX      = results[0]
            pointY      = results[1]
            pointZ      = results[2] + self.height
            self.ax.scatter(pointX, pointY, pointZ, color='blue')
        # Coordinate transformation
        results     = np.matmul(self.rotation_matrix, np.array([point[3], point[4], point[5]]))
        centroidX   = results[0]
        centroidY   = results[1]
        centroidZ   = results[2] + self.height
        self.ax.scatter(centroidX, centroidY, centroidZ, color='red')

    def FeatureAnimate(self, i):
        # Clear previous frame
        self.ax.clear()
        # Reset the scale according to the scene
        self.ax.set_xlim(self.xlim)
        self.ax.set_ylim(self.ylim)
        self.ax.set_zlim(self.zlim)
        # Set the labels
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        # Update the new frame
        for point in self.total_frame[i]:
            self.ax.scatter(point[:, 0], point[:, 1], point[:, 2], color='blue')
            # # Coordinate transformation
            # results     = np.matmul(self.rotation_matrix, np.array([point[:, 0], point[:, 1], point[:, 2]]))
            # pointX      = results[0]
            # pointY      = results[1]
            # pointZ      = results[2] + self.height
            # self.ax.scatter(pointX, pointY, pointZ, color='blue')

    def FeatureAnimate2_GT(self, i):
        # Clear previous frame
        self.ax_GT.clear()
        # Reset the scale according to the scene
        self.ax_GT.set_xlim(self.xlim)
        self.ax_GT.set_ylim(self.ylim)
        self.ax_GT.set_zlim(self.zlim)
        # Set the labels
        self.ax_GT.set_xlabel('X')
        self.ax_GT.set_ylabel('Y')
        self.ax_GT.set_zlabel('Z')
        # Update the new frame
        for point in self.groundtruth[i]:
            self.ax.scatter(point[:, 0], point[:, 1], point[:, 2], color='blue')
            # # Coordinate transformation
            # results     = np.matmul(self.rotation_matrix, np.array([point[:, 0], point[:, 1], point[:, 2]]))
            # pointX      = results[0]
            # pointY      = results[1]
            # pointZ      = results[2] + self.height
            # self.ax.scatter(pointX, pointY, pointZ, color='blue')

    def FeatureAnimate_GT(self, i):
        # Clear previous frame
        self.ax_GT.clear()
        # Reset the scale according to the scene
        self.ax_GT.set_xlim(self.xlim)
        self.ax_GT.set_ylim(self.ylim)
        self.ax_GT.set_zlim(self.zlim)
        # Set the labels
        self.ax_GT.set_xlabel('X')
        self.ax_GT.set_ylabel('Y')
        self.ax_GT.set_zlabel('Z')
        # Update the new frame
        for point in self.groundtruth[i]:
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
            # Coordinate transformation
            results     = np.matmul(self.rotation_matrix, np.array([pointX, pointY, pointZ]))
            pointX      = results[0]
            pointY      = results[1]
            pointZ      = results[2] + self.height
            self.ax_GT.scatter(pointX, pointY, pointZ, color='blue')
        # Coordinate transformation
        results     = np.matmul(self.rotation_matrix, np.array([point[3], point[4], point[5]]))
        centroidX   = results[0]
        centroidY   = results[1]
        centroidZ   = results[2] + self.height
        self.ax_GT.scatter(centroidX, centroidY, centroidZ, color='red')

    def FeatureAnimate2(self, i):
        # Clear previous frame
        self.ax.clear()
        # Reset the scale according to the scene
        self.ax.set_xlim(self.xlim)
        self.ax.set_ylim(self.ylim)
        self.ax.set_zlim(self.zlim)
        # Set the labels
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        # Update the new frame
        for point in self.total_frame[i]:
            self.ax.scatter(point[0], point[1], point[2], color='blue')
            # # Coordinate transformation
            # results     = np.matmul(self.rotation_matrix, np.array([point[:, 0], point[:, 1], point[:, 2]]))
            # pointX      = results[0]
            # pointY      = results[1]
            # pointZ      = results[2] + self.height
            # self.ax.scatter(pointX, pointY, pointZ, color='blue')

    def FeatureAnimate_prediction(self, i):
        # Clear previous frame
        self.ax.clear()
        # Reset the scale according to the scene
        self.ax.set_xlim(self.xlim)
        self.ax.set_ylim(self.ylim)
        self.ax.set_zlim(self.zlim)
        # Set the labels
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        # Update the new frame
        for point in self.total_frame[i]:
            self.ax.scatter(point[:, 0], point[:, 1], point[:, 2], color='blue')
    
    def RawAnimate_GT(self, i):
        # Clear previous frame
        self.ax_GT.clear()
        # Reset the scale according to the scene
        self.ax_GT.set_xlim(self.xlim)
        self.ax_GT.set_ylim(self.ylim)
        self.ax_GT.set_zlim(self.zlim)
        # Set the labels
        self.ax_GT.set_xlabel('X')
        self.ax_GT.set_ylabel('Y')
        self.ax_GT.set_zlabel('Z')
        # Update the new frame
        for point in self.groundtruth[i]:
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
            # Point coordinate transformation
            results     = np.matmul(self.rotation_matrix, np.array([pointX, pointY, pointZ]))
            pointX      = results[0]
            pointY      = results[1]
            pointZ      = results[2] + self.height
            self.ax_GT.scatter(pointX, pointY, pointZ, color='blue')
        # Centroid coordinate transformation
        results      = np.matmul(self.rotation_matrix, np.array([point[3],point[4],point[5]]))
        centroidX    = results[0]
        centroidY    = results[1]
        centroidZ    = results[2] + self.height
        # self.ax_GT.scatter(point[3], point[4], point[5], color='red')
        self.ax_GT.scatter(centroidX, centroidY, centroidZ, color='red')

    def squeeze_pattern(self):
        total_frame_np = np.array(self.total_frame)
        processed_total_frame = list(total_frame_np[:-1, 0, :, :, :])
        print(np.array(processed_total_frame).shape)
        processed_total_frame.extend(total_frame_np[-1, :, :, :, :])
        print(np.array(processed_total_frame).shape)
        self.total_frame = processed_total_frame

    def denormalize(self):
        total_frame_denormalized = []
        for frame in self.total_frame:
            new_frame = []
            for point in frame:
                new_frame.append([(point[0, 0]-self.data_shift)/self.data_scale*(self.def_x_max-self.def_x_min)+self.def_x_min, \
                    (point[0, 1]-self.data_shift)/self.data_scale*(self.def_y_max-self.def_y_min)+self.def_y_min, \
                    (point[0, 2]-self.data_shift)/self.data_scale*(self.def_z_max-self.def_z_min)+self.def_z_min, \
                    (point[0, 3]-self.data_shift)/self.data_scale*(self.def_D_max-self.def_D_min)+self.def_D_min, \
                    (point[0, 4]-self.data_shift)/self.data_scale*(self.def_RCS_max-self.def_RCS_min)+self.def_RCS_min]) 
            total_frame_denormalized.append(np.expand_dims(np.array(new_frame), axis = -2))
        self.total_frame = total_frame_denormalized

    def plot_Raw3D(self):
        fig = plt.figure()
        self.ax = fig.add_subplot(111, projection='3d')
        ani = animation.FuncAnimation(fig, self.RawAnimate, frames=len(self.total_frame), interval=100)
        plt.show()

    def plot_Feature3D(self):
        # self.denormalize()
        fig = plt.figure()
        self.ax = fig.add_subplot(111, projection='3d')
        ani = animation.FuncAnimation(fig, self.FeatureAnimate, frames=len(self.total_frame), interval=100)
        plt.show()

    def plot_Pattern3D(self):
        self.squeeze_pattern()
        self.plot_Feature3D()

    def plot_Pattern3D_2(self):
        fig = plt.figure()
        self.ax = fig.add_subplot(111, projection='3d')
        ani = animation.FuncAnimation(fig, self.FeatureAnimate2, frames=len(self.total_frame), interval=100)
        plt.show()

    def plot_gt_feature_3D(self):
        # self.denormalize()
        fig             = plt.figure()
        self.ax_GT      = fig.add_subplot(121, projection='3d')
        self.ax         = fig.add_subplot(122, projection='3d')
        ani_GT          = animation.FuncAnimation(fig, self.RawAnimate_GT, frames=len(self.groundtruth), interval=100)
        ani_feature     = animation.FuncAnimation(fig, self.FeatureAnimate, frames=len(self.total_frame), interval=100)
        plt.show()

    def plot_gt_pattern_3D(self):
        self.squeeze_pattern()
        # self.denormalize()
        fig             = plt.figure()
        self.ax_GT      = fig.add_subplot(121, projection='3d')
        self.ax         = fig.add_subplot(122, projection='3d')
        ani_GT          = animation.FuncAnimation(fig, self.RawAnimate_GT, frames=len(self.groundtruth), interval=100)
        ani_pattern     = animation.FuncAnimation(fig, self.FeatureAnimate, frames=len(self.total_frame), interval=100)
        plt.show()

    def plot_gt_prediction_3D(self):
        self.squeeze_pattern()
        # self.denormalize()
        fig             = plt.figure()
        self.ax_GT      = fig.add_subplot(121, projection='3d')
        self.ax         = fig.add_subplot(122, projection='3d')
        ani_GT          = animation.FuncAnimation(fig, self.RawAnimate_GT, frames=len(self.groundtruth), interval=100)
        ani_prediction  = animation.FuncAnimation(fig, self.FeatureAnimate_prediction, frames=len(self.total_frame), interval=100)
        plt.show()

    def plot_gt_Pattern3D_2(self):
        fig             = plt.figure()
        self.ax_GT      = fig.add_subplot(121, projection='3d')
        self.ax         = fig.add_subplot(122, projection='3d')
        ani_GT          = animation.FuncAnimation(fig, self.FeatureAnimate2_GT, frames=len(self.groundtruth), interval=100)
        ani_prediction  = animation.FuncAnimation(fig, self.FeatureAnimate2, frames=len(self.total_frame), interval=100)
        plt.show()

    def plot_gt_pattern_2(self):
        fig             = plt.figure()
        self.ax_GT      = fig.add_subplot(121, projection='3d')
        self.ax         = fig.add_subplot(122, projection='3d')
        ani_GT          = animation.FuncAnimation(fig, self.FeatureAnimate_GT, frames=len(self.groundtruth), interval=100)
        ani_prediction  = animation.FuncAnimation(fig, self.FeatureAnimate2, frames=len(self.total_frame), interval=100)
        plt.show()

    def plot_Z(self):
        z_history = []
        for frame in self.groundtruth:
            # Select onme point from the frame
            point = frame[0]
            # Coordinate transformation
            results     = np.matmul(self.rotation_matrix, np.array([point[3], point[4], point[5]]))
            centroidX   = results[0]
            centroidY   = results[1]
            centroidZ   = results[2] + self.height
            z_history.append(centroidZ)
        print("Z std is %s" %(np.std(np.array(z_history))))
        plt.ylim(0.0, 2.0)
        plt.plot(z_history)
        plt.show()

    def plot_RawDoppler(self):
        num_PRIs    = 64
        doppler_res = 0.079 # m/s
        doppler_total_frame = np.zeros((len(self.total_frame), num_PRIs))
        frame_idx = 0

        for frame in self.total_frame:
            doppler_frame = []
            for point in frame:
                doppler_frame.append(int(point[12]/doppler_res + num_PRIs/2))
            doppler_bins = np.unique(np.array(doppler_frame))
            doppler_total_frame[frame_idx, doppler_bins] = 1
            frame_idx += 1
        plt.imshow(np.transpose(doppler_total_frame))  
        plt.show()

    def plot_FeatureDoppler(self):
        self.denormalize()
        num_PRIs    = 64
        doppler_res = 0.079 # m/s
        doppler_total_frame = np.zeros((len(self.total_frame), num_PRIs))
        frame_idx = 0

        for frame in self.total_frame:
            doppler_frame = []
            for point in frame:
                doppler_frame.append(int(point[0, 3]/doppler_res + num_PRIs/2))
            doppler_bins = np.unique(np.array(doppler_frame))
            doppler_total_frame[frame_idx, doppler_bins] = 1
            frame_idx += 1
        plt.imshow(np.transpose(doppler_total_frame))  
        plt.show()

    def plot_PatternDoppler(self):
        self.squeeze_pattern()
        self.plot_FeatureDoppler()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--gtfile', type=str, default=None, help='Load which file. Default: None.')
    parser.add_argument('--binfile', type=str, default=None, help='Load which file. Default: None.')
    args = parser.parse_args()

    ## Plot Doppler pattern
    # data_visualizer(binfile_path=args.binfile).plot_RawDoppler()
    # data_visualizer(binfile_path=args.binfile).plot_FeatureDoppler()
    # data_visualizer(binfile_path=args.binfile).plot_PatternDoppler()

    # Plot Z value
    # data_visualizer(gtfile_path=args.gtfile, binfile_path=args.binfile).plot_Z()

    ## Plot 3D point cloud
    # data_visualizer(binfile_path=args.binfile).plot_Raw3D()
    # data_visualizer(binfile_path=args.binfile).plot_Feature3D()
    # data_visualizer(binfile_path=args.binfile).plot_Pattern3D()
    # data_visualizer(binfile_path=args.binfile, accum_pattern=True).plot_Pattern3D_2()

    # Plot two 3D point cloud 
    # data_visualizer(gtfile_path=args.gtfile, binfile_path=args.binfile).plot_gt_feature_3D()
    # data_visualizer(gtfile_path=args.gtfile, binfile_path=args.binfile).plot_gt_pattern_3D()
    # data_visualizer(gtfile_path=args.gtfile, binfile_path=args.binfile).plot_gt_prediction_3D()
    # data_visualizer(gtfile_path=args.gtfile, binfile_path=args.binfile, accum_pattern=True).plot_gt_Pattern3D_2()
    # data_visualizer(gtfile_path=args.gtfile, binfile_path=args.binfile, GT_accum = True, pattern_accum=True, plot_accum=True).plot_gt_Pattern3D_2()
    data_visualizer(gtfile_path=args.gtfile, binfile_path=args.binfile, GT_accum = False, pattern_accum=True, plot_accum=False).plot_gt_pattern_2()