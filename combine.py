#!/usr/bin/env python
# Author: Feng Jin
# Comments: Combine all the .npy files and the timesheet

import argparse
import numpy as np
import os

class file_preproc:
    def __init__(self):
        pass  

    def combiner(self, filedir):
        self.filecnt = 0
        self.filedir  = filedir
        self.total_pointcloud = []
        self.total_frameidx = []
        num_frames = 0
        for self.file in os.listdir(self.filedir):
            if self.file.endswith(".npy") and self.file != 'total_pointcloud.npy':
                self.filecnt += 1
                # Load the .npy file
                pointcloud  = np.load(self.filedir+self.file, allow_pickle=True)
                self.total_pointcloud.extend(pointcloud)
                if os.path.exists(self.filedir+self.file[:-4] + '.csv'): # Ground truth time index file exist
                    # Load the ground truth timesheet .csv file
                    gt_frameidx = np.genfromtxt(self.filedir+self.file[:-4] + '.csv', delimiter=',').astype(int)           
                    self.total_frameidx.extend((np.array(gt_frameidx)+num_frames))
                num_frames  += len(pointcloud)

        print('*************************************************')
        print('Done. The number of total processed files are:' + str(self.filecnt))
        self.total_pointcloud_path = str(os.path.join(self.filedir,'total_pointcloud'))
        print('Total pointcloud files are combined into:' + str(self.total_pointcloud_path) + '.npy')
        np.save(self.total_pointcloud_path, self.total_pointcloud)
        print('Total ground truth timesheets are combined into:' + str(self.total_pointcloud_path) + '.csv')
        np.savetxt(self.total_pointcloud_path+'.csv', self.total_frameidx, fmt='%i', delimiter=',')

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--filedir', type=str, default=None, help='Load which file. Default: None.')
    args = parser.parse_args()

    file_preproc().combiner(args.filedir)