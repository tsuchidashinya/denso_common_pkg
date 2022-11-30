#!/usr/bin/env python3
import rospy
import rosparam
import h5py

if __name__=='__main__':
    rospy.init_node("hdf5_info")
    param_list = rosparam.get_param(rospy.get_name() + "/get_hdf5_info")
    path = param_list["hdf5_file_path"]
    h5_file = h5py.File(path, 'r')
    file_count = 0
    for k1 in h5_file:
        file_count += 1
        key_1 = k1
    
    print('data size is ', file_count)
    print('key1: ', key_1)
    for k2 in h5_file[key_1].keys():
        print('key2: ', k2)