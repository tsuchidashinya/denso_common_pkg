#!/usr/bin/env python3

import datetime
from cv_bridge import CvBridge
import numpy as np
import os

def get_time_str():
    dt_now = datetime.datetime.now()
    time_str = str(dt_now.month) + "_" + str(dt_now.day) + "_" \
                + str(dt_now.hour) + "_" + str(dt_now.minute) + "_" \
                + str(dt_now.second)
    return time_str

def insert_time_str(filename):
    root, ext = os.path.splitext(filename)
    return root + "_" + get_time_str() + ext

def ext_exist(basename):
    root, ext = os.path.splitext(basename)
    if len(ext) > 0:
        return True
    else:
        return False

def dir_join_and_make(origin_dir, add_dir):
    path = os.path.join(origin_dir, add_dir)
    make_dir(path)
    return path

def make_dir(path):
    if not os.path.exists(path):
        os.makedirs(path)

def decide_allpath(dir_path, file_path):
    dirname, basename = os.path.split(dir_path)
    if not ext_exist(basename):
        make_dir(dir_path)
        return str(os.path.join(dir_path, insert_time_str(file_path)))
    else:
        make_dir(dirname)
        return str(os.path.join(dirname, insert_time_str(file_path)))
        
def make_pose_data(pose):
    data_size = len(pose)
    translation = np.zeros((data_size, 3), dtype=np.float32)
    rotation = np.zeros((data_size, 4), dtype=np.float32)
    for i in range(data_size):
        translation[i, 0] = pose[i].translation.x
        translation[i, 1] = pose[i].translation.y
        translation[i, 2] = pose[i].translation.z
        rotation[i, 0] = pose[i].rotation.x
        rotation[i, 1] = pose[i].rotation.y
        rotation[i, 2] = pose[i].rotation.z
        rotation[i, 3] = pose[i].rotation.w
    return translation, rotation 

def make_pose_mask(translation, rotation):
    pose_mask = np.concatenate([translation, rotation])
    return pose_mask
    
def make_npimg_from_rosimg(rosimg):
    bridge = CvBridge()
    np_img = bridge.imgmsg_to_cv2(rosimg, "bgr8")
    return np_img 

def make_npcam_from_roscam(roscam):
    npcam = np.empty((0))
    for info in roscam.K:
        npcam = np.append(npcam, info)
    return npcam

def make_npcloud_from_cloud(cloud_data):
    np_cloud = np.array((cloud_data.x, cloud_data.y, cloud_data.z, cloud_data.instance))
    np_cloud = np_cloud.T
    return np_cloud

def extract_mask_from_npcloud(npcloud):
    npcloud = npcloud.T
    new_np_cloud = npcloud[:][0:3]
    np_mask = npcloud[:][3:]
    new_np_cloud = new_np_cloud.T 
    np_mask = np_mask.T
    return new_np_cloud, np_mask
