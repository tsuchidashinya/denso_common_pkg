#!/usr/bin/env python3
import h5py
import rospy
import rosparam
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from sensor_msgs.msg import Image
from common_msgs.msg import PoseData, CloudData
from common_srvs.srv import Hdf5OpenService, Hdf5OpenServiceRequest
import hdf5_function

class Hdf5OpenServer():
    def __init__(self):
        self.set_parameter()
        rospy.Service(self.service_name, Hdf5OpenServer, self.service_callback)
    
    def set_parameter(self):
        param_list = rosparam.get_param(rospy.get_name() + "/hdf5_open_server")
        self.service_name = param_list["hdf5_open_service_name"]
        hdf5_file_path = param_list["hdf5_file_path"]
        self.hdf5_object = hdf5_function.open_readed_hdf5(hdf5_file_path)
    
    def service_callback(self, request):
        request = Hdf5OpenServiceRequest()
        index = request.index
        numpy_cloud = self.hdf5_object["data_" + str(index)]['Points'][()]
        mask_data = 
    
    