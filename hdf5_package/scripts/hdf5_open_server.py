#!/usr/bin/env python3
import rospy
import rosparam
from common_srvs.srv import Hdf5OpenService, Hdf5OpenServiceResponse
from common_srvs.srv import Hdf5RealPhoxiOpenService, Hdf5RealPhoxiOpenServiceResponse
from common_srvs.srv import Hdf5SegmentationOpenService, Hdf5SegmentationOpenServiceResponse, Hdf5SegmentationOpenServiceRequest
from hdf5_package import hdf5_function
from util import util_msg_data

class Hdf5OpenServer():
    def __init__(self):
        self.set_parameter()
        rospy.Service(self.service_name, Hdf5OpenService, self.service_callback)
        rospy.Service(self.real_phoxi_service_name, Hdf5RealPhoxiOpenService, self.real_phoxi_service_callback)
        rospy.Service(self.hdf5_segmentation_service_name, Hdf5SegmentationOpenService, self.hdf5_segmentation_service_callback)

    
    def set_parameter(self):
        param_list = rosparam.get_param(rospy.get_name() + "/hdf5_open_server")
        self.service_name = param_list["hdf5_open_service_name"]
        hdf5_file_path = param_list["hdf5_file_path"]
        self.hdf5_object = hdf5_function.open_readed_hdf5(hdf5_file_path)
        data_size = len(self.hdf5_object)
        rospy.set_param("hdf5_data_size", data_size)
        self.real_phoxi_service_name = param_list["hdf5_real_phoxi_open_service_name"]
        self.hdf5_segmentation_service_name = param_list["hdf5_segmentation_service_name"]
    
    def hdf5_segmentation_service_callback(self, request):
        index = request.index
        np_cloud = self.hdf5_object["data_" + str(index)]['Points'][()]
        mask_data = self.hdf5_object["data_" + str(index)]['masks'][()]
        np_concat_cloud = util_msg_data.concatenate_npcloud_and_npmask(np_cloud, mask_data)
        response = Hdf5SegmentationOpenServiceResponse()
        response.cloud_data = util_msg_data.npcloud_to_msgcloud(np_concat_cloud)
        return response
    
    def real_phoxi_service_callback(self, request):
        index = request.index
        np_cloud = self.hdf5_object["data_" + str(index)]['Points'][()]
        image = self.hdf5_object["data_" + str(index)]['image'][()]
        camera_info_list = self.hdf5_object["data_" + str(index)]['camera_info'][()]
        response = Hdf5RealPhoxiOpenServiceResponse()
        response.camera_info = util_msg_data.npcam_to_msgcam(camera_info_list)
        response.image = util_msg_data.npimg_to_rosimg(image)
        response.cloud_data = util_msg_data.npcloud_to_msgcloud(np_cloud)
        return response

    def service_callback(self, request):
        # request = Hdf5OpenServiceRequest()
        index = request.index
        numpy_cloud = self.hdf5_object["data_" + str(index)]['Points'][()]
        mask_data = self.hdf5_object["data_" + str(index)]['masks'][()]
        np_concat_cloud = util_msg_data.concatenate_npcloud_and_npmask(numpy_cloud, mask_data)
        translation = self.hdf5_object["data_" + str(index)]['translation'][()]
        rotation = self.hdf5_object["data_" + str(index)]['rotation'][()]
        image = self.hdf5_object["data_" + str(index)]['image'][()]
        camera_info_list = self.hdf5_object["data_" + str(index)]['camera_info'][()]
        response = Hdf5OpenServiceResponse()
        response.camera_info = util_msg_data.npcam_to_msgcam(camera_info_list)
        response.image = util_msg_data.npimg_to_rosimg(image)
        response.pose_data = util_msg_data.trans_rotate_to_msgposelist(translation, rotation)
        response.cloud_data = util_msg_data.npcloud_to_msgcloud(np_concat_cloud)
        return response

if __name__=='__main__':
    rospy.init_node('hdf5_open_server')
    hdf5_opent = Hdf5OpenServer()
    rospy.spin()

    
    