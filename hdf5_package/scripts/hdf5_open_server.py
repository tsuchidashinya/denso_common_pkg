#!/usr/bin/env python3
import rospy
import rosparam
from common_srvs.srv import Hdf5OpenAccService, Hdf5OpenAccServiceResponse, Hdf5OpenAccServiceRequest
from common_srvs.srv import Hdf5OpenSensorDataService, Hdf5OpenSensorDataServiceResponse, Hdf5OpenSensorDataServiceRequest
from common_srvs.srv import Hdf5OpenSegmentationService, Hdf5OpenSegmentationServiceResponse, Hdf5OpenSegmentationServiceRequest
from hdf5_package import hdf5_function
from util import util_msg_data

class Hdf5OpenServer():
    def __init__(self):
        self.set_parameter()
        rospy.Service(self.service_name, Hdf5OpenAccService, self.service_callback)
        rospy.Service(self.real_phoxi_service_name, Hdf5OpenSensorDataService, self.real_phoxi_service_callback)
        rospy.Service(self.hdf5_open_segmentation_service_name, Hdf5OpenSegmentationService, self.hdf5_open_segmentation_service_callback)

    
    def set_parameter(self):
        param_list = rosparam.get_param(rospy.get_name() + "/hdf5_open_server")
        self.service_name = param_list["hdf5_open_acc_service_name"]
        self.real_phoxi_service_name = param_list["hdf5_open_sensor_data_service_name"]
        self.hdf5_open_segmentation_service_name = param_list["hdf5_open_segmentation_service_name"]
        self.hdf5_open_file_path = ""
    
    def hdf5_open_segmentation_service_callback(self, request):
        # request = Hdf5OpenSegmentationServiceRequest()
        if request.hdf5_open_file_path != self.hdf5_open_file_path:
            self.hdf5_open_file_path = request.hdf5_open_file_path
            self.hdf5_object = hdf5_function.open_readed_hdf5(request.hdf5_open_file_path)
            data_size = len(self.hdf5_object)
            rospy.set_param("hdf5_data_size", data_size)
        index = request.index
        np_cloud = self.hdf5_object["data_" + str(index)]['Points'][()]
        mask_data = self.hdf5_object["data_" + str(index)]['masks'][()]
        np_concat_cloud = util_msg_data.concatenate_npcloud_and_npmask(np_cloud, mask_data)
        response = Hdf5OpenSegmentationServiceResponse()
        response.cloud_data = util_msg_data.npcloud_to_msgcloud(np_concat_cloud)
        response.data_size = len(self.hdf5_object)
        return response
    
    def real_phoxi_service_callback(self, request):
        index = request.index
        if request.hdf5_open_file_path != self.hdf5_open_file_path:
            self.hdf5_open_file_path = request.hdf5_open_file_path
            self.hdf5_object = hdf5_function.open_readed_hdf5(request.hdf5_open_file_path)
            data_size = len(self.hdf5_object)
            rospy.set_param("hdf5_data_size", data_size)
        np_cloud = self.hdf5_object["data_" + str(index)]['Points'][()]
        image = self.hdf5_object["data_" + str(index)]['image'][()]
        camera_info_list = self.hdf5_object["data_" + str(index)]['camera_info'][()]
        response = Hdf5OpenSensorDataServiceResponse()
        response.camera_info = util_msg_data.npcam_to_msgcam(camera_info_list)
        response.image = util_msg_data.npimg_to_rosimg(image)
        response.cloud_data = util_msg_data.npcloud_to_msgcloud(np_cloud)
        return response

    def service_callback(self, request):
        # request = Hdf5OpenAccServiceRequest()
        index = request.index
        if request.hdf5_open_file_path != self.hdf5_open_file_path:
            self.hdf5_open_file_path = request.hdf5_open_file_path
            self.hdf5_object = hdf5_function.open_readed_hdf5(request.hdf5_open_file_path)
            data_size = len(self.hdf5_object)
            rospy.set_param("hdf5_data_size", data_size)
        numpy_cloud = self.hdf5_object["data_" + str(index)]['Points'][()]
        mask_data = self.hdf5_object["data_" + str(index)]['masks'][()]
        np_concat_cloud = util_msg_data.concatenate_npcloud_and_npmask(numpy_cloud, mask_data)
        translation = self.hdf5_object["data_" + str(index)]['translation'][()]
        rotation = self.hdf5_object["data_" + str(index)]['rotation'][()]
        # instance = self.hdf5_object["data_" + str(index)]['instance'][()]
        image = self.hdf5_object["data_" + str(index)]['image'][()]
        camera_info_list = self.hdf5_object["data_" + str(index)]['camera_info'][()]
        response = Hdf5OpenAccServiceResponse()
        response.camera_info = util_msg_data.npcam_to_msgcam(camera_info_list)
        response.image = util_msg_data.npimg_to_rosimg(image)
        response.pose_data = util_msg_data.trans_rotate_to_msgposelist(translation, rotation)
        response.cloud_data = util_msg_data.npcloud_to_msgcloud(np_concat_cloud)
        return response

if __name__=='__main__':
    rospy.init_node('hdf5_open_server')
    hdf5_opent = Hdf5OpenServer()
    rospy.spin()

    
    