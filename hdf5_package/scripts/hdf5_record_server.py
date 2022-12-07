#!/usr/bin/env python3
import os
# from common_msgs.msg import CloudData
from common_srvs.srv import RecordAcc, RecordAccResponse, RecordAccRequest
from common_srvs.srv import RecordClustering, RecordClusteringResponse
from common_srvs.srv import RecordSegmentation, RecordSegmentationResponse
from common_srvs.srv import RecordPoseEstimation, RecordPoseEstimationResponse
from common_srvs.srv import RecordRealSensorData, RecordRealSensorDataResponse
import rospy
import rosparam
from util import util
from util import util_msg_data
from hdf5_package import hdf5_function
from tqdm import tqdm


class RecordServiceClass():
    def __init__(self):
        rospy.init_node('record_service')
        self.set_parameter()
        self.hdf5_initialize()
        rospy.Service(self.record_acc_name, RecordAcc, self.record_acc_service_callback)
        rospy.Service(self.record_segmentation_name, RecordSegmentation, self.record_segmentation_service_callback)
        rospy.Service(self.record_pose_estimation_name, RecordPoseEstimation, self.record_pose_estimation_service_callback)
        rospy.Service(self.record_clustering_name, RecordClustering, self.record_clustering_service_callback)
        rospy.Service(self.record_real_sensor_service_name, RecordRealSensorData, self.record_real_sensor_data_service_callback)

    def hdf5_initialize(self):
        self.hdf5_file_dir = util.dir_join_and_make(self.hdf5_file_dir, util.exclude_ext_str(self.hdf5_file_name) + util.get_timestr_ms())
        filename = util.insert_str(self.hdf5_file_name, util.get_timestr_hms())
        self.hdf5_object = hdf5_function.open_writed_hdf5(util.decide_allpath(self.hdf5_file_dir, filename))
    def set_parameter(self):
        param_list = rosparam.get_param(rospy.get_name() + "/hdf5_record_server/")
        self.record_acc_name = param_list["record_acc_service_name"]
        self.record_segmentation_name = param_list["record_segmentation_service_name"]
        self.record_pose_estimation_name = param_list["record_pose_estimation_service_name"]
        self.record_clustering_name = param_list["record_clustering_service_name"]
        self.hdf5_file_dir = param_list["record_hdf5_file_dir"]
        self.hdf5_file_name = param_list["record_hdf5_file_name"]
        self.hdf5_save_interval = param_list["hdf5_save_interval"]
        self.hdf5_service_counter = 0
        self.record_real_sensor_service_name = param_list["record_real_sensor_data_service_name"]
        
    def record_acc_service_callback(self, request):
        # request = RecordAccRequest()
        response = RecordAccResponse()
        response.ok = True
        index = self.hdf5_service_counter % self.hdf5_save_interval
        if self.hdf5_service_counter == 0:
            self.bar = tqdm(total=request.the_number_of_dataset)
            self.bar.set_description("Progress rate")
            
        elif self.hdf5_service_counter + 1 > request.the_number_of_dataset:
            return response
        elif request.the_number_of_dataset == self.hdf5_service_counter + 1:
            hdf5_function.close_hdf5(self.hdf5_object)
            hdf5_function.concatenate_hdf5(self.hdf5_file_dir, util.decide_allpath(self.hdf5_file_dir, self.hdf5_file_name))
            self.bar.update(1)
            return response
        elif index == 0:
            hdf5_function.close_hdf5(self.hdf5_object)
            filename = util.insert_str(self.hdf5_file_name, util.get_timestr_hms())
            self.hdf5_object = hdf5_function.open_writed_hdf5(util.decide_allpath(self.hdf5_file_dir, filename))
        np_cam = util_msg_data.msgcam_to_npcam(request.camera_info)
        np_img = util_msg_data.rosimg_to_npimg(request.image)
        np_cloud = util_msg_data.msgcloud_to_npcloud(request.cloud_data)
        np_cloud, np_mask = util_msg_data.extract_mask_from_npcloud(np_cloud)
        translation, rotation = util_msg_data.msgposelist_to_trans_rotate(request.pose_data_list)
        data_dict = {"Points": np_cloud, "masks": np_mask, "translation": translation,
            "rotation": rotation, "image": np_img, "camera_info": np_cam}
        hdf5_function.write_hdf5(self.hdf5_object, data_dict, index)
        self.bar.update(1)
        self.hdf5_service_counter = self.hdf5_service_counter + 1
        return response
    
    def record_segmentation_service_callback(self, request):
        self.hdf5_service_counter = self.hdf5_service_counter + 1
        rospy.set_param("record_counter", self.hdf5_service_counter)
        response = RecordSegmentationResponse()
        index = self.hdf5_service_counter % self.hdf5_save_interval
        if self.hdf5_service_counter == 1:
            self.bar = tqdm(total=request.the_number_of_dataset)
            self.bar.set_description("Progress rate")
        elif self.hdf5_service_counter == request.the_number_of_dataset:
            hdf5_function.close_hdf5(self.hdf5_object)
            hdf5_function.concatenate_hdf5(self.hdf5_file_dir, util.decide_allpath(self.hdf5_file_dir, self.hdf5_file_name))
            self.bar.update(1)
            response.ok = True
            return response
        elif self.hdf5_service_counter > request.the_number_of_dataset:
            response.ok = False
            return response
        elif index == 0:
            hdf5_function.close_hdf5(self.hdf5_object)
            filename = util.insert_str(self.hdf5_file_name, util.get_timestr_hms())
            self.hdf5_object = hdf5_function.open_writed_hdf5(util.decide_allpath(self.hdf5_file_dir, filename))
        np_cloud = util_msg_data.msgcloud_to_npcloud(request.cloud_data)
        np_cloud, np_mask = util_msg_data.extract_mask_from_npcloud(np_cloud)
        data_dict = {"Points": np_cloud, "masks": np_mask}
        hdf5_function.write_hdf5(self.hdf5_object, data_dict, index)
        self.bar.update(1)
        
        response.ok = True
        return response
    
    def record_pose_estimation_service_callback(self, request):
        self.hdf5_service_counter = self.hdf5_service_counter + 1
        index = self.hdf5_service_counter % self.hdf5_save_interval
        if self.hdf5_service_counter == 1:
            self.bar = tqdm(total=request.the_number_of_dataset)
            self.bar.set_description("Progress rate")
        elif request.the_number_of_dataset == self.hdf5_service_counter:
            hdf5_function.close_hdf5(self.hdf5_object)
            hdf5_function.concatenate_hdf5(self.hdf5_file_dir, util.decide_allpath(self.hdf5_file_dir, self.hdf5_file_name))
        elif index == 0:
            hdf5_function.close_hdf5(self.hdf5_object)
            filename = util.insert_str(self.hdf5_file_name, util.get_timestr_hms())
            self.hdf5_object = hdf5_function.open_writed_hdf5(util.decide_allpath(self.hdf5_file_dir, filename))
        np_cloud = util_msg_data.msgcloud_to_npcloud(request.cloud_data)
        np_cloud,  = util_msg_data.extract_mask_from_npcloud(np_cloud)
        translation, rotation = util_msg_data.msgposelist_to_trans_rotate(request.pose_datas)
        pose_mask = util_msg_data.make_pose_mask(translation, rotation)
        data_dict = {"pcl": np_cloud, "pose": pose_mask}
        hdf5_function.write_hdf5(self.hdf5_object, data_dict, index)
        self.bar.update(1)
        response = RecordPoseEstimationResponse()
        response.ok = True
        return response

    def record_clustering_service_callback(self, request):
        index = self.hdf5_service_counter % self.hdf5_save_interval
        if self.hdf5_service_counter == 0:
            self.bar = tqdm(total=request.the_number_of_dataset)
            self.bar.set_description("Progress rate")
        elif request.the_number_of_dataset == self.hdf5_service_counter + 1:
            hdf5_function.close_hdf5(self.hdf5_object)
            hdf5_function.concatenate_hdf5(self.hdf5_file_dir, util.decide_allpath(self.hdf5_file_dir, self.hdf5_file_name))
        elif index == 0:
            hdf5_function.close_hdf5(self.hdf5_object)
            filename = util.insert_str(self.hdf5_file_name, util.get_timestr_hms())
            self.hdf5_object = hdf5_function.open_writed_hdf5(util.decide_allpath(self.hdf5_file_dir, filename))
        np_cloud = util_msg_data.msgcloud_to_npcloud(request.cloud_data)
        np_cloud,  = util_msg_data.extract_mask_from_npcloud(np_cloud)
        data_dict = {"pcl": np_cloud, "class": request.class_id}
        hdf5_function.write_hdf5(self.hdf5_object, data_dict, index)
        self.bar.update(1)
        self.hdf5_service_counter = self.hdf5_service_counter + 1
        response = RecordClusteringResponse()
        response.ok = True
        return response
    
    def record_real_sensor_data_service_callback(self, request):
        response = RecordRealSensorDataResponse()
        response.ok = True
        self.hdf5_service_counter = self.hdf5_service_counter + 1
        index = self.hdf5_service_counter % self.hdf5_save_interval
        if self.hdf5_service_counter == 1:
            self.bar = tqdm(total=request.the_number_of_dataset)
            self.bar.set_description("Progress rate")
        elif request.the_number_of_dataset == self.hdf5_service_counter:
            self.bar.update(1)
            hdf5_function.close_hdf5(self.hdf5_object)
            hdf5_function.concatenate_hdf5(self.hdf5_file_dir, util.decide_allpath(self.hdf5_file_dir, self.hdf5_file_name))
            return response
        elif index == 0:
            hdf5_function.close_hdf5(self.hdf5_object)
            filename = util.insert_str(self.hdf5_file_name, util.get_timestr_hms())
            self.hdf5_object = hdf5_function.open_writed_hdf5(util.decide_allpath(self.hdf5_file_dir, filename))
        np_cloud = util_msg_data.msgcloud_to_npcloud(request.cloud_data)
        np_cloud, mask_cloud = util_msg_data.extract_mask_from_npcloud(np_cloud)
        np_cam = util_msg_data.msgcam_to_npcam(request.camera_info)
        np_img = util_msg_data.rosimg_to_npimg(request.image)
        data_dict = {"Points": np_cloud, "masks": mask_cloud, "image": np_img, "camera_info": np_cam}
        hdf5_function.write_hdf5(self.hdf5_object, data_dict, index)
        self.bar.update(1)
        return response
    
        

if __name__=='__main__':
    RecordServiceClass()
    rospy.spin()
    
