#!/usr/bin/env python3
import os, sys
from common_msgs.msg import CloudData
from common_srvs.srv import Hdf5RecordAcc, Hdf5RecordAccResponse
from common_srvs.srv import Hdf5RecordClustering, Hdf5RecordClusteringResponse
from common_srvs.srv import Hdf5RecordSegmentation, Hdf5RecordSegmentationResponse
from common_srvs.srv import Hdf5RecordPoseEstimation, Hdf5RecordPoseEstimationResponse
import rospy
import rosparam
from util import util_python
from util import hdf5_function
from tqdm import tqdm


class RecordServiceClass():
    def __init__(self):
        rospy.init_node('record_service')
        self.set_parameter()
        rospy.Service(self.record_acc_name, Hdf5RecordAcc, self.record_acc_service_callback)
        rospy.Service(self.record_segmentation_name, Hdf5RecordSegmentation, self.record_segmentation_service_callback)
        rospy.Service(self.record_pose_estimation_name, Hdf5RecordPoseEstimation, self.record_pose_estimation_service_callback)
        rospy.Service(self.record_clustering_name, Hdf5RecordClustering, self.record_clustering_service_callback)

    def set_parameter(self):
        param_list = rosparam.get_param(rospy.get_name() + "/record_server/")
        self.record_acc_name = param_list["record_acc_service_name"]
        self.record_segmentation_name = param_list["record_segmentation_service_name"]
        self.record_pose_estimation_name = param_list["record_pose_estimation_service_name"]
        self.record_clustering_name = param_list["record_clustering_service_name"]
        self.hdf5_file_dir = param_list["record_hdf5_file_dir"]
        self.hdf5_file_name = param_list["record_hdf5_file_name"]
        self.hdf5_save_interval = param_list["hdf5_save_interval"]
        self.hdf5_service_counter = 0
        
    def record_acc_service_callback(self, request):
        index = self.hdf5_service_counter % self.hdf5_save_interval
        if self.hdf5_service_counter == 0:
            self.bar = tqdm(total=request.the_number_of_dataset)
            self.bar.set_description("Progress rate")
            self.hdf5_file_dir = os.path.join(self.hdf5_file_dir, request.annotation_task)
            self.hdf5_object = hdf5_function.open_writed_hdf5(util_python.decide_allpath(self.hdf5_file_dir, self.hdf5_file_name))
        elif request.the_number_of_dataset == self.hdf5_service_counter + 1:
            hdf5_function.close_hdf5(self.hdf5_object)
            hdf5_function.concatenate_hdf5(self.hdf5_file_dir, util_python.decide_allpath(self.hdf5_file_dir, self.hdf5_file_name))
        elif index == 0:
            hdf5_function.close_hdf5(self.hdf5_object)
            self.hdf5_object = hdf5_function.open_writed_hdf5(util_python.decide_allpath(self.hdf5_file_dir, self.hdf5_file_name))
        np_cam = util_python.make_npcam_from_roscam(request.camera_info)
        np_img = util_python.make_npimg_from_rosimg(request.image)
        np_cloud = util_python.make_npcloud_from_cloud(request.cloud_data)
        np_cloud, np_mask = util_python.extract_mask_from_npcloud(np_cloud)
        translation, rotation = util_python.make_pose_data(request.pose_datas)
        data_dict = {"Points": np_cloud, "masks": np_mask, "translation": translation,
            "rotation": rotation, "image": np_img, "camera_info": np_cam}
        hdf5_function.write_hdf5(self.hdf5_object, data_dict, index)
        self.bar.update(1)
        self.hdf5_service_counter = self.hdf5_service_counter + 1
        response = Hdf5RecordAccResponse()
        response.ok = True
        return response
    
    def record_segmentation_service_callback(self, request):
        self.hdf5_service_counter = self.hdf5_service_counter + 1
        index = self.hdf5_service_counter % self.hdf5_save_interval
        if self.hdf5_service_counter == 1:
            self.bar = tqdm(total=request.the_number_of_dataset)
            self.bar.set_description("Progress rate")
            self.hdf5_file_dir = util_python.dir_join_and_make(self.hdf5_file_dir, "segmentation/" + util_python.get_time_str())
            self.hdf5_object = hdf5_function.open_writed_hdf5(util_python.decide_allpath(self.hdf5_file_dir, self.hdf5_file_name))
        elif request.the_number_of_dataset == self.hdf5_service_counter:
            hdf5_function.close_hdf5(self.hdf5_object)
            hdf5_function.concatenate_hdf5(self.hdf5_file_dir, util_python.decide_allpath(self.hdf5_file_dir, self.hdf5_file_name))
        elif index == 0:
            hdf5_function.close_hdf5(self.hdf5_object)
            self.hdf5_object = hdf5_function.open_writed_hdf5(util_python.decide_allpath(self.hdf5_file_dir, self.hdf5_file_name))
        np_cloud = util_python.make_npcloud_from_cloud(request.cloud_data)
        # print(np_cloud.shape)
        np_cloud, np_mask = util_python.extract_mask_from_npcloud(np_cloud)
        # print(np_cloud.shape)
        # print(np_mask.shape)
        data_dict = {"Points": np_cloud, "masks": np_mask}
        hdf5_function.write_hdf5(self.hdf5_object, data_dict, index)
        self.bar.update(1)
        
        response = Hdf5RecordSegmentationResponse()
        response.ok = True
        return response
    
    def record_pose_estimation_service_callback(self, request):
        self.hdf5_service_counter = self.hdf5_service_counter + 1
        index = self.hdf5_service_counter % self.hdf5_save_interval
        if self.hdf5_service_counter == 1:
            self.bar = tqdm(total=request.the_number_of_dataset)
            self.bar.set_description("Progress rate")
            self.hdf5_file_dir = os.path.join(self.hdf5_file_dir, "pose_estimation")
            self.hdf5_object = hdf5_function.open_writed_hdf5(util_python.decide_allpath(self.hdf5_file_dir, self.hdf5_file_name))
        elif request.the_number_of_dataset == self.hdf5_service_counter:
            hdf5_function.close_hdf5(self.hdf5_object)
            hdf5_function.concatenate_hdf5(self.hdf5_file_dir, util_python.decide_allpath(self.hdf5_file_dir, self.hdf5_file_name))
        elif index == 0:
            hdf5_function.close_hdf5(self.hdf5_object)
            self.hdf5_object = hdf5_function.open_writed_hdf5(util_python.decide_allpath(self.hdf5_file_dir, self.hdf5_file_name))
        np_cloud = util_python.make_npcloud_from_cloud(request.cloud_data)
        np_cloud,  = util_python.extract_mask_from_npcloud(np_cloud)
        translation, rotation = util_python.make_pose_data(request.pose_datas)
        pose_mask = util_python.make_pose_mask(translation, rotation)
        data_dict = {"pcl": np_cloud, "pose": pose_mask}
        hdf5_function.write_hdf5(self.hdf5_object, data_dict, index)
        self.bar.update(1)
        response = Hdf5RecordPoseEstimationResponse()
        response.ok = True
        return response

    def record_clustering_service_callback(self, request):
        index = self.hdf5_service_counter % self.hdf5_save_interval
        if self.hdf5_service_counter == 0:
            self.bar = tqdm(total=request.the_number_of_dataset)
            self.bar.set_description("Progress rate")
            self.hdf5_file_dir = os.path.join(self.hdf5_file_dir, "clustering")
            self.hdf5_object = hdf5_function.open_writed_hdf5(util_python.decide_allpath(self.hdf5_file_dir, self.hdf5_file_name))
        elif request.the_number_of_dataset == self.hdf5_service_counter + 1:
            hdf5_function.close_hdf5(self.hdf5_object)
            hdf5_function.concatenate_hdf5(self.hdf5_file_dir, util_python.decide_allpath(self.hdf5_file_dir, self.hdf5_file_name))
        elif index == 0:
            hdf5_function.close_hdf5(self.hdf5_object)
            self.hdf5_object = hdf5_function.open_writed_hdf5(util_python.decide_allpath(self.hdf5_file_dir, self.hdf5_file_name))
        np_cloud = util_python.make_npcloud_from_cloud(request.cloud_data)
        np_cloud,  = util_python.extract_mask_from_npcloud(np_cloud)
        data_dict = {"pcl": np_cloud, "class": request.class_id}
        hdf5_function.write_hdf5(self.hdf5_object, data_dict, index)
        self.bar.update(1)
        self.hdf5_service_counter = self.hdf5_service_counter + 1
        response = Hdf5RecordClusteringResponse()
        response.ok = True
        return response
    
        

if __name__=='__main__':
    RecordServiceClass()
    rospy.spin()
    
