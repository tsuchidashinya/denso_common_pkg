#!/usr/bin/env python3
import os
# from common_msgs.msg import CloudData
from common_srvs.srv import Hdf5RecordAcc, Hdf5RecordAccResponse, Hdf5RecordAccRequest
from common_srvs.srv import Hdf5RecordClustering, Hdf5RecordClusteringResponse, Hdf5RecordClusteringRequest
from common_srvs.srv import Hdf5RecordSegmentation, Hdf5RecordSegmentationResponse, Hdf5RecordSegmentationRequest
from common_srvs.srv import Hdf5RecordPoseEstimation, Hdf5RecordPoseEstimationResponse, Hdf5RecordPoseEstimationRequest
from common_srvs.srv import Hdf5RecordRealSensorData, Hdf5RecordRealSensorDataResponse, Hdf5RecordRealSensorDataRequest
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
        rospy.Service(self.record_acc_name, Hdf5RecordAcc, self.record_acc_service_callback)
        rospy.Service(self.record_segmentation_name, Hdf5RecordSegmentation, self.record_segmentation_service_callback)
        rospy.Service(self.record_pose_estimation_name, Hdf5RecordPoseEstimation, self.record_pose_estimation_service_callback)
        rospy.Service(self.record_clustering_name, Hdf5RecordClustering, self.record_clustering_service_callback)
        rospy.Service(self.record_real_sensor_service_name, Hdf5RecordRealSensorData, self.record_real_sensor_data_service_callback)

    # def hdf5_initialize(self):
    #     self.hdf5_file_dir = util.dir_join_and_make(self.hdf5_file_dir, util.exclude_ext_str(self.hdf5_file_name) + util.get_timestr_ms())
    #     filename = util.insert_str(self.hdf5_file_name, util.get_timestr_hms())
    #     self.hdf5_object = hdf5_function.open_writed_hdf5(util.decide_allpath(self.hdf5_file_dir, filename))
    def set_parameter(self):
        param_list = rosparam.get_param(rospy.get_name() + "/hdf5_record_server/")
        self.record_acc_name = param_list["record_acc_service_name"]
        self.record_segmentation_name = param_list["record_segmentation_service_name"]
        self.record_pose_estimation_name = param_list["record_pose_estimation_service_name"]
        self.record_clustering_name = param_list["record_clustering_service_name"]
        self.hdf5_file_dir = param_list["record_hdf5_file_dir"]
        self.hdf5_file_name = param_list["record_hdf5_file_name"]
        self.hdf5_save_interval = param_list["hdf5_save_interval"]
        self.record_real_sensor_service_name = param_list["record_real_sensor_data_service_name"]
        self.counter = 0
        self.hdf5_record_file_path = ""
        
    def record_acc_service_callback(self, request):
        # request = Hdf5RecordAccRequest()
        response = Hdf5RecordAccResponse()
        response.ok = True
        if self.hdf5_record_file_path != request.record_file_path:
            self.counter = 0
            self.bar = tqdm(total=request.the_number_of_dataset)
            self.bar.set_description("Progress rate")
            if os.path.exists(request.record_file_path):
                self.hdf5_object = hdf5_function.open_readed_hdf5(request.record_file_path)
            else:
                self.hdf5_object = hdf5_function.open_writed_hdf5(request.record_file_path)
        np_cam = util_msg_data.msgcam_to_npcam(request.camera_info)
        np_img = util_msg_data.rosimg_to_npimg(request.image)
        np_cloud = util_msg_data.msgcloud_to_npcloud(request.cloud_data)
        np_cloud, np_mask = util_msg_data.extract_mask_from_npcloud(np_cloud)
        translation, rotation = util_msg_data.msgposelist_to_trans_rotate(request.pose_data_list)
        data_dict = {"Points": np_cloud, "masks": np_mask, "translation": translation,
            "rotation": rotation, "image": np_img, "camera_info": np_cam}
        if os.path.exists(request.record_file_path):
            hdf5_read_dict = {}
            for key in self.hdf5_object.keys():
                key2_all = {}
                for key2 in self.hdf5_object[key].keys():
                    key2_all[key2] = self.hdf5_object[key][key2][()]
                hdf5_read_dict[key] = key2_all
            hdf5_function.close_hdf5(self.hdf5_object)
            os.remove(request.record_file_path)
            hdf5_write_object = hdf5_function.open_writed_hdf5(request.record_file_path)
            hdf5_function.write_update_hdf5(hdf5_write_object, hdf5_read_dict, data_dict, request.index)
        else:
            hdf5_function.write_hdf5(self.hdf5_object, data_dict, self.counter)
        self.counter = self.counter + 1
        if self.counter > request.the_number_of_dataset:
            return response
        elif request.the_number_of_dataset == self.counter:
            if os.path.exists(request.record_file_path):
                hdf5_function.close_hdf5(hdf5_write_object)
            else:
                hdf5_function.close_hdf5(self.hdf5_object)
            self.bar.update(1)
            return response
        else:
            self.bar.update(1)
            return response
    
    def record_segmentation_service_callback(self, request):
        # request = Hdf5RecordSegmentationRequest()
        record_dir, record_file_name = os.path.split(request.record_file_path)
        rospy.set_param("record_counter", self.counter)
        response = Hdf5RecordSegmentationResponse()
        index = self.counter % self.hdf5_save_interval
        if self.counter == 0:
            self.bar = tqdm(total=request.the_number_of_dataset)
            self.bar.set_description("Progress rate")
            self.record_temp_dir = util.dir_join_and_make(record_dir, util.exclude_ext_str(record_file_name) + util.get_timestr_ms())
            temp_filename = util.insert_str(record_file_name, util.get_timestr_hms())
            record_temp_all_path = util.decide_allpath(self.record_temp_dir, temp_filename)
            self.hdf5_object = hdf5_function.open_writed_hdf5(record_temp_all_path)
        elif self.counter == request.the_number_of_dataset:
            hdf5_function.close_hdf5(self.hdf5_object)
            final_path = request.record_file_path
            while True:
                if not os.path.exists(final_path):
                    break
                else:
                    final_dir, final_file_name = os.path.split(final_path)
                    final_file_name = util.insert_str(final_file_name, "_copy")
                    final_path = os.path.join(final_dir, final_file_name)
            hdf5_function.concatenate_hdf5(self.record_temp_dir, final_path)
            os.remove(os.path.join(self.record_temp_dir))
            self.bar.update(1)
            response.ok = True
            return response
        elif self.counter > request.the_number_of_dataset:
            response.ok = False
            return response
        elif index == 0:
            hdf5_function.close_hdf5(self.hdf5_object)
            temp_filename = util.insert_str(record_file_name, util.get_timestr_hms())
            self.hdf5_object = hdf5_function.open_writed_hdf5(util.decide_allpath(record_dir, temp_filename))
        np_cloud = util_msg_data.msgcloud_to_npcloud(request.cloud_data)
        np_cloud, np_mask = util_msg_data.extract_mask_from_npcloud(np_cloud)
        data_dict = {"Points": np_cloud, "masks": np_mask}
        hdf5_function.write_hdf5(self.hdf5_object, data_dict, index)
        self.bar.update(1)
        response.ok = True
        self.counter = self.counter + 1
        return response
    
    def record_pose_estimation_service_callback(self, request):
        # request = Hdf5RecordPoseEstimationRequest()
        response = Hdf5RecordPoseEstimationResponse()
        record_dir, record_file_name = os.path.split(request.record_file_path)
        rospy.set_param("record_counter", self.counter)
        index = self.counter % self.hdf5_save_interval
        if self.counter == 0:
            self.bar = tqdm(total=request.the_number_of_dataset)
            self.bar.set_description("Progress rate")
            self.record_temp_dir = util.dir_join_and_make(record_dir, util.exclude_ext_str(record_file_name) + util.get_timestr_ms())
            temp_filename = util.insert_str(record_file_name, util.get_timestr_hms())
            record_temp_all_path = util.decide_allpath(self.record_temp_dir, temp_filename)
            self.hdf5_object = hdf5_function.open_writed_hdf5(record_temp_all_path)
        elif request.the_number_of_dataset == self.counter:
            hdf5_function.close_hdf5(self.hdf5_object)
            final_path = request.record_file_path
            while True:
                if not os.path.exists(final_path):
                    break
                else:
                    final_dir, final_file_name = os.path.split(final_path)
                    final_file_name = util.insert_str(final_file_name, "_copy")
                    final_path = os.path.join(final_dir, final_file_name)
            hdf5_function.concatenate_hdf5(self.record_temp_dir, final_path)
            os.remove(os.path.join(self.record_temp_dir))
            self.bar.update(1)
            response.ok = True
            return response
        elif self.counter > request.the_number_of_dataset:
            response.ok = False
            return response
        elif index == 0:
            hdf5_function.close_hdf5(self.hdf5_object)
            temp_filename = util.insert_str(record_file_name, util.get_timestr_hms())
            self.hdf5_object = hdf5_function.open_writed_hdf5(util.decide_allpath(record_dir, temp_filename))
        np_cloud = util_msg_data.msgcloud_to_npcloud(request.cloud_data)
        np_cloud,  = util_msg_data.extract_mask_from_npcloud(np_cloud)
        translation, rotation = util_msg_data.msgposelist_to_trans_rotate(request.pose_datas)
        pose_mask = util_msg_data.make_pose_mask(translation, rotation)
        data_dict = {"pcl": np_cloud, "pose": pose_mask}
        hdf5_function.write_hdf5(self.hdf5_object, data_dict, index)
        self.bar.update(1)
        response.ok = True
        self.counter = self.counter + 1
        return response

    def record_clustering_service_callback(self, request):
        # request = Hdf5RecordClusteringRequest()
        record_dir, record_file_name = os.path.split(request.record_file_path)
        response = Hdf5RecordClusteringResponse()
        rospy.set_param("record_counter", self.counter)
        index = self.counter % self.hdf5_save_interval
        if self.counter == 0:
            self.bar = tqdm(total=request.the_number_of_dataset)
            self.bar.set_description("Progress rate")
            self.record_temp_dir = util.dir_join_and_make(record_dir, util.exclude_ext_str(record_file_name) + util.get_timestr_ms())
            temp_filename = util.insert_str(record_file_name, util.get_timestr_hms())
            record_temp_all_path = util.decide_allpath(self.record_temp_dir, temp_filename)
            self.hdf5_object = hdf5_function.open_writed_hdf5(record_temp_all_path)
        elif request.the_number_of_dataset == self.counter + 1:
            hdf5_function.close_hdf5(self.hdf5_object)
            final_path = request.record_file_path
            while True:
                if not os.path.exists(final_path):
                    break
                else:
                    final_dir, final_file_name = os.path.split(final_path)
                    final_file_name = util.insert_str(final_file_name, "_copy")
                    final_path = os.path.join(final_dir, final_file_name)
            hdf5_function.concatenate_hdf5(self.record_temp_dir, final_path)
            os.remove(os.path.join(self.record_temp_dir))
            self.bar.update(1)
            response.ok = True
            return response
        elif self.counter > request.the_number_of_dataset:
            response.ok = False
            return response
        elif index == 0:
            hdf5_function.close_hdf5(self.hdf5_object)
            temp_filename = util.insert_str(record_file_name, util.get_timestr_hms())
            self.hdf5_object = hdf5_function.open_writed_hdf5(util.decide_allpath(record_dir, temp_filename))
        np_cloud = util_msg_data.msgcloud_to_npcloud(request.cloud_data)
        np_cloud,  = util_msg_data.extract_mask_from_npcloud(np_cloud)
        data_dict = {"pcl": np_cloud, "class": request.class_id}
        hdf5_function.write_hdf5(self.hdf5_object, data_dict, index)
        self.bar.update(1)
        self.counter = self.counter + 1
        response.ok = True
        return response
    
    def record_real_sensor_data_service_callback(self, request):
        # request = Hdf5RecordRealSensorDataRequest()
        record_dir, record_file_name = os.path.split(request.record_file_path)
        response = Hdf5RecordRealSensorDataResponse()
        response.ok = True
        self.counter = self.counter + 1
        index = self.counter % self.hdf5_save_interval
        if self.counter == 1:
            self.bar = tqdm(total=request.the_number_of_dataset)
            self.bar.set_description("Progress rate")
            self.record_temp_dir = util.dir_join_and_make(record_dir, util.exclude_ext_str(record_file_name) + util.get_timestr_ms())
            temp_filename = util.insert_str(record_file_name, util.get_timestr_hms())
            record_temp_all_path = util.decide_allpath(self.record_temp_dir, temp_filename)
            self.hdf5_object = hdf5_function.open_writed_hdf5(record_temp_all_path)
        elif request.the_number_of_dataset == self.counter:
            self.bar.update(1)
            hdf5_function.close_hdf5(self.hdf5_object)
            final_path = request.record_file_path
            while True:
                if not os.path.exists(final_path):
                    break
                else:
                    final_dir, final_file_name = os.path.split(final_path)
                    final_file_name = util.insert_str(final_file_name, "_copy")
                    final_path = os.path.join(final_dir, final_file_name)
            hdf5_function.concatenate_hdf5(self.record_temp_dir, final_path)
            os.remove(os.path.join(self.record_temp_dir))
            self.bar.update(1)
            response.ok = True
            return response
        elif index == 0:
            hdf5_function.close_hdf5(self.hdf5_object)
            temp_filename = util.insert_str(record_file_name, util.get_timestr_hms())
            self.hdf5_object = hdf5_function.open_writed_hdf5(util.decide_allpath(record_dir, temp_filename))
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
    
