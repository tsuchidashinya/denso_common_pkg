#!/usr/bin/env python3
import os
# from common_msgs.msg import CloudData
from common_srvs.srv import Hdf5RecordAcc, Hdf5RecordAccResponse, Hdf5RecordAccRequest
from common_srvs.srv import Hdf5RecordClustering, Hdf5RecordClusteringResponse, Hdf5RecordClusteringRequest
from common_srvs.srv import Hdf5RecordSegmentation, Hdf5RecordSegmentationResponse, Hdf5RecordSegmentationRequest
from common_srvs.srv import Hdf5RecordPoseEstimation, Hdf5RecordPoseEstimationResponse, Hdf5RecordPoseEstimationRequest
from common_srvs.srv import Hdf5RecordSensorData, Hdf5RecordSensorDataResponse, Hdf5RecordSensorDataRequest
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
        rospy.Service(self.record_real_sensor_service_name, Hdf5RecordSensorData, self.record_real_sensor_data_service_callback)

    def set_parameter(self):
        param_list = rosparam.get_param(rospy.get_name() + "/hdf5_record_server/")
        self.record_acc_name = "record_acc_service"
        self.record_segmentation_name = "record_segmentation_service"
        self.record_pose_estimation_name = "record_pose_estimation_service"
        self.record_clustering_name = "record_clustering_service"
        self.hdf5_save_interval = param_list["hdf5_save_interval"]
        self.record_real_sensor_service_name = "record_real_sensor_data_service"
        self.counter1 = 0
        self.counter2 = 0
        self.counter3 = 0
        self.counter4 = 0
        self.counter5 = 0
        self.hdf5_record_file_path1 = ""
        self.hdf5_record_file_path2 = ""
        self.hdf5_record_file_path3 = ""
        self.hdf5_record_file_path4 = ""
        self.hdf5_record_file_path5 = ""
        print("")
        
    def record_acc_service_callback(self, request):
        # request = Hdf5RecordAccRequest()
        response = Hdf5RecordAccResponse()
        record_dir, record_file_name = os.path.split(request.record_file_path)
        response.ok = True
        if self.hdf5_record_file_path1 != request.record_file_path:
            self.hdf5_record_file_path1 = request.record_file_path
            self.counter1 = 0
            self.record_temp_dir1 = util.dir_join_and_make(record_dir, util.exclude_ext_str(record_file_name) + util.get_timestr_ms())
            self.hdf5_read_dict1 = {}
            if os.path.exists(request.record_file_path):
                hdf5_object = hdf5_function.open_readed_hdf5(request.record_file_path)
                self.hdf5_read_dict1 = hdf5_function.load_hdf5_data_on_dict(hdf5_object)
                hdf5_function.close_hdf5(hdf5_object)
                os.remove(request.record_file_path)
        self.counter1 = self.counter1 + 1
        np_cam = util_msg_data.msgcam_to_npcam(request.camera_info)
        np_img = util_msg_data.rosimg_to_npimg(request.image)
        np_cloud = util_msg_data.msgcloud_to_npcloud(request.cloud_data)
        np_cloud, np_mask = util_msg_data.extract_mask_from_npcloud(np_cloud)
        translation, rotation, ins = util_msg_data.msgposelist_to_trans_rotate_ins(request.pose_data_list)
        data_dict = {"Points": np_cloud, "masks": np_mask, "translation": translation,
            "rotation": rotation, "image": np_img, "camera_info": np_cam, "instance": ins}
        if request.is_overwrite:
            if request.index == -1:
                self.hdf5_read_dict1 = hdf5_function.write_insert_dict(self.hdf5_read_dict1, data_dict)
            else:
                self.hdf5_read_dict1 = hdf5_function.write_overwrite_dict(self.hdf5_read_dict1, data_dict, request.index)
        else:
            self.hdf5_read_dict1 = hdf5_function.write_insert_dict(self.hdf5_read_dict1, data_dict)
        if request.is_end:
            hdf5_object = hdf5_function.open_writed_hdf5(request.record_file_path)
            hdf5_function.write_hdf5_from_dict(hdf5_object, self.hdf5_read_dict1)
            hdf5_function.close_hdf5(hdf5_object)
            print("save on ", request.record_file_path)
            response.save_temp_file_path = request.record_file_path
            self.hdf5_record_file_path1 = ""
            files = os.listdir(self.record_temp_dir1)
            for file in files:
                os.remove(os.path.join(self.record_temp_dir1, file))
            os.rmdir(os.path.join(self.record_temp_dir1))
            return response
        else:
            temp_filename = util.insert_str(record_file_name, util.get_timestr_hms())
            record_temp_all_path = util.decide_allpath(self.record_temp_dir1, temp_filename)
            hdf5_object = hdf5_function.open_writed_hdf5(record_temp_all_path)
            hdf5_function.write_hdf5_from_dict(hdf5_object, self.hdf5_read_dict1)
            hdf5_function.close_hdf5(hdf5_object)
            response.save_temp_file_path = record_temp_all_path
            return response
    
    def record_segmentation_service_callback(self, request):
        # request = Hdf5RecordSegmentationRequest()
        record_dir, record_file_name = os.path.split(request.record_file_path)
        rospy.set_param("record_counter", self.counter2)
        response = Hdf5RecordSegmentationResponse()
        if self.hdf5_record_file_path2 != request.record_file_path:
            self.hdf5_record_file_path2 = request.record_file_path
            self.counter2 = 0
            self.bar2 = tqdm(total=request.the_number_of_dataset)
            self.bar2.set_description("Progress rate")
            self.record_temp_dir2 = util.dir_join_and_make(record_dir, util.exclude_ext_str(record_file_name) + util.get_timestr_ms())
            self.hdf5_read_dict2 = {}
        self.counter2 = self.counter2 + 1
        if self.counter2 > request.the_number_of_dataset:
            response.finish = False
            return response
        index = self.counter2 % self.hdf5_save_interval
        np_cloud = util_msg_data.msgcloud_to_npcloud(request.cloud_data)
        np_cloud, np_mask = util_msg_data.extract_mask_from_npcloud(np_cloud)
        data_dict = {"Points": np_cloud, "masks": np_mask}
        self.hdf5_read_dict2 = hdf5_function.write_insert_dict(self.hdf5_read_dict2, data_dict)
        self.bar2.update(1)
        response.finish = 0
        if self.counter2 == request.the_number_of_dataset:
            final_path = request.record_file_path
            while True:
                if not os.path.exists(final_path):
                    break
                else:
                    final_dir, final_file_name = os.path.split(final_path)
                    final_file_name = util.insert_str(final_file_name, "copy")
                    final_path = os.path.join(final_dir, final_file_name)
            hdf5_object = hdf5_function.open_writed_hdf5(final_path)
            hdf5_function.write_hdf5_from_dict(hdf5_object, self.hdf5_read_dict2)
            hdf5_function.close_hdf5(hdf5_object)
            self.bar2.close()
            print("save on ", final_path)
            files = os.listdir(self.record_temp_dir2)
            for file in files:
                os.remove(os.path.join(self.record_temp_dir2, file))
            os.rmdir(os.path.join(self.record_temp_dir2))
            self.hdf5_record_file_path2 = ""
            response.finish = 1
        elif index == 0:
            temp_filename = util.insert_str(record_file_name, util.get_timestr_mdhms_under())
            record_temp_all_path = util.decide_allpath(self.record_temp_dir2, temp_filename)
            hdf5_object = hdf5_function.open_writed_hdf5(record_temp_all_path)
            hdf5_function.write_hdf5_from_dict(hdf5_object, self.hdf5_read_dict2)
            hdf5_function.close_hdf5(hdf5_object)
            response.finish = 0
        return response
    
    def record_pose_estimation_service_callback(self, request):
        # request = Hdf5RecordPoseEstimationRequest()
        response = Hdf5RecordPoseEstimationResponse()
        record_dir, record_file_name = os.path.split(request.record_file_path)
        rospy.set_param("record_counter", self.counter3)
        index = self.counter3 % self.hdf5_save_interval
        if self.hdf5_record_file_path3 != request.record_file_path or request.is_reload:
            self.hdf5_record_file_path3 = request.record_file_path
            self.bar3 = tqdm(total=request.the_number_of_dataset)
            self.bar3.set_description("Progress rate")
            self.record_temp_dir3 = util.dir_join_and_make(record_dir, util.exclude_ext_str(record_file_name) + util.get_timestr_ms())
            temp_filename = util.insert_str(record_file_name, util.get_timestr_hms())
            record_temp_all_path = util.decide_allpath(self.record_temp_dir3, temp_filename)
            self.hdf5_object3 = hdf5_function.open_writed_hdf5(record_temp_all_path)
        self.counter3 = self.counter3 + 1
        if self.counter3 > request.the_number_of_dataset:
            response.ok = False
            return response
        np_cloud = util_msg_data.msgcloud_to_npcloud(request.cloud_data)
        np_cloud,  = util_msg_data.extract_mask_from_npcloud(np_cloud)
        translation, rotation = util_msg_data.msgposelist_to_trans_rotate(request.pose_datas)
        pose_mask = util_msg_data.make_pose_mask(translation, rotation)
        data_dict = {"pcl": np_cloud, "pose": pose_mask}
        self.hdf5_read_dict3 = hdf5_function.write_insert_dict(self.hdf5_read_dict3, data_dict)
        self.bar3.update(1)
        index = self.counter3 % self.hdf5_save_interval
        if request.the_number_of_dataset == self.counter3:
            final_path = request.record_file_path
            while True:
                if not os.path.exists(final_path):
                    break
                else:
                    final_dir, final_file_name = os.path.split(final_path)
                    final_file_name = util.insert_str(final_file_name, "copy")
                    final_path = os.path.join(final_dir, final_file_name)
            hdf5_object = hdf5_function.open_writed_hdf5(final_path)
            hdf5_function.write_hdf5_from_dict(hdf5_object, self.hdf5_read_dict3)
            hdf5_function.close_hdf5(hdf5_object)
            self.bar3.close()
            print("save on ", final_path)
            files = os.listdir(self.record_temp_dir3)
            for file in files:
                os.remove(os.path.join(self.record_temp_dir3, file))
            os.rmdir(os.path.join(self.record_temp_dir3))
            self.hdf5_record_file_path3 = ""
        elif index == 0:
            temp_filename = util.insert_str(record_file_name, util.get_timestr_mdhms_under())
            record_temp_all_path = util.decide_allpath(self.record_temp_dir3, temp_filename)
            hdf5_object = hdf5_function.open_writed_hdf5(record_temp_all_path)
            hdf5_function.write_hdf5_from_dict(hdf5_object, self.hdf5_read_dict3)
            hdf5_function.close_hdf5(hdf5_object)
        response.ok = True
        return response

    def record_clustering_service_callback(self, request):
        # request = Hdf5RecordClusteringRequest()
        record_dir, record_file_name = os.path.split(request.record_file_path)
        response = Hdf5RecordClusteringResponse()
        rospy.set_param("record_counter", self.counter4)
        if self.hdf5_record_file_path4 != request.record_file_path:
            self.hdf5_record_file_path4 = request.record_file_path
            self.bar4 = tqdm(total=request.the_number_of_dataset)
            self.bar4.set_description("Progress rate")
            self.record_temp_dir4 = util.dir_join_and_make(record_dir, util.exclude_ext_str(record_file_name) + util.get_timestr_ms())
            temp_filename = util.insert_str(record_file_name, util.get_timestr_hms())
            record_temp_all_path = util.decide_allpath(self.record_temp_dir4, temp_filename)
            self.hdf5_object4 = hdf5_function.open_writed_hdf5(record_temp_all_path)
        self.counter4 = self.counter4 + 1
        if self.counter4 > request.the_number_of_dataset:
            response.ok = False
            return response
        np_cloud = util_msg_data.msgcloud_to_npcloud(request.cloud_data)
        np_cloud,  = util_msg_data.extract_mask_from_npcloud(np_cloud)
        data_dict = {"pcl": np_cloud, "class": request.class_id}
        self.hdf5_read_dict4 = hdf5_function.write_insert_dict(self.hdf5_read_dict4, data_dict)
        self.bar4.update(1)
        index = self.counter4 % self.hdf5_save_interval
        if request.the_number_of_dataset == self.counter4:
            final_path = request.record_file_path
            while True:
                if not os.path.exists(final_path):
                    break
                else:
                    final_dir, final_file_name = os.path.split(final_path)
                    final_file_name = util.insert_str(final_file_name, "copy")
                    final_path = os.path.join(final_dir, final_file_name)
            hdf5_object = hdf5_function.open_writed_hdf5(final_path)
            hdf5_function.write_hdf5_from_dict(hdf5_object, self.hdf5_read_dict4)
            hdf5_function.close_hdf5(hdf5_object)
            self.bar4.close()
            print("save on ", final_path)
            files = os.listdir(self.record_temp_dir4)
            for file in files:
                os.remove(os.path.join(self.record_temp_dir4, file))
            os.rmdir(os.path.join(self.record_temp_dir4))
            self.hdf5_record_file_path4 = ""
        elif index == 0:
            temp_filename = util.insert_str(record_file_name, util.get_timestr_mdhms_under())
            record_temp_all_path = util.decide_allpath(self.record_temp_dir4, temp_filename)
            hdf5_object = hdf5_function.open_writed_hdf5(record_temp_all_path)
            hdf5_function.write_hdf5_from_dict(hdf5_object, self.hdf5_read_dict4)
            hdf5_function.close_hdf5(hdf5_object)
        response.ok = True
        return response
    
    def record_real_sensor_data_service_callback(self, request):
        # request = Hdf5RecordSensorDataRequest()
        response = Hdf5RecordSensorDataResponse()
        record_dir, record_file_name = os.path.split(request.record_file_path)
        response.ok = True
        if self.hdf5_record_file_path5 != request.record_file_path:
            self.counter5 = 0
            self.hdf5_record_file_path5 = request.record_file_path
            self.record_temp_dir5 = util.dir_join_and_make(record_dir, util.exclude_ext_str(record_file_name) + util.get_timestr_ms())
            self.hdf5_read_dict5 = {}
            if os.path.exists(request.record_file_path):
                hdf5_object = hdf5_function.open_readed_hdf5(request.record_file_path)
                self.hdf5_read_dict5 = hdf5_function.load_hdf5_data_on_dict(hdf5_object)
                hdf5_function.close_hdf5(hdf5_object)
                os.remove(request.record_file_path)
        self.counter5 = self.counter5 + 1
        np_cloud = util_msg_data.msgcloud_to_npcloud(request.cloud_data)
        np_cloud, mask_cloud = util_msg_data.extract_mask_from_npcloud(np_cloud)
        np_cam = util_msg_data.msgcam_to_npcam(request.camera_info)
        np_img = util_msg_data.rosimg_to_npimg(request.image)
        data_dict = {"Points": np_cloud, "masks": mask_cloud, "image": np_img, "camera_info": np_cam}
        if request.is_overwrite:
            if request.index == -1:
                self.hdf5_read_dict5 = hdf5_function.write_insert_dict(self.hdf5_read_dict5, data_dict)
            else:
                self.hdf5_read_dict5 = hdf5_function.write_overwrite_dict(self.hdf5_read_dict5, data_dict, request.index)
        else:
            self.hdf5_read_dict5 = hdf5_function.write_insert_dict(self.hdf5_read_dict5, data_dict)
        if request.is_end:
            hdf5_object = hdf5_function.open_writed_hdf5(request.record_file_path)
            hdf5_function.write_hdf5_from_dict(hdf5_object, self.hdf5_read_dict5)
            hdf5_function.close_hdf5(hdf5_object)
            print("save on ", request.record_file_path)
            response.save_temp_file_path = request.record_file_path
            self.hdf5_record_file_path5 = ""
            files = os.listdir(self.record_temp_dir5)
            for file in files:
                os.remove(os.path.join(self.record_temp_dir5, file))
            os.rmdir(os.path.join(self.record_temp_dir5))
            return response
        else:
            temp_filename = util.insert_str(record_file_name, util.get_timestr_hms())
            record_temp_all_path = util.decide_allpath(self.record_temp_dir5, temp_filename)
            hdf5_object = hdf5_function.open_writed_hdf5(record_temp_all_path)
            hdf5_function.write_hdf5_from_dict(hdf5_object, self.hdf5_read_dict5)
            hdf5_function.close_hdf5(hdf5_object)
            response.save_temp_file_path = record_temp_all_path
            return response
        

if __name__=='__main__':
    RecordServiceClass()
    rospy.spin()
    
