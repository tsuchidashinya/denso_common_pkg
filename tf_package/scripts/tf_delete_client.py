#!/usr/bin/env python3
from common_srvs.srv import TfDeleteService, TfDeleteServiceRequest
import rospy
import rosparam
from util import util

if __name__=='__main__':
    rospy.init_node("tf_delete_client")
    param_list = rosparam.get_param(rospy.get_name() + "/tf_delete_client/")
    delete_tf_frame = param_list["delete_tf_frame"]
    tf_delete_service_name = "tf_delete_service"
    tf_delete_client = rospy.ServiceProxy(tf_delete_service_name, TfDeleteService)
    request = TfDeleteServiceRequest()
    request.delete_tf_name = delete_tf_frame
    res = util.client_request(tf_delete_client, request, tf_delete_service_name)
