from tf.transformations import quaternion_matrix
import numpy as np
import tf2_ros
import rospy

def conv_quat2mat(vec):
    conv_vec = np.zeros(12)
    conv_vec[0:3] = vec[0:3]
    trans_euler =quaternion_matrix(vec[3:7])
    conv_vec[3:12] = trans_euler[0:3, 0:3].reshape(9)
    return conv_vec

class TfUtil:
    def __init__(self):
        self.br = tf2_ros.TransformBroadcaster()
        self.static_br = tf2_ros.StaticTransformBroadcaster()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
    
    def broadcast(self, trans_stamp):
        trans_stamp.header.stamp = rospy.Time.now()
        self.br.sendTransform(trans_stamp)
    
    def static_broadcast(self, trans_stamp):
        trans_stamp.header.stamp = rospy.Time.now()
        self.static_br.sendTransform(trans_stamp)
    
    def tf_listen(self, target_frame, source_frame):
        while True:
            try:
                trans = self.tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time())
                break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.sleep(0.1)
                continue
        return trans.transform
        
        