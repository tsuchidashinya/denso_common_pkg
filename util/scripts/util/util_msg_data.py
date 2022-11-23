import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from common_msgs.msg import CloudData, PoseData


def get_instance_dict(cloud):
    info = {}
    instance_list = []
    for i in range(len(cloud.x)):
        exist = cloud.instance[i] in instance_list
        if not exist:
            instance_list.append(cloud.instance[i])
            info[str(int(cloud.instance[i]))] = 0
        info[str(int(cloud.instance[i]))] += 1
    return info

def get_max_instance(cloud):
    max = 0
    for i in range(len(cloud.x)):
        if max < cloud.instance[i]:
            max = cloud.instance[i]
    return max

def msgposelist_to_trans_rotate(pose):
    data_size = len(pose)
    translation = np.zeros((data_size, 3), dtype=np.float32)
    rotation = np.zeros((data_size, 4), dtype=np.float32)
    for i in range(data_size):
        translation[i, 0] = pose[i].trans.x
        translation[i, 1] = pose[i].trans.y
        translation[i, 2] = pose[i].trans.z
        rotation[i, 0] = pose[i].rot.x
        rotation[i, 1] = pose[i].rot.y
        rotation[i, 2] = pose[i].rot.z
        rotation[i, 3] = pose[i].rot.w
    return translation, rotation

def trans_rotate_to_msgposelist(translation, rotation):
    pose_list = []
    for i in range(translation.shape[0]):
        pose = PoseData()
        pose.translation.x = translation[i, 0]
        pose.translation.y = translation[i, 1]
        pose.translation.z = translation[i, 2]
        pose.rotation.x = rotation[i, 0]
        pose.rotation.y = rotation[i, 1]
        pose.rotation.z = rotation[i, 2]
        pose.rotation.w = rotation[i, 3]
        pose_list.append(pose)
    return pose_list

def make_pose_mask(translation, rotation):
    pose_mask = np.concatenate([translation, rotation])
    return pose_mask
    
def rosimg_to_npimg(rosimg):
    bridge = CvBridge()
    np_img = bridge.imgmsg_to_cv2(rosimg, "bgr8")
    np_img = np.array(np_img)
    return np_img

def npimg_to_rosimg(npimg):
    bridge = CvBridge()
    try:
        rosimg = bridge.cv2_to_imgmsg(npimg, "bgr8")
    except CvBridgeError as e:
        print(e)
    return rosimg

def roscam_to_npcam(roscam):
    npcam = np.empty((0))
    for info in roscam.K:
        npcam = np.append(npcam, info)
    return npcam

def msgcam_to_npcam(msgcam):
    npcam = np.empty((0))
    for cam in msgcam:
        npcam = np.append(npcam, cam)
    return npcam

def npcam_to_msgcam(npcam):
    msgcam = []
    for i in range(npcam.shape[0]):
        msgcam.append(float(npcam[i]))
    return msgcam

def msgcloud_to_npcloud(cloud_data):
    np_cloud = np.array((cloud_data.x, cloud_data.y, cloud_data.z, cloud_data.instance))
    np_cloud = np_cloud.T
    return np_cloud

def npcloud_to_msgcloud(npcloud):
    msgcloud = CloudData()
    for i in range(npcloud.shape[0]):
        msgcloud.x.append(npcloud[i, 0])
        msgcloud.y.append(npcloud[i, 1])
        msgcloud.z.append(npcloud[i, 2])
        msgcloud.instance.append(int(npcloud[i, 3]))
    return msgcloud

def extract_mask_from_npcloud(npcloud):
    npcloud = npcloud.T
    new_np_cloud = npcloud[:][0:3]
    np_mask = npcloud[:][3:]
    new_np_cloud = new_np_cloud.T 
    np_mask = np_mask.T
    return new_np_cloud, np_mask

def extract_ins_cloud_msg(cloud, ins):
    outcloud = CloudData()
    for i in range(len(cloud.x)):
        if (cloud.instance[i] == ins):
            outcloud.x.append(cloud.x[i])
            outcloud.y.append(cloud.y[i])
            outcloud.z.append(cloud.z[i])
            outcloud.instance.append(cloud.instance[i])
    return outcloud

def concatenate_npcloud_and_npmask(npcloud, npmask):
    outdata = np.hstack([npcloud, npmask])
    return outdata
    