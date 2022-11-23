import datetime
import os


def get_time_str():
    dt_now = datetime.datetime.now()
    time_str = str(dt_now.month) + "_" + str(dt_now.day) + "_" \
                + str(dt_now.hour) + "_" + str(dt_now.minute) + "_" \
                + str(dt_now.second)
    return time_str

def get_time_str_dir():
    dt_now = datetime.datetime.now()
    time_str = str(dt_now.minute)+ str(dt_now.second)
    return time_str
    
def get_time_str_not_second():
    dt_now = datetime.datetime.now()
    time_str = str(dt_now.month) + str(dt_now.day) \
                + str(dt_now.hour) + str(dt_now.minute)+ str(dt_now.second)
    return time_str

def exclude_ext_str(filename):
    root, _ = os.path.splitext(filename)
    return root

def insert_time_str(filename):
    root, ext = os.path.splitext(filename)
    return root + "_" + get_time_str_not_second() + ext

def ext_exist(basename):
    _, ext = os.path.splitext(basename)
    if len(ext) > 0:
        return True
    else:
        return False

def dir_join_and_make(origin_dir, add_dir):
    path = os.path.join(origin_dir, add_dir)
    make_dir(path)
    return path

def make_dir(path):
    if not os.path.exists(path):
        os.makedirs(path)

def decide_allpath(dir_path, file_path):
    dirname, basename = os.path.split(dir_path)
    if not ext_exist(basename):
        make_dir(dir_path)
        return str(os.path.join(dir_path, insert_time_str(file_path)))
    else:
        make_dir(dirname)
        return str(os.path.join(dirname, insert_time_str(file_path)))


        


