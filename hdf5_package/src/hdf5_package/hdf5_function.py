import os
import h5py


def change_data(input_file_path, out_file_path):
    keys_1 = []
    keys_2 = []
    all_data = []
    h_object = open_readed_hdf5(input_file_path)
    for k1 in h_object.keys():
        keys_1.append(k1)
    for k2 in h_object[keys_1[0]].keys():
        print(k2)
        keys_2.append(k2)
    for i in range(len(keys_1)):
        part_data = []
        for j in range(len(keys_2)):
            part_data.append(h_object[keys_1[i]][keys_2[j]][()])
        all_data.append(part_data)
    f = h_w_object = open_writed_hdf5(out_file_path)
    for i in range(len(all_data)):
        print("data_" + str(i))
        f.create_group('data_' + str(i+1))
        for j in range(len(keys_2)):
            if keys_2[j] == "masks":
                for k in range(all_data[i][j].shape[0]):
                    if all_data[i][j][k] == 2 or all_data[i][j][k] == 3:
                        # print(all_data[i][j][k])
                        all_data[i][j][k] = 0
            f['data_' + str(i+1)].create_dataset(keys_2[j], data=all_data[i][j], compression="lzf")
    


def concatenate_hdf5(dir_path, out_file_path):
    files = os.listdir(dir_path)
    keys_1_all = []
    keys_2 = []
    all_data = []
    for i in range(len(files)):
        with h5py.File(os.path.join(dir_path, files[i]), mode="r") as f:
            if i == 0:
                keys_1 = []
                for k1 in f.keys():
                    keys_1.append(k1)
                for k2 in f[keys_1[0]].keys():
                    keys_2.append(k2)
                keys_1_all.append(keys_1)
            else:
                keys_1 = []
                for k1 in f.keys():
                    keys_1.append(k1)
                keys_1_all.append(keys_1)
    for i in range(len(files)):
        keys_1 = []
        with h5py.File(os.path.join(dir_path, files[i]), mode="r") as f:
            for j in range(len(f.keys())):
                part_data = []
                for k in range(len(keys_2)):
                    part_data.append(f[keys_1_all[i][j]][keys_2[k]][()])
                all_data.append(part_data)
    for ff in files:
        os.remove(os.path.join(dir_path, ff))
    with h5py.File(out_file_path, mode="w") as f:
        for i in range(len(all_data)):
            f.create_group('data_' + str(i))
            for j in range(len(keys_2)):
                f['data_' + str(i)].create_dataset(keys_2[j], data=all_data[i][j], compression="lzf")
    


def write_hdf5(h5pyObject, data_dict, index):
    # h5pyObject = h5py.File()
    index_str = "data_" + str(index)
    is_exist = index_str in h5pyObject.keys()
    if is_exist:
        for key, value in data_dict.items():
            h5pyObject[index_str][key] = value
            data_group = h5pyObject[index_str]
            data_group.update()
            # data_group.create_dataset(key, data=value, compression="lzf")
    else:   
        data_group = h5pyObject.create_group(index_str)
        for key, value in data_dict.items():
            data_group.create_dataset(key, data=value, compression="lzf")
    h5pyObject.flush()

def write_overwrite_hdf5(h5pyObject_write, h5pyObject_read, data_dict, index_update):
    # h5pyObject_write = h5py.File()
    # h5pyObject_read = h5py.File()
    update_index = "data_" + str(index_update)
    for index in h5pyObject_read.keys():
        data_group = h5pyObject_write.create_group(index)
        if index == update_index:  
            for key, value in data_dict.items():
                data_group.create_dataset(key, data=value, compression="lzf")
        else:
            for key, value in h5pyObject_read[index].items():
                data_group.create_dataset(key, data=value, compression="lzf")
    h5pyObject_write.flush()

def write_overwrite_dict(hdf5_read_dict, data_dict, index_update):
    for key, value in data_dict.items():
        hdf5_read_dict["data_" + str(index_update)][key] = value
    return hdf5_read_dict

def load_hdf5_data_on_dict(hdf5_object):
    hdf5_read_dict = {}
    for key in hdf5_object.keys():
        key2_all = {}
        for key2 in hdf5_object[key].keys():
            key2_all[key2] = hdf5_object[key][key2][()]
        hdf5_read_dict[key] = key2_all
    return hdf5_read_dict

def write_hdf5_from_dict(h5pyObject_write, h5pyObject_read):
    for index in h5pyObject_read.keys():
        data_group = h5pyObject_write.create_group(index)
        for key, value in h5pyObject_read[index].items():
            data_group.create_dataset(key, data=value, compression="lzf")
        h5pyObject_write.flush()

def write_insert_hdf5(h5pyObject_write, h5pyObject_read, data_dict):
    # h5pyObject_write = h5py.File()
    # h5pyObject_read = h5py.File()
    start_index_num = 0
    for index in h5pyObject_read.keys():
        start_index_num = start_index_num + 1
        data_group = h5pyObject_write.create_group(index)
        for key, value in h5pyObject_read[index].items():
            data_group.create_dataset(key, data=value, compression="lzf")
    data_group = h5pyObject_write.create_group("data_" + str(start_index_num))
    for key, value in data_dict.items():
        data_group.create_dataset(key, data=value, compression="lzf")
    h5pyObject_write.flush()

def write_insert_dict(hdf5_read_dict, data_dict):
    start_index_num = 0
    for _ in hdf5_read_dict.keys():
        start_index_num = start_index_num + 1
    update_dict = {"data_" + str(start_index_num): data_dict}
    hdf5_read_dict.update(update_dict)
    # hdf5_read_dict["data_" + str(start_index_num)][key] = value
    return hdf5_read_dict


def open_writed_hdf5(file_path):
    if not os.path.exists(os.path.dirname(file_path)):
        print("hdf5 file is not found!!")
        raise Exception
    return h5py.File(file_path, "w")

def close_hdf5(h5pyObject):
    h5pyObject.flush()
    h5pyObject.close()

def open_readed_hdf5(file_path):
    if not os.path.exists(file_path):
        print("hdf5 file is not found!!")
        raise Exception
    return h5py.File(file_path, "r")

def get_len_hdf5(hdf5_object):
    file_count = 0
    for k1 in hdf5_object:
        file_count += 1
    return file_count

if __name__=='__main__':
    path = "/media/ericlab/DE59-9C00/test/concate"
    concatenate_hdf5(path, os.path.join(path, "real_HV8_data.hdf5"))
    # input_path = "/home/ericlab/tsuchida/2022_12/annotation/Semseg/multi_object/kai3228/kai.hdf5"
    # out_path = "/home/ericlab/tsuchida/2022_12/annotation/Semseg/multi_object/kai3228/kai_1.hdf5"
    # change_data(input_path, out_path)
