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
                        print(all_data[i][j][k])
                        all_data[i][j][k] = 0
            f['data_' + str(i+1)].create_dataset(keys_2[j], data=all_data[i][j], compression="lzf")
    


def concatenate_hdf5(dir_path, out_file_name):
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
    with h5py.File(os.path.join(dir_path, out_file_name), mode="w") as f:
        for i in range(len(all_data)):
            f.create_group('data_' + str(i+1))
            for j in range(len(keys_2)):
                f['data_' + str(i+1)].create_dataset(keys_2[j], data=all_data[i][j], compression="lzf")
    for ff in files:
        os.remove(os.path.join(dir_path, ff))


def write_hdf5(h5pyObject, data_dict, index):
    data_group = h5pyObject.create_group("data_" + str(index))
    for key, value in data_dict.items():
        data_group.create_dataset(key, data=value, compression="lzf")
    h5pyObject.flush()

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
    # path = "/home/dl-box/tsuchida/2022_11/annotation/sum"
    # concatenate_hdf5(path, "acc_real.hdf5")
    input_path = "/home/ericlab/tsuchida/2022_11/annotation/segmentation/multi_object_randomize/temp/saikou.hdf5"
    out_path = "/home/ericlab/tsuchida/2022_11/annotation/segmentation/multi_object_randomize/temp/saikou_1.hdf5"
    change_data(input_path, out_path)
