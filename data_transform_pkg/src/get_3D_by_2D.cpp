#include <data_transform_pkg/get_3D_by_2D.hpp>

Get3DBy2D::Get3DBy2D(std::vector<float> cinfo_list, ImageSize im_size)
{
    cinfo_list_ = cinfo_list;
    im_size_ = im_size;
}



std::vector<ArrayInt> Get3DBy2D::write_2d_instance_multi(std::vector<common_msgs::BoxPosition> b_boxs)
{
    std::vector<ArrayInt> outdata;
    outdata.resize(im_size_.height);
    for (int i = 0; i < outdata.size(); i++) {
        outdata[i].resize(im_size_.width);
        for (int j = 0; j < outdata[i].size(); j++) {
            outdata[i][j] = 0;
        }
    }
    for (int i = 0; i < b_boxs.size(); i++) {
        int x1 = int(b_boxs[i].x_one);
        int x2 = int(b_boxs[i].x_two);
        int y1 = int(b_boxs[i].y_one);
        int y2 = int(b_boxs[i].y_two);
        if (x1 > x2) {
            std::swap(x1, x2);
        }
        if (y1 > y2) {
            std::swap(y1, y2);
        }
        for (int k = y1; k <= y2; k++) {
            for (int l = x1; l <= x2; l++) {
                outdata[k][l] = i+1;
            }
        }
    }
    return outdata;
}

std::vector<ArrayInt> Get3DBy2D::write_2d_instance(common_msgs::BoxPosition b_box)
{
    std::vector<ArrayInt> outdata;
    outdata.resize(im_size_.height);
    for (int i = 0; i < outdata.size(); i++) {
        outdata[i].resize(im_size_.width);
        for (int j = 0; j < outdata[i].size(); j++) {
            outdata[i][j] = 0;
        }
    }
    int x1 = int(b_box.x_one);
    int x2 = int(b_box.x_two);
    int y1 = int(b_box.y_one);
    int y2 = int(b_box.y_two);
    if (x1 > x2) {
        std::swap(x1, x2);
    }
    if (y1 > y2) {
        std::swap(y1, y2);
    }
    for (int k = y1; k <= y2; k++) {
        for (int l = x1; l <= x2; l++) {
            outdata[k][l] = 1;
        }
    }
    return outdata;
}

std::vector<common_msgs::CloudData> Get3DBy2D::extract_data_multi(common_msgs::CloudData cloud, std::vector<ArrayInt> write_instance, std::vector<common_msgs::BoxPosition> b_boxs)
{
    std::vector<common_msgs::CloudData> outdata;
    outdata.resize(b_boxs.size());
    FuncDataConvertion data_convert;
    for (int i = 0; i < cloud.x.size(); i++) {
        Point3D point3d(cloud.x[i], cloud.y[i], cloud.z[i]);
        Point2D point2d = data_convert.func_3d_to_2d_pointcloud(point3d, cinfo_list_);
        if (FuncDataConvertion::point_is_in_image(point2d, im_size_, 1)) {
            int x = int(point2d.x);
            int y = int(point2d.y);
            if (write_instance[y][x] > 0) {
                int write_ins_point = write_instance[y][x] - 1;
                outdata[write_ins_point].x.push_back(cloud.x[i]);
                outdata[write_ins_point].y.push_back(cloud.y[i]);
                outdata[write_ins_point].z.push_back(cloud.z[i]);
                outdata[write_ins_point].instance.push_back(cloud.instance[i]);
                outdata[write_ins_point].cloud_name = b_boxs[write_ins_point].tf_name;
            }
        }
    }
    return outdata;
}

common_msgs::CloudData Get3DBy2D::extract_data(common_msgs::CloudData cloud, std::vector<ArrayInt> write_instance, common_msgs::BoxPosition b_box)
{
    common_msgs::CloudData outdata;
    FuncDataConvertion data_convert;
    for (int i = 0; i < cloud.x.size(); i++) {
        Point3D point3d(cloud.x[i], cloud.y[i], cloud.z[i]);
        Point2D point2d = data_convert.func_3d_to_2d_pointcloud(point3d, cinfo_list_);
        if (FuncDataConvertion::point_is_in_image(point2d, im_size_, 1)) {
            int x = int(point2d.x);
            int y = int(point2d.y);
            if (write_instance[y][x] == 1) {
                outdata.x.push_back(cloud.x[i]);
                outdata.y.push_back(cloud.y[i]);
                outdata.z.push_back(cloud.z[i]);
                outdata.instance.push_back(cloud.instance[i]);
                outdata.cloud_name = b_box.tf_name;
            }
        }
    }
    return outdata;
}

std::vector<common_msgs::CloudData> Get3DBy2D::get_out_data(common_msgs::CloudData cloud, std::vector<common_msgs::BoxPosition> b_boxs)
{
    std::vector<common_msgs::CloudData> out_data;
    out_data.resize(b_boxs.size());
    for (int i = 0; i < out_data.size(); i++) {
        std::vector<ArrayInt> image_instance = write_2d_instance(b_boxs[i]);
        out_data[i] = extract_data(cloud, image_instance, b_boxs[i]);
    }
    return out_data;
}