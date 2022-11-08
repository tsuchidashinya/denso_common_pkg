#include <data_convert_pkg/get_3D_by_2D.hpp>

Get3DBy2D::Get3DBy2D(sensor_msgs::CameraInfo cinfo, ImageSize im_size)
{
    cinfo_ = cinfo;
    im_size_ = im_size;
}



std::vector<ArrayInt> Get3DBy2D::write_2d_instance(std::vector<common_msgs::BoxPosition> b_boxs, ImageSize im_size)
{
    std::vector<ArrayInt> outdata;
    outdata.resize(im_size.height);
    for (int i = 0; i < outdata.size(); i++) {
        outdata[i].resize(im_size.width);
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

std::vector<common_msgs::CloudData> Get3DBy2D::extract_data(common_msgs::CloudData cloud, std::vector<ArrayInt> write_instance, ImageSize im_size)
{
    std::vector<common_msgs::CloudData> outdata;
    outdata.resize(box_num_);
    FuncDataConvertion data_convert;
    for (int i = 0; i < cloud.x.size(); i++) {
        Point3D point3d(cloud.x[i], cloud.y[i], cloud.z[i]);
        Point2D point2d = data_convert.func_3d_to_2d_pointcloud(point3d, cinfo_);
        if (FuncDataConvertion::point_is_in_image(point2d, im_size, 1)) {
            int x = int(point2d.x);
            int y = int(point2d.y);
            if (write_instance[y][x] > 0) {
                outdata[write_instance[y][x] - 1].x.push_back(cloud.x[i]);
                outdata[write_instance[y][x] - 1].y.push_back(cloud.y[i]);
                outdata[write_instance[y][x] - 1].z.push_back(cloud.z[i]);
                outdata[write_instance[y][x] - 1].instance.push_back(0);
            }
        }
    }
    return outdata;
}

std::vector<common_msgs::CloudData> Get3DBy2D::get_out_data(common_msgs::CloudData cloud, std::vector<common_msgs::BoxPosition> b_boxs)
{
    std::vector<ArrayInt> image_instance = write_2d_instance(b_boxs, im_size_);
    return extract_data(cloud, image_instance, im_size_);
}