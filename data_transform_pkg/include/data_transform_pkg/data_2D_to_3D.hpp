#pragma once
#include "func_data_convertion.hpp"

class Data2Dto3D
{
public:
    Data2Dto3D(std::vector<float>, ImageSize);
    std::vector<common_msgs::CloudData> get_out_data(common_msgs::CloudData, std::vector<common_msgs::BoxPosition>);
    
    
private:
    std::vector<float> cinfo_list_;
    ImageSize im_size_;
    std::vector<ArrayInt> write_2d_instance_multi(std::vector<common_msgs::BoxPosition>);
    std::vector<ArrayInt> write_2d_instance(common_msgs::BoxPosition);
    std::vector<common_msgs::CloudData> extract_data_multi(common_msgs::CloudData, std::vector<ArrayInt>, std::vector<common_msgs::BoxPosition>);
    common_msgs::CloudData extract_data(common_msgs::CloudData, std::vector<ArrayInt>, common_msgs::BoxPosition);
};