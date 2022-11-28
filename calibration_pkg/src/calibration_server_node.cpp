#include <calibration_pkg/calibration_server.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "calibration_server");
    ros::NodeHandle nh;
    CalibrationServer calib_server(nh);
    ros::spin();
    return 0;
}