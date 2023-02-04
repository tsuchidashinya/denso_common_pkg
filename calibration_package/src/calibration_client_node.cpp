#include <calibration_package/calibration_client.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "calibration_client");
    ros::NodeHandle nh;
    CalibrationClient calib_client(nh);
    calib_client.main();
    return 0;
}