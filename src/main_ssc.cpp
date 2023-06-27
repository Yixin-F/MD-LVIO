#include "ssc/ssc.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "ssc");
    // ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug); 
    ROS_INFO("\033[1;32m----> ssc Started.\033[0m");

    // test 2023.6.28
    // ocl

    ros::spin();

    return 0;
}