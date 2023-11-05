#include "track_bar/bar.hpp"
#include "ros/ros.h"
#include "std_srvs/Empty.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "intrinsic_bar_val");
    ros::NodeHandle nh("~");
    std::string bar_name = "bar";
    bar mybar(bar_name, nh);
    while(ros::ok())
    {
        char key = waitKey(1);
        if(key == 27)
        {
            break;
        }
    }
    ros::spin();
    return 0;
}