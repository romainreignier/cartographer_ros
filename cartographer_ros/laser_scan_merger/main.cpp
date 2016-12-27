#include "LaserScanMerger.h"
#include <ros/ros.h>

int main(int _argc, char** _argv)
{
    ros::init(_argc, _argv, "LaserScanMerger");

    LaserScanMerger scanMerger;

    ros::spin();
    return 0;
}
