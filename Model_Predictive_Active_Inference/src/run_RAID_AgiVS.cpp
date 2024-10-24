#include "RAID_AgiVS_for_test.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "optimizer_node");
    ros::NodeHandle nh("~");
    // 获取参数
    RAID_AgiVS controller;
    ros::spin();
    return 0;
}