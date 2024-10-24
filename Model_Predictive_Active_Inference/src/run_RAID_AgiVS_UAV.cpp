#include "RAID_AgiVS_for_UAV.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "control_publisher");
    ros::NodeHandle nh("~");
    
    int index;
    nh.param("index", index, 0);

    RAID_AgiVS controller(index);

    ros::spin();
    return 0;
}