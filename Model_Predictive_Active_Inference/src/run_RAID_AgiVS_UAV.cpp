#include "RAID_AgiVS_for_UAV.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "control_publisher");
    ros::NodeHandle nh("~");
    RAID_AgiVS controller(nh);
    // Start the spinner in the main thread
    ros::AsyncSpinner spinner(3);
    spinner.start();

    // Keep the main thread alive to process callbacks and signals
    ros::waitForShutdown();
    return 0;
}