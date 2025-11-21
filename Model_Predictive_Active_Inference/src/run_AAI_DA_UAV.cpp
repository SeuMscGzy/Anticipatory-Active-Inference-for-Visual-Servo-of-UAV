#include "AAI_DA.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "control_publisher");
    ros::NodeHandle nh("~");
    AAI_DA controller(nh);
    // Start the spinner in the main thread
    ros::spin();
    return 0;
}