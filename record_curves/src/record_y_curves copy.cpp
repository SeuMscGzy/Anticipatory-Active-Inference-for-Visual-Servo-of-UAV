#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <fstream>
using namespace std;

class DataRecorder
{
public:
    DataRecorder()
    {
        // Initialize ROS node handle
        ros::NodeHandle nh;

        // Subscribe to the /hat_x_topic
        subscriber_ = nh.subscribe("/hat_x_topic", 0, &DataRecorder::callback, this);

        // Create or overwrite the CSV file
        file_.open("data.csv", std::ofstream::out | std::ofstream::trunc);
        // Write header row to CSV
        file_ << "data[0],data[1],data[2],data[3],data[4],data[5],data[6],data[7],data[8],data[9],"
                 "data[10],data[11],data[12],data[13],data[14],data[15],data[16],data[17],data[18],data[19],data[20],data[21],data[22],data[23],data[24]\n";
    }

    ~DataRecorder()
    {
        // Close the file on shutdown
        file_.close();
    }

    void callback(const std_msgs::Float64MultiArray::ConstPtr &msg)
    {
        // std::cout << msg->data.size() << std::endl;
        if (msg->data.size() >= 25)
        {
            // Write each data element as comma-separated values
            file_ << msg->data[0] << "," << msg->data[1] << "," << msg->data[2] << ","
                  << msg->data[3] << "," << msg->data[4] << "," << msg->data[5] << ","
                  << msg->data[6] << "," << msg->data[7] << "," << msg->data[8] << ","
                  << msg->data[9] << "," << msg->data[10] << "," << msg->data[11] << ","
                  << msg->data[12] << "," << msg->data[13] << "," << msg->data[14] << ","
                  << msg->data[15] << "," << msg->data[16] << "," << msg->data[17] << ","
                  << msg->data[18] << "," << msg->data[19] << "," << msg->data[20] << ","
                  << msg->data[21] << "," << msg->data[22] << "," << msg->data[23] << "," << msg->data[24] << "\n";
        }
    }

private:
    ros::Subscriber subscriber_;
    std::ofstream file_;
};

int main(int argc, char **argv)
{
    // Initialize ROS node
    ros::init(argc, argv, "data_recorder");
    DataRecorder recorder;
    // Enter ROS event loop
    ros::spin();
    return 0;
}