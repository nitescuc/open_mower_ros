#include "ros/ros.h"
#include "mower_msgs/Status.h"
#include "rosbag_snapshot_msgs/TriggerSnapshot.h"

ros::ServiceClient snapshotClient;
bool emergency = false, emergency_sent = false;
ros::Time last_snapshot_time(0.0);
std::recursive_mutex snapshot_ctrl_mutex;

void onStatus(const mower_msgs::Status::ConstPtr &msg) {
    std::lock_guard<std::recursive_mutex> lk{snapshot_ctrl_mutex};
    emergency = msg->emergency;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "snapshot_controller");

    ros::NodeHandle n;
    ros::NodeHandle paramNh("~");

    ros::Subscriber status = paramNh.subscribe("/mower/status", 10, onStatus);

    snapshotClient = n.serviceClient<rosbag_snapshot_msgs::TriggerSnapshot>("/trigger_snapshot");
    ROS_INFO("Waiting for snapshot service");
    if (!snapshotClient.waitForExistence(ros::Duration(10.0, 0.0))) {
        ROS_ERROR("Snapshot service not found.");

        return 1;
    }

    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        if (emergency && !emergency_sent && (ros::Time::now() - last_snapshot_time).toSec() > 30.0) {
            ROS_WARN_STREAM("Taking emergency snapshot");
            rosbag_snapshot_msgs::TriggerSnapshot srv;
            srv.request.filename = "emergency";
            snapshotClient.call(srv);
            last_snapshot_time = ros::Time::now();
            emergency_sent = true;
        }
        if (!emergency) {
            emergency_sent = false;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}