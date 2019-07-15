#include <math.h>
#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char **argv)
{
    int counter = 0;
    ros::init(argc, argv, "dummy_pose_publisher");
    ros::NodeHandle n("~");
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 0;
    pose.pose.orientation.w = 1;
    ros::Publisher pose_publisher = n.advertise<geometry_msgs::PoseStamped>("/pose", 1);
    ros::Rate loop_rate(50);

    while (ros::ok())
    {
        if (counter == 628)
        {
            counter = 0;
        }
        pose.header.stamp = ros::Time::now();
        pose.pose.position.x = sin(((float)counter)/100);
        pose.pose.position.y = cos(((float)counter)/100);
        pose_publisher.publish(pose);
        ros::spinOnce();
        loop_rate.sleep();
        counter++;
    }
    return 0;
}
