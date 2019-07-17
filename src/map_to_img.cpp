#include "ros/ros.h"
#include <std_msgs/Int16.h>
#include <MapAsImageProvider.h>

using namespace std;

MapAsImageProvider *map_image_provider;

void map_zoom_callback(const std_msgs::Int16 &scale)
{
  map_image_provider->setScale((float)scale.data / 2500);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "map_to_image_node");
  ros::NodeHandle n("~");
  ros::Rate loop_rate(50);
  ROS_INFO("Init MapAsImageProvider object");
  map_image_provider = new MapAsImageProvider(n);
  ros::Subscriber map_zoom_sub = n.subscribe("/map_zoom", 1, map_zoom_callback);

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
    map_image_provider->publishFullMap();
    map_image_provider->publishMapTile();
  }
  return 0;
}
