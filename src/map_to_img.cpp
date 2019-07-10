#include "ros/ros.h"

#include <nav_msgs/GetMap.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <Eigen/Geometry>
#include <sensor_msgs/image_encodings.h>
#include <hector_map_tools/HectorMapTools.h>
#include <std_msgs/Int16.h>

using namespace std;

int map_scale = 50;
/**
 * @brief This node provides occupancy grid maps as images via image_transport, so the transmission consumes less bandwidth.
 * The provided code is a incomplete proof of concept.
 */
class MapAsImageProvider
{
public:
cv::Mat *map_mat;
  MapAsImageProvider()
    : pn_("~")
  {

    image_transport_ = new image_transport::ImageTransport(n_);
    image_transport_publisher_full_ = image_transport_->advertise("map_image/full", 1);
    image_transport_publisher_tile_ = image_transport_->advertise("map_image/tile", 1);

    pose_sub_ = n_.subscribe("pose", 1, &MapAsImageProvider::poseUpdate, this);
    map_sub_ = n_.subscribe("map", 1, &MapAsImageProvider::mapUpdate, this);

    map_mat  = &cv_img_full_.image;
    *map_mat = cv::Mat(256,256,CV_8U, 127);
    map_mat  = &cv_img_tile_.image;
    *map_mat = cv::Mat(256,256,CV_8U, 127);

    //Which frame_id makes sense?
    cv_img_full_.header.frame_id = "map_image";
    cv_img_full_.encoding = sensor_msgs::image_encodings::MONO8;

    cv_img_tile_.header.frame_id = "map_image";
    cv_img_tile_.encoding = sensor_msgs::image_encodings::MONO8;

    //Fixed cell width for tile based image, use dynamic_reconfigure for this later
    p_size_tiled_map_image_x_ = 256;
    p_size_tiled_map_image_y_ = 256;

    ROS_INFO("Map to Image node started.");
  }

  ~MapAsImageProvider()
  {
    delete image_transport_;
  }

  void additionalPublisher()
  { 
    image_transport_publisher_full_.publish(cv_img_full_.toImageMsg());
    image_transport_publisher_tile_.publish(cv_img_tile_.toImageMsg());
    //ROS_INFO("test.");
  }

  //We assume the robot position is available as a PoseStamped here (querying tf would be the more general option)
  void poseUpdate(const geometry_msgs::PoseStampedConstPtr& pose)
  { 
    pose_ptr_ = pose;
    ROS_INFO("New pose received.");
  }

  //The map->image conversion runs every time a new map is received at the moment
  void mapUpdate(const nav_msgs::OccupancyGridConstPtr& map)
  { 
    int size_x = map->info.width;
    int size_y = map->info.height;

    if ((size_x < 3) || (size_y < 3) ){
      ROS_INFO("Map size is only x: %d,  y: %d . Not running map to image conversion", size_x, size_y);
      return;
    }

    // Only if someone is subscribed to it, do work and publish full map image
      map_mat  = &cv_img_full_.image;

      // resize cv image if it doesn't have the same dimensions as the map
      if ( (map_mat->rows != size_y) && (map_mat->cols != size_x)){
        *map_mat = cv::Mat(size_y, size_x, CV_8U);
      }

      const std::vector<int8_t>& map_data (map->data);

      unsigned char *map_mat_data_p=(unsigned char*) map_mat->data;

      //We have to flip around the y axis, y for image starts at the top and y for map at the bottom
      int size_y_rev = size_y-1;

      //for (int y = 0; y >= size_y_rev; ++y){

      for (int y = size_y_rev; y >= 0; --y){

        int idx_map_y = size_x * y;
        int idx_img_y = size_x * (size_y -y);

        for (int x = 0; x < size_x; ++x){

          int idx = idx_img_y + x;

          switch (map_data[idx_map_y + x])
          {
          case -1:
            map_mat_data_p[idx] = 127;
            break;

          case 0:
            map_mat_data_p[idx] = 255;
            break;

          case 100:
            map_mat_data_p[idx] = 0;
            break;
          }
        }
      image_transport_publisher_full_.publish(cv_img_full_.toImageMsg());
    }

    // Only if someone is subscribed to it, do work and publish tile-based map image Also check if pose_ptr_ is valid
    if ((image_transport_publisher_tile_.getNumSubscribers() > 0) && (pose_ptr_)){

      world_map_transformer_.setTransforms(*map);

      Eigen::Vector2f rob_position_world (pose_ptr_->pose.position.x, pose_ptr_->pose.position.y);
      Eigen::Vector2f rob_position_map (world_map_transformer_.getC2Coords(rob_position_world));

      Eigen::Vector2i rob_position_mapi (rob_position_map.cast<int>());

      Eigen::Vector2i tile_size_lower_halfi (p_size_tiled_map_image_x_*map_scale/100, p_size_tiled_map_image_y_*map_scale/100);

      Eigen::Vector2i min_coords_map (rob_position_mapi - tile_size_lower_halfi);

      //Clamp to lower map coords
      if (min_coords_map[0] < 0){
        min_coords_map[0] = 0;
      }

      if (min_coords_map[1] < 0){
        min_coords_map[1] = 0;
      }

      Eigen::Vector2i max_coords_map (min_coords_map + Eigen::Vector2i(p_size_tiled_map_image_x_*map_scale/50,p_size_tiled_map_image_y_*map_scale/50));

      //Clamp to upper map coords
      if (max_coords_map[0] > size_x){

        int diff = max_coords_map[0] - size_x;
        min_coords_map[0] -= diff;

        max_coords_map[0] = size_x;
      }

      if (max_coords_map[1] > size_y){

        int diff = max_coords_map[1] - size_y;
        min_coords_map[1] -= diff;

        max_coords_map[1] = size_y;
      }

      //Clamp lower again (in case the map is smaller than the selected visualization window)
      if (min_coords_map[0] < 0){
        min_coords_map[0] = 0;
      }

      if (min_coords_map[1] < 0){
        min_coords_map[1] = 0;
      }

      Eigen::Vector2i actual_map_dimensions(max_coords_map - min_coords_map);

      cv::Mat* map_mat  = &cv_img_tile_.image;

      // resize cv image if it doesn't have the same dimensions as the selected visualization window
      if ( (map_mat->rows != actual_map_dimensions[0]) || (map_mat->cols != actual_map_dimensions[1])){
        *map_mat = cv::Mat(actual_map_dimensions[0], actual_map_dimensions[1], CV_8U);
      }

      const std::vector<int8_t>& map_data (map->data);

      unsigned char *map_mat_data_p=(unsigned char*) map_mat->data;

      //We have to flip around the y axis, y for image starts at the top and y for map at the bottom
      int y_img = max_coords_map[1]-1;

      for (int y = min_coords_map[1]; y < max_coords_map[1];++y){

        int idx_map_y = y_img-- * size_x;
        int idx_img_y = (y-min_coords_map[1]) * actual_map_dimensions.x();

        for (int x = min_coords_map[0]; x < max_coords_map[0];++x){

          int img_index = idx_img_y + (x-min_coords_map[0]);

          switch (map_data[idx_map_y+x])
          {
          case 0:
            map_mat_data_p[img_index] = 255;
            break;

          case -1:
            map_mat_data_p[img_index] = 127;
            break;

          case 100:
            map_mat_data_p[img_index] = 0;
            break;
          }
        }        
      }
      image_transport_publisher_tile_.publish(cv_img_tile_.toImageMsg());
      ROS_INFO("New map received.");
    }
  }

  ros::Subscriber map_sub_;
  ros::Subscriber pose_sub_;

  image_transport::Publisher image_transport_publisher_full_;
  image_transport::Publisher image_transport_publisher_tile_;

  image_transport::ImageTransport* image_transport_;

  geometry_msgs::PoseStampedConstPtr pose_ptr_;

  cv_bridge::CvImage cv_img_full_;
  cv_bridge::CvImage cv_img_tile_;

  ros::NodeHandle n_;
  ros::NodeHandle pn_;

  int p_size_tiled_map_image_x_;
  int p_size_tiled_map_image_y_;

  HectorMapTools::CoordinateTransformer<float> world_map_transformer_;

};

void task_callback(const std_msgs::Int16 &scale)
{
  map_scale = scale.data;
}

int main(int argc, char** argv)
{   
  ros::init(argc, argv, "map_to_image_node");
  ros::NodeHandle pn("~");
  ros::NodeHandle n("~");
  //void task_callback();
  ros::Rate loop_rate(5); 
  ros::Subscriber map_zoom_sub = n.subscribe("/map_zoom", 1, task_callback);
  MapAsImageProvider map_image_provider;

  while (ros::ok())
   { 
      ROS_INFO("Scale %d", map_scale);
      map_image_provider.additionalPublisher();
      ros::spinOnce();
      loop_rate.sleep();
   }
  return 0;
}
