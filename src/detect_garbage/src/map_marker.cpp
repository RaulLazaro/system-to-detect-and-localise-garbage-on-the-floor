#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/Marker.h>
#include "detect_garbage/pixel_coordinates.h"

sensor_msgs::PointCloud2 my_pcl;
ros::NodeHandle n;
ros::Publisher marker_pub;
int count = 0;

bool calc(detect_garbage::pixel_coordinates::Request  &req, detect_garbage::pixel_coordinates::Response &res);

void callback(const sensor_msgs::PointCloud2ConstPtr& msg)
 {
    my_pcl = *msg;    
  }

int main (int argc, char **argv)
{
  ros::init(argc, argv, "map_marker");

  ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 1, callback);
  
  ros::ServiceServer service = n.advertiseService("pixel_coordinates", calc);

  ros::spin();

  return 0;
}

bool calc(detect_garbage::pixel_coordinates::Request  &req, detect_garbage::pixel_coordinates::Response &res)
{
  int arrayPosition = req.v*my_pcl.row_step + req.u*my_pcl.point_step;
  int arrayPosX = arrayPosition + my_pcl.fields[0].offset; // X has an offset of 0
  int arrayPosY = arrayPosition + my_pcl.fields[1].offset; // Y has an offset of 4
  int arrayPosZ = arrayPosition + my_pcl.fields[2].offset; // Z has an offset of 8
  
  memcpy(&res.x, &my_pcl.data[arrayPosX], sizeof(float));
  memcpy(&res.y, &my_pcl.data[arrayPosY], sizeof(float));
  memcpy(&res.z, &my_pcl.data[arrayPosZ], sizeof(float));

  // TF

  // Set our shape type to be a sphere
  uint32_t shape = visualization_msgs::Marker::SPHERE;

  visualization_msgs::Marker marker;
  // Set the frame ID and timestamp.
  marker.header.frame_id = "/camera_color_frame";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.
  marker.ns = "basic_shapes";
  marker.id = count;

  // Set the marker type.
  marker.type = shape;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = res.x;
  marker.pose.position.y = res.y;
  marker.pose.position.z = res.z;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();

  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 20, 1);

  while(marker_pub.getNumSubscribers()==0)
  {
    ROS_ERROR("Waiting for subscibers");
    sleep(10);
  }
  ROS_ERROR("Got subscriber");

  marker_pub.publish(marker);

  count++;

  return true;
}
