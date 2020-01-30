# Appendix 1: Code {.unnumbered}

## img_processing.cpp {.unnumbered}

```cpp
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "detect_garbage/pixel_coordinates.h"

// Global variables
cv::Mat src, src_gray, dst, mask;

int size_top = 15;
int size_dilate = 5;
int size_dilate1 = 5;
int max_size = 100;
int threshold_RGB = 153;
int threshold_Y_min = 0;
int threshold_Y_max = 215;
int threshold_Cb_min = 115;
int threshold_Cb_max = 255;
int const max_threshold = 255;

static const std::string ORIGINAL = "Original";
static const std::string FINAL = "Final";
static const std::string window_1 = "Top Hat";
static const std::string window_2 = "Color RGB Threshold";
static const std::string window_3 = "Color YCrCb Threshold";

void imageCallback (const sensor_msgs::ImageConstPtr& msg);
void tophat(int, void*);
void colorThreshold( int, void* );
ros::ServiceClient *clientPtr; // Pointer for a client

int main (int argc, char **argv)
{
  ros::init(argc, argv, "img_processing");
  ros::NodeHandle n;

  cv::namedWindow(ORIGINAL);
  cv::startWindowThread();

  image_transport::ImageTransport it(n);
  image_transport::Subscriber sub = it.subscribe(
    "/camera/color/image_raw", 1, imageCallback);

  ros::ServiceClient client = n.serviceClient<
    detect_garbage::pixel_coordinates>("pixel_coordinates");

  //give the address of the client to the clientPtr
  clientPtr = &client;

  ros::spin();

  return 0;
}

void imageCallback (const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg,
      sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Image Processing
  cv::Mat dImg = cv_ptr->image;

  // Divide in half
  src = dImg(cv::Rect(0, dImg.rows/2, dImg.cols, dImg.rows/2));

  cv::imshow( ORIGINAL, src );

  // Create a matrix of the same type and size as src (for dst)
  dst.create( src.size(), src.type() );

  // Convert the image to grayscale
  cv::cvtColor( src, src_gray, CV_BGR2GRAY );

  cv::namedWindow( window_1, CV_WINDOW_AUTOSIZE );
  cv::namedWindow(window_2, CV_WINDOW_AUTOSIZE );
  cv::namedWindow(window_3, CV_WINDOW_AUTOSIZE );

  cv::createTrackbar( "Size Top Hat:",
    window_1, &size_top, max_size, tophat );
  cv::createTrackbar( "Size Dilate:",
    window_1, &size_dilate, max_size, tophat );
  tophat(0, 0);

  cv::createTrackbar("Threshold RGB:",
    window_2, &threshold_RGB, max_threshold, colorThreshold );
  cv::createTrackbar("Threshold Y_min:",
    window_3, &threshold_Y_min, max_threshold, colorThreshold );
  cv::createTrackbar("Threshold Y_max:",
    window_3, &threshold_Y_max, max_threshold, colorThreshold );
  cv::createTrackbar("Threshold Cb_min:",
    window_3, &threshold_Cb_min, max_threshold, colorThreshold );
  cv::createTrackbar("Threshold Cb_max:",
    window_3, &threshold_Cb_max, max_threshold, colorThreshold );
  cv::createTrackbar( "Size Dilate:",
    FINAL, &size_dilate1, max_size, colorThreshold );
  colorThreshold( 0, 0 );

  cv::Mat canny_output;
  std::vector<std::vector<cv::Point> > contours;
  std::vector<cv::Vec4i> hierarchy;

  // Detect edges using canny
  cv::Canny( dst, canny_output, 50, 150, 3 );

  // Find contours
  cv::findContours( canny_output, contours, hierarchy,
    cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

  // Get the moments
  std::vector<cv::Moments> mu(contours.size());
  for( int i = 0; i<contours.size(); i++ )
  {
    mu[i] = cv::moments( contours[i], false );
  }

  // Get the centroid of figures.
  std::vector<cv::Point2f> mc(contours.size());
  for( int i = 0; i<contours.size(); i++)
  {
    mc[i] = cv::Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 );
  }

  // Draw contours
  cv::Mat drawing(canny_output.size(),
    CV_8UC3, cv::Scalar(255,255,255));
  for( int i = 0; i<contours.size(); i++ )
  {
    cv::Scalar color = cv::Scalar(167,151,0); // B G R values
    cv::drawContours(drawing, contours, i, color, 2, 8,
      hierarchy, 0, cv::Point());
    cv::circle( drawing, mc[i], 4, color, -1, 8, 0 );

    // Mark on map
    detect_garbage::pixel_coordinates srv;
    srv.request.u = mc[i].x;
    srv.request.v = mc[i].y+dImg.rows/2;

    //dereference the clientPtr
    ros::ServiceClient client = (ros::ServiceClient)*clientPtr;

    if (client.call(srv))
    {
      ROS_INFO("x: %f", (float)srv.response.x);
      ROS_INFO("y: %f", (float)srv.response.y);
      ROS_INFO("z: %f", (float)srv.response.z);
    }
    else
    {
      ROS_ERROR("Failed to call service from pixel_coordinates");
    }
  }

  // Show the resultant image
  cv::namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
  cv::imshow( "Contours", drawing );

  cv::waitKey(3);
}

void tophat(int, void*)
{
  cv::Mat im;

  // Reduce noise with a kernel 3x3
  blur( src_gray, im, cv::Size(3,3) );

  cv::Mat element = getStructuringElement(
    cv::MORPH_ELLIPSE, cv::Size( size_top, size_top ));

  // Apply the tophat morphology operation
  cv::morphologyEx( im, im, cv::MORPH_TOPHAT, element );

  cv::threshold(im, im, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU );

  cv::Mat element2 = getStructuringElement(
    cv::MORPH_ELLIPSE, cv::Size( 3, 3 ));
  cv::Mat element3 = getStructuringElement(
    cv::MORPH_ELLIPSE, cv::Size( size_dilate, size_dilate ));

  cv::morphologyEx( im, im, cv::MORPH_OPEN, element2 );
  cv::morphologyEx( im, im, cv::MORPH_DILATE, element3 );

  mask = cv::Scalar::all(0);

  src.copyTo( mask, im);

  cv::imshow( window_1, mask );
}

void colorThreshold( int, void* )
{
  cv::Mat im1, im2;

  cv::inRange(mask, threshold_RGB,
    cv::Scalar(max_threshold, max_threshold, max_threshold), im1);
  cv::imshow( window_2, im1);

  cv::cvtColor(mask, im2, CV_BGR2YCrCb);
  cv::inRange(im2,
    cv::Scalar(threshold_Y_min, 0, threshold_Cb_min),
    cv::Scalar(threshold_Y_max, max_threshold, threshold_Cb_max), im2);
  im2=255-im2;
  cv::imshow( window_3, im2);

  dst=im1+im2;

  cv::Mat element2 = getStructuringElement(
    cv::MORPH_ELLIPSE, cv::Size( 3, 3 ));
  cv::Mat element3 = getStructuringElement(
    cv::MORPH_ELLIPSE, cv::Size( size_dilate1, size_dilate ));

  cv::morphologyEx( dst, dst, cv::MORPH_OPEN, element2 );
  cv::morphologyEx( dst, dst, cv::MORPH_DILATE, element3 );

  cv::imshow( FINAL, dst);
}
```

\newpage

## map_marker.cpp {.unnumbered}

```cpp
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

int count = 0;

bool calc(
  detect_garbage::pixel_coordinates::Request  &req,
  detect_garbage::pixel_coordinates::Response &res);

void callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  my_pcl = *msg;    
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "map_marker");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>
    ("/camera/depth_registered/points", 1, callback);

  ros::ServiceServer service = n.advertiseService(
    "pixel_coordinates", calc);

  ros::spin();

  return 0;
}

bool calc(
  detect_garbage::pixel_coordinates::Request  &req,
  detect_garbage::pixel_coordinates::Response &res)
{
  ros::NodeHandle nh;
  ros::Publisher marker_pub;

  int arrayPosition = req.v*my_pcl.row_step + req.u*my_pcl.point_step;
  int arrayPosX = arrayPosition
    + my_pcl.fields[0].offset; // X has an offset of 0
  int arrayPosY = arrayPosition
    + my_pcl.fields[1].offset; // Y has an offset of 4
  int arrayPosZ = arrayPosition
    + my_pcl.fields[2].offset; // Z has an offset of 8

  float X = 0;
  float Y = 0;
  float Z = 0;

  memcpy(&X, &my_pcl.data[arrayPosX], sizeof(float));
  memcpy(&Y, &my_pcl.data[arrayPosY], sizeof(float));
  memcpy(&Z, &my_pcl.data[arrayPosZ], sizeof(float));

  ROS_INFO("%f %f %f", X,Y,Z);

  res.x = X;
  res.y = Y;
  res.z = Z;

  /*
  // TF
  tf::Vector3 point(res.x,res.y,res.z);
  tf::TransformListener listener;
  tf::StampedTransform transform;
  try{
    listener.lookupTransform("/map",
      "/camera_color_frame", ros::Time::now(), transform);
  }
  catch (tf::TransformException ex){
    ROS_WARN("Map to camera transform unavailable %s", ex.what());
  }

  tf::Vector3 point_bl = transform * point;

  ROS_INFO("%f %f %f", point_bl[0],point_bl[1],point_bl[2]);
  */

  // Set our shape type to be a sphere
  uint32_t shape = visualization_msgs::Marker::SPHERE;

  visualization_msgs::Marker marker;
  // Set the frame ID and timestamp.
  marker.header.frame_id = "/camera_color_optical_frame";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.
  marker.ns = "basic_shapes";
  marker.id = count;

  // Set the marker type.
  marker.type = shape;

  // Set the marker action.  
  // Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.  
  // This is a full 6DOF pose
  // relative to the frame/time specified in the header
  marker.pose.position.x = res.x;
  marker.pose.position.y = res.y;
  marker.pose.position.z = res.z;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();

  marker_pub = nh.advertise<visualization_msgs::Marker>
    ("/visualization_marker", 20, 1);

  ROS_ERROR("Waiting for subscibers");

  while(marker_pub.getNumSubscribers()==0)
  {

  }
  ROS_ERROR("Got subscriber");

  marker_pub.publish(marker);

  count++;

  return true;
}
```

## laser_measure.py {.unnumbered}

```python
#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan

def callback(msg):
  print msg.ranges[270]

rospy.init_node('sub_node')
sub = rospy.Subscriber("/scan", LaserScan, callback)

rospy.spin()
```

\newpage

## collect_data_node.py {.unnumbered}

```python
#!/usr/bin/env python

import argparse       #Read command line arguments
import numpy as np    #Arrays and opencv images

import rospy          #ros python module
import cv2

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, LaserScan

#-----------------------------------------
#--- GLOBAL VARIABLES
#-----------------------------------------
bridge = CvBridge()
cv_image = []
laser_scan = []
image_stamp = []

#-----------------------------------------
#--- FUNCTION DEFINITION
#-----------------------------------------
def ImageReceivedCallback(data):
  global cv_image
  global bridge
  global image_stamp

  #print("Received image")

  try:
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    image_header = data.header
    image_stamp = data.header.stamp

  except CvBridgeError as e:
    print(e)

def LaserReceivedCallback(data):
  global laser_scan

  print("Received laser")
  laser_scan = data

#-----------------------------------------
#--- MAIN
#-----------------------------------------
if __name__ == "__main__":

  global cv_image
  global image_stamp

  #---------------------------------------
  #--- Argument parser
  #---------------------------------------
  ap = argparse.ArgumentParser()
  ap.add_argument("-p", "--capture_path",
    help = "path to the capture folder", default = ".")
  args = vars(ap.parse_args())

  #---------------------------------------
  #--- Intitialization
  #---------------------------------------
  rospy.init_node('collect_data_node') #ros node init
  cv2.namedWindow("Camera", cv2.WINDOW_NORMAL)

  h = open('laser.txt','w')
  hi = open('image_stamps.txt','w')

  image_sub = rospy.Subscriber("/camera/color/image_raw",
    Image, ImageReceivedCallback)
  laser_sub = rospy.Subscriber("/scan",
    LaserScan, LaserReceivedCallback)

  #time for the tf listener to receive some transforms
  rospy.sleep(rospy.Duration(0.1))

  rate = rospy.Rate(100) # 10hz
  count = 1

  while not rospy.is_shutdown():

    #print("One iteration complete")

    #cv2.imshow("Camera", cv_image)

    key = (cv2.waitKey(20) & 255)
    #print("key = " + str(key))

    #<timestamp> StartAngleRads AngleIncrementRads
    #EndAngleRads RangeUnitType NoAngles [Ranges]
    range_unit_type = 3 #for meters
    ss_ranges = " ".join(["%.8f" % i for i in laser_scan.ranges])

    ss_time = str(laser_scan.header.stamp.secs) + "."
      + str(laser_scan.header.stamp.nsecs)
    ss = ss_time + " " + str(laser_scan.angle_min)
      + " " + str(laser_scan.angle_increment)  + " "
      + str(laser_scan.angle_max) + " "
      + str(range_unit_type) + " " + str(len(laser_scan.ranges))
      + " " + ss_ranges + "\n"

    h.write(ss)

    if key == 113: #q for quit
      print("Quit")
      break
    elif key == 115: #s for save
      print("Saving image and laser scan number " + str(count))
      cv2.imwrite("image_" + str(count) + ".bmp", cv_image)

      ss = str(image_stamp.secs) + "." + str(image_stamp.nsecs)
        + " " + str(image_stamp.secs) + "."
        + str(image_stamp.nsecs) + "\n"
      hi.write(ss)

      count += 1

    rate.sleep()

  h.close()
  hi.close()
```
