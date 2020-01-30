#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "detect_garbage/pixel_coordinates.h"

/// Global variables
cv::Mat src, src_gray, dst, mask;

int size_top = 15;
int size_dilate = 5;
int max_size = 100;
int threshold_RGB = 215;
int threshold_Y_min = 0;
int threshold_Y_max = 215;
int threshold_Cb_min = 115;
int threshold_Cb_max = 255;
int const max_threshold = 255;

static const std::string ORIGINAL = "Original";
static const std::string FINAL = "Final";
static const std::string window_1 = "Tophat";
static const std::string window_2 = "Color RGB Threshold";
static const std::string window_3 = "Color YCrCb Threshold";

void imageCallback (const sensor_msgs::ImageConstPtr& msg);
void tophat(int, void*);
void colorThreshold( int, void* );
ros::ServiceClient *clientPtr; //pointer for a client

int main (int argc, char **argv)
{
  ros::init(argc, argv, "img_processing");
  ros::NodeHandle n;

  cv::namedWindow(ORIGINAL);
  cv::startWindowThread();

  image_transport::ImageTransport it(n);
  image_transport::Subscriber sub = it.subscribe("/camera/color/image_raw", 1, imageCallback);
  
  ros::ServiceClient client = n.serviceClient<detect_garbage::pixel_coordinates>("pixel_coordinates");

  clientPtr = &client; //give the address of the client to the clientPtr

  ros::spin();

  return 0;

}

void imageCallback (const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    //Image Processing
    cv::Mat dImg = cv_ptr->image;

    // Divide in half
    src = dImg;//(cv::Rect(0, dImg.rows/2, dImg.cols, dImg.rows/2));
    
    cv::imshow( ORIGINAL, src );

    /// Create a matrix of the same type and size as src (for dst)
    dst.create( src.size(), src.type() );

    /// Convert the image to grayscale
    cv::cvtColor( src, src_gray, CV_BGR2GRAY );

    cv::namedWindow( window_1, CV_WINDOW_AUTOSIZE );
    cv::namedWindow(window_2, CV_WINDOW_AUTOSIZE );
    cv::namedWindow(window_3, CV_WINDOW_AUTOSIZE );

    cv::createTrackbar( "Size Top Hat:", window_1, &size_top, max_size, tophat );
    cv::createTrackbar( "Size Dilate:", window_1, &size_dilate, max_size, tophat );
    tophat(0, 0);

    cv::createTrackbar("Threshold RGB:", window_2, &threshold_RGB, max_threshold, colorThreshold );
    cv::createTrackbar("Threshold Y_min:", window_3, &threshold_Y_min, max_threshold, colorThreshold );
    cv::createTrackbar("Threshold Y_max:", window_3, &threshold_Y_max, max_threshold, colorThreshold );
    cv::createTrackbar("Threshold Cb_min:", window_3, &threshold_Cb_min, max_threshold, colorThreshold );
    cv::createTrackbar("Threshold Cb_max:", window_3, &threshold_Cb_max, max_threshold, colorThreshold );
    colorThreshold( 0, 0 );
    
    cv::waitKey(3);


    detect_garbage::pixel_coordinates srv;
    srv.request.u = 240;
    srv.request.v = 2;

    ros::ServiceClient client = (ros::ServiceClient)*clientPtr; //dereference the clientPtr

    if (client.call(srv))
    {
      ROS_INFO("angle: %f", (float)srv.response.x);
    }
    else
    {
        ROS_ERROR("Failed to call service from pixel_coordinates"); 
    }
}

void tophat(int, void*)
{
  cv::Mat im;

  /// Reduce noise with a kernel 3x3
  blur( src_gray, im, cv::Size(3,3) );

  cv::Mat element = getStructuringElement( cv::MORPH_ELLIPSE, cv::Size( size_top, size_top ));

  /// Apply the tophat morphology operation
  cv::morphologyEx( im, im, cv::MORPH_TOPHAT, element );

  cv::threshold(im, im, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU );

  cv::Mat element2 = getStructuringElement( cv::MORPH_ELLIPSE, cv::Size( 3, 3 ));
  cv::Mat element3 = getStructuringElement( cv::MORPH_ELLIPSE, cv::Size( size_dilate, size_dilate ));

  cv::morphologyEx( im, im, cv::MORPH_OPEN, element2 );
  cv::morphologyEx( im, im, cv::MORPH_DILATE, element3 );

  mask = cv::Scalar::all(0);

  src.copyTo( mask, im);
  
  cv::imshow( window_1, mask );
}

void colorThreshold( int, void* )
{
  cv::Mat im1, im2;

  cv::inRange(mask, threshold_RGB, cv::Scalar(max_threshold, max_threshold, max_threshold), im1);
  cv::imshow( window_2, im1);

  cv::cvtColor(mask, im2, CV_BGR2YCrCb);
  cv::inRange(im2, cv::Scalar(threshold_Y_min, 0, threshold_Cb_min), cv::Scalar(threshold_Y_max, max_threshold, threshold_Cb_max), im2);
  im2=255-im2;
  cv::imshow( window_3, im2);

  dst=im1+im2;
  cv::imshow( FINAL, dst);
}
