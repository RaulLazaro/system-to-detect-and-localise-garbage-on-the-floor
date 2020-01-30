# Implementation of dirt detection

## Introduction

In the following sections I will detail the work carried out during these months, detailing all the decisions taken.

This time could be divide in three different stages:

- Learning
  - Learning ROS
  - Knowing Turtlebot3
- Get and analysis data
  - Preparing robot with 3D camera for scanning the store
  - Analysis
- Test different techniques
  - Processing of images (detecting objects)
  - Positioning objects (get angle and distance)
  - Put markers on the map
  - Build elevation map
  - Try to remove parts of the image that are fixed objects

## Learning ROS

ROS is a flexible framework for software development for robots that provides the functionality of an operating system.

It is a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robotic platforms.

ROS provides the standard services of an operating system such as hardware abstraction, control of low-level devices, implementation of commonly used functionality, passing of messages between processes and maintenance of packages.

It is based on a graph architecture where the processing takes place in the nodes that can receive, send and multiplex messages from sensors, control, states, schedules and actuators, among others. The library is oriented for a UNIX system (Linux).

I passed the first two weeks of the project learning it. Firstly, I installed Ubuntu 16 and ROS Kinetic following the instructions of the documentation [^5].

[^5]: http://wiki.ros.org/kinetic/Installation/Ubuntu

I have installed it in a virtual machine using the free VirtualBox software, that was because it is easier, faster, and more flexible for me, I only have to problems:

- Loss of performance.
- Need to change the network configuration of the virtual machine in order to properly connect with Robots.
  - Use “Bridge Mode”.

Finally, with everything installed and working, I have followed all the beginner tutorials [^6] of the documentation and code small programs to test.

[^6]: http://wiki.ros.org/ROS/Tutorials

\newpage

## Knowing Turtlebot3

![Turtlebot3 Burger: version used in this project. \label{turtlebot3}](source/figures/turtlebot-3.jpg){ width=50%}

TurtleBot3 is a small, affordable, programmable, ROS-based mobile robot for the use in education, research, hobby, and product prototyping.

The TurtleBot can run SLAM(simultaneous localization and mapping) algorithms to build a map and can drive around in a room. Also, it can be controlled remotely from a laptop.

I received a ROS and Turtlebot3 workshop in which I started controlling the robot and testing all of its functionalities.

Then I followed all the tutorials of the Turtlebot3 e-Manual [^7] emphasizing on the part of the creation and the usage of the maps. As well as testing autonomous navigation programs.

[^7]: http://emanual.robotis.com/docs/en/platform/turtlebot3/overview/

\newpage

Continuing with my learning of ROS and Turtlebot3 I code some programs:

- I modified the simple_navigation_goals.cpp program [^8] to use coordinates relatives to the map.
- I followed the markers tutorial [^9] to create a subscriber node that can be used to notify the garbage in the map.

[^8]: http://wiki.ros.org/navigation/Tutorials/SendingSimpleGoals
[^9]: http://wiki.ros.org/rviz/Tutorials/Markers%3A%20Basic%20Shapes

### Modifications

For the goals of this project we need to add some sensors to the Turtelbot. We decided to use one RGBD camera. The chosen one was the Intel Realsense d435.

![Intel Realsense d435 camera. \label{realsense}](source/figures/realsense-d435.jpg){ width=75%}

In order to use the camera with the TurtleBot3 I need to compile the drivers in the own Raspberry Pi of the robot. The realsense drivers are downloaded without compiling and then they were compiled following the steps of the guide [^10] but problems emerged.

[^10]: https://github.com/IntelRealSense/librealsense/blob/development/doc/installation.md

\newpage

Firstly, a power supply must be used given the time it takes to perform the compilation.

The compilation will fail due to the lack of ram memory of the Raspberry Pi, I had to create a 2Gb swap partition following this steps [^11]. With this I was able to compile it perfectly.

[^11]: https://linuxize.com/post/how-to-add-swap-space-on-ubuntu-18-04/

The next step was to download the necessary ROS package [^12] into the workspace of the Raspberry Pi, compiled and finally tested the camera. I initialized the camera with the launch file `rs_camera.launch`.

[^12]: https://github.com/IntelRealSense/realsense-ros

There is a problem, the camera requires a usb 3.0 connection for the large amount of data handled, and the raspberry pi uses usb 2.0 so the resolution and frame rate must be lowered. I modified the launch file and use a resolution of 424x240 and 15 fps.

I place the camera on the front, in the third level of the TurtleBot3, fixed by a 3D printing custom camera screw [^13].

[^13]: https://www.thingiverse.com/thing:2749041

![Camera position. \label{camera_tb3}](source/figures/camera_tb3.png){width=50%}

\newpage

I also created routines to easily start everything when scanning the store using aliases:

| Alias   | Where execute | Description  	                                          |
|---      |:-------------:|---	                                                    |
| bringup | [TurtleBot]   | Robot and camera initialization.                        |
| uclock  | [Remote PC]   | Synchronize internal CPU clocks of the robot and PC.  	|
| slam  	| [Remote PC]   | Start creating the map. 	                              | 	
| bag     | [Remote PC]   | Start record bag file.                                  |
| teleop  | [Remote PC]   | Start teleoperation node to control remotely the robot. |
| savemap | [Remote PC]   | Save the map created.                                   |

Table: Alias routines created for easy use of the robot. \label{alias}

## Results of the mapping

With the robot prepared and the map creation tested we were invited to one of the Colruyt Group stores to collect data.

We went through two corridors with the robot twice, a first without garbage to create the map mainly, and a second one with garbage on the ground.

Next, I will analyze the information obtained and discuss the methods used to recognize garbage.

### Image of the color camera:

![Source image of the color camera. \label{original-color}](source/figures/original-color.png)

In this image we see that the interesting part, where the floor is located, is the lower half of the image.

The most interesting is the dark and uniform color of the floor, except for the reflected shine of the lamps.

If it is possible to eliminate the brightness, it should be relatively easy to distinguish brightly colored objects from the dark ground.

\newpage

### Image of the depth camera:

![Source image of the depth camera. \label{original-depth}](source/figures/original-depth.png){ width=83%}

This image is created with the depth information, in it the blue areas are closer and the red ones are farther away. I can see a problem, most of the lower half of the image, the one belonging to the ground and interesting to us, appears as a large blue spot in which I can not distinguish any object.

I have investigated the reason for this and has been concluded that the position of the camera is not ideal for being very close to the ground. In the specifications of the camera has been observed that there is a minimum range and maximum distance for usage of the camera. If the object is closer than the minimum will not be differentiable.

The camera should be placed in a higher position and away from the ground to solve this problem and be able to use the depth information.

\newpage

### Map:

![Occupancy map created. \label{original-map}](source/figures/original-map.png)

This is the map created in the first pass without garbage. It is quite well built and only a simple processing will be necessary to eliminate the noise and complete it.

![Occupancy map processed. \label{processed-map}](source/figures/processed-map.png)

\newpage

## Implementation of the computer vision algorithm

I start working in two objectives:

- Remove brightness.
- Recognize objects of different color to the ground.

I take frames of the recorded video and use Matlab for processing it. I use two apps of Matlab:

### Color threshold:

![Color threshold Matlab app. \label{color-threshold}](source/figures/color-threshold.png)

With this app I can apply threshold to the images in different space of colors (RGB, HSV...) and based on the histogram.

I started working to recognise the garbage. The idea is to end with a binary image where white points are garbage and the rest will be black.

I tested different ways of binarizing the image based on the color. The most effective way is to change points with the highest RGB values to white and the rest to black, I tried different thresholds.

I apply color threshold to all the RGB channels of the image in different frames and test:

![Color threshold around 200. \label{color-threshold-200}](source/figures/color-threshold-200.png){ width=80%}

In this image the threshold is fixed in the point that almost only the garbage appears in white, but some objects like plastics are a problem.

In the next image I lowered the threshold, now the plastics are more visible but the reflected shine of the lamps on the floor also appears.

![Color threshold lower. \label{color-threshold-lower}](source/figures/color-threshold-lower.png){ width=80%}

### Morphological operations:

![Image segmenter Matlab app. \label{image-segmenter}](source/figures/image-segmenter.png)

With this app I can test different morphological operations in the images.

I have read about the different operations in the [@OpenCV] [^14].

[^14]: https://docs.opencv.org/trunk/d9/d61/tutorial_py_morphological_ops.html

Morphological transformations are some simple operations based on the image shape. It is normally performed on binary images. It needs two inputs, one is our original image, second one is called structuring element or kernel which decides the nature of operation.

Working towards eliminating the brightness of the image floor, several morphological transformations are tested.

\newpage

One of them is Top Hat, it is the difference between input image and opening of the image. Below example is done for a 9x9 kernel.

![Top Hat example. \label{tophat}](source/figures/tophat.png){ width=30%}

Opening is just another name of erosion followed by dilation. It is useful in removing noise.

![Opening example. \label{opening}](source/figures/opening.png){ width=30%}

- Top Hat = source image - opening of source image
  - Opening = erosion of source image -> dilation of source image
    - Erosion = erodes away the boundaries of foreground object
    - Dilation = It is just opposite of erosion

| Original | Erosion | Dilation |
|:---:|:-------:|:-------:|
|![](source/figures/j.png){width=15%}|![](source/figures/erosion.png){width=15%}|![](source/figures/dilation.png){width=15%}|

Table: Erosion and dilation examples. \label{erosion-dilation}

I discovered that using Top Hat, the floor has a more uniform tone without losing much information of the rest so it can be used to create a mask and eliminate the problem of the floor, leaving an image in which the usage of segmentation based on colour can be performed more easy.

![Top Hat masked image. \label{image-tophat}](source/figures/image-tophat.png){ width=80%}

I applied a blur filter to reduce noise, with a kernel 3x3 aperture.

Top Hat is applied to the gray scale image and then it is binarised using the Otsu algorithm [@1979:ots] to define the threshold.

I have tested with different structural elements to apply Top Hat. A disc of 15px diameter is the finally choosen for the algorithm.

Then, the morphological transformations, open and dilate, have been used to remove additional noise and increase the mask.

With all this tested I passed the program to ROS and OpenCV. I create a node that subscribes to the topic of the camera, converts it to be used in OpenCV and publishes the position of the detected garbage to mark it on the map.

\newpage

Example code of a ROS node that subscribes to an image and convert it with `cv_bridge` to be able to use it in OpenCV:

```cpp
int main (int argc, char **argv)
{
  ros::init(argc, argv, "img_processing");
  ros::NodeHandle n;

  image_transport::ImageTransport it(n);
  image_transport::Subscriber sub = it.subscribe(
    "/camera/color/image_raw", 1, imageCallback);

  ros::spin();
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

  ...
```
\newpage

This will be the main program to process the image, it should process the image, first create the Top Hat masked image and then apply color threshold based on the RGB color space.

## Positioning objects

The next step was to isolate each object and calculate the relative position respect to the robot and later transform to map coordinates.

I have created a service that receives the X, Y coordinates of the center pixel that belongs to the object in the image and return the position.

I came up with two methods.

### Angle and distance:

![Positioning garbage explanation. \label{position}](source/figures/position.png)

\newpage

For calculating the angle, I used a simple formula based on the field of view of the RGBD Camera
Intel Realsense d435: 85.2° x 58° (+/- 3°)

> $angle=\frac{85.2}{\frac{imageWidth}{2}} \cdot X - 85.2$

![How to get angle and distance of the garbage. \label{angle}](source/figures/angle.png)

For the distance I use the depth camera info, activating the feature to align the depth image with the color image.

`roslaunch realsense2_camera rs_camera.launch align_depth:=true`

### Point Cloud:

If I start the camera with the command `roslaunch realsense2_camera rs_camera.launch filters:=pointcloud` a point cloud will be published. This point cloud is of type unorganised, which is a one row array of points. Because of the unorganised representation, the calculation of a point that corresponds to a certain pixel in the colour image, becomes difficult.

For getting a organized point cloud data that uses 2D array of points with the same size of the color image I should start the camera with the command `roslaunch realsense2_camera rs_rgbd.launch`. That one uses the color and depth image to create the point cloud using the ROS package rgbd_launch.

With this point cloud I could get the coordinates respect to the camera of each pixel of the image easily. This was the method that I finally use to position the objects.

Example code of a ROS node that subscribes to an PCL calc the point that corresponds to a certain pixel:

```cpp
int main (int argc, char **argv)
{
  ros::init(argc, argv, "map_marker");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>
    ("/camera/depth_registered/points", 1, callback);
  ros::spin();
}

void callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  my_pcl = *msg;

  int arrayPosition = v*my_pcl.row_step + u*my_pcl.point_step;
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

  ...
```
