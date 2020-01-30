# Literature survey

## Introduction

For the development of this project, I searched and read some documentation and research papers. In this chapter, I will discuss and comment the interesting topics as well as the implementation them in the project.

I have searched for related projects, but it is a very specific problem, so there are not many projects that deal with it, that is why I have extended the research to any project related to autonomous platforms that recognize objects in dynamic environments.

The projects found use different technologies, either individually or combining them, such as cameras, distance sensors, machine learning...

I was also read traversability analysis papers, in which could read about different type of maps, how to create them and their uses.

\newpage

## Vision methods

One of the related projects that inspires me was [@3D-SALM] [^1] that is a:

[^1]: https://github.com/ROBOTIS-JAPAN-GIT/turtlebot3_slam_3d

> "Sample repository for creating a three dimensional map of the environment in real-time and navigating through it. Object detection using YOLO is also performed, showing how neural networks can be used to take advantage of the image database stored by RTAB-Map and use it to e.g. localize objects in the map."

It uses the Turtlebot3 platform to which a RGBD camera has been installed, using the images of the cameras to recognize objects with the YOLO neural network [@Redmon2016YouOL] and the depth image to position the objects on the map. In this project we can see the implementation of different techniques in a combination way.

![Operation diagram of a related project. \label{3D-SLAM-diagram}](source/figures/3D-SLAM-diagram.png){ width=75%}

So at first my attention was focused on using the information from the cameras to detect garbage based on computer vision, a field already known by my. The first idea was to try to recognize the garbage making tests in Matlab and later using the Opencv library.

\newpage

## Traversability analysis methods

One of the most interesting papers about traversability analysis was [@guerrero:hal-01518756]. It talks about the different type of maps, how to create them and their uses.

Coming up next the types of maps found will be detailed, we will also talk about being able to integrate them into this project and its pros and cons.

### Occupancy map:

It is one of the most used methods for terrain mapping. Every cell in an occupancy map contains an occupancy probability which is used to determine if the cell is free, occupied or not explored. Figure \ref{occupancy-map} depicts an example of an occupancy grid map.

![Example of an occupancy grid map. \label{occupancy-map}](source/figures/occupancy-map.png)

This is the kind of maps you can create with the Turtlebot3 right out of the box, just using odometry and laser information. Is very useful using them for the autonomous navigation of robots in plane floors and controlled environments.

### Elevation map:

Alternatively to the occupancy map, an elevation map is a 2D grid in which every cell contains height values of the terrain mapped. Figure \ref{elevation-map} is an example of an elevation map.

![Example of elevation map. \label{elevation-map}](source/figures/elevation-map.jpg){ width=75%}

Elevation maps are also known as 2.5D maps. Similarly to the occupancy map, the computational requirements are not so important as for 3D mapping. An important disadvantage of 2.5D mapping is the fact that overhanging structures will be considered as obstacles.

They are very useful for moving robots in on uneven floors or to determine obstacles on the way. They are the most usable for the purpose of this project.

I found a ROS package [^2] to create elevation maps with robots which are equipped with a pose estimation (e.g. IMU & odometry) and a distance sensor (e.g. structured light (Kinect, RealSense), laser range sensor, stereo camera).

This package is based on the works of the papers [@Fankhauser2018ProbabilisticTerrainMapping] and [@Fankhauser2014RobotCentricElevationMapping] that I have also read.

[^2]: https://github.com/ANYbotics/elevation_mapping

\newpage

### 3D map:

Figure \ref{3D-map} depicts an example of 3D map.

![Example of 3D map. \label{3D-map}](source/figures/3D-map.png)

They are a type of maps more complete and easy to observe but at the same time they are difficult to obtain and manage given their high computational requirement.

For 3D mapping I found several package in ROS to create them with different approach, such as [@hornung13auro ] [^3] or [@RTAB-Map] [^4].

[^3]: http://octomap.github.io/
[^4]: http://introlab.github.io/rtabmap/

## Conclusions

Finally we noticed two clear ideas to apply in our project to recognize objects on the floor:

- Image processing (OpenCV, neural networks...).

- Using depth camera info to create different elevation maps of the terrain.

In our case, a combination can be used by placing the 3D camera pointing to the ground, being able to detect the objects by distance and using OpenCV based on colors and shapes.
