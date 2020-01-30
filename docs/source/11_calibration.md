# Camera calibration

## Introduction

I need to calibrate the camera position in the robot to transforms the coordinates correctly.

In this chapter I will resume the methods that I have found to calibrate the camera as well the results that were obtained.

An important point is to know the concepts of intrinsic and extrinsic parameters of a camera.

- Intrinsic:
  - Focal length
  - Principal point
  - Skew
  - Radial distortion
  - Tangential distortion
- Extrinsic:
  - Translation vectors
  - Rotation vectors

![Calibration coordinates camera intrinsic and extrinsic. \label{calibration_coordinate_blocks}](source/figures/calibration_coordinate_blocks.png)

The extrinsic parameters denote the coordinate system transformations from 3D world coordinates to 3D camera coordinates. Equivalently, the extrinsic parameters define the position of the camera, these are the important ones for our purpose.

Another important point to keep in mind is to know the system of axes that each method uses to be able to interpret them correctly.

| ROS | RADLOCC | OPTICAL |
|:---:|:-------:|:-------:|
|![](source/figures/ROS_axes.png){width=30%}|![](source/figures/RADLOCC_axes.png){width=30%}|![](source/figures/optical_axes.png){width=30%}|

Table: Axes transformations used. X red, Y green and Z blue. \label{axes}

\newpage

## RADLOCC Toolbox

I have found a Matlab toolbox called RADLOCC[^15] based on the works of the papers [@Kassir] and [@Peynot].

[^15]: http://www-personal.acfr.usyd.edu.au/akas9185/AutoCalib/index.html

> "This paper proposes algorithms that augment two existing trustful calibration methods with an automatic extraction of the calibration object from the sensor data. The result is a complete procedure that allows for automatic camera-laser calibration."

For using this toolbox I need a special dataset that should contain a set of files:

- `laser.txt` containing the laser data, in the format `timestamp angle_min angle_increment angle_max unit_type number_of_points ranges`;
- `image_stamps.txt` containing the timestamps of the captured images;
- `image_XX.bmp`, which are the image files, which need to start by the number 01 (and not 00). For example image_01.bmp, image_002.bmp, ...

This dataset was exported from ROS using a python script, taking photos with the color camera of a checkerboard in different positions.

\newpage

The RADLOCC method for getting a calibration was:

1. In a directory, extract both RADOCC and RADLOCC Toolkits.
1. In MATLAB, add both toolkits to the path.  
    - `addpath RADOCCToolbox`
    - `addpath RADOCCToolbox\CornerFinder`
    - `addpath RADLOCCToolbox`
    - `addpath RADLOCCToolbox\Functions.`
1. Open the dataset as the root.
1. The first step is to obtain the intrinsic calibration of the camera and extract the checkerboard planes. To do this, run the `calib` command. Then follow the steps:
    - `Image Names` to set the image names (select `image_` and `b` for `bmp`)
    - `Extract Grid Corners` (press enter for all images)
    - `Calibration` to get the intrinsic parameters
    - `Save`, which saves the calibration to a file in the dataset's directory (`Calib_Results.mat`)
1. Run the RADLOCC Toolbox. Load both the laser and the image data with `Read Data`.
1. `Manual Select` to segment the laser. Choose only the more straight and clear parts.
1. `Calibrate` to run the calibration algorithm. The values appear on the console of MATLAB.
1. `Laser into Image` to check the validity of the calibration. If a range of images is what is wanted, then input something like `1:10`.

The best results obtained have been with a dataset of many images (22) in the highest possible resolution (1280 × 720) and placing the checkerboard plane as close as possible to the camera to capture more points of the laser.

\newpage

RADLOCC axis
$$
\Delta = \begin{pmatrix}
0.0308\\
-0.0652\\
-0.0796\\
\end{pmatrix}
\pm
\begin{pmatrix}
0.011\\
0.0136\\
0.00737\\
\end{pmatrix}
m
$$
$$
\Phi = \begin{pmatrix}
-0.105\\
-3.53\\
-179\\
\end{pmatrix}
\pm
\begin{pmatrix}
1.38\\
0.615\\
0.631\\
\end{pmatrix}
deg
$$

>Total rms error = 0.00616

A very large error can be observed for the dimensions that are being measured, this is because the laser is not very precise and does not recognize the plane as a completely straight line of points.

![An example of a laser beam projected on the XZ-plane. \label{laser_plane_example}](source/figures/laser_plane_example.png)

Perhaps this method would be more accurate with a better laser or with post processed laser data that calculates the average of the position of the points during the time that the checkerboard plane is in the same position, avoiding outliers.

The advantage of this method is that it can be used with any type of camera, a RGBD camera is not necessary.

## Point cloud to laser scan and 2D calibration

Other approach for the calibration of the camera is to use the point cloud and the laser sensor. There are several ROS packages to convert a PCL in laser scan data given certain parameters [^16].

[^16]: http://wiki.ros.org/pointcloud_to_laserscan

With the two laser scan files, one for the laser and other of the conversion, and tools like in Matlab `matchScans` [^17] I could get the X, Y and theta parameters. In this case I need to suppose that the other two rotations are equal to 0 and measure the Z distance to perform the PCL to laser scan conversion at the same height of the real laser.

![Match Lidar Scans Example \label{MatchLidarScansExample}](source/figures/MatchLidarScansExample.png){width=85%}

[^17]: https://es.mathworks.com/help/robotics/ref/matchscans.html

For this reasons, the lack of precision of the laser seen in the previous method, and the lack of time to implement the algorithms in Maltlab and export the data in a correctly, this method has not been proven.

## Third point method

The problem with the calibrations of the camera is that I can not measure the distance of the hardware by hand because this is not the actual position from which the photo was taken, either by the internal lenses or by the position of the camera in the interior of the frame.

But with a checkerboard and using the Matlab toolbox to calibrate cameras it is easy to get the position with respect to the checkerboard plane, being the origin the first corner.

![Checkerboard example. Origin and axes. \label{checkerboard}](source/figures/checkerboard.png){width=75%}

\newpage

If I make that point to match with the height of the laser, known by the URDF model of the robot, and the plane is placed parallel to the robot, the transformation between that point and the laser would be equal to 0 in all the axes and angles except in the X axes that would be the measurement given by the laser.

![Origin_plane-laser transformation. \label{origin_plane-laser}](source/figures/origin_plane-laser.png){width=75%}

Finally I will have the two transformations, one camera-origin_plane and another origin_plane-laser, that gives us the searched transformation camera-laser.

For the first, I used the Matlab toolbox to calibrate cameras with the same dataset that was used for the RADLOCC method, adding an image with the origin of the plane matching with the laser.

\newpage

From this method I get this transformation camera-checkerboard:

Optical axes
$$
\Delta = \begin{pmatrix}
0.0230\\
-0.0694\\
0.6606\\
\end{pmatrix}
\pm
\begin{pmatrix}
0.0004\\
0.0006\\
0.0010\\
\end{pmatrix}
m
$$
$$
\Phi = \begin{pmatrix}
-0.0186\\
-0.0234\\
0.0206\\
\end{pmatrix}
\pm
\begin{pmatrix}
0.0026\\
0.0022\\
0.0003\\
\end{pmatrix}
rad
$$

For the second, the distance to the point has been measured with the laser several times and the average has been made.

ROS axes
$$
\Delta = \begin{pmatrix}
0,6044\\
0\\
0\\
\end{pmatrix}
\pm
\begin{pmatrix}
0.001\\
0.001\\
0.001\\
\end{pmatrix}
m
$$

The final laser-camera transformation would be:

ROS axes  
$$
\Delta = \begin{pmatrix}
0.0562\\
0.0230\\
-0.0694\\
\end{pmatrix}
\pm
\begin{pmatrix}
0.0020\\
0.0014\\
0.0016\\
\end{pmatrix}
m
$$
$$
\Phi = \begin{pmatrix}
0.0206\\
-0.0186\\
-0.0234\\
\end{pmatrix}
\pm
\begin{pmatrix}
0.0178\\
0.0201\\
0.0197\\
\end{pmatrix}
rad
$$  

\newpage

## Conclusions

External calibration of a camera to a laser is a common prerequisite on today’s multi-sensor mobile robot platforms. However, the process of doing so is relatively poorly documented and almost always time-consuming.

For our project I do not currently need this calibration with high precision, so after observing the results of the last method and comparing several measures of laser and PCL, I have chosen to give the calibration as sufficient, documenting all methods in case in the future will be needed.

The final result in the URDF model of the robot is:

![Camera position relative to laser scan. \label{camera_position}](source/figures/camera_position.png){width=75%}
