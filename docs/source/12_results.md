# Results and conclusions

## Introduction

In this part I will comment the results of different test and talk about the conclusions that I have reached in this project.

I will summarize some ideas or techniques that could not be proved due to lack of time and knowledge, the problems encountered will also be detailed and we will discuss how they could be solved.

## Tests

To see the results of the works and programs carried out, I have done different tests.

In this project there are two main parts that have been developed and should be tested, the image processing to detect garbage and the positioning on the map of this garbage.

These two parts have been tested separately under different conditions, this is because the processing of the image to detect garbage is prepared for the conditions of the store (color of the floor, surrounding furniture color, lights) and these are not easy to replicate.

\newpage

### Recognizing

To test how the garbage recognition algorithm works, the recorded data during the visit to one of the stores was used, specifically the recorded by the color camera installed in the robot.

In this recording you can see examples with different types of materials.

The algorithm recognizes most of the garbage with ease and because being in movement, different points of view of the same object are had, reason why if it is not recognized in a frame it is very probable be recognized in another one (especially when it is more centered and near)

Errors are also observed as parts that are recognized as garbage and are not, such as:

- Lines or indications painted on the floor.
- Parts of the furniture of the store that differ greatly from the color of the floor. (racks, legs, ...)
- Parts of the image that correspond to very distant things.

Below there are some images collected from the recording as an example of the different situations and materials tested.

\newgeometry{left=0.5in}

| Original | Result |
|:--------:|:------:|
|![](source/figures/o1.png){width=55%}|![](source/figures/n1.png){width=55%}|
|![](source/figures/o2.png){width=55%}|![](source/figures/n2.png){width=55%}|
|![](source/figures/o3.png){width=55%}|![](source/figures/n3.png){width=55%}|
|![](source/figures/o4.png){width=55%}|![](source/figures/n4.png){width=55%}|
|![](source/figures/o5.png){width=55%}|![](source/figures/n5.png){width=55%}|
|![](source/figures/o6.png){width=55%}|![](source/figures/n6.png){width=55%}|

Table:Results of the image processing algorithm. \label{results}

\begin{changemargin}{1in}{0in}

In them you can see the original image and the result, a binary image in which the white pixels correspond to the objects detected.

\end{changemargin}

\restoregeometry

\newpage

### Positioning

To test the positioning of the garbage on a 2D map, a small test facility is built using four wooden boards. With the circuit finished I go through it with the robot and built the map.

As the conditions are different from those of the store (color of the floor, surrounding furniture color, lights) the algorithm to recognize garbage did not work and I had to adjust it (change color thresholds and kernels sizes) to recognize objects only white and thus be able to test the positioning part.

| Test circuit | Map created |
|:------------:|:-----------:|
|![Test circuit](source/figures/test_circuit.jpg){width=70%}|![Map created of the test circuit](source/figures/test_map.png){width=25%}|

Table:Test circuit and map created. \label{test_circuit}

The objective is to recognize and correctly position the different white objects placed by the circuit in the created map.

The algorithms have been thought to run in real time while the robot moves and so it was initially tested. With this first test the results of the following image were obtained.

![Result of positioning garbage while moving the robot. \label{m0}](source/figures/m0.png){width=50%}

At first glance one could say that the program failed, marking multiple points that do not correspond to garbage along the entire map, but analyzing why this happened and seeing that some points were correct, some conclusions were reached:

- The noise of the point cloud causes errors in the measurements when positioning.
- The processing of so many algorithms at the same time by the robot added to the large amount of data from the camera and the management of the wifi for the connection exceeds the CPU power of the Raspberry Pi of the Turtlebot3.
- Given this slow processing, the necessary transformations to position the objects on the map are not performed immediately, this causes them to be wrong.

For all these reasons I tried again but this time only executing the code to position garbage with the robot completely stopped and interrupting it when I want to move the robot.

Below are the different stops of the robot and the points that are marked on the map cumulatively at each stop.

\newpage

Table: Result of positioning with the robot stoped in different positions. \label{m1-m5}

| Pose 1. | Pose 2. | Pose 3. |
|:-------:|:-------:|:-------:|
|![](source/figures/m1.png){width=30%}|![](source/figures/m2.png){width=30%}|![](source/figures/m3.png){width=30%}|

| Pose 4. | Pose 5. |
|:-------:|:-------:|
|![](source/figures/m4.png){width=46%}|![](source/figures/m5.png){width=46%}|

This time it can be said that the positioning of the garbage has been effective, errors are still observed as the same object is positioned a little different depending on where the object is seen by the robot.

\newpage

## Future work

Here I will comment, from my point of view, how this project should continue to develop, what should be the topics to investigate and the changes to be made.

Starting with the things that I have lacked time to try, I have two ideas:

- The creation of elevation maps with the RGBD camera. I can create a map without garbage and another map containing garbage, perform the subtraction leaving only the different points that should correspond with the garbage, the result can be superimposed with the occupation map to see its position.

- In the processing of the image one of the biggest challenges is to isolate the part of the image belonging to the ground, so as not to identify the furniture as garbage. The idea would be to analyze pixel by pixel comparing the information of the position given by the PCL and the situation on the map to determine if it corresponds with parts that should not be analyzed and eliminate that pixel.

One of the first changes that I would make would be the Raspberry Pi of the Turtlebot3, this presents several problems such as the lack of usb 3.0 or higher for a correct communication with the camera, the lack of RAM to compile certain packages and finally the lack of power for run many very demanding nodes in real time.

Another step in this project would be to adapt the algorithm of image processing to recognize garbage to be used in any environment and combine it with more techniques to achieve better results (Other vision techniques, neural networks...)

## Conclusion

The implementation of this project has been based on the application of computer vision techniques almost exclusively. This is due to the lack of time and knowledge to try other techniques, such as those mentioned in the literature analysis about the construction of different kind of maps.

Even so, the work done serves as a first approach and serves as a basis for future work related to this and others that use:

- Recognition of objects in dynamic environments either through image processing or depth cameras.
- Positioning and marking objects on the map.
- Calibration of the camera position in multi-sensor mobile robot platforms.

It also demonstrate  that is possible the automation of this process, reducing cost and time for the company.

During the time that I have been developing this project I have learned to handle ROS with some ease and how to handle a robot and integrate new sensors, I have applied and reinforced my knowledge of programming in C++ and OpenCV.
