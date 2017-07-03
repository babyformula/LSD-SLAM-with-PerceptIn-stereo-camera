# LSD-SLAM-with-PerceptIn-stereo-camera
## 1. Quickstart
### 1.1 Prepare environment
(1) Check the running requirement of [ROS](http://www.ros.org) + [LSD-SLAM](https://github.com/tum-vision/lsd_slam), and rosmake or catkin_make LSD-SLAM.<br>
(2) Initialize the workspace naming `ros_opencv`<br>
```
cd ros_opencv/src
catkin_init_workspace
catkin_create_pkg opencv_publisher sensor_msgs opencv2 cv_bridge roscpp std_msgs image_transport
```
Make sure the CMakeLists.txt in path `ros_opencv/src/opencv_publisher` is same as the file in this git. And related `include` and `lib` files are in right paths.<br>
### 1.2 Compile
```
cd ros_opencv
caktkin_make
```
NOTE: Remove and compile using your latest OpenCV recommended from [cv_bridge](https://github.com/ros-perception/vision_opencv) and [image_pipeline](https://github.com/ros-perception/image_pipeline). Put them under `ros_opencv/src`
```
sudo apt-get remove ros-[YOUR ROS DISTRIBUTION]-cv-bridge ros-[YOUR ROS DISTRIBUTION]-image-pipeline
catkin_make
catkin_make --pkg cv_bridge
```
## 2. Usage
```
rosrun opencv_publisher opencv_publisher_node [YOUR calibration].json
```
Call on your node in ROS, and then use LSD-SLAM to subscribe the image message.<br>
Please check related tools' usage on their page.
