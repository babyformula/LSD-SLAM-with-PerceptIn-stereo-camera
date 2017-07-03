/**
 * Copyright 2017 PerceptIn
 *
 * This End-User License Agreement (EULA) is a legal agreement between you
 * (the purchaser) and PerceptIn regarding your use of
 * PerceptIn Robotics Vision System (PIRVS), including PIRVS SDK and
 * associated documentation (the "Software").
 *
 * IF YOU DO NOT AGREE TO ALL OF THE TERMS OF THIS EULA, DO NOT INSTALL,
 * USE OR COPY THE SOFTWARE.
 */

/**
 * Editorial edition by Tong ZHANG
 */

// Don't use  GCC CXX11 ABI by default
#ifndef _GLIBCXX_USE_CXX11_ABI
#define _GLIBCXX_USE_CXX11_ABI 0
#endif

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <mutex>
#include <thread>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <pirvs.h>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

std::shared_ptr<PIRVS::PerceptInDevice> gDevice = NULL;
namespace enc = sensor_msgs::image_encodings;

void exit_handler(int s){
  if (gDevice != NULL) {
    gDevice->StopDevice();
  }
  cv::destroyAllWindows();
  exit(1);
}

//static const char WINDOW[] = "Stream window";
static void help()
{
    printf("\nThis program demonstrates converting OpenCV stream to ROS stream messages  \n"
        );

}

// Callback function for OpenCV trackbar to set exposure of the device.
void ExposureTrackBarCallback(int value, void *ptr) {
  std::shared_ptr<PIRVS::PerceptInDevice>* device_ptr =
      static_cast<std::shared_ptr<PIRVS::PerceptInDevice>*>(ptr);
  if (!device_ptr) {
    return;
  }
  (*device_ptr)->SetExposure(value);
}

int main(int argc, char** argv)
{
  help();
  const std::string file_calib(argv[1]);

  // install SIGNAL handler
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = exit_handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);

  // Create an initial state for feature detection + matching + triangulation.
  std::shared_ptr<PIRVS::FeatureState> state;
  if (!PIRVS::InitFeatureState(file_calib, &state)){
    printf("Failed to InitFeatureState.\n");
    return -1;
  }

  // Create an interface to stream the PerceptIn V1 device.
  if (!PIRVS::CreatePerceptInV1Device(&gDevice) || !gDevice) {
    printf("Failed to create device.\n");
    return -1;
  }
  // Start streaming from the device.
  if (!gDevice->StartDevice()) {
    printf("Failed to start device.\n");
    return -1;
  }

  // Image and window to draw 2d features.
  cv::Mat img_2d;
  cv::namedWindow("Detected features");

  // Image and window to draw 3d features.
  cv::Mat img_depth;
  cv::namedWindow("Sparse depth");

  // Reading stream from the Mat
  cv::Mat img_l;
  cv::namedWindow("Streaming Image");

  // Add a trackbar to the window to tune the exposure of the stereo camera.
  uint32_t exposure_value_u;
  if (!gDevice->GetExposure(&exposure_value_u)) {
    printf("Failed to get exposure.\n");
  }
  int exposure_value = exposure_value_u;
  cv::createTrackbar("Exposure", "Detected features", &exposure_value, 2000,
                     ExposureTrackBarCallback, &gDevice);

  // ROS header and initialization
  ros::init(argc, argv, "image_publisher");

  // Define node
  ros::NodeHandle node;
  image_transport::ImageTransport it(node);
  image_transport::Publisher pub = it.advertise("image_raw", 1); 
  ros::Time time=ros::Time::now(); 

  // ROS message and loop rate
  sensor_msgs::ImagePtr msg;
  ros::Rate loop_rate(60);
  ROS_INFO("Streaming Successfully!");

  // Stream data from the device and update the feature state.
  while (node.ok()) {
    // Get the newest data from the device.
    // Note, it could be either an ImuData or a StereoData.
    std::shared_ptr<const PIRVS::Data> data;
    if (!gDevice->GetData(&data)) {
      continue;
    }
    // RunFeature only accept StereoData.
    std::shared_ptr<const PIRVS::StereoData> stereo_data =
        std::dynamic_pointer_cast<const PIRVS::StereoData>(data);
    if (!stereo_data) {
      continue;
    }

    // Update feature state according to the stereo data.
    // Note, the last input argument of RunFeature() is a flag to specify
    // whether or not to match + triangulate features between the two sensors.
    // Set get_3d = false, if 3d features are not required.
    const bool get_3d = true;
    PIRVS::RunFeature(stereo_data, state, get_3d);

    // Get the 2d features from both sensor in the stereo camera to do all sorts
    // of cool stuff.
    //
    // Sample code:
    // std::vector<cv::Point2d> features_l;
    // std::vector<cv::Point2d> features_r;
    // if (state->Get2dFeatures(&features_l, &features_r)) {
    //   // Your cool stuff here.
    // }
    // Get the 3d features from the stereo camera to do all sorts of cool stuff.
    // Note, 3d features are only available if get_3d is true.
    // std::vector<StereoFeature> features;
    // if (get_3d && state->GetStereoFeatures(&features)) {
    //   // Your cool stuff here.
    // }

    // Visualize the 2d detected features on both sensors in the stereo camera.
    if (PIRVS::Draw2dFeatures(stereo_data, state, &img_2d)) {
      if(!img_2d.empty()){
          cv::imshow("Detected features", img_2d);
      }
    }
    // Visualize the 3d depth of the features.
    if (get_3d && PIRVS::DrawStereoFeatures(stereo_data, state, &img_depth)) {
      if(!img_depth.empty()){
          cv::imshow("Sparse depth", img_depth);
          // Visualize what is being streaming.
          cv::imshow("Streaming Image", stereo_data->img_l);
          cv::waitKey(3);
      // Use cv_bridge to transfer data from cv format to ROS message.
      // *NOTE* Need to recompile cv_bridge and image_pipeline by using latest OpenCV!
      msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", stereo_data->img_l).toImageMsg();
      pub.publish(msg);
      }
    }

    ros::spinOnce();    
    loop_rate.sleep(); 

    // Press ESC to stop.
    char key = cv::waitKey(1);
    if (key == 27) {
      printf("Stopped.\n");
      break;
    }
  }

  return 0;
}
