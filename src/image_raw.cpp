#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <stdlib.h>

const static int NUM_IMAGE = 20;

int main(int argc, char **argv) {

  ros::init(argc, argv, "image_pub");
  ros::NodeHandle nh;

  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("image_raw", 1);

  ros::Rate loop_rate(5);

  for (int i = 1; i <= 20; i++) {
    if (!nh.ok())
      break;

    std::ostringstream stringStream;
    stringStream << "/home/staringsky/Camera_out/" << i << ".JPG";
    cv::Mat image = cv::imread(stringStream.str(), CV_LOAD_IMAGE_COLOR);
   
    cv::waitKey(30);
    if (image.empty()) {

      ROS_ERROR("Failed to load image");
      return -1;
    }
    ROS_INFO("Load image: %d", i);
    sensor_msgs::ImagePtr msg =
        cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

    pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
