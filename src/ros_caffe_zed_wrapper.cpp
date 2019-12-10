/* This is the segmentation code which uses the zed-ros-wrapper to get RGB, depth images and point clouds*/

// General C++ headers
#include <math.h>
#include <iostream>
#include <string>
#include <boost/make_shared.hpp>

// ROS include files
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

// Segmentation header
#include "Classifier.h"

using namespace std;

const string PUB_SEG = "/enet/seg";

Classifier* classifier;
std::string model_path;
std::string weights_path;
std::string label_file;

class segPublish{
public:
  ros::NodeHandle nh;
  image_transport::ImageTransport it;
  image_transport::CameraSubscriber rgb;
  image_transport::Publisher pub_seg;
  cv::Mat frame;
  string mLeftCamFrameId;

  segPublish():it(nh){
    pub_seg = it.advertise(PUB_SEG, 5);
    rgb = it.subscribeCamera("/zed/rgb/image_rect_color", 1, &segPublish::getRGBImage, this);
    nh.param<std::string>("general/left_camera_frame", mLeftCamFrameId, "left_camera_frame");
  }

  // Get the left camera RGB image
  void getRGBImage(const sensor_msgs::ImageConstPtr& seg_img, const sensor_msgs::CameraInfoConstPtr& info){
    boost::shared_ptr<cv_bridge::CvImage> cvPtr(new cv_bridge::CvImage);
    try{
      cvPtr = cv_bridge::toCvCopy(seg_img, sensor_msgs::image_encodings::BGR8);
      frame = cvPtr->image;
    }
    catch(cv::Exception &e){
      cout << "Hold on : " << e.what() << endl;
    }
  }

  // Publish the segmented image
  void publishImage(cv::Mat out_seg){
    cv_bridge::CvImage seg_bridge;
    sensor_msgs::Image seg_msg;

    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = mLeftCamFrameId;

    seg_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, out_seg);
    seg_bridge.toImageMsg(seg_msg);
    pub_seg.publish(seg_msg);
  }
}; // End of class

int main(int argc, char** argv){
  // Initialize paths
  const std::string ROOT_SAMPLE = "/home/khashayar/qin";
  model_path = ROOT_SAMPLE + "/model/bn_conv_merged_model.prototxt";
  weights_path = ROOT_SAMPLE + "/model/bn_conv_merged_weights.caffemodel";
  label_file = ROOT_SAMPLE + "/cityscapes19.png";

  // Initialize ROS node
  ros::init(argc, argv, "ros_caffe_zed_wrapper");
  segPublish SPObject;

  // Initialize calssifier
  Classifier classifier(model_path, weights_path);

  cv::Mat out_seg;
  char key = ' ';

  while(true){
    try{
      out_seg = classifier.Predict(SPObject.frame, label_file);
      SPObject.publishImage(out_seg);
    }
    catch(cv::Exception &e){
      cout << "No image : " << e.what() << endl;
    }
    ros::spinOnce();
  }
  ros::shutdown();
  return 0;
}
