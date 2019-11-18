// General C++ headers
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <string>

// ROS include files
#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

// Segmentation header
#include "Classifier.h"

// Point cloud include files
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

const std::string PUB_RGB = "/enet/rgb";
const std::string PUB_DEPTH = "/enet/depth";
const std::string PUB_SEG = "/enet/seg";
const std::string PUB_POINT_CLOUD = "/enet/point_cloud";

Classifier* classifier;
std::string model_path;
std::string weights_path;
std::string label_file;

// ZED SDK include
#include <sl/Camera.hpp>
using namespace sl;

class SegmentationPublish{

  public:

  ros::NodeHandle nh;
  image_transport::ImageTransport it;
  image_transport::Publisher pub_rgb;
  image_transport::Publisher pub_depth;
  image_transport::Publisher pub_seg;
  ros::Publisher pub_cloud;

  // Point cloud variables
  string point_cloud_frame_id = "";
  ros::Time point_cloud_time;
  string camera_frame_id;

  SegmentationPublish():it(nh){
      pub_rgb = it.advertise(PUB_RGB, 5);
      pub_rgb = it.advertise(PUB_DEPTH, 5);
      pub_seg = it.advertise(PUB_SEG, 5);
      pub_cloud = nh.advertise<sensor_msgs::PointCloud2>(PUB_POINT_CLOUD, 5);
      nh.param<std::string>("camera_frame", camera_frame_id, "camera_frame");
    }

    void PublishImage(cv::Mat rgb, cv::Mat depth, cv::Mat seg){
      cv_bridge::CvImage rgb_bridge;
      cv_bridge::CvImage depth_bridge;
      cv_bridge::CvImage seg_bridge;

      sensor_msgs::Image rgb_msg;
      sensor_msgs::Image depth_msg;
      sensor_msgs::Image seg_msg;

      std_msgs::Header header; // empty header
      header.stamp = ros::Time::now(); // time

      rgb_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGRA8, rgb);
      depth_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGBA8, depth);
      seg_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, seg);

      rgb_bridge.toImageMsg(rgb_msg);
      depth_bridge.toImageMsg(depth_msg);
      seg_bridge.toImageMsg(seg_msg);


      pub_rgb.publish(rgb_msg);
      pub_depth.publish(depth_msg);
      pub_seg.publish(seg_msg);
    }

    void publishPointCloud(int width, int height, sl::Mat cloud) {
      point_cloud_frame_id = camera_frame_id;
      point_cloud_time = ros::Time::now();

      pcl::PointCloud<pcl::PointXYZRGB> point_cloud;
      point_cloud.width = width;
      point_cloud.height = height;
      int size = width*height;
      point_cloud.points.resize(size);

      sl::Vector4<float>* mem_cloud = cloud.getPtr<sl::float4>();
      for (int i = 0; i < size; i++) {
          point_cloud.points[i].x = mem_cloud[i][2];
          point_cloud.points[i].y = -mem_cloud[i][0];
          point_cloud.points[i].z = -mem_cloud[i][1];
          point_cloud.points[i].rgb = mem_cloud[i][3];
      }

      sensor_msgs::PointCloud2 output;
      pcl::toROSMsg(point_cloud, output); // Convert the point cloud to a ROS message
      output.header.frame_id = point_cloud_frame_id; // Set the header values of the ROS message
      output.header.stamp = point_cloud_time;
      output.height = height;
      output.width = width;
      output.is_bigendian = false;
      output.is_dense = false;
      pub_cloud.publish(output);
    }

}; // End of class

//slMat to cvMat
cv::Mat slMat2cvMat(sl::Mat& input)
{

	//convert MAT_TYPE to CV_TYPE
	int cv_type = -1;
	switch (input.getDataType())
	{
	case sl::MAT_TYPE_32F_C1: cv_type = CV_32FC1; break;
	case sl::MAT_TYPE_32F_C2: cv_type = CV_32FC2; break;
	case sl::MAT_TYPE_32F_C3: cv_type = CV_32FC3; break;
	case sl::MAT_TYPE_32F_C4: cv_type = CV_32FC4; break;
	case sl::MAT_TYPE_8U_C1: cv_type = CV_8UC1; break;
	case sl::MAT_TYPE_8U_C2: cv_type = CV_8UC2; break;
	case sl::MAT_TYPE_8U_C3: cv_type = CV_8UC3; break;
	case sl::MAT_TYPE_8U_C4: cv_type = CV_8UC4; break;
	default: break;
	}

	// cv::Mat data requires a uchar* pointer. Therefore, we get the uchar1 pointer from sl::Mat (getPtr<T>())
	//cv::Mat and sl::Mat will share the same memory pointer
	return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(MEM_CPU));
}


int main(int argc, char **argv) {
    // Initialize paths
    const std::string ROOT_SAMPLE = "/home/khashayar/qin";
    model_path = ROOT_SAMPLE + "/model/bn_conv_merged_model.prototxt";
    weights_path = ROOT_SAMPLE + "/model/bn_conv_merged_weights.caffemodel";
    label_file = ROOT_SAMPLE + "/cityscapes19.png";

    // ZED
    Camera zed; //Change back if not working
    InitParameters init_params;
    init_params.camera_resolution = RESOLUTION_VGA;
    init_params.depth_mode = DEPTH_MODE_PERFORMANCE;
    init_params.coordinate_units = sl::UNIT_METER;
    init_params.coordinate_system = sl::COORDINATE_SYSTEM_RIGHT_HANDED_Z_UP;

    ERROR_CODE err; // error state for all ZED SDK functions
    ERROR_CODE err_tracking; // error state for all ZED SDK functions

    //Open the camera
    err = zed.open(init_params);

    if (err != SUCCESS) {
      std::cout << errorCode2str(err) << std::endl;
      zed.close();
      return EXIT_FAILURE; // quit if an error occurred
    }

    //Create a Mat to store images
    Resolution image_size = zed.getResolution();
    sl::Mat zed_image_left(image_size, sl::MAT_TYPE_8U_C4);
    sl::Mat depth_map;
    cv::Mat input_rgb = slMat2cvMat(zed_image_left);
    cv::Mat input_depth = slMat2cvMat(depth_map);
    sl::Pose deltaOdom;
    sl::Mat cloud;

    // Initialize ROS node
    ros::init(argc, argv, "ros_caffe_brick");
    SegmentationPublish SPObject;

    // Initialize calssifier
    Classifier classifier(model_path, weights_path);

    cv::Mat out_seg;

    // Loop until 'q' is pressed
    char key = ' ';
    //cloud.points.resize(230400);
    while (key != 'q') {
      // Grab images and process them
      if (zed.grab() == SUCCESS) {
        //Retrieve left camera frame, depth and point cloud
        zed.retrieveImage(zed_image_left, VIEW_LEFT);
        zed.retrieveImage(depth_map, VIEW_DEPTH);
	zed.retrieveMeasure(cloud, MEASURE_XYZRGBA);
        SPObject.publishPointCloud(image_size.width, image_size.height, cloud);

	//Convert to cv images
        cv::Mat input_rgb = slMat2cvMat(zed_image_left);
        cv::Mat input_depth = slMat2cvMat(depth_map);

      }

      out_seg = classifier.Predict(input_rgb, label_file);

      // Publish RGB image and segmented image
      SPObject.PublishImage(input_rgb, input_depth, out_seg);
      ros::spinOnce();

    }

    zed.close();
    ros::shutdown();
    return 0;
}
