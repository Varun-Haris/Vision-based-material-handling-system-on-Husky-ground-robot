// General C++ headers
#include <string>
#include <algorithm>
#include <math.h>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <boost/make_shared.hpp>

// OpenCV headers
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// ROS include files
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <image_transport/image_transport.h>

// Kinova driver specific include files
#include "kinova_driver/kinova_api.h"
#include "kinova_driver/kinova_arm.h"
#include "kinova_driver/kinova_tool_pose_action.h"
#include "kinova_driver/kinova_joint_angles_action.h"
#include "kinova_driver/kinova_fingers_action.h"
#include "kinova_driver/kinova_joint_trajectory_controller.h"
#include "kinova_driver/kinova_ros_types.h"

#include <actionlib/client/simple_action_client.h>

using namespace std;
using namespace cv;

// Setting up the actionlib
typedef actionlib::SimpleActionClient<kinova_msgs::ArmPoseAction> armPose;
typedef actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction> fingerPose;

bool homeSet = false;
const int width = 640;
const int height = 360;
RNG rng(12345);

//Global point cloud pointer
boost::shared_ptr<sensor_msgs::PointCloud2> point_cloud(new sensor_msgs::PointCloud2);

// Converts euler angles (in rad) to normalized quaternions
geometry_msgs::Quaternion euler_to_quaterion(double roll, double pitch, double yaw){
	float cy = cos(yaw*0.5);
	  float sy = sin(yaw*0.5);
	  float cp = cos(pitch*0.5);
	  float sp = sin(pitch*0.5);
	  float cr = cos(roll*0.5);
	  float sr = sin(roll*0.5);

	  geometry_msgs::Quaternion quat;

	  quat.w = cy * cp * cr + sy * sp * sr;
	  quat.x = cy * cp * sr - sy * sp * cr;
	  quat.y = sy * cp * sr + cy * sp * cr;
	  quat.z = sy * cp * cr - cy * sp * sr;

	return quat;
}

// Class for the arm node containing necessary attributes and member functions
class armJaco{
public:
	image_transport::ImageTransport it;
	ros::NodeHandle nh;
	ros::Subscriber sub, pc_sub;
	image_transport::Subscriber seg, depth;
	
	// The next pose is the pre-grasp pose (0.2 m above the detected object)
	geometry_msgs::PoseStamped home, currentPose, nextPose, dropPose, firstPose, graspPose;
	geometry_msgs::Vector3 c[width][height], m;
	geometry_msgs::Point pts;
	cv_bridge::CvImagePtr cvPtr, depthPtr;
	
	bool grab;
	cv::Mat image;


	armJaco():nh(), it(nh){
		sub = nh.subscribe("/j2n6s300_driver/out/tool_pose",1,&armJaco::currentPoseFeedback,this);
		seg = it.subscribe("/enet/seg",1,&armJaco::getSegmentedImage,this);
		depth = it.subscribe("/zed/depth/depth_registered",1,&armJaco::getDepthImage,this);
		pc_sub = nh.subscribe("/zed/point_cloud/cloud_registered",1,&armJaco::getPointCloud,this);
		ROS_INFO_STREAM("Setup successful !!");
	}

	// Pose callback
	void currentPoseFeedback(const geometry_msgs::PoseStamped& pose){
	    currentPose = pose;

	    if (homeSet == false){
		    home = currentPose;
		    ROS_INFO_STREAM("Home position set !!");
	            cout << "Home Position \n" << home << endl;
		    homeSet = true;
	    }
	}

	// Segmented image callback
	void getSegmentedImage(const sensor_msgs::ImageConstPtr& seg_img){
		cvPtr = cv_bridge::toCvCopy(seg_img, sensor_msgs::image_encodings::BGR8);
	}

	// Point cloud callback
	void getPointCloud(const boost::shared_ptr<sensor_msgs::PointCloud2> cloud){
		point_cloud = cloud;
	}

	// Depth image callback
	void getDepthImage(const sensor_msgs::ImageConstPtr& depth_img){
		depthPtr = cv_bridge::toCvCopy(depth_img, sensor_msgs::image_encodings::TYPE_32FC1);
	}

	void sendArmPoseGoal(geometry_msgs::PoseStamped &goal_pose){
	    armPose client("/j2n6s300_driver/pose_action/tool_pose", true);
	    kinova_msgs::ArmPoseGoal goal;
	    client.waitForServer();

	    goal_pose.header.stamp = ros::Time::now();
	    goal_pose.header.frame_id = "j2n6s300_link_base";
	    goal.pose = goal_pose;
	    client.sendGoal(goal);
	}

	void sendFingerPoseGoal(float finger_turn){
		fingerPose client("/j2n6s300_driver/fingers_action/finger_positions", true);
		kinova_msgs::SetFingersPositionGoal goal;
		client.waitForServer();

		if (finger_turn < 0) finger_turn = 0;
		else finger_turn = std::min(finger_turn, float(55));

		goal.fingers.finger1 = finger_turn;
		goal.fingers.finger2 = goal.fingers.finger1;
		goal.fingers.finger3 = goal.fingers.finger2;
		client.sendGoal(goal);
	}

	void goHome(){
		home.header.stamp = ros::Time::now();
		home.header.frame_id = "j2n6s300_link_base";

		sendFingerPoseGoal(0);
		sendArmPoseGoal(home);
		ROS_INFO_STREAM("Back home");
	}

	// Converts 2D pixel midpoint to 3D coordinates w.r.t camera frame of reference
	geometry_msgs::Point pixel2PointCloud(const int u, const int v){
		int w = point_cloud->width;
		int h = point_cloud->height;

		int arrayPos = v*point_cloud->row_step + u*point_cloud->point_step;

		int arrayPosX = arrayPos + point_cloud->fields[1].offset; // X has an offset of 0
    		int arrayPosY = arrayPos + point_cloud->fields[2].offset; // Y has an offset of 4
	    	int arrayPosZ = arrayPos + point_cloud->fields[0].offset; // Z has an offset of 8

    		float X = 0.0;
   		float Y = 0.0;
   		float Z = 0.0;

		memcpy(&X, &point_cloud->data[arrayPosX], sizeof(float));
    		memcpy(&Y, &point_cloud->data[arrayPosY], sizeof(float));
    		memcpy(&Z, &point_cloud->data[arrayPosZ], sizeof(float));

		geometry_msgs::Point p;
		// put data into the point p
		p.x = -X; 
		p.y = -Y; 
		// Use the depth image from z for accurate depth estimation
		p.z = depthPtr->image.at<float>(u,v);

		return p;
	}

	// Classifies whether a given pixel is inside the segmented object or not
	cv::Mat seg2obj(){
		cv::Mat frame = cvPtr->image;
		cv::Mat gray, thresh;
		int sum = 0;

		cvtColor(frame, gray, CV_RGB2GRAY);
		threshold(gray, image, 200, 255, cv::THRESH_BINARY);
	}

	// Compute pixel midpoint
	void computeMidPoint(ros::Rate loop_rate){
		seg2obj();
		cv::Mat cImage, canny;

		// To find the pixel centroids of various objects in the frame
		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;
		findContours(image, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0,0));

		if(!contours.size()) return;
		cout << contours.size() << endl;

		// This calculates the centroid of the object closest to the origin of the camera
		Point2f centroid;
		for(int i=0;i<contours.size();i++){
			Moments m = moments(contours[i], false);
			centroid = Point2f(m.m10/m.m00, m.m01/m.m00);
			if(!isnan(centroid.x) || !isnan(centroid.y)) break;
		}

		// To find the orientation
		Canny(image, canny, 40, 100, 3);
		cvtColor(canny, cImage, CV_GRAY2BGR);
		vector<Vec2f> lines;
		HoughLines(canny, lines, 1, CV_PI/180, 40, 0, 0);

		// Display the hough lines (can be commented out for faster operation)
		for( size_t i = 0; i < lines.size(); i++ ){
		     float rho = lines[i][0], theta = lines[i][1];
		     Point pt1, pt2;
		     double a = cos(theta), b = sin(theta);
		     double x0 = a*rho, y0 = b*rho;
		     pt1.x = cvRound(x0 + 1000*(-b));
		     pt1.y = cvRound(y0 + 1000*(a));
		     pt2.x = cvRound(x0 - 1000*(-b));
		     pt2.y = cvRound(y0 - 1000*(a));
		     line( cImage, pt1, pt2, Scalar(0,0,255), 3, CV_AA);
		}

		 if(abs(centroid.x) < width && abs(centroid.y) < height){
			 // Get the 3D coordinates (pose) with respect to the camera
			 pts = pixel2PointCloud(centroid.x,centroid.y);
		 }
		 else{
			 pts.x = 0;
			 pts.y = 0;
			 pts.z = 0;
		 }

		 if(pts.x == 0 && pts.y == 0 && pts.z == 0) grab = false;
		 else grab = true;

		 printf("Centroid coordinates: %f %f %f\n", pts.x, pts.y, pts.z);

			// Generate next waypoint based on frame transformation matrices ==> With respect to the base of the arm
			generateNextWaypoint();
			// Go for grabbing the object only if it is in the range and none of the values are NaN
			if(abs(nextPose.pose.position.x) > 0.15 && abs(nextPose.pose.position.x) < 0.65 && abs(nextPose.pose.position.y) < 0.65 && grab){
				grasp(loop_rate);
			}
	}

	void generateNextWaypoint(){
		if(grab){
			nextPose.header.stamp = ros::Time::now();
			nextPose.header.frame_id = "j2n6s300_link_base";

			// Trasformation from the camera frame to the hand frame
			float mean_x_pose, mean_y_pose, mean_z_pose;
			mean_x_pose = 0.05 - pts.x;
			mean_y_pose = pts.y + 0.26;
			mean_z_pose = 0.77 - pts.z;

			// Arm has a mean error of 9cm in the x direction
			// The z-coordinate of the waypoint is 10cm above the object
			nextPose.pose.position.x = mean_x_pose - 0.09;
			nextPose.pose.position.y = mean_y_pose;
			nextPose.pose.position.z = mean_z_pose + 0.1;
			// Capping off the z value 
			// This is to ensure that the arm doesn't go too low to a pose where the fingers can't close
			// This is primarily to overcome the depth estimation issue of the zed camera
			if (nextPose.pose.position.z < -0.1635) nextPose.pose.position.z = -0.1635;
			nextPose.pose.orientation = home.pose.orientation;

			cout << "Next waypoint  is: " << "\n" << nextPose.pose << endl;
		}
	}

	void grasp(ros::Rate loop_rate){
		ROS_INFO_STREAM("Moving to firstPose");
		firstPose.pose.position.x = 0.4;
		firstPose.pose.position.y = 0.05;
		firstPose.pose.position.z = 0.3;

		// Go to inital pose
		geometry_msgs::Quaternion q = euler_to_quaterion(double(3.14), double(0), double(-1));
		firstPose.pose.orientation = q;
		sendArmPoseGoal(firstPose);
		loop_rate.sleep();

		// Go to pre-grasp pose
		ROS_INFO_STREAM("Going to pre-grasp pose");
		nextPose.pose.orientation = firstPose.pose.orientation;
		sendArmPoseGoal(nextPose);
		loop_rate.sleep();

		// Go to grasp pose
		ROS_INFO_STREAM("Going to grasp pose");
		graspPose.pose.orientation = firstPose.pose.orientation;
		graspPose = nextPose;
		graspPose.pose.position.z = nextPose.pose.position.z - 0.1;
		sendArmPoseGoal(graspPose);
		loop_rate.sleep();

		// Pick up the object
		ROS_INFO_STREAM("Picking up the object");
		sendFingerPoseGoal(55);
		loop_rate.sleep();

		// Go up 10cm pre-grasp pose
		dropPose = nextPose;
		dropPose.pose.position.z = dropPose.pose.position.z + 0.1;
		dropPose.pose.orientation = firstPose.pose.orientation;
		sendArmPoseGoal(dropPose);

		// Move to drop pose
		ROS_INFO_STREAM("Moving to drop pose");
		//dropPose.pose.position.z = dropPose.pose.position.z + 0.1;
		dropPose.pose.position.x = 0.4;
		dropPose.pose.position.y = -0.5;
		sendArmPoseGoal(dropPose);
		loop_rate.sleep();

		// Drop the object
		ROS_INFO_STREAM("Dropping object");
		sendFingerPoseGoal(0);
		loop_rate.sleep();

		// Return home
		goHome();
		nextPose = home;
		loop_rate.sleep();
	}
}; // End of class

int main(int argc, char** argv){
	ros::init(argc, argv, "Jaco");
	ros::NodeHandle nh;
	ros::Rate loop_rate(0.2);

	// System initialization
	armJaco *arm = new armJaco;
	ros::Duration(5).sleep();

	while(true){
		ros::spinOnce();

		arm->computeMidPoint(loop_rate);
		if(arm->grab){
			ROS_INFO_STREAM("Pick and place executed !!");
			// To ensure there are no false positives or the arm doesn't go to visited waypoint
			arm->grab = false;
			ros::Duration(3).sleep();
		}
	}
	return 0;
}
