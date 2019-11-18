// General C++ headers
#include <string>
#include <algorithm>
#include <math.h>
#include <vector>

// ROS include files
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>

// Kinova driver specific include files
#include "kinova_driver/kinova_api.h"
#include "kinova_driver/kinova_arm.h"
#include "kinova_driver/kinova_tool_pose_action.h"
#include "kinova_driver/kinova_joint_angles_action.h"
#include "kinova_driver/kinova_fingers_action.h"
#include "kinova_driver/kinova_joint_trajectory_controller.h"
#include <actionlib/client/simple_action_client.h>

// ZED SDK
#include <sl/Camera.hpp>

using namespace std;
using namespace cv;
using namespace sl;

// Setting up the actionlib
typedef actionlib::SimpleActionClient<kinova_msgs::ArmPoseAction> armPose;
typedef actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction> fingerPose;

bool homeSet = false;
const int width = 640;
const int height = 360;

class armJaco{
public:
	ros::NodeHandle nh;
	ros::Subscriber sub, seg, pc_sub;
	
	// The next pose is the pre-grasp pose (0.2 m above the detected object)
	geometry_msgs::PoseStamped home, currentPose, nextPose, dropPose;
	geometry_msgs::Vector3 c[width][height];
	sensor_msgs::PointCloud point_cloud;
	int isObject[width][height],t[width][height];
	float mean_x = 0.0, mean_y = 0.0, mean_z = 0.0;
	cv_bridge::CvImagePtr cvPtr;

	armJaco():nh(){
		sub = nh.subscribe("/j2n6s300_driver/out/tool_pose",1,&armJaco::currentPoseFeedback,this);
		seg = nh.subscribe("/enet/seg",1,&armJaco::getSegmentedImage,this);
		pc_sub = nh.subscribe("/enet/point_cloud",1,&armJaco::getPointCloud,this);
		ROS_INFO_STREAM("Setup successful !!");
	}

	// Pose callback
	void currentPoseFeedback(const geometry_msgs::PoseStamped& pose){
	    	currentPose = pose;
		
		if (homeSet == false){
			home = currentPose;
			ROS_INFO_STREAM("Home position set !!");
			homeSet = true;
		}
	}

	// Segmented image callback
	void getSegmentedImage(const sensor_msgs::ImageConstPtr& seg_img){
		cvPtr = cv_bridge::toCvCopy(seg_img, sensor_msgs::image_encodings::BGR8);
	}

	// Point cloud callback
	void getPointCloud(const sensor_msgs::PointCloud2& cloud){
		sensor_msgs::convertPointCloud2ToPointCloud(cloud, point_cloud);
	}

	void sendArmPoseGoal(geometry_msgs::PoseStamped &goal_pose){
	    armPose client("/j2n6s300_driver/pose_action/tool_pose", true);
	    kinova_msgs::ArmPoseGoal goal;
	    client.waitForServer();

	    goal.pose = goal_pose;
	    ROS_INFO_STREAM("Arm pose goal sent !!");
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
		ROS_INFO_STREAM("Finger goal sent !!");
		client.sendGoal(goal);
	}

	void goHome(){
		home.header.stamp = ros::Time::now();
		home.header.frame_id = "j2n6s300_link_base";

		this->sendArmPoseGoal(home);
		ROS_INFO_STREAM("Back home");
	}

	// Classifies whether a given pixel lies inside the identified object from the segmented image
	void seg2obj(){
		cv::Mat frame = cvPtr->image;
		int sum = 0;
		for(int i=0; i<frame.rows; i++){
			for(int j=0; j<frame.cols; j++){
				int g = frame.at<Vec3b>(i,j)[1];

				if(g>35) t[i][j] = 0;
				else t[i][j] = 1;
			}
		}
	}

	void computeMidPoint(){
		seg2obj();

		double sum_x = 0, sum_y = 0, sum_z = 0, cx = 0, cy = 0, cz = 0;
		int object_point_count = 0;
		for (int i = 0; i < width; i++){
			for (int j = 0; j < height; j++){
				isObject[i][j] = t[i][j];

				if (!(isnan(point_cloud.points[i*j].x) || isinf(point_cloud.points[i*j].x))) cx = double(point_cloud.points[i*j].x);
				if (!(isnan(point_cloud.points[i*j].y) || isinf(point_cloud.points[i*j].y))) cy = double(point_cloud.points[i*j].y);
				if (!(isnan(point_cloud.points[i*j].z) || isinf(point_cloud.points[i*j].z))) cz = double(point_cloud.points[i*j].z);

				c[i][j].x = cx;
				c[i][j].y = cy;
				c[i][j].z = cz;

				if (isObject[i][j] && c[i][j].z < 0) {
					sum_x += i;
					sum_y += j;
					object_point_count++;
				}
			}
		}
		
		// calculate min and max in each direction
		double max_xdir[2] ;
		double min_xdir[2] ;
		double max_ydir[2] ;
		double min_ydir[2] ;
		max_xdir[0] = 0;
		max_xdir[1] = 0;
		min_xdir[0] = width;
		min_xdir[1] = width;
		max_ydir[0] = 0;
		max_ydir[1] = 0;
		min_ydir[0] = height;
		min_ydir[1] = height;
		for (int i = 0; i < width; i++){
			for (int j = 0; j < height; j++) {
				if (isObject[i][j] && c[i][j].z < 0) {
					//sum_x += i;

					if(i>max_xdir[0]){
						max_xdir[0]=i;
						max_xdir[1]=j;
					}
					if(i<min_xdir[0]){
						min_xdir[0]=i;
						min_xdir[1]=j;
					}
					if(j>max_ydir[1]){
						max_ydir[0]=i;
						max_ydir[1]=j;
					}
					if(j<min_ydir[1]){
						min_ydir[0]=i;
						min_ydir[1]=j;
					}
					//object_point_count++;
				}
			}
		}
		
		max_xdir[1] = -1*max_xdir[1];
		min_xdir[1] = -1*min_xdir[1];
		max_ydir[1] = -1*max_ydir[1];
		min_ydir[1] = -1*min_ydir[1];


		double d1=0, d2=0;
		double alpha = 0;
		d1 = sqrt((max_xdir[0]-max_ydir[0])*(max_xdir[0]-max_ydir[0])+(max_xdir[1]-max_ydir[1])*(max_xdir[1]-max_ydir[1]));
		d2 = sqrt((min_xdir[0]-max_ydir[0])*(min_xdir[0]-max_ydir[0])+(min_xdir[1]-max_ydir[1])*(min_xdir[1]-max_ydir[1]));
		alpha = atan((max_xdir[1]-max_ydir[1])/(max_xdir[0]-max_ydir[0]));
		alpha = (alpha*180/3.14);
		if(d2>d1) alpha = (alpha)+90;

		// Mid point in the pixel domain
		int pixel_x = int(round(sum_x / object_point_count));
		int pixel_y = int(round(sum_y / object_point_count));

		// Extract the actual from the point cloud ==> With respect to camera
		if (abs(pixel_x) < width && abs(pixel_y) < height){
			mean_x = c[pixel_x][pixel_y].x;
			mean_y = c[pixel_x][pixel_y].y;
			mean_z = c[pixel_x][pixel_y].z;
		}

		// Generate next waypoint based on frame transformation matrices
		generateNextWaypoint();
	}	// Compute mid point function

	void generateNextWaypoint(){
		nextPose.header.stamp = ros::Time::now();
		nextPose.header.frame_id = "j2n6s300_link_base";

		// Trasformation from the camera frame to the hand frame
		float mean_x_pose, mean_y_pose, mean_z_pose;
		float delta_x=0, delta_y=350, delta_z=780;
		
		// The actual poses with respect to the base of the arm
		mean_x_pose = -(mean_x)/1000+0.08;
		mean_y_pose = (mean_y + 200)/1000 + 0.06;
		mean_z_pose = (delta_z - mean_z) /1000 - 0.08;

		nextPose.pose.position.x = mean_x_pose;
		nextPose.pose.position.y = mean_y_pose;
		nextPose.pose.position.z = mean_z_pose + 0.2;

		nextPose.pose.orientation = home.pose.orientation;
	}

	void grasp(ros::Rate loop_rate){
		// Go to pre-grasp pose
		sendArmPoseGoal(nextPose);
		loop_rate.sleep();

		// Go to grasp pose
		nextPose.pose.position.z = nextPose.pose.position.z - 0.2;
		sendArmPoseGoal(nextPose);
		loop_rate.sleep();

		// Pick up the object
		sendFingerPoseGoal(55);
		loop_rate.sleep();

		// Move to drop pose
		dropPose = currentPose;
		dropPose.pose.position.y = dropPose.pose.position.y + 0.5;
		loop_rate.sleep();

		// Drop the object
		sendFingerPoseGoal(0);
		loop_rate.sleep();

		// Return home
		goHome();
		nextPose = home;
	}
}; // End of class

int main(int argc, char** argv){
	ros::init(argc, argv, "Jaco");
	ros::NodeHandle nh;
	ros::Rate loop_rate(0.2);
	
	armJaco *arm = new armJaco;
	usleep(1000*1000);
	geometry_msgs::PoseStamped goal, home, w1, w2;

	while(true){
		ros::spinOnce();

		arm->computeMidPoint();
		cout << "Next waypoint  is: " << arm->nextPose << endl;
		loop_rate.sleep();
		
		// If the segmented object is within the reach of the arm, go for the pick and place, otherwise ignore the waypoint
		if (arm->nextPose.pose.position.x > arm->home.pose.position.x + 2){
			arm->grasp();
		}
		//break;
	}
	return 0;
}
