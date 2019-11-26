// General C++ headers
#include <string>
#include <algorithm>
#include <math.h>
#include <vector>
#include <stdio.h>
#include <boost/make_shared.hpp>

// ROS include files
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <image_transport/image_transport.h>
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
bool waypointSet = false;
const double FINGER_MAX = 6400;
const int width = 640;
const int height = 360;

//Global point cloud pointer
boost::shared_ptr<sensor_msgs::PointCloud2> point_cloud(new sensor_msgs::PointCloud2);

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

class armJaco{
public:
	image_transport::ImageTransport it;
	ros::NodeHandle nh;
	ros::Subscriber sub, pc_sub;
	image_transport::Subscriber seg;
	// The next pose is the pre-grasp pose (0.2 m above the detected object)
	geometry_msgs::PoseStamped home, currentPose, nextPose, dropPose, firstPose;
	geometry_msgs::Vector3 c[width][height], m;
	geometry_msgs::Point pts;

	int isObject[width][height],t[width][height];
	cv_bridge::CvImagePtr cvPtr;


	armJaco():nh(), it(nh){
		sub = nh.subscribe("/j2n6s300_driver/out/tool_pose",1,&armJaco::currentPoseFeedback,this);
		seg = it.subscribe("/enet/seg",1,&armJaco::getSegmentedImage,this);
		pc_sub = nh.subscribe("/enet/point_cloud",1,&armJaco::getPointCloud,this);
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

		this->sendArmPoseGoal(home);
		ROS_INFO_STREAM("Back home");
	}

	geometry_msgs::Point pixel2PointCloud(const int u, const int v){
		int w = point_cloud->width;
		int h = point_cloud->height;

		cout << "Pointcloud dimensions " << w << " " << h << endl;

		int arrayPos = v*point_cloud->row_step + u*point_cloud->point_step;

		int arrayPosX = arrayPos + point_cloud->fields[0].offset; // X has an offset of 0
    int arrayPosY = arrayPos + point_cloud->fields[2].offset; // Y has an offset of 4
    int arrayPosZ = arrayPos + point_cloud->fields[1].offset; // Z has an offset of 8

    float X = 0.0;
    float Y = 0.0;
    float Z = 0.0;

		memcpy(&X, &point_cloud->data[arrayPosX], sizeof(float));
    memcpy(&Y, &point_cloud->data[arrayPosY], sizeof(float));
    memcpy(&Z, &point_cloud->data[arrayPosZ], sizeof(float));

		geometry_msgs::Point p;
    // put data into the point p
    p.x = Y+0.33;
    p.y = X+0.26;
    p.z = Z;

		return p;
	}

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

				if (isObject[i][j]) {
					sum_x += i;
					sum_y += j;
					object_point_count++;
				}
			}
		}

		// calculate min and max in each direction
		double a;
		double b;
		a= 0;
		b= 0;

		double sigma_x2;
		double sigma_x;
		double n;
		double sigma_xy;
		double sigma_y;

		sigma_x2 =0;
		sigma_x =0;
		n =0;
		sigma_xy =0;
		sigma_y =0;

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
				if (isObject[i][j]) {
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

					sigma_x += i;
					sigma_y += -1*j;
					sigma_xy += i*-1*j;
					sigma_x2 += i*i;
					n++;


					//object_point_count++;
				}
			}
		}

		a= ((sigma_y*sigma_x2)-(sigma_x*sigma_xy))/((n*sigma_x2)-(sigma_x*sigma_x));
		b= ((n*sigma_xy)-(sigma_x*sigma_y))/((n*sigma_x2)-(sigma_x*sigma_x));

		double slope;
		slope = atan(b)*180/3.14;

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

		int pixel_x = int(round(sum_x / object_point_count));
		int pixel_y = int(round(sum_y / object_point_count));

		float x = sum_x / object_point_count;
		float y = sum_y / object_point_count;

		printf("Pixel Centroid: %d %d\n", pixel_x, pixel_y);

		// Extract the actual from the point cloud ==> With respect to camera
		if (abs(pixel_x) < width && abs(pixel_y) < height){
			pts = pixel2PointCloud(pixel_x,pixel_y);
		}

		printf("Centroid coordinates: %f %f %f\n", pts.x, pts.y, pts.z);

		// Generate next waypoint based on frame transformation matrices
		generateNextWaypoint();
	}	// Compute mid point function

	void generateNextWaypoint(){
		nextPose.header.stamp = ros::Time::now();
		nextPose.header.frame_id = "j2n6s300_link_base";

		// Trasformation from the camera frame to the hand frame
		float mean_x_pose, mean_y_pose, mean_z_pose;
   		mean_x_pose = pts.x + 0.1;
    		mean_y_pose = pts.y + 0.4;
    		mean_z_pose = 0.78 - pts.z;

		nextPose.pose.position.x = mean_x_pose;
		nextPose.pose.position.y = mean_y_pose;
		nextPose.pose.position.z = mean_z_pose + 0.1;

		nextPose.pose.orientation = home.pose.orientation;
	}

	void grasp(ros::Rate loop_rate){
		ROS_INFO_STREAM("Moving to firstPose");
		firstPose.pose.position.x = 0.4;
		firstPose.pose.position.y = 0.05;
		firstPose.pose.position.z = 0.3;

		//tf::Quaternion q = kinova::EulerXYZ2Quaternion(float(180), float(0), float(-57.2958)).normalize();
		// Go to inital pose
		geometry_msgs::Quaternion q = euler_to_quaterion(double(3.14), double(0), double(-1));
		firstPose.pose.orientation = q;
		sendArmPoseGoal(firstPose);
		ros::Duration(5).sleep();

		// Go to pre-grasp pose
		ROS_INFO_STREAM("Going to pre-grasp pose");
		nextPose.pose.orientation = firstPose.pose.orientation;
		sendArmPoseGoal(nextPose);
		loop_rate.sleep();

		// Go to grasp pose
		ROS_INFO_STREAM("Going to grasp pose");
		nextPose.pose.orientation = firstPose.pose.orientation;
		nextPose.pose.position.z = nextPose.pose.position.z - 0.1;
		sendArmPoseGoal(nextPose);
		loop_rate.sleep();

		// Pick up the object
		ROS_INFO_STREAM("Picking up the object");
		sendFingerPoseGoal(55);
		loop_rate.sleep();

		// Move to drop pose
		ROS_INFO_STREAM("Moving to drop pose");
		dropPose = nextPose;
		dropPose.pose.position.z = -0.1;
		dropPose.pose.orientation = firstPose.pose.orientation;
		dropPose.pose.position.y = dropPose.pose.position.y - 0.3;
		sendArmPoseGoal(dropPose);
		loop_rate.sleep();

		// Drop the object
		ROS_INFO_STREAM("Dropping object");
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

	while(true){
		ros::spinOnce();
		arm->computeMidPoint();
		cout << "Next waypoint  is: " << "\n" << arm->nextPose.pose << endl;

		if(!(isnan(arm->nextPose.pose.position.x) || isnan(arm->nextPose.pose.position.y) || isnan(arm->nextPose.pose.position.z))){
			if(abs(arm->nextPose.pose.position.x) < 0.5 && abs(arm->nextPose.pose.position.y) < 0.5){
				arm->grasp(loop_rate);
				ROS_INFO_STREAM("Pick and place executed !!");
			}
			else ROS_WARN("Cant't reach the arm to the pose, please move closer");
			break;
		}
  }
	return 0;
}
