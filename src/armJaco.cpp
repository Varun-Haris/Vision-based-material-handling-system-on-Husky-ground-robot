#include <string>
#include <algorithm>
#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include "kinova_driver/kinova_api.h"
#include "kinova_driver/kinova_arm.h"
#include "kinova_driver/kinova_tool_pose_action.h"
#include "kinova_driver/kinova_joint_angles_action.h"
#include "kinova_driver/kinova_fingers_action.h"
#include "kinova_driver/kinova_joint_trajectory_controller.h"

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/LinearMath/Scalar.h>
#include <tf/transform_listener.h>

#include <actionlib/client/simple_action_client.h>

using namespace std;

// Setting up the actionlib
typedef actionlib::SimpleActionClient<kinova_msgs::ArmPoseAction> armPose;
typedef actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction> fingerPose;

bool getCurrentCommand = false;
const double FINGER_MAX = 6400;


geometry_msgs::Quaternion quaternionNorm(geometry_msgs::Quaternion q){
	float q_norm = sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);	
	q.x = q.x/q_norm;
	q.y = q.y/q_norm;
	q.z = q.z/q_norm;
	q.w = q.w/q_norm;
	return q;
}

class armJaco{
public:
	ros::NodeHandle nh;
	ros::Subscriber sub;
	kinova_msgs::KinovaPose current_pose;
	tf::Pose home_pose; // home position

	armJaco():nh(){
		sub = nh.subscribe("/j2n6s300_driver/out/cartesian_command",1,&armJaco::currentPoseFeedback,this);
		ROS_INFO_STREAM("Setup successful !!");
	}

	void currentPoseFeedback(const kinova_msgs::KinovaPoseConstPtr pose){
	    current_pose.X = pose->X;
	    current_pose.Y = pose->Y;
	    current_pose.Z = pose->Z;
	    current_pose.ThetaX = pose->ThetaX;
	    current_pose.ThetaY = pose->ThetaY;
	    current_pose.ThetaZ = pose->ThetaZ;

	    if (getCurrentCommand == false){
	        home_pose.setOrigin(tf::Vector3(current_pose.X, current_pose.Y, current_pose.Z));
	        tf::Quaternion q = kinova::EulerXYZ2Quaternion(current_pose.ThetaX, current_pose.ThetaY, current_pose.ThetaZ);
	        home_pose.setRotation(q);

	        ROS_INFO_STREAM("Home position set");
	        getCurrentCommand = true;
	    }
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
		geometry_msgs::PoseStamped home;
		home.header.stamp = ros::Time::now();
		home.header.frame_id = "j2n6s300_link_base";
		tf::poseTFToMsg(home_pose, home.pose);
		home.pose.orientation = quaternionNorm(home.pose.orientation); // Quaternion normalization from tf::Quaternion to geometry_msgs::Quaternion

		sendArmPoseGoal(home);
		sendFingerPoseGoal(0);
		ROS_INFO_STREAM("Back home !!");
	}

};

int main(int argc, char** argv){
	ros::init(argc, argv, "Jaco");
	ros::NodeHandle nh;
	ros::Rate loop_rate(0.1);
	armJaco *arm = new armJaco;
	cout << arm->current_pose <<endl;

	geometry_msgs::PoseStamped goal, home;
	tf::poseTFToMsg(arm->home_pose, home.pose);
	home.pose.orientation = quaternionNorm(home.pose.orientation);

	goal.header.stamp = ros::Time::now();
	goal.header.frame_id = "j2n6s300_link_base";
	goal.pose = home.pose;
	goal.pose.position.x = goal.pose.position.x + 1;

	while(true){
		ros::spinOnce();
		arm->sendArmPoseGoal(goal);
		loop_rate.sleep();
		arm->sendFingerPoseGoal(float(55)); //Closing
    		loop_rate.speep();
    		arm->sendFingerPoseGoal(float(0)); //Opening
    		loop_rate.sleep();
    		arm->goHome();
		loop_rate.sleep();
		break;
    }
	return 0;
}
