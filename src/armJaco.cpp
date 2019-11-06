#include <string>
#include <algorithm>
#include <math.h>
#include <vector>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include "kinova_driver/kinova_api.h"
#include "kinova_driver/kinova_arm.h"
#include "kinova_driver/kinova_tool_pose_action.h"
#include "kinova_driver/kinova_joint_angles_action.h"
#include "kinova_driver/kinova_fingers_action.h"
#include "kinova_driver/kinova_joint_trajectory_controller.h"

#include <actionlib/client/simple_action_client.h>

using namespace std;

// Setting up the actionlib
typedef actionlib::SimpleActionClient<kinova_msgs::ArmPoseAction> armPose;
typedef actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction> fingerPose;

bool homeSet = false;
bool waypointSet = false;
const double FINGER_MAX = 6400;

class armJaco{
public:
	ros::NodeHandle nh;
	ros::Subscriber sub;
	geometry_msgs::PoseStamped currentPose;

	armJaco():nh(){
		sub = nh.subscribe("/j2n6s300_driver/out/tool_pose",1,&armJaco::getArmPose,this);
		ROS_INFO_STREAM("Setup successful !!");
	}

	void getArmPose(const geometry_msgs::PoseStamped& pose){
	    currentPose = pose;
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

	void goHome(geometry_msgs::PoseStamped home){
		home.header.stamp = ros::Time::now();
		home.header.frame_id = "j2n6s300_link_base";
		
		this->sendArmPoseGoal(home);
		ROS_INFO_STREAM("Back home");
	}

};

int main(int argc, char** argv){
	ros::init(argc, argv, "Jaco");
	ros::NodeHandle nh;
	ros::Rate loop_rate(0.2);
	armJaco *arm = new armJaco;
	usleep(1000*1000);

	geometry_msgs::PoseStamped goal, home, w1, w2;
	std::vector<geometry_msgs::PoseStamped> waypoints;

	while(true){
		ros::spinOnce();
		if (homeSet == false){
			home = arm->currentPose;
			ROS_INFO_STREAM("Home position set !!");
			homeSet = true;
			continue;
		}

		goal = home;
		// Go to a defined cartesian waypoint
		goal.pose.position.x = goal.pose.position.x + 0.3;
		arm->sendArmPoseGoal(goal);
		loop_rate.sleep();
		// Close all the fingers (Equivalent to object grasping)
		arm->sendFingerPoseGoal(float(55));
		loop_rate.sleep();
		// Open all the fingers
		arm->sendFingerPoseGoal(float(0));
		loop_rate.sleep();
		// Return to home position after the task execution
		arm->goHome(home);
		break;
    }
	return 0;
}
