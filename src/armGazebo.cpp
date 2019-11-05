/* This file is still under development. I'm aiming to make a simple gazebo interface driver for the Jaco 6DoF arm for efficient 
simulation of various robotic system scenarios (Cartesian control in my case)*/

#include <ros/ros.h>
#include <string>
#include <algorithm>
#include <math.h>
#include <geometry_msgs/PoseStamped.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ModelStates.h>

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

//typedef actionlib::SimpleActionClient<gazebo_msgs::ModelState> armPose;
//typedef actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction> fingerPose;

namespace kinovaArm{
	class armVirtual{
	public:
		ros::NodeHandle nh;
		ros::Subscriber sub;
		gazebo_msgs::ModelStates state;

		armVirtual():nh(){
			sub = nh.subscribe("/gazebo/model_states",1,
				&kinovaArm::armVirtual::getCurrentState, this);
			ROS_INFO("Subscriber setup!!");
		}

		void getCurrentState(const gazebo_msgs::ModelStates& msg){
			state = msg;
			cout << state << endl;
		}

		/*void sendNextPose(geometry_msgs::PoseStamped &msg){
			armPose client("/gazebo/set_model_state", true);
			gazebo_msgs::ModelState goal;
			client.waitForServer();

			goal.pose = msg.pose;
			client.sendGoal(goal);
		}*/
	}; //armVirtual class
} //Namespace

using namespace kinovaArm;

int main(int argc, char** argv)
{
	ros::init(argc,argv,"Virtual_Arm");
	ros::NodeHandle nh;
	ros::Rate loop_rate(0.1);
	armVirtual *arm = new armVirtual;

	while(ros::ok()){
		ros::spinOnce();
	}
	return 0;
}
