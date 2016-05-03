/**
 ** Simple ROS Publisher Node
 **/
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
//#include <primeiro/PathPosition.h> // TODO: Uncomment this line
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
//#include	"ros/ros.h"
#include "segundo/Path_Posit.h"
#include <iostream>
#include <sstream>
using namespace	std;

bool demo_service_callback(segundo::Path_Posit::Request &req, segundo::Path_Posit::Response &res)
{
	std::stringstream ss;
	ss<<"1";
	res.out	=ss.str();
	ROS_INFO("RECEBI[%f], CONFIRMEI[%s]", (double) req.x,res.out.c_str());
	moveit::planning_interface::MoveGroup group("manipulator");
  	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  	geometry_msgs::Pose pose1;
  	
    pose1.position.x=req.x;
    pose1.position.y=req.y;
    pose1.position.z=req.z;
    
    pose1.orientation.x=req.a;
    pose1.orientation.y=req.b;
    pose1.orientation.z=req.c;
    pose1.orientation.w=req.w;
    
    
//    if(pose2
	//    ros::waitForShutdown();
//		ROS_INFO("PASSEI AQUI000");
		group.setPoseTarget(pose1);
//		ROS_INFO("PASSEI AQUI111");
		group.move();
		ROS_INFO("POSICAO ATINGIDA!!!!");
	return	true;
} 


int main(int argc, char **argv)
{
	ros::init(argc,	argv,	"service_server");
	ros::NodeHandle	n;
	
	 // start a background "spinner", so our node can process ROS messages
    //  - this lets us know when the move is completed
    ros::AsyncSpinner spinner(1);
    spinner.start();
	
	ros::ServiceServer service = n.advertiseService("demo_service",	 demo_service_callback);
	ROS_INFO("Ready	to receive from	client.");
	ros::spin();
	return	0;
}//END MAIN
