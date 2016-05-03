/**
 ** Simple ROS Subscriber Node
 **/
#include <ros/ros.h>
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
#include	<iostream>
#include	"segundo/Path_Posit.h"
#include	<iostream>
#include	<sstream>
using	namespace	std;


int	main(int argc, char **argv)
{

		ros::init(argc,	argv, "service_client");
		ros::NodeHandle	n;
		ros::Rate	loop_rate(10);
		ros::ServiceClient client = n.serviceClient<segundo::Path_Posit>("demo_service");
		
		int sitio=0;
		std::string caca;
		
		while (ros::ok())
		{
				segundo::Path_Posit srv;
				float ss;
//				ss<<99;
//				srv.request.in = 99.88;
				
				if(sitio==0 && caca=="1" ){
//					 srv.request.x = -0.1028377;
//					 srv.request.y = -0.551449;
//					 srv.request.z = 0.673022;
//					srv.request.a =-0.9;
//					srv.request.b =-0.64875;
//					 srv.request.c =0.256041;
//					 srv.request.w =-0.0428377;
					 
					srv.request.x = 0.0;
				  srv.request.y = 0.54;
				  srv.request.z = 0.78; //0.67
				  
				  srv.request.a = 0;
				  srv.request.b = 0;
				  srv.request.c = 0;
				  srv.request.w = 0;
				 }
				 
				 if(sitio==1 && caca=="1" ){
//					 srv.request.x = 0.0528377;
//					 srv.request.y = -0.551449;
//					 srv.request.z = 0.673022;
//					srv.request.a =-0.9;
//					srv.request.b =-0.64875;
//					 srv.request.c =0.256041;
//					 srv.request.w =-0.0128377;
					 
					 					srv.request.x = 0.20;
				  srv.request.y = 0.54;
				  srv.request.z = 0.78; //0.67
				  
				  srv.request.a = 0;
				  srv.request.b = 0;
				  srv.request.c = 0;
				  srv.request.w = 0;
				 }
				// 
				 if(sitio==2 && caca=="1" ){
//					 srv.request.x = 0.0528377;
//					 srv.request.y = -0.601449;
//					 srv.request.z = 0.673022;
//					srv.request.a =-0.9;
//					srv.request.b =-0.64875;
//					 srv.request.c =0.256041;
//					srv.request.w =-0.0428377;
					
										srv.request.x = 0.20;
				  srv.request.y = 0.74;
				  srv.request.z = 0.78; //0.67
				  
				  srv.request.a = 0;
				  srv.request.b = 0;
				  srv.request.c = 0;
				  srv.request.w = 0;
				 }
				// 
				 if(sitio==3 && caca=="1" ){
//					 srv.request.x = -0.1028377;
//					 srv.request.y = -0.601449;
//					 srv.request.z = 0.673022;
//					srv.request.a =-0.9;
//					srv.request.b =-0.64875;
//					 srv.request.c =0.256041;
//					srv.request.w =-0.0428377;
					
										srv.request.x = 0.0;
				  srv.request.y = 0.74;
				  srv.request.z = 0.78; //0.67
				  
				  srv.request.a = 0;
				  srv.request.b = 0;
				  srv.request.c = 0;
				  srv.request.w = 0;
				 }
				 
				caca="0";
				if (client.call(srv))
				{
						ROS_INFO("Client [%f], enviou [%s]",(double) srv.request.x,srv.response.out.c_str());
				}
				else
				{
						ROS_ERROR("Failed to call service");
						return	1;				
				}
				
			caca=srv.response.out;
			
			sitio=sitio+1;
			if(sitio==4){
		     	sitio=0;
		     }
				
		ros::spinOnce();
		loop_rate.sleep();
		
		}
		return	0;
}//END MAIN
