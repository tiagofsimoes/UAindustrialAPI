/**
 ** Simple ROS Publisher Node
 **/
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <primeiro/PathPosition.h> // TODO: Uncomment this line

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "MOTION_publisher");
    ros::NodeHandle node;

    // Create a 1Hz update rate
    ros::Rate loop_rate(0.30);
    

    // Advertise that we're publishing the topic "position", of type PathPosition
    ros::Publisher pub = node.advertise<primeiro::PathPosition>("position", 1000); // TODO: Uncomment this line
    primeiro::PathPosition pose;
        // We can plan a motion for this group to a desired pose for the end-effector.

int sitio=0;

while(ros::ok()) {


if(sitio==0){
	 pose.position.x = -0.1028377;
	 pose.position.y = -0.581449;
	 pose.position.z = 0.673022;
	pose.orientation.x =-0.9;
	pose.orientation.y =-0.964875;
	 pose.orientation.z =0.256041;
	 pose.orientation.w =-0.0428377;
 }
 
 if(sitio==1){
	 pose.position.x = 0.1028377;
	 pose.position.y = -0.581449;
	 pose.position.z = 0.673022;
	pose.orientation.x =-0.9;
	pose.orientation.y =-0.964875;
	 pose.orientation.z =0.256041;
	 pose.orientation.w =-0.0428377;
 }
// 
 if(sitio==2){
	 pose.position.x = 0.1028377;
	 pose.position.y = -0.621449;
	 pose.position.z = 0.673022;
	pose.orientation.x =-0.9;
	pose.orientation.y =-0.964875;
	 pose.orientation.z =0.256041;
	 pose.orientation.w =-0.0428377;
 }
// 
 if(sitio==3){
	 pose.position.x = -0.1028377;
	 pose.position.y = -0.621449;
	 pose.position.z = 0.673022;
	pose.orientation.x =-0.9;
	pose.orientation.y =-0.964875;
	 pose.orientation.z =0.256041;
	 pose.orientation.w =-0.0428377;
 }

 
 

			sitio=sitio+1;
		    pub.publish(pose);
		    ros::spinOnce();
		    ROS_INFO("Published message %f %f", pose.position.x,pose.position.y);
	//        // Wait 1 second to publish again
		    loop_rate.sleep();
		    
		     if(sitio==4){
		     	sitio=0;
		     }
    }

    return 0;
}//END MAIN
