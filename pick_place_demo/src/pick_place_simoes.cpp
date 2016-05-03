
////******************************************************************************************************
  // Todos os includes
////******************************************************************************************************

#include <ros/ros.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group.h>
#include <geometric_shapes/solid_primitive_dims.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>


static const std::string ROBOT_DESCRIPTION="robot_description";


////******************************************************************************************************
  // approach_pick, que faz o ponto de aproximação para o pick
////******************************************************************************************************
void approach_pick(moveit::planning_interface::MoveGroup &group,double x,double y, double z)
{
//group.asyncMove ();
  	 group.setEndEffectorLink("tool0");
  	 	group.setPlannerId("RRTConnectkConfigDefault");//SBLkConfigDefault
  	 	
	group.setPlanningTime(0);
	
	unsigned int num_planning_attempts=0;
	group.setNumPlanningAttempts (num_planning_attempts);
	
	double tolerance=15;
	group.setGoalJointTolerance (tolerance);
	
  std::vector<moveit_msgs::Grasp> grasps;

  geometry_msgs::PoseStamped p;
  p.header.frame_id = "base_link";
//  p.pose.position.x = x;
//  p.pose.position.y = y;
//  p.pose.position.z = z;
//  p.pose.orientation.x = 0;
//  p.pose.orientation.y = 0;
//  p.pose.orientation.z = 0;
//  p.pose.orientation.w = 1;
  
  // We can plan a motion for this group to a desired pose for the 
  // end-effector.
  geometry_msgs::Pose target_pose1;
  geometry_msgs::Pose target_pose2;
//  geometry_msgs::Pose target_pose3;

	 // Orientation
    double angle =M_PI;
    Eigen::Quaterniond quat(Eigen::AngleAxis<double>(double(angle), Eigen::Vector3d::UnitZ()));
    
	 target_pose1.position.x = x;
	 target_pose1.position.y = y;
	 target_pose1.position.z = z;
    target_pose1.orientation.x = quat.x();
    target_pose1.orientation.y = quat.y();
    target_pose1.orientation.z = quat.z();
    target_pose1.orientation.w = quat.w();
    
    
    
    angle =M_PI;
    Eigen::Quaterniond quat1(Eigen::AngleAxis<double>(double(angle), Eigen::Vector3d::UnitY()));
    target_pose2.position.x=target_pose1.position.x;
	target_pose2.position.y = target_pose1.position.y;
	target_pose2.position.z = target_pose1.position.z;
	target_pose2.orientation.x = quat1.x();
    target_pose2.orientation.y = quat1.y();
    target_pose2.orientation.z = quat1.z();
    target_pose2.orientation.w = quat1.w();
	 

	p.pose.position.x = x;
	p.pose.position.y = y;
	p.pose.position.z = z;
 	p.pose.orientation.x = quat1.x();
    p.pose.orientation.y = quat1.y();
    p.pose.orientation.z = quat1.z();
    p.pose.orientation.w = quat1.w();
  
    group.setPoseTarget(target_pose1);
    group.setPoseTarget(target_pose2);
    group.setPoseTarget(p);
  
  
  // Now, we call the planner to compute the plan
  // and visualize it.
  // Note that we are just planning, not asking move_group 
  // to actually move the robot.
  moveit::planning_interface::MoveGroup::Plan my_plan1;
  bool success = group.plan(my_plan1);

  ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");  
    /* Sleep to give Rviz time to visualize the plan. */
//  sleep(1.0);
  
//  	group.asyncExecute(my_plan);
    group.move();
//    group.execute(my_plan1);
    
}





























////******************************************************************************************************
  // pick, pick do objecto
////******************************************************************************************************
void pick(moveit::planning_interface::MoveGroup &group,double x,double y, double z,std::string obj)
{
//group.asyncMove ();
  	group.setEndEffectorLink("tool0");
  	group.setPlannerId("RRTConnectkConfigDefault");//SBLkConfigDefault
  	 	
	group.setPlanningTime(0);
	
	unsigned int num_planning_attempts=0;
	group.setNumPlanningAttempts (num_planning_attempts);
	
	double tolerance=15;
	group.setGoalJointTolerance (tolerance);
	
  	std::vector<moveit_msgs::Grasp> grasps;

 	 geometry_msgs::PoseStamped p;
 		 p.header.frame_id = "base_link";
//  p.pose.position.x = x;
//  p.pose.position.y = y;
//  p.pose.position.z = z;
//  p.pose.orientation.x = 0;
//  p.pose.orientation.y = 0;
//  p.pose.orientation.z = 0;
//  p.pose.orientation.w = 1;
  
  // We can plan a motion for this group to a desired pose for the 
  // end-effector.
  geometry_msgs::Pose target_pose1;
  geometry_msgs::Pose target_pose2;
//  geometry_msgs::Pose target_pose3;

	 // Orientation
    double angle =M_PI;
    Eigen::Quaterniond quat(Eigen::AngleAxis<double>(double(angle), Eigen::Vector3d::UnitZ()));
    
	 target_pose1.position.x = x;
	 target_pose1.position.y = y;
	 target_pose1.position.z = z;
    target_pose1.orientation.x = quat.x();
    target_pose1.orientation.y = quat.y();
    target_pose1.orientation.z = quat.z();
    target_pose1.orientation.w = quat.w();
    
    
    
    angle =M_PI;
    Eigen::Quaterniond quat1(Eigen::AngleAxis<double>(double(angle), Eigen::Vector3d::UnitY()));
    target_pose2.position.x=target_pose1.position.x;
	target_pose2.position.y = target_pose1.position.y;
	target_pose2.position.z = target_pose1.position.z;
	target_pose2.orientation.x = quat1.x();
    target_pose2.orientation.y = quat1.y();
    target_pose2.orientation.z = quat1.z();
    target_pose2.orientation.w = quat1.w();
	 

	p.pose.position.x = x;
	p.pose.position.y = y;
	p.pose.position.z = z;
 	p.pose.orientation.x = quat1.x();
    p.pose.orientation.y = quat1.y();
    p.pose.orientation.z = quat1.z();
    p.pose.orientation.w = quat1.w();
  
    group.setPoseTarget(target_pose1);
    group.setPoseTarget(target_pose2);
    group.setPoseTarget(p);
  
  
  // Now, we call the planner to compute the plan
  // and visualize it.
  // Note that we are just planning, not asking move_group 
  // to actually move the robot.
  moveit::planning_interface::MoveGroup::Plan my_plan2;
  bool success = group.plan(my_plan2);

  ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");  
    /* Sleep to give Rviz time to visualize the plan. */
//  sleep(1.0);
  
  
  
//  	group.asyncExecute(my_plan2);
    group.move();
//    group.execute(my_plan2);  


  group.setSupportSurfaceName("table");
	group.attachObject(obj);
}





























////******************************************************************************************************
  // approach_place, que faz o ponto de aproximação para o place
////******************************************************************************************************
void approach_place(moveit::planning_interface::MoveGroup &group,double x,double y,double z)
{
//group.asyncMove ();

 group.setEndEffectorLink("tool0");
  	 	group.setPlannerId("RRTConnectkConfigDefault");//SBLkConfigDefault
  	 	
	group.setPlanningTime(0);
	
	unsigned int num_planning_attempts=0;
	group.setNumPlanningAttempts (num_planning_attempts);
	
	double tolerance=15;
	group.setGoalJointTolerance (tolerance);
	
  std::vector<moveit_msgs::Grasp> grasps;

  geometry_msgs::PoseStamped p;
  p.header.frame_id = "base_link";
//  p.pose.position.x = x;
//  p.pose.position.y = y;
//  p.pose.position.z = z;
//  p.pose.orientation.x = 0;
//  p.pose.orientation.y = 0;
//  p.pose.orientation.z = 0;
//  p.pose.orientation.w = 1;
  
  // We can plan a motion for this group to a desired pose for the 
  // end-effector.
  geometry_msgs::Pose target_pose1;
  geometry_msgs::Pose target_pose2;
//  geometry_msgs::Pose target_pose3;

	 // Orientation
    double angle =M_PI;
    Eigen::Quaterniond quat(Eigen::AngleAxis<double>(double(angle), Eigen::Vector3d::UnitZ()));
    
	 target_pose1.position.x = x;
	 target_pose1.position.y = y;
	 target_pose1.position.z = z;
    target_pose1.orientation.x = quat.x();
    target_pose1.orientation.y = quat.y();
    target_pose1.orientation.z = quat.z();
    target_pose1.orientation.w = quat.w();
    
    
    
    angle =M_PI;
    Eigen::Quaterniond quat1(Eigen::AngleAxis<double>(double(angle), Eigen::Vector3d::UnitY()));
    target_pose2.position.x=target_pose1.position.x;
	target_pose2.position.y = target_pose1.position.y;
	target_pose2.position.z = target_pose1.position.z;
	target_pose2.orientation.x = quat1.x();
    target_pose2.orientation.y = quat1.y();
    target_pose2.orientation.z = quat1.z();
    target_pose2.orientation.w = quat1.w();
	 

	p.pose.position.x = x;
	p.pose.position.y = y;
	p.pose.position.z = z;
 	p.pose.orientation.x = quat1.x();
    p.pose.orientation.y = quat1.y();
    p.pose.orientation.z = quat1.z();
    p.pose.orientation.w = quat1.w();
  
    group.setPoseTarget(target_pose1);
    group.setPoseTarget(target_pose2);
    group.setPoseTarget(p);
  
  
  // Now, we call the planner to compute the plan
  // and visualize it.
  // Note that we are just planning, not asking move_group 
  // to actually move the robot.
  moveit::planning_interface::MoveGroup::Plan my_plan3;
  bool success = group.plan(my_plan3);

  ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");  
    /* Sleep to give Rviz time to visualize the plan. */
//  sleep(1.0);
  
//  	group.asyncExecute(my_plan3);
    group.move();
//    group.execute(my_plan3);    

}





























////******************************************************************************************************
  // place, place do objecto
////******************************************************************************************************
void place(moveit::planning_interface::MoveGroup &group,double x,double y,double z,std::string obj)
{
//group.asyncMove ();

 group.setEndEffectorLink("tool0");
  	 	group.setPlannerId("RRTConnectkConfigDefault");//SBLkConfigDefault
	group.setPlanningTime(0);
	
	unsigned int num_planning_attempts=0;
	group.setNumPlanningAttempts (num_planning_attempts);
	
	double tolerance=15;
	group.setGoalJointTolerance (tolerance);
	
  std::vector<moveit_msgs::Grasp> grasps;

  geometry_msgs::PoseStamped p;
  p.header.frame_id = "base_link";
//  p.pose.position.x = x;
//  p.pose.position.y = y;
//  p.pose.position.z = z;
//  p.pose.orientation.x = 0;
//  p.pose.orientation.y = 0;
//  p.pose.orientation.z = 0;
//  p.pose.orientation.w = 1;
  
  // We can plan a motion for this group to a desired pose for the 
  // end-effector.
  geometry_msgs::Pose target_pose1;
  geometry_msgs::Pose target_pose2;
//  geometry_msgs::Pose target_pose3;

	 // Orientation
    double angle =M_PI;
    Eigen::Quaterniond quat(Eigen::AngleAxis<double>(double(angle), Eigen::Vector3d::UnitZ()));
    
	 target_pose1.position.x = x;
	 target_pose1.position.y = y;
	 target_pose1.position.z = z;
    target_pose1.orientation.x = quat.x();
    target_pose1.orientation.y = quat.y();
    target_pose1.orientation.z = quat.z();
    target_pose1.orientation.w = quat.w();
    
    
    
    angle =M_PI;
    Eigen::Quaterniond quat1(Eigen::AngleAxis<double>(double(angle), Eigen::Vector3d::UnitY()));
    target_pose2.position.x=target_pose1.position.x;
	target_pose2.position.y = target_pose1.position.y;
	target_pose2.position.z = target_pose1.position.z;
	target_pose2.orientation.x = quat1.x();
    target_pose2.orientation.y = quat1.y();
    target_pose2.orientation.z = quat1.z();
    target_pose2.orientation.w = quat1.w();
	 

	p.pose.position.x = x;
	p.pose.position.y = y;
	p.pose.position.z = z;
 	p.pose.orientation.x = quat1.x();
    p.pose.orientation.y = quat1.y();
    p.pose.orientation.z = quat1.z();
    p.pose.orientation.w = quat1.w();
  
    group.setPoseTarget(target_pose1);
    group.setPoseTarget(target_pose2);
    group.setPoseTarget(p);
  
  
  // Now, we call the planner to compute the plan
  // and visualize it.
  // Note that we are just planning, not asking move_group 
  // to actually move the robot.
  moveit::planning_interface::MoveGroup::Plan my_plan4;
  bool success = group.plan(my_plan4);

  ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");  
    /* Sleep to give Rviz time to visualize the plan. */
//  sleep(1.0);
  
//  	group.asyncExecute(my_plan);
    group.move();
//    group.execute(my_plan4);

    
      group.detachObject(obj);  

}


































int main(int argc, char **argv)
{
  ros::init (argc, argv, "pick_place_SIMOES");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  

  ros::NodeHandle nh;
  ros::Publisher pub_co = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 10);
  ros::Publisher pub_aco1 = nh.advertise<moveit_msgs::AttachedCollisionObject>("attached_collision_object", 10);

//  ros::WallDuration(1.0).sleep();

  moveit::planning_interface::MoveGroup group("manipulator");
  
  	 group.setEndEffectorLink("tool0");
  
//  group.setPlanningTime(5.0);

  moveit_msgs::CollisionObject co;
  co.header.stamp = ros::Time::now();
  co.header.frame_id = "base_link";

  
   // remove pole
  co.id = "table";
  co.operation = moveit_msgs::CollisionObject::REMOVE;
  pub_co.publish(co);
//ROS_INFO("ERRO AQUI!! 1111111111111111111.\n");
////////////  // add pole
  co.operation = moveit_msgs::CollisionObject::ADD;
  co.primitives.resize(1);
  co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  co.primitives[0].dimensions.resize(geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.900;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 1.250;
  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.035;
  co.primitive_poses.resize(1);
  co.primitive_poses[0].position.x = 0.355;
  co.primitive_poses[0].position.y = 0;
  co.primitive_poses[0].position.z = 0;
  co.primitive_poses[0].orientation.w = 0;
  pub_co.publish(co);



////////////  // remove table
//  co.id = "table";
//  co.operation = moveit_msgs::CollisionObject::REMOVE;
//  pub_co.publish(co);

////////////  // add table
//  co.operation = moveit_msgs::CollisionObject::ADD;
//  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.900;//0.900
//  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 2.250;//1.250
//  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.035;
//  co.primitive_poses[0].position.x = (0.450);
//  co.primitive_poses[0].position.y = (2.250)/2;
//  co.primitive_poses[0].position.z = -0.2;
//  pub_co.publish(co);


	co.id = "part1";
	  co.operation = moveit_msgs::CollisionObject::REMOVE;
	  pub_co.publish(co);
	 
	  moveit_msgs::AttachedCollisionObject aco1;
	  aco1.object = co;
	  pub_aco1.publish(aco1);
	 
	  co.operation = moveit_msgs::CollisionObject::ADD;
	  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.090;
	  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.100;
	  co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.200;

	  co.primitive_poses[0].position.x = 0.320;
	  co.primitive_poses[0].position.y = -0.300;
	  co.primitive_poses[0].position.z = 0.200;
	  pub_co.publish(co);
  
  ROS_INFO("Inicio pick\n");
  approach_pick(group,0.320,-0.300,0.450);

  			pick(group,0.320,-0.300,0.350,"part1");
  approach_pick(group,0.320,-0.300,0.450);
ROS_INFO("Fim PICK\n");

ROS_INFO("Inicio place\n");
  approach_place(group,0.320,0.000,0.450);
 		   place(group,0.320,0.000,0.350,"part1");
  approach_place(group,0.320,0.000,0.450);
ROS_INFO("Fim PLACE\n");

  sleep(4.0);
//  ros::waitForShutdown();
  return 0;
}

