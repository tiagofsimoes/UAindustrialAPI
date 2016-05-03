#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lesson_moveit");
  moveit::planning_interface::MoveGroup group("manipulator");

  // start a background "spinner", so our node can process ROS messages
  //  - this lets us know when the move is completed
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // INSERT MOTION COMMANDS HERE

  group.setNamedTarget("home");
std::cout << "Passei aqui: home ";
  group.move();
  sleep(5.0);
  
  
  group.getCurrentJointValues ();
  
  
  Eigen::Affine3d approach = Eigen::Translation3d(0.1, 0, 0.1) *
                             Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitY());
  group.setPoseTarget(approach);
std::cout << "Passei aqui: approach ";
  group.move();
  sleep(1.0);





  Eigen::Affine3d pick = approach.translate(0.1*Eigen::Vector3d::UnitZ());
  group.setPoseTarget(pick);
std::cout << "Passei aqui: pick ";
group.move();
  sleep(5.0);




  Eigen::Affine3d retreat1 = pick.translate(-0.1*Eigen::Vector3d::UnitZ());
  group.setPoseTarget(retreat1);
std::cout << "Passei aqui: retreat ";
  group.move();
  sleep(5.0);
  
  group.setNamedTarget("ff");
  
  Eigen::Affine3d retreat = pick.translate(-0.1*Eigen::Vector3d::UnitY());
  group.setPoseTarget(retreat);
std::cout << "Passei aqui: retreat ";
  group.move();
  sleep(5.0);





  std::vector<double> inspectPos;
  inspectPos.push_back(-0.35); inspectPos.push_back(0.45); inspectPos.push_back(0.57);
  inspectPos.push_back(-0.64); inspectPos.push_back(-1.03); inspectPos.push_back(0.29);
  group.setJointValueTarget(inspectPos);
std::cout << "Passei aqui: inspectPos ";
  group.move();
  sleep(5.0);
  
  





  group.setNamedTarget("home");
  std::cout << "Passei aqui: home ";
    group.move();
    sleep(5.0);
  
}
