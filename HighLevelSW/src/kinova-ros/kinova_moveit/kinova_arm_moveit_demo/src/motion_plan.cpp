#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <kinova_driver/kinova_ros_types.h>



int main(int argc, char **argv)
{
  ros::init(argc, argv, "motion_plan");
  ros::NodeHandle node_handle;  
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Setup
  // ^^^^^
  // 
  // The :move_group_interface:`MoveGroup` class can be easily 
  // setup using just the name
  // of the group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface group("arm");

  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to deal directly with the world.
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  

  // (Optional) Create a publisher for visualizing plans in Rviz.
  ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
  
  // We can also print the name of the end-effector link for this group.
  ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

  // Home position for jaco:
//  currentCartesianCommand = [feedback.X, feedback.Y, feedback.Z, feedback.ThetaX, feedback.ThetaY, feedback.Z]
//  currentCartesianCommand = [0.212322831154, -0.257197618484, 0.509646713734, 1.63771402836, 1.11316478252, 0.134094119072]

// Parameters //

  group.setMaxVelocityScalingFactor(1);
  group.setMaxAccelerationScalingFactor(1);
  
  group.setPlannerId("RRTstarkConfigDefault");
  //group.setPlanningTime(5);
  const double sleeptime = 3.0;
  
  

// Poses //


 // Home joint angles //
 
   std::vector<double> Home_Angles = {4.700178386786677, 2.8063496745128744, 6.257508173213478, 0.7187058186410845, -7.9793254412079335, 4.5172762115455605, -1.441835281010845};

  // Grasp Pose //
  
  tf::Pose Grasp;
  Grasp.setOrigin(tf::Vector3(0.1, 0.6,0.8));
  Grasp.setRotation(kinova::EulerXYZ2Quaternion(1.54, 3.14, 0));

  // Place Pose //
  
  tf::Pose Place;
  Place.setOrigin(tf::Vector3(0.1, -0.5, 0.15));
  Place.setRotation(kinova::EulerXYZ2Quaternion(1.54, 0, 0));
  
  
  
// Planning and executing


  // Planning to Home

  group.setJointValueTarget(Home_Angles);

  // Call the planner to compute the plan
  // and visualize it.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan1;
  bool success1 = (group.plan(my_plan1) == moveit_msgs::MoveItErrorCodes::SUCCESS);

  ROS_INFO("Visualizing plan 1 (pose goal) %s",success1?"":"FAILED");    
  /* Sleep to give Rviz time to visualize the plan. */
  //sleep(sleeptime);

  // Moving to home
group.move(); 

/*

  // Planning to Grasp
/*  geometry_msgs::Pose Grasp_pose;
  tf::poseTFToMsg(Grasp, Grasp_pose);
  group.setPoseTarget(Grasp_pose);
*/

  // Call the planner to compute the plan
  // and visualize it.
 /*
  moveit::planning_interface::MoveGroupInterface::Plan my_plan2;
  bool success2 = (group.plan(my_plan2) == moveit_msgs::MoveItErrorCodes::SUCCESS);

  ROS_INFO("Visualizing plan 2 (pose goal) %s",success2?"":"FAILED");    
*/
  /* Sleep to give Rviz time to visualize the plan. */
  //sleep(sleeptime);

  
  // Moving to a pose goal
//group.move();  */

  // Planning to Place


  moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = "j2s7s300_end_effector";
  ocm.header.frame_id = "j2s7s300_link_base";
  

  geometry_msgs::Pose Curr = group.getCurrentPose().pose;


  ocm.orientation.x = Curr.orientation.x;
  ocm.orientation.y =  Curr.orientation.y;
  ocm.orientation.z =  Curr.orientation.z;
  ocm.orientation.w =  Curr.orientation.w;
  ocm.absolute_x_axis_tolerance = 0.1;
  ocm.absolute_y_axis_tolerance = 0.1;
  ocm.absolute_z_axis_tolerance = 0.1;
  ocm.weight = 1.0; 
  
    // Now, set it as the path constraint for the group.
  moveit_msgs::Constraints test_constraints;
  test_constraints.orientation_constraints.push_back(ocm);
  group.setPathConstraints(test_constraints);


  geometry_msgs::Pose Place_pose;
  tf::poseTFToMsg(Place, Place_pose);
  group.setPoseTarget(Place_pose);


  moveit::planning_interface::MoveGroupInterface::Plan my_plan3;
  bool success3 = (group.plan(my_plan3) == moveit_msgs::MoveItErrorCodes::SUCCESS);

  ROS_INFO("Visualizing plan 3 (constraints) %s",success3?"":"FAILED");
  /* Sleep to give Rviz time to visualize the plan. */
  sleep(sleeptime);
  
  // Moving to a pose goal
group.move(); 

  // When done with the path constraint be sure to clear it.
  group.clearPathConstraints();


  ros::shutdown();  
  return 0;
}
