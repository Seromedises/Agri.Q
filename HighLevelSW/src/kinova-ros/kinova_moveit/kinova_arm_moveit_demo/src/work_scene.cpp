//////// Caricamento del work-scene -- Modifica del file originale Kinova////////
/*
Autore: Giovanni Colucci
Data: 23/09
*/


#include <ros/ros.h>

#include <tf_conversions/tf_eigen.h>  // Necessaria per dichiarare tf::Quaternion


#include <geometry_msgs/Pose.h>       // Necessari per dichiarare la posa degli ogetti ed il fatto che siano delle mesh e fare operazioni su di esse
#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"
#include <geometric_shapes/shape_operations.h>


#include <moveit/planning_scene_interface/planning_scene_interface.h>


#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/ApplyPlanningScene.h>


tf::Quaternion EulerZYZ_to_Quaternion(double tz1, double ty, double tz2)  //Necessario per poter passare da Rappresentazione in angoli di Eulero ZYZ in quaternione
{
    tf::Quaternion q;
    tf::Matrix3x3 rot;
    tf::Matrix3x3 rot_temp;
    rot.setIdentity();

    rot_temp.setEulerYPR(tz1, 0.0, 0.0);
    rot *= rot_temp;
    rot_temp.setEulerYPR(0.0, ty, 0.0);
    rot *= rot_temp;
    rot_temp.setEulerYPR(tz2, 0.0, 0.0);
    rot *= rot_temp;
    rot.getRotation(q);
    return q;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "work_scene");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::NodeHandle nh;

    // Display debug information in teminal
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
        ros::console::notifyLoggerLevelsChanged();
    }


    // Define publisher to update work scene
    ros::Publisher pub_work_scene = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    while(pub_work_scene.getNumSubscribers() < 1)
    {
        ros::WallDuration sleep_t(0.5);
        sleep_t.sleep();
    }

  // Parametri

    Eigen::Vector3d vectorScale(0.001, 0.001, 0.001);  // Mi serve per scalare gli .stl


    // Define agri.q 
    shape_msgs::Mesh agriq;
    shapes::Mesh *meshObject = shapes::createMeshFromResource(
          "file:///home/agriq/catkin_ws/src/kinova-ros/kinova_moveit/kinova_arm_moveit_demo/src/meshes/00_agriq_semplificato.STL", vectorScale);
          
    shapes::ShapeMsg meshMessage;
    shapes::constructMsgFromShape(meshObject, meshMessage);
    agriq = boost::get<shape_msgs::Mesh>(meshMessage);
    
    tf::Quaternion q;
    q = EulerZYZ_to_Quaternion(180*M_PI/180,0*M_PI/180,0*M_PI/180);
  
   geometry_msgs::Pose agriQ_Pose;
   agriQ_Pose.orientation.w = q.w();
   agriQ_Pose.orientation.x = q.x();
   agriQ_Pose.orientation.y = q.y();
   agriQ_Pose.orientation.z = q.z();
   agriQ_Pose.position.x = -0.10;
   agriQ_Pose.position.y = 0.45;
   agriQ_Pose.position.z = -0.65+0.12;
 

    // Define collision objects
    moveit_msgs::CollisionObject collision_objects;
    collision_objects.id = "agriq";
    collision_objects.header.frame_id = "root";
    collision_objects.meshes.push_back(agriq);
    collision_objects.mesh_poses.push_back(agriQ_Pose);
    collision_objects.operation = collision_objects.ADD;



  // Define Grape
  
    shape_msgs::Mesh Grape;
    shapes::Mesh *meshObject1 = shapes::createMeshFromResource(
          "file:///home/agriq/catkin_ws/src/kinova-ros/kinova_moveit/kinova_arm_moveit_demo/src/meshes/grape.stl", vectorScale);
          
    shapes::ShapeMsg meshMessage1;
    shapes::constructMsgFromShape(meshObject1, meshMessage1);
    Grape = boost::get<shape_msgs::Mesh>(meshMessage1);
    
    tf::Quaternion q2;
    q2 = EulerZYZ_to_Quaternion(180*M_PI/180,0*M_PI/180,0*M_PI/180);
  
   geometry_msgs::Pose Grape_Pose;
   Grape_Pose.orientation.w = q.w();
   Grape_Pose.orientation.x = q.x();
   Grape_Pose.orientation.y = q.y();
   Grape_Pose.orientation.z = q.z();
   Grape_Pose.position.x = 0.1 - 0.01;  // Lo 0.01 serve per mettere a posto il sistema di riferimento della mesh
   Grape_Pose.position.y = 0.6;
   Grape_Pose.position.z = 0.8 - 0.1;  // Lo 0.1 serve per mettere a posto il sistema di riferimento dell stl
 

    // Define collision objects
    moveit_msgs::CollisionObject collision_objects1;
    collision_objects1.id = "Grape";
    collision_objects1.header.frame_id = "root";
    collision_objects1.meshes.push_back(Grape);
    collision_objects1.mesh_poses.push_back(Grape_Pose);
    collision_objects1.operation = collision_objects1.ADD;

// Aggiunto per non farlo sbattere sul muro //

    // Define table plane
    shape_msgs::SolidPrimitive table;
    table.type = table.BOX;
    table.dimensions.resize(3);
    table.dimensions[0] = 0.1; // x
    table.dimensions[1] = 1; // y
    table.dimensions[2] = 1.; // z
    // Define table position
    geometry_msgs::Pose table_pose;
    table_pose.orientation.w = 1.0;
    table_pose.position.x = 0.6;
    table_pose.position.y = table.dimensions[1]/2.0;
    table_pose.position.z =  table.dimensions[2]/2.0;

    // Define collision objects
    moveit_msgs::CollisionObject collision_objects2;
    collision_objects2.id = "table";
    collision_objects2.header.frame_id = "root";
    collision_objects2.primitives.push_back(table);
    collision_objects2.primitive_poses.push_back(table_pose);
    collision_objects2.operation = collision_objects2.ADD;




    // Add all objects to environment
    ROS_INFO("Adding the all objects to the work scene.");
    moveit_msgs::PlanningScene work_scene;
//    work_scene.world.collision_objects.push_back(attached_objects.object);
    work_scene.world.collision_objects.push_back(collision_objects);
    work_scene.world.collision_objects.push_back(collision_objects1);
    work_scene.world.collision_objects.push_back(collision_objects2);
    work_scene.is_diff = true;
    pub_work_scene.publish(work_scene);
    ros::WallDuration(1).sleep();


    ros::shutdown();
    return 0;
}
