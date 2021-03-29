#include <stdio.h>      /* printf, NULL */
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <backup_recovery.h>
#include <pluginlib/class_list_macros.h>
//#include <nav_core/parameter_magic.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <angles/angles.h>
#include <algorithm>
#include <string>
#include <ros/console.h>
#include <std_srvs/Empty.h>


// register this planner as a RecoveryBehavior plugin
PLUGINLIB_EXPORT_CLASS(BackupRecovery, nav_core::RecoveryBehavior)

BackupRecovery::BackupRecovery():
    local_costmap_(NULL), initialized_(false), world_model_(NULL)
{
}

void BackupRecovery::initialize(std::string name, tf::TransformListener*, costmap_2d::Costmap2DROS*, costmap_2d::Costmap2DROS* local_costmap)
{
  if (!initialized_)
  {
    local_costmap_ = local_costmap;

    world_model_ = new base_local_planner::CostmapModel(*local_costmap_->getCostmap());

    srand(time(NULL));

    initialized_ = true;
  }
  else
  {
    ROS_ERROR("You should not call initialize twice on this object, doing nothing");
  }
}

BackupRecovery::~BackupRecovery()
{
  delete world_model_;
}

void BackupRecovery::runBehavior()
{
  if (!initialized_)
  {
    ROS_ERROR("This object must be initialized before runBehavior is called");
    return;
  }

  if (local_costmap_ == NULL)
  {
    ROS_ERROR("The costmap passed to the BackupRecovery object cannot be NULL. Doing nothing.");
    return;
  }
  ROS_WARN("Backup recovery behavior started.");

//  ros::Rate r(2);
  ros::NodeHandle n;

  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);

  //geometry_msgs::PoseStamped global_pose;
  //local_costmap_->getRobotPose(global_pose);

  //double current_angle = tf2::getYaw(global_pose.pose.orientation);
  //double start_angle = current_angle;

     double   vel = 0.3;

    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = -0.2;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = (rand() % 10 + 5) / 100;

    vel_pub.publish(cmd_vel);

   ros::Duration(3).sleep(); 

    cmd_vel.linear.x = 0;
    cmd_vel.angular.z = 0;

    vel_pub.publish(cmd_vel);
}
