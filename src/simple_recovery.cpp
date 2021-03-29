#include <simple_recovery.h>
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
PLUGINLIB_EXPORT_CLASS(SimpleRecovery, nav_core::RecoveryBehavior)

SimpleRecovery::SimpleRecovery():
    local_costmap_(NULL), initialized_(false), world_model_(NULL)
{
}

void SimpleRecovery::initialize(std::string name, tf::TransformListener*, costmap_2d::Costmap2DROS*, costmap_2d::Costmap2DROS* local_costmap)
{
  if (!initialized_)
  {
    local_costmap_ = local_costmap;

    world_model_ = new base_local_planner::CostmapModel(*local_costmap_->getCostmap());

    initialized_ = true;
  }
  else
  {
    ROS_ERROR("You should not call initialize twice on this object, doing nothing");
  }
}

SimpleRecovery::~SimpleRecovery()
{
  delete world_model_;
}

void SimpleRecovery::runBehavior()
{
  if (!initialized_)
  {
    ROS_ERROR("This object must be initialized before runBehavior is called");
    return;
  }

  if (local_costmap_ == NULL)
  {
    ROS_ERROR("The costmap passed to the SimpleRecovery object cannot be NULL. Doing nothing.");
    return;
  }
  ROS_WARN("Simple recovery behavior started.");

  ros::Rate r(2);
  ros::NodeHandle n;

  ros::ServiceClient client = n.serviceClient<std_srvs::Empty>("next_goal");

  std_srvs::Empty srv;

  if (client.call(srv)) {
    ROS_INFO("service call successfull");
  }
  else {
    ROS_INFO("sfailed to call service");
  }
}
