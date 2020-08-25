/*********************************************************************
* Software License Agreement (BSD License)
* Author: anchuanxu
*********************************************************************/
#include <smart_recovery/smart_recovery.h>
#include <pluginlib/class_list_macros.h>
#include <nav_core/parameter_magic.h>
#include <tf2/utils.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <angles/angles.h>
#include <algorithm>
#include <string>

// register this planner as a RecoveryBehavior plugin
PLUGINLIB_EXPORT_CLASS(smart_recovery::SmartRecovery, nav_core::RecoveryBehavior)

namespace smart_recovery
{
SmartRecovery::SmartRecovery(): local_costmap_(NULL), initialized_(false), world_model_(NULL)
{
}

void SmartRecovery::initialize(std::string name, tf2_ros::Buffer*,
                                costmap_2d::Costmap2DROS*, costmap_2d::Costmap2DROS* local_costmap)
{
  if (!initialized_)
  {
    local_costmap_ = local_costmap;

    // get some parameters from the parameter server
    ros::NodeHandle private_nh("~/" + name);
    ros::NodeHandle blp_nh("~/TrajectoryPlannerROS");

    //获取激光数据
    ros::NodeHandle scan_nh;
    ros::Subscriber scan_sub = scan_nh.subscribe<sensor_msgs::LaserScan>("/scan", 1, boost::bind(&SmartRecovery::scanCallback),this,1);

    // we'll simulate every degree by default
    private_nh.param("sim_granularity", sim_granularity_, 0.017);
    private_nh.param("frequency", frequency_, 20.0);

    acc_lim_th_ = nav_core::loadParameterWithDeprecation(blp_nh, "acc_lim_theta", "acc_lim_th", 3.2);
    max_rotational_vel_ = nav_core::loadParameterWithDeprecation(blp_nh, "max_vel_theta", "max_rotational_vel", 1.0);
    min_rotational_vel_ = nav_core::loadParameterWithDeprecation(blp_nh, "min_in_place_vel_theta", "min_in_place_rotational_vel", 0.4);
    blp_nh.param("yaw_goal_tolerance", tolerance_, 0.10);

    world_model_ = new base_local_planner::CostmapModel(*local_costmap_->getCostmap());

    initialized_ = true;
    canGofoword = false;
    canGoback = false;
    pi = 3.14;
  }
  else
  {
    ROS_ERROR("You should not call initialize twice on this object, doing nothing");
  }
}

SmartRecovery::~SmartRecovery()
{
  delete world_model_;
}

void SmartRecovery::runBehavior()
{
  ROS_WARN("------Into SmartRecovery------");

  //判断有没有全局路径规划出来
  std::vector<geometry_msgs::PoseStamped> global_plan;
  if(!global_plan.empty())
  {
      //存在全局规划路径，尝试进行前后挪移
      int turnCount = 0;
      while(turnCount < 4)
      {

          //前方可以行进
          if(canGofoword)
          {
              goStraight(0.05); //前进5cm
              publishZeroVelocity();
          }
          //前方不可前进
          else {
              if(canGoback)
              {
                goBack(0.05);//后退5cm
                publishZeroVelocity();
              }
              else { //不能前进也不能后退，旋转90度试试别的方位
                  turnAngle(pi / 2);
                  turnCount ++;
              }
          }
      }
  }
  else
  {
      //没有全局规划路径，旋转180度，尝试能否给出规划
      rotate180();
  }
}
void SmartRecovery::goStraight(double distance)
{
    double linear_speed = 0.2;
    double linear_duration = distance / linear_speed;
    ros::Rate r(10);
    int ticks = int(linear_duration * 10);
    geometry_msgs::Twist cmd_vel;
    for(int i = 0; i < ticks; i++)
    {
       cmd_vel.linear.x = linear_speed;
       vel_pub_->publish(cmd_vel);
       r.sleep();
    }
    return ;
}
void SmartRecovery::goBack(double distance)
{
    double linear_speed = -0.2;
    double linear_duration = distance / fabs(linear_speed);
    ros::Rate r(10);
    int ticks = int(linear_duration * 10);
    geometry_msgs::Twist cmd_vel;
    for(int i = 0; i < ticks; i++)
    {
       cmd_vel.linear.x = linear_speed;
       vel_pub_->publish(cmd_vel);
       r.sleep();
    }
    return ;
}
void SmartRecovery::publishZeroVelocity()
{
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    vel_pub_->publish(cmd_vel);
}
void SmartRecovery::scanCallback(const sensor_msgs::LaserScan::ConstPtr msg)
{
    int obs_count = 0;
    int obs_back_count = 0;
    for (int i = 0; i < 180; i++) {
      if (0.1 < msg->ranges[i] && msg->ranges[i] < 0.5) {
        obs_count++;
      }
    }
    for (int i = 540; i < 720; i++) {
      if (0.1 < msg->ranges[i] && msg->ranges[i] < 0.5) {
        obs_count++;
      }
    }
    for (int i = 180; i < 540; i++) {
        if (0.1 < msg->ranges[i] && msg->ranges[i] < 0.5) {
          obs_back_count++;
        }
    }
    if(obs_count > 20)
        canGofoword = false;
    else {
        canGofoword = true;
    }
    if(obs_back_count > 20)
        canGoback = false;
    else {
        canGoback = true;
    }
}
bool SmartRecovery::turnAngle(double angle)
{
    double angle_speed = 0.2;
    double angle_duration = angle / angle_speed;
    ros::Rate r(10);
    int ticks = int(angle_duration * 10);
    geometry_msgs::Twist cmd_vel;
    for(int i = 0; i < ticks; i++)
    {
       cmd_vel.linear.z = angle_speed;
       vel_pub_->publish(cmd_vel);
       r.sleep();
    }
    return true;
}
bool SmartRecovery::turnToGlobalPath(){
//TODO
}
bool SmartRecovery::rotate180()
{
    if (!initialized_)
    {
      ROS_ERROR("This object must be initialized before runBehavior is called");
      return false;
    }

    if (local_costmap_ == NULL)
    {
      ROS_ERROR("The costmap passed to the SmartRecovery object cannot be NULL. Doing nothing.");
      return false;
    }
    ROS_WARN("Smart recovery behavior started.");

    ros::Rate r(frequency_);
    ros::NodeHandle n;
    ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    geometry_msgs::PoseStamped global_pose;
    local_costmap_->getRobotPose(global_pose);

    double current_angle = tf2::getYaw(global_pose.pose.orientation);
    double start_angle = current_angle;

    bool got_180 = false;

    while (n.ok() &&
           (!got_180 ||
            std::fabs(angles::shortest_angular_distance(current_angle, start_angle)) > tolerance_))
    {
      // Update Current Angle
      local_costmap_->getRobotPose(global_pose);
      current_angle = tf2::getYaw(global_pose.pose.orientation);

      // compute the distance left to Smart
      double dist_left;
      if (!got_180)
      {
        // If we haven't hit 180 yet, we need to Smart a half circle plus the distance to the 180 point
        double distance_to_180 = std::fabs(angles::shortest_angular_distance(current_angle, start_angle + M_PI));
        dist_left = M_PI + distance_to_180;

        if (distance_to_180 < tolerance_)
        {
          got_180 = true;
        }
      }
      else
      {
        // If we have hit the 180, we just have the distance back to the start
        dist_left = std::fabs(angles::shortest_angular_distance(current_angle, start_angle));
      }

      double x = global_pose.pose.position.x, y = global_pose.pose.position.y;

      // check if that velocity is legal by forward simulating
      double sim_angle = 0.0;
      while (sim_angle < dist_left)
      {
        double theta = current_angle + sim_angle;

        // make sure that the point is legal, if it isn't... we'll abort
        double footprint_cost = world_model_->footprintCost(x, y, theta, local_costmap_->getRobotFootprint(), 0.0, 0.0);
        if (footprint_cost < 0.0)
        {
          ROS_ERROR("Smart recovery can't run in place because there is a potential collision. Cost: %.2f",
                    footprint_cost);
          return false;
        }

        sim_angle += sim_granularity_;
      }

      // compute the velocity that will let us stop by the time we reach the goal
      double vel = sqrt(2 * acc_lim_th_ * dist_left);

      // make sure that this velocity falls within the specified limits
      vel = std::min(std::max(vel, min_rotational_vel_), max_rotational_vel_);

      geometry_msgs::Twist cmd_vel;
      cmd_vel.linear.x = 0.0;
      cmd_vel.linear.y = 0.0;
      cmd_vel.angular.z = vel;

      vel_pub.publish(cmd_vel);

      r.sleep();
    }
    return true;
}
};  // namespace smart_recovery
