/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*********************************************************************/
#include <detect_recovery/detect_recovery.h>
#include <pluginlib/class_list_macros.h>
#include <nav_core/parameter_magic.h>
#include <tf2/utils.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <angles/angles.h>
#include <algorithm>
#include <string>
#include <std_msgs/Int16MultiArray.h>


// register this planner as a RecoveryBehavior plugin
PLUGINLIB_EXPORT_CLASS(detect_recovery::DetectRecovery, nav_core::RecoveryBehavior)

namespace detect_recovery
{
DetectRecovery::DetectRecovery(): local_costmap_(NULL), initialized_(false), world_model_(NULL)
{

}

void DetectRecovery::initialize(std::string name, tf2_ros::Buffer*,
                                costmap_2d::Costmap2DROS*, costmap_2d::Costmap2DROS* local_costmap)
{
  if (!initialized_)
  {
    local_costmap_ = local_costmap;

    // get some parameters from the parameter server
    ros::NodeHandle private_nh("~/" + name);
    ros::NodeHandle blp_nh("~/TrajectoryPlannerROS");

    // we'll simulate every degree by default
    private_nh.param("sim_granularity", sim_granularity_, 0.017);
    private_nh.param("frequency", frequency_, 20.0);


    acc_lim_th_ = nav_core::loadParameterWithDeprecation(blp_nh, "acc_lim_theta", "acc_lim_th", 3.2);
    max_rotational_vel_ = nav_core::loadParameterWithDeprecation(blp_nh, "max_vel_theta", "max_rotational_vel", 1.0);
    min_rotational_vel_ = nav_core::loadParameterWithDeprecation(blp_nh, "min_in_place_vel_theta", "min_in_place_rotational_vel", 0.4);
    blp_nh.param("yaw_goal_tolerance", tolerance_, 0.10);

    world_model_ = new base_local_planner::CostmapModel(*local_costmap_->getCostmap());
    initialized_ = true;

    canGofoword = true;
    canGoback = true;
    pi = 3.14;
  }
  else
  {
    ROS_ERROR("You should not call initialize twice on this object, doing nothing");
  }
}

DetectRecovery::~DetectRecovery()
{
  delete world_model_;
}

void DetectRecovery::runBehavior()
{
    ROS_INFO("---Detect Recovery started---");
    //获取激光数据
    ros::Rate r(frequency_);
    ros::NodeHandle scan_nh;
    ros::Subscriber scan_sub;
    scan_sub = scan_nh.subscribe<sensor_msgs::LaserScan>("/scan",1,boost::bind(&DetectRecovery::scanCallback,this,_1));
    int turnCount = 0;
    int goStrCount = 0;

    while(turnCount < 4)
    {
        //前方可以行进
        if(canGofoword)
        {
            goStraight(0.05); //前进5cm
            goStrCount++;
            if(goStrCount > 10)
            {
                ROS_INFO("There are few obstacles ahead, Detect recovery end.");
                return;
            }
        }
        //前方不可前进
        else {
               bool turnResult = turnAngle(90);
               if(turnResult)
               {
                 turnCount++;
                 goStrCount = 0;
               }
               else {
                 ROS_ERROR("I can't turn my body!");
               }
            }
        }
    ROS_ERROR("---I have tried many times and recover over---");
    return;
}
void DetectRecovery::goStraight(double distance)
{
    ros::Rate r(frequency_);
    ros::NodeHandle nv;
    ros::Publisher vel_pub_ = nv.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    double linear_speed = 0.2;
    double linear_duration = distance / fabs(linear_speed);

    int ticks = int(linear_duration * frequency_);
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    while(ticks > 0)
    {
        ticks --;
        cmd_vel.linear.x = linear_speed;
        vel_pub_.publish(cmd_vel);
        r.sleep();
    }
    cmd_vel.linear.x = 0.0;
    vel_pub_.publish(cmd_vel);
    ROS_ERROR("---go stright end---");
}
void DetectRecovery::goBack(double distance)
{

    ros::Rate r(frequency_);
    ros::NodeHandle nv;
    ros::Publisher vel_pub_ = nv.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    double linear_speed = -0.2;
    double linear_duration = distance / fabs(linear_speed);

    int ticks = int(linear_duration * frequency_);
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    while(ticks > 0)
    {
        ticks --;
        cmd_vel.linear.x = linear_speed;
        vel_pub_.publish(cmd_vel);
        r.sleep();
    }
    cmd_vel.linear.x = 0.0;
    vel_pub_.publish(cmd_vel);
}
void DetectRecovery::publishZeroVelocity()
{
    ros::Rate r(frequency_);
    ros::NodeHandle nv;
    ros::Publisher vel_pub_ = nv.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    vel_pub_.publish(cmd_vel);
}
void DetectRecovery::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    int16_t obs_count = 0;
    sensor_msgs::LaserScan message = *msg;
    //xbot use this code
//    for (int i = 0; i < 90; i++) {
//      if (0.1 < message.ranges[i] && message.ranges[i] < 0.5) {
//        obs_count++;
//      }
//    }
//    for (int i = 630; i < 720; i++) {
//      if (0.1 < message.ranges[i] && message.ranges[i] < 0.5) {
//        obs_count++;
//      }
//    }

    //others bot use this code
    for (int i = 90; i < 270; i++) {
        if(0.1 < message.ranges[i] && message.ranges[i] < 1.0 )
            obs_count++;
    }
    std::cout << "obs_count = "<< obs_count << std::endl;
    if(obs_count > 20){
        canGofoword = false;
    }
    else {
        canGofoword = true;
    }
}
bool DetectRecovery::turnAngle(double angle)
{
    ros::Rate r(frequency_);
    ros::NodeHandle nv;
    ros::Publisher vel_pub_ = nv.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    double angle_speed = 0.2;
    double angle_duration = (angle / 360 * 2 * pi) / angle_speed;
    int ticks = int(angle_duration * frequency_);
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    while(ticks > 0)
    {
        ticks --;
        cmd_vel.angular.z = angle_speed;
        vel_pub_.publish(cmd_vel);
        r.sleep();
    }
    cmd_vel.angular.z = 0.0;
    vel_pub_.publish(cmd_vel);
    return true;
}
};  // namespace detect_recovery
