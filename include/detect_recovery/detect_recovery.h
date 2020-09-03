/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2020, Anchuanxu.
*  All rights reserved.
* Author: Chuanxu An
*********************************************************************/
#ifndef DETECT_RECOVERY_DETECT_RECOVERY_H
#define DETECT_RECOVERY_DETECT_RECOVERY_H
#include <nav_core/recovery_behavior.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/buffer.h>
#include <base_local_planner/costmap_model.h>
#include <string>
#include <sensor_msgs/LaserScan.h>

namespace detect_recovery
{
/**
 * @class RoRecovery
 * @brief A recovery behavior that ros the robot in-place to attempt to clear out space
 */
class DetectRecovery : public nav_core::RecoveryBehavior
{
public:
  /**
   * @brief  Constructor, make sure to call initialize in addition to actually initialize the object
   */
  DetectRecovery();

  /**
   * @brief  Initialization function for the DetectRecovery recovery behavior
   * @param name Namespace used in initialization
   * @param tf (unused)
   * @param global_costmap (unused)
   * @param local_costmap A pointer to the local_costmap used by the navigation stack
   */
  void initialize(std::string name, tf2_ros::Buffer*,
                  costmap_2d::Costmap2DROS*, costmap_2d::Costmap2DROS* local_costmap);

  /**
   * @brief  Run the DetectRecovery recovery behavior.
   */
  void runBehavior();

  /**
   * @brief  Destructor for the detect recovery behavior
   */
  ~DetectRecovery();

private:
  costmap_2d::Costmap2DROS* local_costmap_;
  bool initialized_;
  double sim_granularity_, min_rotational_vel_, max_rotational_vel_, acc_lim_th_, tolerance_, frequency_;
  base_local_planner::CostmapModel* world_model_;
  bool canGofoword,canGoback;
  double pi;
  bool turnAngle(double angle);
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
  void publishZeroVelocity();
  void goStraight(double distance);
  void goBack(double distance);
};
};  // namespace detect_recovery
#endif  // DETECT_RECOVERY_DETECT_RECOVERY_H
