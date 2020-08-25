/*********************************************************************
* Software License Agreement (BSD License)
* Author: anchuanxu
*********************************************************************/
#ifndef SMART_RECOVERY_SMART_RECOVERY_H
#define SMART_RECOVERY_SMART_RECOVERY_H
#include <nav_core/recovery_behavior.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/buffer.h>
#include <base_local_planner/costmap_model.h>
#include <string>
#include <sensor_msgs/LaserScan.h>

namespace smart_recovery
{
/**
 * @class SmartRecovery
 * @brief A recovery behavior that to relieve the plight of the robot
 */
class SmartRecovery : public nav_core::RecoveryBehavior
{
public:
  /**
   * @brief  Constructor, make sure to call initialize in addition to actually initialize the object
   */
  SmartRecovery();

  /**
   * @brief  Initialization function for the SmartRecovery recovery behavior
   * @param name Namespace used in initialization
   * @param tf (unused)
   * @param global_costmap (unused)
   * @param local_costmap A pointer to the local_costmap used by the navigation stack
   */
  void initialize(std::string name, tf2_ros::Buffer*,
                  costmap_2d::Costmap2DROS*, costmap_2d::Costmap2DROS* local_costmap);

  /**
   * @brief  Run the SmartRecovery recovery behavior.
   */
  void runBehavior();

  /**
   * @brief  Destructor for the smart recovery behavior
   */
  ~SmartRecovery();

private:
  costmap_2d::Costmap2DROS* local_costmap_;
  bool initialized_;
  double sim_granularity_, min_rotational_vel_, max_rotational_vel_, acc_lim_th_, tolerance_, frequency_;
  base_local_planner::CostmapModel* world_model_;
  ros::Publisher* vel_pub_;
  bool canGofoword,canGoback;
  double pi;
  bool rotate180();
  bool turnToGlobalPath();
  bool turnAngle(double angle);
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr msg);
  void publishZeroVelocity();
  void goStraight(double distance);
  void goBack(double distance);
};
};  // namespace smart_recovery
#endif  // SMART_RECOVERY_SMART_RECOVERY_H
