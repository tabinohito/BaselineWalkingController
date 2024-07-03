#pragma once

#include <BaselineWalkingController/State.h>

#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>

namespace BWC
{
/** \brief FSM state to walk with Tactileeration. */
struct TactileState : State
{
public:
  /** \brief Start. */
  void start(mc_control::fsm::Controller & ctl) override;

  /** \brief Run. */
  bool run(mc_control::fsm::Controller & ctl) override;

  /** \brief Teardown. */
  void teardown(mc_control::fsm::Controller & ctl) override;

protected:
  /** \brief ROS callback of tactile topic. */
  void tactileCallback(const std_msgs::Float32MultiArray::ConstPtr & tactileMsg);

protected:
  //! Relative min target foot contact pose (x [m], y [m])
  std::unordered_map<Foot, Eigen::Vector3d> min_pose_;

  //! Relative max target foot contact pose (x [m], y [m])
  std::unordered_map<Foot, Eigen::Vector3d> max_pose_;

  //! Sensor based detect touch down
  std::unordered_map<Foot, bool> touchDown_;

  //! 4 points of contact area　both foot
  std::unordered_map<Foot, std::vector<Eigen::Vector2d>> contactArea_;

  //! Tactile sensor name list
  std::array<Foot, 2> feet = {Foot::Left, Foot::Right};

  //! ROS variables
  //! @{
  std::unique_ptr<ros::NodeHandle> nh_;
  ros::CallbackQueue callbackQueue_;
  ros::Subscriber tactileSub_;
  //! @}
};
} // namespace BWC
