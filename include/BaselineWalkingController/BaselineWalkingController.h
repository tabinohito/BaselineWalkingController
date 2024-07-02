#pragma once

#include <mc_control/fsm/Controller.h>

#include <BaselineWalkingController/FootTypes.h>

#ifdef USE_ICEORYX
#  include "iceoryx_posh/popo/publisher.hpp"
#  include "iceoryx_posh/runtime/posh_runtime.hpp"
#  include "iox/signal_watcher.hpp"
#endif

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_msgs/Float32MultiArray.h>

namespace mc_tasks
{
struct CoMTask;
struct OrientationTask;

namespace force
{
struct FirstOrderImpedanceTask;
}
} // namespace mc_tasks

namespace BWC
{
class FootManager;
class CentroidalManager;

#ifdef USE_ICEORYX
/** \brief RadarObject. */
struct foot_info
{
  struct force
  {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
  } force;
  struct torque
  {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
  } torque;
};

struct tactileInfo
{
  double time = 0.0;
  foot_info left;
  foot_info right;
};
#endif

/** \brief Humanoid walking controller with various baseline methods. */
struct BaselineWalkingController : public mc_control::fsm::Controller
{
public:
  /** \brief Constructor.
      \param rm robot module
      \param dt control timestep
      \param _config controller configuration
      \param allowEmptyManager whether to allow the managers to be empty (assuming initialized in the parent class)
   */
  BaselineWalkingController(mc_rbdyn::RobotModulePtr rm,
                            double dt,
                            const mc_rtc::Configuration & _config,
                            bool allowEmptyManager = false);

  /** \brief Reset a controller.

      This method is called when starting the controller.
   */
  void reset(const mc_control::ControllerResetData & resetData) override;

  /** \brief Run a controller.

      This method is called every control period.
   */
  bool run() override;

  /** \brief Stop a controller.

      This method is called when stopping the controller.
   */
  void stop() override;

  /** \brief Get controller name. */
  inline const std::string & name() const
  {
    return name_;
  }

  /** \brief Get current time. */
  inline double t() const noexcept
  {
    return t_;
  }

  /** \brief Get timestep. */
  inline double dt() const
  {
    return solver().dt();
  }

  /** \brief Set default anchor. */
  void setDefaultAnchor();

public:
  //! CoM task
  std::shared_ptr<mc_tasks::CoMTask> comTask_;

  //! Base link orientation task
  std::shared_ptr<mc_tasks::OrientationTask> baseOriTask_;

  //! Foot tasks
  std::unordered_map<Foot, std::shared_ptr<mc_tasks::force::FirstOrderImpedanceTask>> footTasks_;

  //! Foot manager
  std::shared_ptr<FootManager> footManager_;

  //! Centroidal manager
  std::shared_ptr<CentroidalManager> centroidalManager_;

  //! Whether to enable manager update
  bool enableManagerUpdate_ = false;

protected:
  //! Controller name
  std::string name_ = "BWC";

  //! Current time [sec]
  double t_ = 0;

private:
#ifdef USE_ICEORYX
  //! method to publish and subscribe communication
  void update_iceoryx();

  //! Name of APP
  static constexpr char APP_NAME[] = "iox-cpp-publisher-helloworld";

  //! Create a publisher for tactileInfo
  std::shared_ptr<iox::popo::Publisher<tactileInfo>> publisher_ = nullptr;
#endif
  std::unique_ptr<ros::NodeHandle> nh_;
  ros::CallbackQueue callbackQueue_;
  ros::Subscriber contact_area_sub_;
};
} // namespace BWC
