#include <mc_rtc/gui/Button.h>
#include <mc_rtc/ros.h>

#include <BaselineWalkingController/BaselineWalkingController.h>
#include <BaselineWalkingController/FootManager.h>
#include <BaselineWalkingController/states/TactileState.h>

using namespace BWC;

void TactileState::start(mc_control::fsm::Controller & _ctl)
{
  State::start(_ctl);

  // Skip if ROS is not initialized
  if(!mc_rtc::ROSBridge::get_node_handle())
  {
    mc_rtc::log::warning("[TactileState] ROS is not initialized.");
    output("OK");
    return;
  }

  // Load configuration
  std::string tactileTopicName = "/tactile_sensor_system_client/contact_area";
  if(config_.has("configs"))
  {
    // 無視する領域のconfigを読み込む

    // if(config_("configs").has("velScale"))
    // {
    //   velScale_ = config_("configs")("velScale");
    //   velScale_[2] = mc_rtc::constants::toRad(velScale_[2]);
    // }
    config_("configs")("tactileTopicName", tactileTopicName);
  }

  // Setup ROS
  nh_ = std::make_unique<ros::NodeHandle>();
  // Use a dedicated queue so as not to call callbacks of other modules
  nh_->setCallbackQueue(&callbackQueue_);
  tactileSub_ = nh_->subscribe<std_msgs::Float32MultiArray>(tactileTopicName, 1, &TactileState::tactileCallback, this);

  output("OK");
}

bool TactileState::run(mc_control::fsm::Controller &)
{
  // Finish if ROS is not initialized
  if(!mc_rtc::ROSBridge::get_node_handle())
  {
    return true;
  }

  // Call ROS callback
  callbackQueue_.callAvailable(ros::WallDuration());

  //   // Set target velocity
  //   if(ctl().footManager_->velModeEnabled())
  //   {
  //     ctl().footManager_->setRelativeVel(targetVel_);
  //   }

  return false;
}

void TactileState::teardown(mc_control::fsm::Controller &) {}

void TactileState::tactileCallback(const std_msgs::Float32MultiArray::ConstPtr & tactileMsg)
{
  mc_rtc::log::info("[TactileState] Tactile callback");
  for(auto foot : feet)
  {
    int footIndex_offset = foot == Foot::Left ? 0 : 9;
    auto tactileData = tactileMsg->data;
    contactArea_[foot]["x_max"] = Eigen::Vector2d(tactileData[0 + footIndex_offset], tactileData[1 + footIndex_offset]);
    contactArea_[foot]["x_min"] = Eigen::Vector2d(tactileData[2 + footIndex_offset], tactileData[3 + footIndex_offset]);
    contactArea_[foot]["y_max"] = Eigen::Vector2d(tactileData[4 + footIndex_offset], tactileData[5 + footIndex_offset]);
    contactArea_[foot]["y_min"] = Eigen::Vector2d(tactileData[6 + footIndex_offset], tactileData[7 + footIndex_offset]);
    touchDown_[foot] = tactileData[8 + footIndex_offset] ? true : false;
  }
}

EXPORT_SINGLE_STATE("BWC::Tactile", TactileState)
