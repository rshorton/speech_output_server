#include <functional>
#include <memory>

#include "speech_action_interfaces/action/speak.hpp"
#include "face_control_interfaces/msg/smile.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "speech_output_proc.h"

namespace speech_output_action_server
{
class SpeechOutputActionServer : public rclcpp::Node
{
public:
  using Speak = speech_action_interfaces::action::Speak;
  using GoalHandleSpeak = rclcpp_action::ServerGoalHandle<Speak>;

  explicit SpeechOutputActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("speech_output_action_server", options),
	speech_proc_(NULL)
  {
    using namespace std::placeholders;

    smile_publisher_ = this->create_publisher<face_control_interfaces::msg::Smile>("/head/smile", 2);
    speech_active_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/head/speaking", 2);

    this->action_server_ = rclcpp_action::create_server<Speak>(
      this,
      "speak",
      std::bind(&SpeechOutputActionServer::handle_goal, this, _1, _2),
      std::bind(&SpeechOutputActionServer::handle_cancel, this, _1),
      std::bind(&SpeechOutputActionServer::handle_accepted, this, _1));

    speech_proc_ = new SpeechOutputProc();
    speech_proc_->Open();
    speech_proc_->SetSpeakCB(std::bind(&SpeechOutputActionServer::speech_finished, this, _1));
    speech_proc_->SetSmileCB(std::bind(&SpeechOutputActionServer::set_smile_mode, this, _1));
    speech_proc_->SetSpeechActiveCB(std::bind(&SpeechOutputActionServer::set_speaking_active, this, _1));
  }

  void speech_finished(SpeechProcStatus status)
  {
	  auto result = std::make_shared<Speak::Result>();
	  if (status == SpeechProcStatus_Ok) {
		  RCLCPP_INFO(this->get_logger(), "Speech output complete");
		  result->result = "OK";
	  } else {
		  RCLCPP_INFO(this->get_logger(), "Error outputing speech");
		  result->result = "ERROR";
	  }
	  goal_handle_->succeed(result);
  }

  void set_smile_mode(std::string mode)
  {
	  RCLCPP_INFO(this->get_logger(), "Set smile mode: %s", mode.c_str());
	  auto message = face_control_interfaces::msg::Smile();
      message.mode = mode;
      message.level = 0;
      message.duration_ms = 0;
      message.use_as_default = false;
      smile_publisher_->publish(message);
  }

  void set_speaking_active(bool bSpeaking)
  {
	  RCLCPP_INFO(this->get_logger(), "Set speaking: %d", bSpeaking);
	  auto message = std_msgs::msg::Bool();
      message.data = bSpeaking;
      speech_active_publisher_->publish(message);
  }

private:
  rclcpp_action::Server<Speak>::SharedPtr action_server_;

  std::shared_ptr<GoalHandleSpeak> goal_handle_;

  SpeechOutputProc *speech_proc_;

  rclcpp::Publisher<face_control_interfaces::msg::Smile>::SharedPtr smile_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr speech_active_publisher_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Speak::Goal> goal)
  {

	//std::cout << "Received goal request with text: " << goal->text << std::endl;

    RCLCPP_INFO(this->get_logger(), "Received goal request with text [%s]", goal->text.c_str());
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleSpeak> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    speech_proc_->SpeakStop();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleSpeak> goal_handle)
  {
    using namespace std::placeholders;
    goal_handle_ = goal_handle;
    const auto goal = goal_handle->get_goal();
    if (speech_proc_->SpeakStart(goal->text) == SpeechProcStatus_Error) {
	  auto result = std::make_shared<Speak::Result>();
	  result->result = "ERROR";
	  goal_handle_->succeed(result);
	  RCLCPP_INFO(this->get_logger(), "Goal failed to start");
    }
  }

};  // class SpeechOutputActionServer

}  // namespace speech_output_action_server

RCLCPP_COMPONENTS_REGISTER_NODE(speech_output_action_server::SpeechOutputActionServer)
