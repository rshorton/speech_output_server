/*
Copyright 2021 Scott Horton

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/


#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include "speech_action_interfaces/action/speak.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace speech_output_action_server
{
class SpeakActionClient : public rclcpp::Node
{
public:
  using Speak = speech_action_interfaces::action::Speak;
  using GoalHandleSpeak = rclcpp_action::ClientGoalHandle<Speak>;

  explicit SpeakActionClient(const rclcpp::NodeOptions & options)
  : Node("speak_action_client", options)
  {
    this->client_ptr_ = rclcpp_action::create_client<Speak>(
      this,
      "speak");

    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&SpeakActionClient::send_goal, this));
  }

  void send_goal()
  {
    using namespace std::placeholders;

    this->timer_->cancel();

    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = Speak::Goal();
    goal_msg.text = "Hello, I'm alive";

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<Speak>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&SpeakActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&SpeakActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&SpeakActionClient::result_callback, this, _1);
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<Speak>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;

  void goal_response_callback(const GoalHandleSpeak::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
    GoalHandleSpeak::SharedPtr,
    const std::shared_ptr<const Speak::Feedback> feedback)
  {
    std::stringstream ss;
    ss << "State: " << feedback->state;
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
  }

  void result_callback(const GoalHandleSpeak::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
    std::stringstream ss;
    ss << "Result received: " << result.result->result;
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    rclcpp::shutdown();
  }
};  // class SpeakActionClient

}  // namespace speech_output_action_server

RCLCPP_COMPONENTS_REGISTER_NODE(speech_output_action_server::SpeakActionClient)
