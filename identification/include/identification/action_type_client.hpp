#include "rclcpp/rclcpp.hpp"
#include "identification_interface/srv/action_type.hpp"

namespace identification
{
    class ActionidentifClient
    {
        public:
            using actionType = identification_interface::srv::ActionType;

            ActionidentifClient(const rclcpp::Node::SharedPtr& node_ptr) : node_ptr_(node_ptr)
            {
                client_ = node_ptr_->create_client<actionType>("action_type");
            }

            std::string send_request(bool askedToIdentify)
            {
                auto request = std::make_shared<actionType::Request>();
                request->action_type_request = askedToIdentify;

                // wait for service
                using namespace std::chrono_literals;
                if (!client_->wait_for_service(1s))
                {
                RCLCPP_ERROR(rclcpp::get_logger("action_classification_client"), "Interrupted while waiting for the service. Exiting.");
                return "Nothing";
                }

                // send request and wait for response
                auto future_result = client_->async_send_request(request);
                if (future_result.wait_for(120s) != std::future_status::ready)
                {
                RCLCPP_ERROR(rclcpp::get_logger("action_classification_client"), "Failed to identify the action...");
                }

                RCLCPP_INFO(rclcpp::get_logger("action_classification_client"), "Action identified successfully !");
                return future_result.get()->data_folder_name;
            }

            private:
                rclcpp::Node::SharedPtr node_ptr_;
                rclcpp::Client<actionType>::SharedPtr client_;
    };
}
