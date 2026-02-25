#include "rclcpp/rclcpp.hpp"
#include "identification_interface/msg/action_info.hpp"

namespace identification
{
    class ActionidentifSubscriber
    {
        public:
            using actionType = identification_interface::msg::ActionInfo;

            actionidentifClient(const rclcpp::Node::SharedPtr& node_ptr) : node_ptr_(node_ptr)
            {
                subscriber_ = node_ptr_->create_subscription<actionType>("action_info",10,
                    std::bind(&MinimalSubscriber::topic_callback,this,_1));
            }
            
            //If you want to add things to the callback
            void topic_callback(const actionType &msg) const
            {
                RCLCPP_INFO(this->get_logger(),"The action is : '%s'",msg.action_name.c_str())
            }

            private:
                rclcpp::Subscription<actionType>::SharedPtr subscriber_;
    };
}
