#include "rclcpp/rclcpp.hpp"
#include "iot_msgs/msg/instance.hpp"
#include "iot_msgs/ValueHelper.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:

  MinimalSubscriber () : Node ("iot_subscriber")
  {
    // Create subscription, registering update handler

    subscription_ = this->create_subscription <iot_msgs::msg::Instance>
      ("IOTData", std::bind (&MinimalSubscriber::topic_callback, this, _1));
  }

private:

  void topic_callback (iot_msgs::msg::Instance::SharedPtr instance)
  {
    // Print instance type and key

    printf ("%s (%s)\n", instance->type_name.c_str (), instance->key.c_str ());

    // Iterate and print values 

    std::vector<iot_msgs::msg::NVP>::iterator it = instance->values.begin ();
    while (it != instance->values.end ())
    {
      iot_msgs::ValueHelper::print (*it);
      it++;
    }
  }

  rclcpp::Subscription<iot_msgs::msg::Instance>::SharedPtr subscription_;
};

int main (int argc, char * argv[])
{
  rclcpp::init (argc, argv);
  rclcpp::spin (std::make_shared<MinimalSubscriber> ());
  rclcpp::shutdown ();
  return 0;
}
