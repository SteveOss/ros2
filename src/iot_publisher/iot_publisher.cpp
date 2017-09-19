#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging.h"
#include "iot_msgs/msg/instance.hpp"
#include "iot_msgs/ValueHelper.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:

  MinimalPublisher () : Node ("iot_publisher"), count_ (0)
  {
    // Create pubisher and register timed callback

    publisher_ = this->create_publisher <iot_msgs::msg::Instance> ("IOTData");
    timer_ = this->create_wall_timer (500ms, std::bind (&MinimalPublisher::timer_callback, this));
  }

private:

  void timer_callback ()
  {
    auto instance = iot_msgs::msg::Instance ();
    std::vector<iot_msgs::msg::NVP> vals;
    rcutils_time_point_value_t now = 0;
    rcutils_ret_t status;

    // Get timestamp, logging any errors

    status = rcutils_system_time_now (&now);
    if (status != RCUTILS_RET_OK)
    {
      rcutils_log (NULL, RCUTILS_LOG_SEVERITY_ERROR, "rcutils_system_time_now", "returned %d", status);
    }

    // Set some sample named values

    iot_msgs::ValueHelper::add<std::string> (vals, "Location", "Room 101");
    iot_msgs::ValueHelper::add<bool> (vals, "Active", true);
    iot_msgs::ValueHelper::add<uint16_t> (vals, "Sample", count_++);
    iot_msgs::ValueHelper::add<uint64_t> (vals, "Timestamp", now);
    iot_msgs::ValueHelper::add<int32_t> (vals, "T1 Current", 18);
    iot_msgs::ValueHelper::add<int32_t> (vals, "T1 Min", -5);
    iot_msgs::ValueHelper::add<int32_t> (vals, "T1 Max", 55);
    instance.set__values (vals);

    // Set instance name and key

    instance.set__type_name ("Temperature Sensor");
    instance.set__key ("1");

    // Publish instance

    publisher_->publish (instance);
  }

  uint16_t count_;
  rclcpp::Publisher<iot_msgs::msg::Instance>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main (int argc, char * argv[])
{
  rclcpp::init (argc, argv);
  rcutils_logging_initialize ();
  rclcpp::spin (std::make_shared<MinimalPublisher> ());
  rclcpp::shutdown ();
  return 0;
}
