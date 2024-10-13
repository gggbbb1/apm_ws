/**
 * @brief AHRS plugin
 * @file ahrs2.cpp
 * @author GG BB <gggbbb@dev.com>
 *
 * @addtogroup plugin
 * @{
 */

#include <string>

#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

namespace mavros
{
namespace extra_plugins
{
using namespace std::placeholders;      // NOLINT

/**
 * @brief Mavlink AHRS2 plugin.
 * @plugin ahrs2
 *
 * This plugin publishes GPS sensor data from a Mavlink compatible FCU to ROS.
 */
class AHRS2Plugin : public plugin::Plugin
{
public:
  explicit AHRS2Plugin(plugin::UASPtr uas_)
  : Plugin(uas_, "ahrs2")
  {

    enable_node_watch_parameters();

    pose_pub = node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("~/alt", 10);

    node_declare_and_watch_parameter(
      "alt_stdev", 0.0, [&](const rclcpp::Parameter & p) {
        alt_stdev = p.as_double();
      });
    // tf params
    node_declare_and_watch_parameter(
      "tf.frame_id", "map", [&](const rclcpp::Parameter & p) {
        tf_frame_id = p.as_string();
      });
  }

  Subscriptions get_subscriptions() override
  {
    return {
      make_handler(&AHRS2Plugin::handle_ahrs2),
    };
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub;

  double alt_stdev;
  std::string tf_frame_id;

  /* -*- callbacks -*- */
  /**
   * @brief Publish <a href="https://mavlink.io/en/messages/common.html#GPS_RAW_INT">mavlink GPS_RAW_INT message</a> into the gps1/raw topic.
   */
  void handle_ahrs2(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    mavlink::ardupilotmega::msg::AHRS2 & mav_msg,
    plugin::filter::SystemAndOk filter [[maybe_unused]])
    {
    auto ros_msg = geometry_msgs::msg::PoseWithCovarianceStamped();

    ros_msg.header.stamp = node->now();
    ros_msg.header.frame_id = std::string(tf_frame_id);
    ros_msg.pose.pose.position.z = mav_msg.altitude;
    ros_msg.pose.covariance[14] = alt_stdev*alt_stdev;
    pose_pub->publish(ros_msg);
  }
};
}       // namespace extra_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::extra_plugins::AHRS2Plugin)
