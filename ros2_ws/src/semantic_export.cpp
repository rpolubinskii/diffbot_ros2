#include <functional>
#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <image_transport/image_transport.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace diffbot
{

using Image = sensor_msgs::msg::Image;
using CameraInfo = sensor_msgs::msg::CameraInfo;
using SyncPolicy = message_filters::sync_policies::ApproximateTime<Image, Image>;

// Robot-side semantic export: subscribes to the hardware-synced color + aligned
// depth, rate-limits to a keyframe rate, looks up the map-frame camera pose from
// TF, and republishes a single coherent, identically-stamped bundle
// {color, depth, camera_info, odom} for the offboard semantic map (DualMap).
// Compression is handled by image_transport plugins on the throttled output, so
// only a few Mbit/s of compressed RGB-D + a tiny Odometry cross WiFi. RTAB-Map
// keeps consuming the raw camera topics directly and is untouched.
class SemanticExport : public rclcpp::Node
{
public:
  SemanticExport()
  : Node("diffbot_semantic_export")
  {
    color_topic_ = declare_parameter<std::string>("color_topic", "/camera/camera/color/image_raw");
    depth_topic_ = declare_parameter<std::string>(
      "depth_topic", "/camera/camera/aligned_depth_to_color/image_raw");
    camera_info_topic_ =
      declare_parameter<std::string>("camera_info_topic", "/camera/camera/color/camera_info");

    out_color_topic_ = declare_parameter<std::string>("out_color_topic", "/semantic/color/image_raw");
    out_depth_topic_ =
      declare_parameter<std::string>("out_depth_topic", "/semantic/aligned_depth/image_raw");
    out_camera_info_topic_ =
      declare_parameter<std::string>("out_camera_info_topic", "/semantic/color/camera_info");
    out_odom_topic_ = declare_parameter<std::string>("out_odom_topic", "/semantic/odom");

    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    camera_frame_ = declare_parameter<std::string>("camera_frame", "camera_color_optical_frame");
    target_rate_hz_ = declare_parameter<double>("target_rate_hz", 4.0);
    tf_timeout_sec_ = declare_parameter<double>("tf_timeout_sec", 0.1);

    min_period_ = rclcpp::Duration::from_seconds(target_rate_hz_ > 0.0 ? 1.0 / target_rate_hz_ : 0.0);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    camera_info_sub_ = create_subscription<CameraInfo>(
      camera_info_topic_, rclcpp::SensorDataQoS(),
      [this](CameraInfo::ConstSharedPtr msg) { last_camera_info_ = msg; });

    color_pub_ = image_transport::create_publisher(this, out_color_topic_);
    depth_pub_ = image_transport::create_publisher(this, out_depth_topic_);
    camera_info_pub_ = create_publisher<CameraInfo>(out_camera_info_topic_, rclcpp::SensorDataQoS());
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      out_odom_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

    color_sub_.subscribe(this, color_topic_, rmw_qos_profile_sensor_data);
    depth_sub_.subscribe(this, depth_topic_, rmw_qos_profile_sensor_data);
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), color_sub_, depth_sub_);
    sync_->registerCallback(std::bind(&SemanticExport::on_frame, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(
      get_logger(),
      "semantic_export: %s + %s @ %.1f Hz -> %s/{color,aligned_depth,camera_info,odom} "
      "| pose %s->%s",
      color_topic_.c_str(), depth_topic_.c_str(), target_rate_hz_, "/semantic",
      map_frame_.c_str(), camera_frame_.c_str());
  }

private:
  void on_frame(const Image::ConstSharedPtr & color, const Image::ConstSharedPtr & depth)
  {
    const rclcpp::Time stamp(color->header.stamp);

    if (last_pub_time_.nanoseconds() != 0 && (stamp - last_pub_time_) < min_period_) {
      return;  // rate gate (cheap; runs before the TF lookup)
    }

    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(
        map_frame_, camera_frame_, stamp, tf2::durationFromSec(tf_timeout_sec_));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "TF %s->%s unavailable at frame stamp, skipping frame: %s",
        map_frame_.c_str(), camera_frame_.c_str(), ex.what());
      return;  // no pose -> don't emit a frame the mapper can't place
    }
    last_pub_time_ = stamp;

    // One stamp for the whole bundle so the offboard time-sync is exact.
    const auto frame_stamp = color->header.stamp;

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = frame_stamp;
    odom.header.frame_id = map_frame_;
    odom.child_frame_id = camera_frame_;
    odom.pose.pose.position.x = tf.transform.translation.x;
    odom.pose.pose.position.y = tf.transform.translation.y;
    odom.pose.pose.position.z = tf.transform.translation.z;
    odom.pose.pose.orientation = tf.transform.rotation;

    color_pub_.publish(*color);

    Image depth_out = *depth;
    depth_out.header.stamp = frame_stamp;
    depth_pub_.publish(depth_out);

    if (last_camera_info_) {
      CameraInfo camera_info = *last_camera_info_;
      camera_info.header.stamp = frame_stamp;
      camera_info_pub_->publish(camera_info);
    }

    odom_pub_->publish(odom);
  }

  std::string color_topic_;
  std::string depth_topic_;
  std::string camera_info_topic_;
  std::string out_color_topic_;
  std::string out_depth_topic_;
  std::string out_camera_info_topic_;
  std::string out_odom_topic_;
  std::string map_frame_;
  std::string camera_frame_;
  double target_rate_hz_{4.0};
  double tf_timeout_sec_{0.1};
  rclcpp::Duration min_period_{0, 0};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  message_filters::Subscriber<Image> color_sub_;
  message_filters::Subscriber<Image> depth_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  rclcpp::Subscription<CameraInfo>::SharedPtr camera_info_sub_;
  CameraInfo::ConstSharedPtr last_camera_info_;

  image_transport::Publisher color_pub_;
  image_transport::Publisher depth_pub_;
  rclcpp::Publisher<CameraInfo>::SharedPtr camera_info_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  rclcpp::Time last_pub_time_{0, 0, RCL_ROS_TIME};
};

}  // namespace diffbot

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<diffbot::SemanticExport>());
  rclcpp::shutdown();
  return 0;
}
