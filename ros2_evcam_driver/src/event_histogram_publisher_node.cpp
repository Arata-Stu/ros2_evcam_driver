#include <memory>
#include <chrono>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "evcam_msgs/msg/event_histogram.hpp"  

#include <metavision/sdk/driver/camera.h>
#include <metavision/sdk/base/utils/log.h>

#include "Buffer.hpp"
#include "EventHistogram.hpp"

using namespace std::chrono_literals;

class EventHistogramPublisherNode : public rclcpp::Node {
public:
  EventHistogramPublisherNode()
  : Node("event_histogram_publisher_node")
  {
    this->declare_parameter<int>("publish_period_ms", 10);
    this->declare_parameter<int>("histogram_window_ms", 100);
    this->declare_parameter<int64_t>("count_cutoff", 0LL);
    this->declare_parameter<bool>("downsample", false);
    this->declare_parameter<int>("bins", 1);

    parameter_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&EventHistogramPublisherNode::onParameterChanged, this, std::placeholders::_1)
    );

    publisher_ = this->create_publisher<evcam_msgs::msg::EventHistogram>("event_histogram", 10);

    try {
      camera_ = std::make_shared<Metavision::Camera>(Metavision::Camera::from_first_available());
    } catch (const Metavision::CameraException &e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open camera: %s", e.what());
      rclcpp::shutdown();
      return;
    }

    const auto &geometry = camera_->geometry();
    sensor_width_  = geometry.width();
    sensor_height_ = geometry.height();
    RCLCPP_INFO(this->get_logger(), "Sensor geometry: width=%d, height=%d", sensor_width_, sensor_height_);

    callback_id_ = camera_->cd().add_callback(
      [this](const Metavision::EventCD *ev_begin, const Metavision::EventCD *ev_end) {
        event_buffer_.addEvents(ev_begin, ev_end);
      }
    );

    try {
      camera_->start();
    } catch (const Metavision::CameraException &e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to start camera: %s", e.what());
      rclcpp::shutdown();
      return;
    }

    int publish_period_ms = this->get_parameter("publish_period_ms").as_int();
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(publish_period_ms),
      std::bind(&EventHistogramPublisherNode::timerCallback, this)
    );

  }

  ~EventHistogramPublisherNode() {
    if (camera_) {
      camera_->stop();
      camera_->cd().remove_callback(callback_id_);
    }
  }

private:
  rcl_interfaces::msg::SetParametersResult onParameterChanged(const std::vector<rclcpp::Parameter> &parameters)
  {
    for (const auto &param : parameters) {
      if (param.get_name() == "publish_period_ms") {
        int new_period = param.as_int();
        timer_->cancel();
        timer_ = this->create_wall_timer(
          std::chrono::milliseconds(new_period),
          std::bind(&EventHistogramPublisherNode::timerCallback, this)
        );
        RCLCPP_INFO(this->get_logger(), "Updated publish_period_ms: %d", new_period);
      } else if (param.get_name() == "histogram_window_ms") {
        int duration = param.as_int();
        RCLCPP_INFO(this->get_logger(), "Updated histogram_window_ms: %d", duration);
      } else if (param.get_name() == "count_cutoff") {
        int64_t cutoff = param.as_int();
        RCLCPP_INFO(this->get_logger(), "Updated count_cutoff: %ld", cutoff);
      } else if (param.get_name() == "downsample") {
        RCLCPP_INFO(this->get_logger(), "Updated downsample: %s", param.as_bool() ? "true" : "false");
      } else if (param.get_name() == "bins") {
        int val = param.as_int();
        RCLCPP_INFO(this->get_logger(), "Updated bins: %d", val);
      }
    }

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "Parameters updated successfully";
    return result;
  }


  void timerCallback() {
    int histogram_window_ms = this->get_parameter("histogram_window_ms").as_int();
    int64_t window_duration_us = static_cast<int64_t>(histogram_window_ms) * 1000;
  
    int64_t cutoff_int = this->get_parameter("count_cutoff").as_int();
    uint8_t count_cutoff = static_cast<uint8_t>(std::min<int64_t>(cutoff_int, 255));
    bool downsample = this->get_parameter("downsample").as_bool();
    int bins = this->get_parameter("bins").as_int();
  
    // camera 時間ベースの now を取得
    int64_t now_us = event_buffer_.peekLatestTimestamp();
    if (now_us == -1) {
      RCLCPP_WARN(this->get_logger(), "No events yet, skipping this frame.");
      return;
    }
  
    // 過去 N μs のイベントを取得
    auto events_chunk = event_buffer_.getRecentEvents(now_us, window_duration_us);
    // RCLCPP_INFO(this->get_logger(), "events_chunk size: %zu", events_chunk.size());
    if (events_chunk.empty()) return;
  
    EventHistogram histogram(bins, sensor_width_, sensor_height_, count_cutoff, downsample);
    histogram.construct(events_chunk.data(), events_chunk.data() + events_chunk.size());
    // histogram.printDimensions();
  
    std::vector<uint8_t> histogram_on, histogram_off;
    histogram.getHistogramSeparated(histogram_on, histogram_off);
  
    evcam_msgs::msg::EventHistogram msg;
    msg.header.stamp = this->now();
    msg.width = static_cast<uint16_t>(sensor_width_);
    msg.height = static_cast<uint16_t>(sensor_height_);
    msg.bins = static_cast<uint8_t>(bins);
    msg.histogram_on = histogram_on;
    msg.histogram_off = histogram_off;
  
    publisher_->publish(msg);
  }
  
  

  rclcpp::Publisher<evcam_msgs::msg::EventHistogram>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<Metavision::Camera> camera_;
  EventBuffer event_buffer_;
  int callback_id_;
  int sensor_width_;
  int sensor_height_;
  OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<EventHistogramPublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
