#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "evcam_msgs/msg/event_histogram.hpp"

#include <opencv2/opencv.hpp>
#include "Visualization.hpp"

class EventHistogramVisualizerNode : public rclcpp::Node {
public:
  EventHistogramVisualizerNode()
  : Node("event_histogram_visualizer_node")
  {
    subscription_ = this->create_subscription<evcam_msgs::msg::EventHistogram>(
      "event_histogram", 10,
      std::bind(&EventHistogramVisualizerNode::histogramCallback, this, std::placeholders::_1)
    );

    image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image_topic", 10);
  }

private:
  void histogramCallback(const evcam_msgs::msg::EventHistogram::SharedPtr msg) {
    int width = msg->width;
    int height = msg->height;
    int bins = msg->bins;

    std::size_t single_size = width * height * bins;

    // ON/OFF を直接マージ（uint8_t のまま）
    std::vector<uint8_t> merged_histogram;
    merged_histogram.reserve(single_size * 2);
    merged_histogram.insert(merged_histogram.end(), msg->histogram_on.begin(), msg->histogram_on.end());
    merged_histogram.insert(merged_histogram.end(), msg->histogram_off.begin(), msg->histogram_off.end());

    // visualize
    cv::Mat image = visualizeHistogram(merged_histogram, bins, width, height);

    std_msgs::msg::Header header;
    header.stamp = msg->header.stamp;
    header.frame_id = "camera_frame";

    cv_bridge::CvImage cv_image(header, "mono8", image);
    image_publisher_->publish(*cv_image.toImageMsg());
  }

  rclcpp::Subscription<evcam_msgs::msg::EventHistogram>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<EventHistogramVisualizerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
