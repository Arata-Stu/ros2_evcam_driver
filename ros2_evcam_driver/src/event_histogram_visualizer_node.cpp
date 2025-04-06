#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "evcam_msgs/msg/event_histogram.hpp"  // 別パッケージのカスタムメッセージ

#include <opencv2/opencv.hpp>
#include "Visualization.hpp"

class EventHistogramVisualizerNode : public rclcpp::Node {
public:
  EventHistogramVisualizerNode()
  : Node("event_histogram_visualizer_node")
  {
    // ヒストグラムメッセージのサブスクライバー作成
    subscription_ = this->create_subscription<evcam_msgs::msg::EventHistogram>(
      "event_histogram", 10,
      std::bind(&EventHistogramVisualizerNode::histogramCallback, this, std::placeholders::_1)
    );

    // 画像をパブリッシュするパブリッシャー（Rviz2で可視化するため）
    image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image_topic", 10);
  }

private:
  void histogramCallback(const evcam_msgs::msg::EventHistogram::SharedPtr msg) {
    int width = msg->width;
    int height = msg->height;
    int bins = msg->bins;

    std::size_t single_size = width * height * bins;

    // uint32_t で受け取ってから...
    std::vector<uint32_t> merged_histogram_u32(single_size * 2);
    for (size_t i = 0; i < single_size; ++i) {
      merged_histogram_u32[i] = msg->histogram_on[i];
      merged_histogram_u32[i + single_size] = msg->histogram_off[i];
    }

    // uint64_t に変換
    std::vector<uint64_t> merged_histogram_u64(merged_histogram_u32.begin(), merged_histogram_u32.end());

    // visualize
    cv::Mat image = visualizeHistogram(merged_histogram_u64, bins, width, height);

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
