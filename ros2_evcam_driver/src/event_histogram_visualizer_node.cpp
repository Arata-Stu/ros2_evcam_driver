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

    // bins=1 でも stacked 関数を使えばOK
    cv::Mat image = visualizeHistogram(msg->histogram, bins, width, height);

    // 見やすくリサイズ
    cv::Mat image_resized;
    cv::resize(image, image_resized, cv::Size(width * 2, height * 2), 0, 0, cv::INTER_NEAREST);

    // 画像メッセージに変換してpublish
    std_msgs::msg::Header header;
    header.stamp = this->get_clock()->now();
    header.frame_id = "camera_frame";

    cv_bridge::CvImage cv_image(header, "mono8", image_resized);
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
