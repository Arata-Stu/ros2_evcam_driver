#include "rclcpp/rclcpp.hpp"
#include "evcam_msgs/msg/event_histogram.hpp"
#include "event_histogram_accumulator/histogram_buffer.hpp"

class HistogramAccumulatorNode : public rclcpp::Node {
public:
  HistogramAccumulatorNode()
  : Node("histogram_accumulator_node"),
    hist_buffer_(declare_parameter<int>("buffer_size", 100))
  {
    output_bins_ = this->declare_parameter<int>("output_bins", 10);

    subscription_ = this->create_subscription<evcam_msgs::msg::EventHistogram>(
      "event_histogram", 10,
      std::bind(&HistogramAccumulatorNode::histogramCallback, this, std::placeholders::_1)
    );

    stacked_publisher_ = this->create_publisher<evcam_msgs::msg::EventHistogram>(
      "stacked_event_histogram", 10
    );

    RCLCPP_INFO(this->get_logger(), "HistogramAccumulatorNode started with buffer_size=%d, output_bins=%d",
                hist_buffer_.size(), output_bins_);
  }

private:
  void histogramCallback(const evcam_msgs::msg::EventHistogram::SharedPtr msg) {
    hist_buffer_.addHistogram(*msg);

    if (hist_buffer_.isReady(output_bins_)) {
      auto stacked = hist_buffer_.generateStackedHistogram(output_bins_);
      stacked_publisher_->publish(stacked);
      // RCLCPP_INFO(this->get_logger(), "Published stacked histogram (bins=%d, size_on=%zu, size_off=%zu)",
      //             output_bins_, stacked.histogram_on.size(), stacked.histogram_off.size());
    } else {
      // RCLCPP_DEBUG(this->get_logger(), "Waiting for enough histograms: %ld / %d",
      //              hist_buffer_.size(), output_bins_);
    }
  }

  rclcpp::Subscription<evcam_msgs::msg::EventHistogram>::SharedPtr subscription_;
  rclcpp::Publisher<evcam_msgs::msg::EventHistogram>::SharedPtr stacked_publisher_;
  HistogramBuffer hist_buffer_;
  int output_bins_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HistogramAccumulatorNode>());
  rclcpp::shutdown();
  return 0;
}
