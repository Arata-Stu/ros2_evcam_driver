#pragma once

#include <deque>
#include <vector>
#include <cstddef>

#include "evcam_msgs/msg/event_histogram.hpp"

class HistogramBuffer {
public:
  explicit HistogramBuffer(std::size_t max_size);

  void addHistogram(const evcam_msgs::msg::EventHistogram &msg);
  bool isReady(std::size_t required_count) const;

  evcam_msgs::msg::EventHistogram generateStackedHistogram(std::size_t num_bins) const;

  std::size_t size() const { return buffer_.size(); }

private:
  std::deque<evcam_msgs::msg::EventHistogram> buffer_;
  std::size_t max_size_;
};
