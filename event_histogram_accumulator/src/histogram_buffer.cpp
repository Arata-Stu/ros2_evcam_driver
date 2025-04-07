#include "event_histogram_accumulator/histogram_buffer.hpp"
#include <algorithm>  // for std::copy

HistogramBuffer::HistogramBuffer(std::size_t max_size)
: max_size_(max_size)
{}

void HistogramBuffer::addHistogram(const evcam_msgs::msg::EventHistogram &msg) {
  if (buffer_.size() >= max_size_) {
    buffer_.pop_front();
  }
  buffer_.push_back(msg);
}

bool HistogramBuffer::isReady(std::size_t required_count) const {
  return buffer_.size() >= required_count;
}

evcam_msgs::msg::EventHistogram HistogramBuffer::generateStackedHistogram(std::size_t num_bins) const {
  evcam_msgs::msg::EventHistogram result;

  if (buffer_.size() < num_bins) {
    return result;
  }

  const auto &first = buffer_.front();
  result.width = first.width;
  result.height = first.height;
  result.bins = static_cast<uint8_t>(num_bins);  // ← uint8 に変更済み
  result.header.stamp = buffer_.back().header.stamp;

  std::size_t single_size = first.histogram_on.size();

  result.histogram_on.reserve(single_size * num_bins);
  result.histogram_off.reserve(single_size * num_bins);

  for (std::size_t i = buffer_.size() - num_bins; i < buffer_.size(); ++i) {
    const auto &h = buffer_[i];

    result.histogram_on.insert(result.histogram_on.end(), h.histogram_on.begin(), h.histogram_on.end());
    result.histogram_off.insert(result.histogram_off.end(), h.histogram_off.begin(), h.histogram_off.end());
  }

  return result;
}
