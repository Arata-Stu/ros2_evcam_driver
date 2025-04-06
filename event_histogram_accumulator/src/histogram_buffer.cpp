#include "event_histogram_accumulator/histogram_buffer.hpp"

HistogramBuffer::HistogramBuffer(std::size_t max_size)
: max_size_(max_size)
{}

void HistogramBuffer::addHistogram(const evcam_msgs::msg::EventHistogram &msg) {
  if (buffer_.size() >= max_size_) {
    buffer_.pop_front();  // 古いものを削除
  }
  buffer_.push_back(msg);  // 新しいものを追加
}

bool HistogramBuffer::isReady(std::size_t required_count) const {
  return buffer_.size() >= required_count;
}

evcam_msgs::msg::EventHistogram HistogramBuffer::generateStackedHistogram(std::size_t num_bins) const {
  evcam_msgs::msg::EventHistogram result;

  if (buffer_.size() < num_bins) {
    return result;  // 空のメッセージを返す
  }

  const auto &first = buffer_.front();
  result.width = first.width;
  result.height = first.height;
  result.bins = static_cast<uint32_t>(num_bins);

  std::size_t single_hist_size = first.histogram.size();
  result.histogram.reserve(single_hist_size * num_bins);

  for (std::size_t i = buffer_.size() - num_bins; i < buffer_.size(); ++i) {
    const auto &h = buffer_[i];
    result.histogram.insert(result.histogram.end(), h.histogram.begin(), h.histogram.end());
  }

  return result;
}
