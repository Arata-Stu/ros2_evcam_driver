#include "Buffer.hpp"
#include <metavision/sdk/base/utils/log.h>

void EventBuffer::addEvents(const Metavision::EventCD *ev_begin, const Metavision::EventCD *ev_end) {
    std::lock_guard<std::mutex> lock(mutex_);
    buffer_.insert(buffer_.end(), ev_begin, ev_end);
}

std::vector<Metavision::EventCD> EventBuffer::retrieveAndClear() {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<Metavision::EventCD> events(buffer_.begin(), buffer_.end());
    buffer_.clear();
    return events;
}

std::vector<Metavision::EventCD> EventBuffer::getRecentEvents(int64_t current_time_us, int64_t window_duration_us) {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<Metavision::EventCD> result;

    int64_t window_start = current_time_us - window_duration_us;

    // 古いイベントを破棄
    while (!buffer_.empty() && buffer_.front().t < window_start) {
        buffer_.pop_front();
    }

    result.assign(buffer_.begin(), buffer_.end());
    return result;
}
