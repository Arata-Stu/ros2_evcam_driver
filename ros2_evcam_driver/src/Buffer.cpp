#include "Buffer.hpp"
#include <metavision/sdk/base/utils/log.h>

void EventBuffer::addEvents(const Metavision::EventCD *ev_begin, const Metavision::EventCD *ev_end) {
    std::lock_guard<std::mutex> lock(mutex_);
    size_t count = ev_end - ev_begin;
    buffer_.insert(buffer_.end(), ev_begin, ev_end);
    MV_LOG_INFO() << "Added " << count << " events. Total events in buffer: " << buffer_.size();
}

std::vector<Metavision::EventCD> EventBuffer::retrieveAndClear() {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<Metavision::EventCD> events;
    events.swap(buffer_);
    return events;
}
