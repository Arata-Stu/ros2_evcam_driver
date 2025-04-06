#ifndef EVENTHISTOGRAM_HPP
#define EVENTHISTOGRAM_HPP

#include <vector>
#include <cstdint>
#include <iostream>
#include <metavision/sdk/driver/camera.h>

class EventHistogram {
public:
    EventHistogram(int bins, int width, int height, uint64_t count_cutoff = 0, bool downsample = false);

    void construct(const Metavision::EventCD* ev_begin, const Metavision::EventCD* ev_end);
    std::vector<uint64_t> getHistogram() const;

    /// 追加：ON/OFFに分離して取得する
    void getHistogramSeparated(std::vector<uint64_t> &on, std::vector<uint64_t> &off) const;

    void printDimensions() const;

private:
    int width_;
    int height_;
    int bins_;
    uint64_t count_cutoff_;
    bool downsample_;
    std::vector<uint64_t> histogram_; // (2 * bins * H * W)
};

#endif // EVENTHISTOGRAM_HPP
