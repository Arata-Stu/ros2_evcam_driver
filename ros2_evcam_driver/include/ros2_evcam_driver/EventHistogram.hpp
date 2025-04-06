#ifndef EVENTHISTOGRAM_HPP
#define EVENTHISTOGRAM_HPP

#include <vector>
#include <cstdint>
#include <iostream>
#include <metavision/sdk/driver/camera.h>

class EventHistogram {
public:
    EventHistogram(int bins, int width, int height, uint8_t count_cutoff = 255, bool downsample = false);

    void construct(const Metavision::EventCD* ev_begin, const Metavision::EventCD* ev_end);
    std::vector<uint8_t> getHistogram() const;
    void getHistogramSeparated(std::vector<uint8_t> &on, std::vector<uint8_t> &off) const;

    void printDimensions() const;

private:
    int width_;
    int height_;
    int bins_;
    uint8_t count_cutoff_;
    bool downsample_;
    std::vector<uint8_t> histogram_;  // 軽量化後: 2 * bins * W * H
};

#endif // EVENTHISTOGRAM_HPP
