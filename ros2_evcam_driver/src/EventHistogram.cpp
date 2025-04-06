#include "EventHistogram.hpp"
#include <algorithm>
#include <limits>

EventHistogram::EventHistogram(int bins, int width, int height, uint64_t count_cutoff, bool downsample)
    : width_(width), height_(height), bins_(bins), count_cutoff_(count_cutoff), downsample_(downsample) {
    histogram_.resize(2 * bins_ * width_ * height_, 0);
}


void EventHistogram::construct(const Metavision::EventCD* ev_begin, const Metavision::EventCD* ev_end) {
    std::fill(histogram_.begin(), histogram_.end(), 0);

    if (ev_begin == nullptr || ev_end == nullptr || ev_begin >= ev_end) {
        std::cerr << "[Warning] Invalid event range." << std::endl;
        return;
    }

    auto t0 = ev_begin->t;
    auto t1 = (ev_end - 1)->t;

    if (t1 < t0) {
        std::cerr << "[Warning] Non-monotonic timestamps (t1 < t0): " << t1 << " < " << t0 << std::endl;
        return;
    }

    int64_t dt = std::max<int64_t>(1, t1 - t0);  // avoid div by zero

    for (const Metavision::EventCD* ev = ev_begin; ev != ev_end; ++ev) {
        // 座標チェック
        if (ev->x >= static_cast<uint16_t>(width_) || ev->y >= static_cast<uint16_t>(height_)) {
            continue;
        }

        int pol = (ev->p == 1) ? 0 : 1;
        int bin_idx = static_cast<int>(((ev->t - t0) * bins_) / dt);
        if (bin_idx < 0) bin_idx = 0;
        if (bin_idx >= bins_) bin_idx = bins_ - 1;

        size_t idx =
            pol * (bins_ * width_ * height_) +
            bin_idx * (width_ * height_) +
            ev->y * width_ + ev->x;

        if (idx >= histogram_.size()) {
            std::cerr << "[Error] Out-of-bounds histogram index: " << idx << " (size=" << histogram_.size() << ")" << std::endl;
            continue;  // 念のため回避
        }

        histogram_[idx]++;
    }

    // カウント制限
    if (count_cutoff_ > 0) {
        for (auto &count : histogram_) {
            if (count > count_cutoff_) {
                count = count_cutoff_;
            }
        }
    }
}


std::vector<uint64_t> EventHistogram::getHistogram() const {
    if (!downsample_) {
        return histogram_;
    } else {
        int new_width = width_ / 2;
        int new_height = height_ / 2;
        std::vector<uint64_t> down_hist(2 * bins_ * new_width * new_height, 0);

        for (int ch = 0; ch < 2; ++ch) {
            for (int b = 0; b < bins_; ++b) {
                for (int y = 0; y < new_height; ++y) {
                    for (int x = 0; x < new_width; ++x) {
                        int orig_y = y * 2;
                        int orig_x = x * 2;

                        size_t base_idx = ch * bins_ * width_ * height_ + b * width_ * height_;
                        uint64_t sum = 0;
                        sum += histogram_[base_idx + orig_y * width_ + orig_x];
                        sum += histogram_[base_idx + orig_y * width_ + orig_x + 1];
                        sum += histogram_[base_idx + (orig_y + 1) * width_ + orig_x];
                        sum += histogram_[base_idx + (orig_y + 1) * width_ + orig_x + 1];

                        down_hist[ch * bins_ * new_width * new_height + b * new_width * new_height + y * new_width + x] = sum;
                    }
                }
            }
        }
        return down_hist;
    }
}

void EventHistogram::printDimensions() const {
    int w = downsample_ ? width_ / 2 : width_;
    int h = downsample_ ? height_ / 2 : height_;
    std::cout << "Histogram shape: (" << 2 * bins_ << ", " << h << ", " << w << ")" << std::endl;
}
