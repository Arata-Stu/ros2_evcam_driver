#ifndef VISUALIZATION_HPP
#define VISUALIZATION_HPP

#include <vector>
#include <cstdint>
#include <opencv2/opencv.hpp>

// bins対応済み visualizeHistogram（統一関数）
cv::Mat visualizeHistogram(const std::vector<uint64_t>& hist, int bins, int width, int height, bool use_ratio = true);

#endif // VISUALIZATION_HPP
