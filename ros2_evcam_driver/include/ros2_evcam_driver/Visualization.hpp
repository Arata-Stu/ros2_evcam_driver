#ifndef VISUALIZATION_HPP
#define VISUALIZATION_HPP

#include <vector>
#include <cstdint>
#include <opencv2/opencv.hpp>

// bins対応済み visualizeHistogram（uint8_tベース）
cv::Mat visualizeHistogram(const std::vector<uint8_t>& hist, int bins, int width, int height, bool use_ratio = true);

#endif // VISUALIZATION_HPP
