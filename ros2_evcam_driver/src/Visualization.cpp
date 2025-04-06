#include "Visualization.hpp"
#include <iostream>

cv::Mat visualizeHistogram(const std::vector<uint64_t>& hist, int bins, int width, int height, bool use_ratio) {
    cv::Mat img(height, width, CV_8UC1, cv::Scalar(127));  // 初期値：中間グレー

    if (hist.size() != static_cast<size_t>(2 * bins * width * height)) {
        std::cerr << "[visualizeHistogram] Invalid histogram size. Got "
                  << hist.size() << ", expected " << 2 * bins * width * height << std::endl;
        return img;
    }

    cv::Mat pos_sum(height, width, CV_32SC1, cv::Scalar(0));
    cv::Mat neg_sum(height, width, CV_32SC1, cv::Scalar(0));

    for (int b = 0; b < bins; ++b) {
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                size_t idx_on  = 0 * (bins * width * height) + b * width * height + y * width + x;
                size_t idx_off = 1 * (bins * width * height) + b * width * height + y * width + x;
                pos_sum.at<int>(y, x) += static_cast<int>(hist[idx_on]);
                neg_sum.at<int>(y, x) += static_cast<int>(hist[idx_off]);
            }
        }
    }

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int on = pos_sum.at<int>(y, x);
            int off = neg_sum.at<int>(y, x);
            unsigned char pixel_val = 127;

            if (use_ratio) {
                int total = on + off;
                if (total > 0) {
                    double ratio = static_cast<double>(on) / total;
                    pixel_val = static_cast<unsigned char>(ratio * 255.0);
                }
            } else {
                if (on > off)
                    pixel_val = 255;
                else if (on < off)
                    pixel_val = 0;
            }

            img.at<uchar>(y, x) = pixel_val;
        }
    }

    return img;
}
