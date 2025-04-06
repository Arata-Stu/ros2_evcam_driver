#ifndef BUFFER_HPP
#define BUFFER_HPP

#include <deque>
#include <vector>
#include <mutex>
#include <metavision/sdk/driver/camera.h>

/**
 * @brief イベントを蓄積し、任意の時間範囲で取り出せるバッファ
 */
class EventBuffer {
public:
    void addEvents(const Metavision::EventCD *ev_begin, const Metavision::EventCD *ev_end);

    /// @brief すべてのイベントを取得してクリア（互換目的）
    std::vector<Metavision::EventCD> retrieveAndClear();

    /// @brief 現在時刻から過去window_durationマイクロ秒以内のイベントを返す
    std::vector<Metavision::EventCD> getRecentEvents(int64_t current_time_us, int64_t window_duration_us);

private:
    std::deque<Metavision::EventCD> buffer_;
    std::mutex mutex_;
};

#endif // BUFFER_HPP
