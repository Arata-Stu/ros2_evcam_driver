#ifndef BUFFER_HPP
#define BUFFER_HPP

#include <vector>
#include <mutex>
#include <metavision/sdk/driver/camera.h>

/**
 * @brief EventBuffer クラスは、Metavision SDK から受信したイベントを一時的に蓄積するためのバッファです。
 */
class EventBuffer {
public:
    /**
     * @brief イベントをバッファに追加します。
     * @param ev_begin イベントの開始ポインタ
     * @param ev_end   イベントの終了ポインタ
     */
    void addEvents(const Metavision::EventCD *ev_begin, const Metavision::EventCD *ev_end);

    /**
     * @brief 蓄積されたイベントを取得し、内部バッファをクリアします。
     * @return 蓄積されたイベントの std::vector
     */
    std::vector<Metavision::EventCD> retrieveAndClear();

private:
    std::vector<Metavision::EventCD> buffer_;
    std::mutex mutex_;
};

#endif // BUFFER_HPP
