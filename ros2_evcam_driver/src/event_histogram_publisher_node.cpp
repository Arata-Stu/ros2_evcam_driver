#include <memory>
#include <chrono>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "evcam_msgs/msg/event_histogram.hpp"  // 別パッケージのカスタムメッセージ

#include <metavision/sdk/driver/camera.h>
#include <metavision/sdk/base/utils/log.h>

#include "Buffer.hpp"
#include "EventHistogram.hpp"

using namespace std::chrono_literals;

class EventHistogramPublisherNode : public rclcpp::Node {
public:
  EventHistogramPublisherNode()
  : Node("event_histogram_publisher_node")
  {
    // パラメータの宣言（初期値を設定）
    this->declare_parameter<int>("accumulation_period_ms", 10);
    this->declare_parameter<int64_t>("count_cutoff", 0LL);
    this->declare_parameter<bool>("downsample", false);
    this->declare_parameter<int>("bins", 1);  


    // パラメータの動的更新用コールバックの登録
    parameter_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&EventHistogramPublisherNode::onParameterChanged, this, std::placeholders::_1)
    );

    // ヒストグラムメッセージのパブリッシャー作成
    publisher_ = this->create_publisher<evcam_msgs::msg::EventHistogram>("event_histogram", 10);

    // カメラの初期化
    try {
      camera_ = std::make_shared<Metavision::Camera>(Metavision::Camera::from_first_available());
    } catch (const Metavision::CameraException &e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open camera: %s", e.what());
      rclcpp::shutdown();
      return;
    }

    // カメラからセンサのジオメトリを取得してサイズを設定
    const auto &geometry = camera_->geometry();
    sensor_width_  = geometry.width();
    sensor_height_ = geometry.height();
    RCLCPP_INFO(this->get_logger(), "Sensor geometry: width=%d, height=%d", sensor_width_, sensor_height_);

    // カメライベントの受信コールバック登録（EventBuffer に追加）
    callback_id_ = camera_->cd().add_callback(
      [this](const Metavision::EventCD *ev_begin, const Metavision::EventCD *ev_end) {
        event_buffer_.addEvents(ev_begin, ev_end);
      }
    );

    // カメラストリーミング開始
    try {
      camera_->start();
    } catch (const Metavision::CameraException &e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to start camera: %s", e.what());
      rclcpp::shutdown();
      return;
    }

    // タイマーを作成（蓄積時間はパラメータから取得）
    int accumulation_period_ms = this->get_parameter("accumulation_period_ms").as_int();
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(accumulation_period_ms),
      std::bind(&EventHistogramPublisherNode::timerCallback, this)
    );
  }

  ~EventHistogramPublisherNode() {
    if (camera_) {
      camera_->stop();
      camera_->cd().remove_callback(callback_id_);
    }
  }

private:
  // パラメータ変更時のコールバック
  rcl_interfaces::msg::SetParametersResult onParameterChanged(
      const std::vector<rclcpp::Parameter> &parameters)
  {
    for (const auto &param : parameters) {
      if (param.get_name() == "accumulation_period_ms") {
        int new_period = param.as_int();
        // タイマーの更新：タイマーを再作成することで変更を反映
        timer_->cancel();
        timer_ = this->create_wall_timer(
          std::chrono::milliseconds(new_period),
          std::bind(&EventHistogramPublisherNode::timerCallback, this)
        );
        RCLCPP_INFO(this->get_logger(), "Updated accumulation_period_ms: %d", new_period);
      } else if (param.get_name() == "count_cutoff") {
        int64_t cutoff = param.as_int();
        RCLCPP_INFO(this->get_logger(), "Updated count_cutoff: %ld", cutoff);
      } else if (param.get_name() == "downsample") {
        RCLCPP_INFO(this->get_logger(), "Updated downsample: %s", param.as_bool() ? "true" : "false");
      } else if (param.get_name() == "bins") {
        int val = param.as_int();
        RCLCPP_INFO(this->get_logger(), "Updated bins: %d", val);
      }
      
    }
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "Parameters updated successfully";
    return result;
  }

  void timerCallback() {
    // パラメータ取得
    int accumulation_period_ms = this->get_parameter("accumulation_period_ms").as_int();
    int64_t cutoff_int = this->get_parameter("count_cutoff").as_int();
    uint64_t count_cutoff = static_cast<uint64_t>(cutoff_int);
    bool downsample = this->get_parameter("downsample").as_bool();
    int bins = this->get_parameter("bins").as_int();
  
    // バッファから蓄積イベント取得
    auto events_chunk = event_buffer_.retrieveAndClear();
    if (events_chunk.empty()) {
      return;
    }
  
    // ヒストグラム構築（bin対応版）
    EventHistogram histogram(bins, sensor_width_, sensor_height_, count_cutoff, downsample);
    histogram.construct(events_chunk.data(), events_chunk.data() + events_chunk.size());
    histogram.printDimensions();  // オプション：ログ出力
  
    // ON/OFF分離ヒストグラム取得
    std::vector<uint64_t> histogram_on_u64, histogram_off_u64;
    histogram.getHistogramSeparated(histogram_on_u64, histogram_off_u64);
  
    // uint64_t → uint32_tに変換
    std::vector<uint32_t> histogram_on(histogram_on_u64.begin(), histogram_on_u64.end());
    std::vector<uint32_t> histogram_off(histogram_off_u64.begin(), histogram_off_u64.end());
  
    // メッセージ作成＆パブリッシュ
    evcam_msgs::msg::EventHistogram msg;
    msg.header.stamp = this->now();  // 時間情報追加
    msg.width = sensor_width_;
    msg.height = sensor_height_;
    msg.bins = static_cast<uint32_t>(bins);
    msg.histogram_on = histogram_on;
    msg.histogram_off = histogram_off;
  
    publisher_->publish(msg);
  }
  
  

  rclcpp::Publisher<evcam_msgs::msg::EventHistogram>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<Metavision::Camera> camera_;
  EventBuffer event_buffer_;
  int callback_id_;
  int sensor_width_;
  int sensor_height_;
  OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<EventHistogramPublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
