#pragma once
#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <mutex>
#include <functional>
#include <cstdint>

#include "tactile_sensor/serial_port.hpp"

namespace tactile {

// 可配置参数
struct Config {
    // —— 主要可配置项 ——
    std::string port      = "/dev/ttyUSB0";
    int         baudrate  = 921600;
    double      poll_rate = 200.0;   // Hz
    int         threshold = 100;

    // —— 传感器/协议参数 ——
    int      rows          = 12;
    int      cols          = 7;
    bool     single_sensor = true;
    uint8_t  dev_addr      = 0x34;
    uint32_t start_addr    = 0x00001C00;
    int      buffer_len    = 100;   // 上电校准帧数
};

// 一帧解析结果
struct TactileFrame {
    std::vector<int>   raw_data;   // 原始 uint16
    std::vector<int>   fine_data;  // 去基线 + 阈值后
    std::vector<float> dev_data;   // 统计量 (mean,max,sum[,...])
};

using DataCallback = std::function<void(const TactileFrame&)>;

class TactileSensor {
public:
    explicit TactileSensor(const Config& cfg);
    ~TactileSensor();

    bool start();                 // 打开串口并启动后台轮询线程
    void stop();
    bool isRunning() const { return running_.load(); }
    bool isCalibrated() const { return calibrated_.load(); }

    void setDataCallback(DataCallback cb);  // 每帧有效数据回调
    void recalibrate();                     // 重新进行底噪校准
    bool getLatest(TactileFrame& out);      // 线程安全获取最近一帧

private:
    void pollLoop();
    void processPayload(const std::vector<uint8_t>& payload);

    Config       cfg_;
    SerialPort   serial_;
    std::thread  thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> calibrated_{false};

    size_t expected_points_ = 0;
    size_t expected_bytes_  = 0;

    std::vector<std::vector<int>> calib_buffer_;
    std::vector<double>           baseline_;

    DataCallback  cb_;
    std::mutex    cb_mtx_;

    TactileFrame  latest_;
    std::mutex    latest_mtx_;
};

} // namespace tactile