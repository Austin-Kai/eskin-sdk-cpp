#pragma once
#include <chrono>
#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <mutex>
#include <functional>
#include <memory>
#include <cstdint>

#include "tactile_sensor/api_export.h"
#include "tactile_sensor/tangential_sensor.hpp"

namespace tactile {

class SerialPort;  // 前向声明, 实现细节不暴露给 API 用户

// 可配置参数
struct TACTILE_API Config {
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

    // —— 切向力感知 ——
    float tang_threshold_factor = 5.0f;  ///< 动态阈值倍数: thresh = factor × mean(前 N 帧总压力)
    int   tang_collect_frames   = 20;    ///< 学习阈值的样本帧数 (≤0 直接设为 0)
    int   tang_stability_frames = 5;     ///< 连续低压帧数触发原点复位
    int   tang_reset_at_frame   = 0;     ///< 第 N 帧自动复位原点 (0=禁用)
    int   tang_refine_cnt       = 10;    ///< CoP 稳定帧数触发二次精修 (0=禁用)
    float tang_refine_distance  = 0.1f;  ///< 判定稳定的欧氏距离阈值 (cells, 0=禁用)
};

// 一帧解析结果
struct TACTILE_API TactileFrame {
    std::vector<int> rawArrayData;              // 原始 uint16
    std::vector<int> fineArrayData;             // 去基线 + 阈值后

    float minValue        = 0.0f;               // 最小值
    float maxValue        = 0.0f;               // 最大值
    float sumValue        = 0.0f;               // 总和
    float meanValue       = 0.0f;               // 均值
    float forceDerivative = 0.0f;               // 力导数 (df/dt)
    float copX            = 0.0f;               // 压力中心 X (列方向, cells)
    float copY            = 0.0f;               // 压力中心 Y (行方向, cells)
    int   sensingArea     = 0;                  // 感应面积 (非零像素数)

    float tangentialForceAngle = 0.0f;          // 切向力角度 (0~360°)
};

using DataCallback = std::function<void(const TactileFrame&)>;

class TACTILE_API TactileSensor {
public:
    explicit TactileSensor(const Config& cfg);
    ~TactileSensor();

    bool start();                 // 打开串口并启动后台轮询线程
    void stop();
    bool isRunning() const { return running_.load(); }
    bool isCalibrated() const { return calibrated_.load(); }

    void setDataCallback(DataCallback cb);    // 每帧有效数据回调
    void recalibrate();                       // 重新进行底噪校准
    void recalibrateTangential();             // 重新锁定切向力原点
    bool getLatest(TactileFrame& out);        // 线程安全获取最近一帧

private:
    void pollLoop();
    void processPayload(const std::vector<uint8_t>& payload);

    Config                        cfg_;
    std::unique_ptr<SerialPort>   serial_;
    TangentialSensor  tangential_;
    std::thread       thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> calibrated_{false};

    size_t expected_points_ = 0;
    size_t expected_bytes_  = 0;

    std::vector<std::vector<int>> calib_buffer_;
    std::vector<double>           baseline_;

    DataCallback  cb_;
    std::mutex    cb_mtx_;

    TactileFrame        latest_;
    std::mutex          latest_mtx_;

    float                                    prevForce_ = 0.0f;
    std::chrono::steady_clock::time_point    lastProcessTime_;
};

} // namespace tactile