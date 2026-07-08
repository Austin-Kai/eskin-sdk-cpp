#include "tactile_sensor/tactile_sensor.hpp"
#include "tactile_sensor/protocol.hpp"

#include <chrono>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <iostream>

namespace tactile {

using clock = std::chrono::steady_clock;

TactileSensor::TactileSensor(const Config& cfg) : cfg_(cfg) {
    expected_points_ = static_cast<size_t>(cfg_.rows) * cfg_.cols;
    expected_bytes_  = expected_points_ * 2;
}

TactileSensor::~TactileSensor() { stop(); }

bool TactileSensor::start() {
    if (running_.load()) return true;
    if (!serial_.open(cfg_.port, cfg_.baudrate)) {
        std::cerr << "[TactileSensor] 无法打开串口: " << cfg_.port << std::endl;
        return false;
    }
    serial_.flushInput();
    running_.store(true);
    thread_ = std::thread(&TactileSensor::pollLoop, this);
    std::cout << "[TactileSensor] 已启动, 端口=" << cfg_.port
              << " 波特率=" << cfg_.baudrate
              << " 频率=" << cfg_.poll_rate << "Hz" << std::endl;
    return true;
}

void TactileSensor::stop() {
    if (running_.exchange(false)) {
        if (thread_.joinable()) thread_.join();
    }
    serial_.close();
}

void TactileSensor::setDataCallback(DataCallback cb) {
    std::lock_guard<std::mutex> lk(cb_mtx_);
    cb_ = std::move(cb);
}

void TactileSensor::recalibrate() {
    calibrated_.store(false);
    calib_buffer_.clear();
}

bool TactileSensor::getLatest(TactileFrame& out) {
    std::lock_guard<std::mutex> lk(latest_mtx_);
    if (latest_.raw_data.empty()) return false;
    out = latest_;
    return true;
}

void TactileSensor::pollLoop() {
    const auto req = protocol::buildRequestFrame(
        cfg_.dev_addr, cfg_.start_addr, static_cast<uint16_t>(expected_bytes_));
    protocol::FrameParser parser(expected_bytes_);

    const auto period = std::chrono::duration_cast<clock::duration>(
        std::chrono::duration<double>(1.0 / cfg_.poll_rate));

    std::vector<uint8_t> rbuf(1024);
    std::vector<uint8_t> payload;

    while (running_.load()) {
        const auto t0 = clock::now();

        serial_.flushInput();
        parser.reset();
        serial_.write(req);

        // 在一个短时间窗内读取并尝试解析出一帧
        const auto deadline = clock::now() + std::chrono::milliseconds(50);
        while (running_.load() && clock::now() < deadline) {
            int n = serial_.read(rbuf.data(), rbuf.size(), 10);
            if (n > 0) {
                parser.feed(rbuf.data(), static_cast<size_t>(n));
                if (parser.nextFrame(payload)) {
                    processPayload(payload);
                    break;
                }
            }
        }

        const auto elapsed = clock::now() - t0;
        if (elapsed < period) std::this_thread::sleep_for(period - elapsed);
    }
}

static void quadrantStats(const std::vector<int>& data, int rows, int cols,
                          int r0, int r1, int c0, int c1,
                          float& mean, float& mx, float& sum) {
    double s = 0; int m = 0; int cnt = 0;
    for (int r = r0; r < r1; ++r)
        for (int c = c0; c < c1; ++c) {
            int v = data[r * cols + c];
            s += v; if (v > m) m = v; ++cnt;
        }
    sum  = static_cast<float>(s);
    mx   = static_cast<float>(m);
    mean = cnt ? static_cast<float>(s / cnt) : 0.f;
}

void TactileSensor::processPayload(const std::vector<uint8_t>& payload) {
    const size_t n = expected_points_;
    std::vector<int> values(n);
    for (size_t i = 0; i < n; ++i)
        values[i] = payload[2 * i] | (payload[2 * i + 1] << 8);

    // —— 上电校准: 前 buffer_len 帧求均值作为底噪 ——
    if (!calibrated_.load()) {
        calib_buffer_.push_back(values);
        if (static_cast<int>(calib_buffer_.size()) >= cfg_.buffer_len) {
            baseline_.assign(n, 0.0);
            for (auto& v : calib_buffer_)
                for (size_t i = 0; i < n; ++i) baseline_[i] += v[i];
            for (size_t i = 0; i < n; ++i) baseline_[i] /= calib_buffer_.size();
            calibrated_.store(true);
            calib_buffer_.clear();
            std::cout << "[TactileSensor] 触觉传感器校准完成" << std::endl;
        }
        return;
    }

    // —— 去基线 + 阈值 ——
    std::vector<int> fine(n);
    for (size_t i = 0; i < n; ++i) {
        int d = static_cast<int>(std::lround(std::fabs(values[i] - baseline_[i])));
        fine[i] = (d < cfg_.threshold) ? 0 : d;
    }

    // —— 统计量 ——
    std::vector<float> dev;
    if (cfg_.single_sensor) {
        double s = std::accumulate(fine.begin(), fine.end(), 0.0);
        int mx = *std::max_element(fine.begin(), fine.end());
        dev = { static_cast<float>(s / n), static_cast<float>(mx), static_cast<float>(s) };
    } else {
        float m1, x1, s1, m2, x2, s2;
        quadrantStats(fine, cfg_.rows, cfg_.cols, 0, cfg_.rows/2, 0, cfg_.cols/2, m1, x1, s1);
        quadrantStats(fine, cfg_.rows, cfg_.cols, cfg_.rows/2, cfg_.rows, cfg_.cols/2, cfg_.cols, m2, x2, s2);
        dev = { m1, x1, s1, m2, x2, s2 };
    }

    TactileFrame frame;
    frame.raw_data  = std::move(values);
    frame.fine_data = std::move(fine);
    frame.dev_data  = std::move(dev);

    { std::lock_guard<std::mutex> lk(latest_mtx_); latest_ = frame; }
    { std::lock_guard<std::mutex> lk(cb_mtx_); if (cb_) cb_(frame); }
}

} // namespace tactile