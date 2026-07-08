#include "tactile_sensor/tactile_sensor.hpp"
#include <iostream>
#include <iomanip>
#include <csignal>
#include <cstdlib>
#include <thread>
#include <atomic>

static std::atomic<bool> g_stop{false};
static void onSig(int) { g_stop.store(true); }

int main(int argc, char** argv) {
    tactile::Config cfg;   // 默认值

    // 命令行覆盖 4 个主要参数
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if      (a == "--port"      && i + 1 < argc) cfg.port      = argv[++i];
        else if (a == "--baud"      && i + 1 < argc) cfg.baudrate  = std::atoi(argv[++i]);
        else if (a == "--rate"      && i + 1 < argc) cfg.poll_rate = std::atof(argv[++i]);
        else if (a == "--threshold" && i + 1 < argc) cfg.threshold = std::atoi(argv[++i]);
    }

    const int rows = cfg.rows;
    const int cols = cfg.cols;

    tactile::TactileSensor sensor(cfg);
    sensor.setDataCallback([rows, cols](const tactile::TactileFrame& f) {
        // 统计量
        std::cout << "mean=" << f.dev_data[0]
                  << " max=" << f.dev_data[1]
                  << " sum=" << f.dev_data[2] << std::endl;

        // 完整阵列 (rows × cols)
        std::cout << "阵列 [" << rows << "x" << cols << "]:" << std::endl;
        for (int r = 0; r < rows; ++r) {
            for (int c = 0; c < cols; ++c) {
                std::cout << std::setw(5) << f.fine_data[r * cols + c] << " ";
            }
            std::cout << std::endl;
        }
        std::cout << std::string(40, '-') << std::endl;
    });

    if (!sensor.start()) return 1;

    std::signal(SIGINT, onSig);
    while (!g_stop.load()) std::this_thread::sleep_for(std::chrono::milliseconds(100));

    sensor.stop();
    std::cout << "已退出" << std::endl;
    return 0;
}