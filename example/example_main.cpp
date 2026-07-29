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
    // ============================================================
    //  Config — 全部可配置参数 (以下均为默认值)
    // ============================================================
    tactile::Config cfg;

    // —— 连接 ——
    cfg.port      = "/dev/ttyUSB2";   // 串口设备路径
    cfg.baudrate  = 921600;           // 波特率
    cfg.poll_rate = 200.0;            // 轮询频率 (Hz)

    // —— 传感器几何 ——
    cfg.rows          = 12;           // 行数
    cfg.cols          = 7;            // 列数
    cfg.single_sensor = true;         // true: 单片; false: 双片四象限

    // —— 协议 ——
    cfg.dev_addr   = 0x34;            // 设备地址
    cfg.start_addr = 0x00001C00;      // 读取起始地址

    // —— 校准 ——
    cfg.threshold  = 200;             // 去基线后阈值 (低于此值归零)
    cfg.buffer_len = 100;             // 上电校准帧数

    // —— 切向力感知 ——
    cfg.tang_threshold_factor = 5.0f; // 动态阈值倍数
    cfg.tang_collect_frames   = 20;   // 阈值学习帧数 (≤0=默认阈值0)
    cfg.tang_stability_frames = 5;    // 连续低压帧数→自动复位原点
    cfg.tang_reset_at_frame   = 0;    // 第N帧自动复位 (0=禁用)
    cfg.tang_refine_cnt       = 10;   // 稳定帧数→二次精修原点 (0=禁用)
    cfg.tang_refine_distance  = 0.1f; // 稳定判定距离 (cells, 0=禁用)

    // 命令行覆盖 (可选)
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
        // 特征打印 (固定占位)
        std::cout << std::fixed << std::setprecision(1)
                  << "min="  << std::setw(5) << f.minValue
                  << " max=" << std::setw(5) << f.maxValue
                  << " sum=" << std::setw(8) << f.sumValue
                  << " mean=" << std::setw(7) << f.meanValue
                  << " dfdt=" << std::setw(9) << f.forceDerivative
                  << " copX=" << std::setw(5) << f.copX
                  << " copY=" << std::setw(5) << f.copY
                  << " area=" << std::setw(4) << f.sensingArea
                  << " angle=" << std::setw(6) << f.tangentialForceAngle << std::endl;

        // 完整阵列 (rows × cols)
        std::cout << "阵列 (fineData)[" << rows << "x" << cols << "]:" << std::endl;
        for (int r = 0; r < rows; ++r) {
            for (int c = 0; c < cols; ++c) {
                std::cout << std::setw(5) << f.fineArrayData[r * cols + c] << " ";
            }
            std::cout << std::endl;
        }
        std::cout << std::string(40, '-') << std::endl;
    });

    if (!sensor.start()) return 1;

    std::signal(SIGINT, onSig);
    while (!g_stop.load()) std::this_thread::sleep_for(std::chrono::milliseconds(50));

    sensor.stop();
    std::cout << "已退出" << std::endl;
    return 0;
}