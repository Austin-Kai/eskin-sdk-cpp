/**
 * record_press — 触觉传感器按压数据录制工具
 *
 * 用法:
 *   ./record_press [--port /dev/ttyUSB0] [--baud 921600] [--out ./data]
 *
 * 操作:
 *   [空格] 开始/停止录制  (每次停止自动保存 JSON)
 *   [q]    退出
 *   [Ctrl+C] 退出
 *
 * 输出: 每次按压保存为一个独立 JSON, 文件名含时间戳, 不会覆盖
 */

#include "tactile_sensor/tactile_sensor.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <ctime>
#include <deque>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <string>
#include <sys/select.h>
#include <termios.h>
#include <thread>
#include <unistd.h>

// ============================================================
// 终端非阻塞按键
// ============================================================
static struct termios g_old_tio;

static void termRaw() {
    tcgetattr(STDIN_FILENO, &g_old_tio);
    struct termios raw = g_old_tio;
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VMIN]  = 0;
    raw.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
}

static void termRestore() {
    tcsetattr(STDIN_FILENO, TCSANOW, &g_old_tio);
}

static bool kbhit() {
    fd_set set;
    FD_ZERO(&set);
    FD_SET(STDIN_FILENO, &set);
    struct timeval tv = {0, 0};
    return select(STDIN_FILENO + 1, &set, nullptr, nullptr, &tv) > 0;
}

static char getch() {
    char c = 0;
    read(STDIN_FILENO, &c, 1);
    return c;
}

// ============================================================
// 帧缓冲区
// ============================================================
struct FrameRecord {
    int64_t time_us;                     // 相对录制起始的微秒偏移
    std::vector<int> raw_data;
    std::vector<int> fine_data;
};

// ============================================================
// JSON 写入 (无外部依赖)
// ============================================================
static void writeJson(const std::string& path,
                      int rows, int cols,
                      const std::vector<FrameRecord>& frames) {
    std::ofstream of(path);
    of << "{\n";
    of << "  \"rows\": " << rows << ",\n";
    of << "  \"cols\": " << cols << ",\n";
    of << "  \"frame_count\": " << frames.size() << ",\n";
    of << "  \"frames\": [\n";

    for (size_t fi = 0; fi < frames.size(); ++fi) {
        const auto& fr = frames[fi];
        of << "    {\n";
        of << "      \"time_us\": " << fr.time_us << ",\n";

        of << "      \"raw_data\": [";
        for (size_t i = 0; i < fr.raw_data.size(); ++i) {
            if (i) of << ", ";
            of << fr.raw_data[i];
        }
        of << "],\n";

        of << "      \"fine_data\": [";
        for (size_t i = 0; i < fr.fine_data.size(); ++i) {
            if (i) of << ", ";
            of << fr.fine_data[i];
        }
        of << "]\n";

        of << "    }";
        if (fi + 1 < frames.size()) of << ",";
        of << "\n";
    }

    of << "  ]\n";
    of << "}\n";
    of.close();
}

// ============================================================
// 文件名生成
// ============================================================
static std::string makeSnapFilename(const std::string& dir, int snapIdx) {
    auto now = std::chrono::system_clock::now();
    auto t   = std::chrono::system_clock::to_time_t(now);
    auto ms  = std::chrono::duration_cast<std::chrono::milliseconds>(
                   now.time_since_epoch()) % 1000;

    std::ostringstream ss;
    ss << dir << "/snap_" << std::setfill('0') << std::setw(3) << snapIdx
       << "_" << std::put_time(std::localtime(&t), "%Y%m%d_%H%M%S")
       << "_" << std::setfill('0') << std::setw(3) << ms.count()
       << ".json";
    return ss.str();
}

static std::string makeFilename(const std::string& dir, int pressIdx) {
    auto now = std::chrono::system_clock::now();
    auto t   = std::chrono::system_clock::to_time_t(now);
    auto ms  = std::chrono::duration_cast<std::chrono::milliseconds>(
                   now.time_since_epoch()) % 1000;

    std::ostringstream ss;
    ss << dir << "/press_" << std::setfill('0') << std::setw(3) << pressIdx
       << "_" << std::put_time(std::localtime(&t), "%Y%m%d_%H%M%S")
       << "_" << std::setfill('0') << std::setw(3) << ms.count()
       << ".json";
    return ss.str();
}

// ============================================================
// main
// ============================================================
static std::atomic<bool> g_running{true};

static void onSig(int) { g_running.store(false); }

int main(int argc, char** argv) {
    termRaw();
    std::atexit(termRestore);
    std::signal(SIGINT, onSig);

    // —— 参数 ——
    std::string outDir = "./record_data";

    tactile::Config cfg;
    cfg.port          = "/dev/ttyUSB2";
    cfg.baudrate      = 921600;
    cfg.poll_rate     = 200.0;
    cfg.rows          = 12;
    cfg.cols          = 7;
    cfg.single_sensor = true;
    cfg.dev_addr      = 0x34;
    cfg.start_addr    = 0x00001C00;
    cfg.threshold     = 200;
    cfg.buffer_len    = 100;
    cfg.tang_threshold_factor = 5.0f;
    cfg.tang_collect_frames   = 20;
    cfg.tang_stability_frames = 5;

    // 命令行覆盖
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if      (a == "--port" && i + 1 < argc) cfg.port      = argv[++i];
        else if (a == "--baud" && i + 1 < argc) cfg.baudrate  = std::atoi(argv[++i]);
        else if (a == "--rate" && i + 1 < argc) cfg.poll_rate = std::atof(argv[++i]);
        else if (a == "--out"  && i + 1 < argc) outDir        = argv[++i];
    }

    // 创建输出目录
    std::system(("mkdir -p " + outDir).c_str());

    // —— 传感器 ——
    tactile::TactileSensor sensor(cfg);

    // 录制状态
    bool recording = false;
    std::vector<FrameRecord> buffer;
    std::chrono::steady_clock::time_point recStart;
    int pressCount   = 0;   // 持续录制序号
    int snapCount    = 0;   // 单帧快照序号
    std::mutex latestMtx;
    tactile::TactileFrame latestFrame;

    const int rows = cfg.rows;
    const int cols = cfg.cols;

    sensor.setDataCallback([&](const tactile::TactileFrame& f) {
        // 缓存最新帧供单帧快照使用
        {
            std::lock_guard<std::mutex> lk(latestMtx);
            latestFrame = f;
        }

        // 特征打印 (固定占位)
        std::cout << (recording ? "[●]" : "[ ]")
                  << (snapCount > 0 ? " snap:" + std::to_string(snapCount) : "")
                  << " "
                  << std::fixed << std::setprecision(1)
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
            for (int c = 0; c < cols; ++c)
                std::cout << std::setw(5) << f.fineArrayData[r * cols + c] << " ";
            std::cout << std::endl;
        }
        std::cout << std::string(40, '-') << std::endl;

        if (!recording) return;

        auto now = std::chrono::steady_clock::now();
        int64_t us = std::chrono::duration_cast<std::chrono::microseconds>(
                         now - recStart).count();

        FrameRecord fr;
        fr.time_us   = us;
        fr.raw_data  = f.rawArrayData;
        fr.fine_data = f.fineArrayData;
        buffer.push_back(std::move(fr));
    });

    if (!sensor.start()) {
        std::cerr << "无法启动传感器" << std::endl;
        return 1;
    }

    std::cout << "\n===================================\n"
              << "  触觉传感器按压录制工具\n"
              << "  [空格] 持续录制 开始/停止\n"
              << "  [s]    单帧快照\n"
              << "  [q]    退出\n"
              << "  输出目录: " << outDir << "\n"
              << "===================================\n\n";

    while (g_running.load()) {
        if (kbhit()) {
            char c = getch();
            if (c == ' ') {
                if (!recording) {
                    recording = true;
                    buffer.clear();
                    recStart = std::chrono::steady_clock::now();
                    ++pressCount;
                    std::cout << "[录制] 第 " << pressCount
                              << " 次按压开始..." << std::endl;
                } else {
                    recording = false;
                    if (!buffer.empty()) {
                        std::string path = makeFilename(outDir, pressCount);
                        writeJson(path, cfg.rows, cfg.cols, buffer);
                        std::cout << "[保存] " << path
                                  << "  (" << buffer.size() << " 帧)" << std::endl;
                    } else {
                        std::cout << "[跳过] 无数据" << std::endl;
                    }
                }
            } else if (c == 's') {
                // 单帧快照
                tactile::TactileFrame snap;
                {
                    std::lock_guard<std::mutex> lk(latestMtx);
                    snap = latestFrame;
                }
                if (!snap.rawArrayData.empty()) {
                    ++snapCount;
                    std::vector<FrameRecord> oneFrame;
                    FrameRecord fr;
                    fr.time_us   = 0;
                    fr.raw_data  = snap.rawArrayData;
                    fr.fine_data = snap.fineArrayData;
                    oneFrame.push_back(std::move(fr));
                    std::string path = makeSnapFilename(outDir, snapCount);
                    writeJson(path, cfg.rows, cfg.cols, oneFrame);
                    std::cout << "[快照] " << path << std::endl;
                }
            } else if (c == 'q') {
                g_running.store(false);
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // 退出前保存未停止的录制
    if (recording && !buffer.empty()) {
        std::string path = makeFilename(outDir, pressCount);
        writeJson(path, cfg.rows, cfg.cols, buffer);
        std::cout << "[保存] " << path
                  << " (" << buffer.size() << " 帧)" << std::endl;
    }

    sensor.stop();
    termRestore();
    std::cout << "已退出" << std::endl;
    return 0;
}
