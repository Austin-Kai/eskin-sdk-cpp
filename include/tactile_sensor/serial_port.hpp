#pragma once
#include <string>
#include <vector>
#include <cstdint>
#include <cstddef>

namespace tactile {

// 简单的 Linux 串口封装 (termios, raw 模式)
class SerialPort {
public:
    SerialPort();
    ~SerialPort();

    bool open(const std::string& port, int baudrate);
    void close();
    bool isOpen() const { return fd_ >= 0; }

    int write(const uint8_t* data, size_t len);
    int write(const std::vector<uint8_t>& data) { return write(data.data(), data.size()); }

    // 读取最多 maxlen 字节, 最多阻塞 timeout_ms; 返回读到的字节数(0=超时)
    int read(uint8_t* buf, size_t maxlen, int timeout_ms);

    void flushInput();

private:
    int fd_;
};

} // namespace tactile