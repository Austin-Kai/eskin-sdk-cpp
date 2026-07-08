#include "tactile_sensor/serial_port.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
#include <cstring>

namespace tactile {

static bool speedFromBaud(int baud, speed_t& sp) {
    switch (baud) {
        case 9600:   sp = B9600;   return true;
        case 19200:  sp = B19200;  return true;
        case 38400:  sp = B38400;  return true;
        case 57600:  sp = B57600;  return true;
        case 115200: sp = B115200; return true;
        case 230400: sp = B230400; return true;
        case 460800: sp = B460800; return true;
        case 921600: sp = B921600; return true;
        default:     return false;
    }
}

SerialPort::SerialPort() : fd_(-1) {}
SerialPort::~SerialPort() { close(); }

bool SerialPort::open(const std::string& port, int baudrate) {
    close();
    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) return false;

    struct termios tty;
    std::memset(&tty, 0, sizeof(tty));
    if (tcgetattr(fd_, &tty) != 0) { close(); return false; }

    speed_t sp;
    if (!speedFromBaud(baudrate, sp)) { close(); return false; }
    cfsetispeed(&tty, sp);
    cfsetospeed(&tty, sp);

    cfmakeraw(&tty);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~PARENB;   // 无校验
    tty.c_cflag &= ~CSTOPB;   // 1 停止位
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;       // 8 数据位
    tty.c_cflag &= ~CRTSCTS;  // 无硬件流控
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) { close(); return false; }
    tcflush(fd_, TCIOFLUSH);
    return true;
}

void SerialPort::close() {
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
}

int SerialPort::write(const uint8_t* data, size_t len) {
    if (fd_ < 0) return -1;
    size_t sent = 0;
    while (sent < len) {
        ssize_t n = ::write(fd_, data + sent, len - sent);
        if (n < 0) return -1;
        sent += static_cast<size_t>(n);
    }
    return static_cast<int>(sent);
}

int SerialPort::read(uint8_t* buf, size_t maxlen, int timeout_ms) {
    if (fd_ < 0) return -1;
    fd_set set;
    FD_ZERO(&set);
    FD_SET(fd_, &set);
    struct timeval tv;
    tv.tv_sec  = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

    int rv = select(fd_ + 1, &set, nullptr, nullptr, &tv);
    if (rv <= 0) return 0;                 // 超时或错误
    ssize_t n = ::read(fd_, buf, maxlen);
    return (n < 0) ? 0 : static_cast<int>(n);
}

void SerialPort::flushInput() {
    if (fd_ >= 0) tcflush(fd_, TCIFLUSH);
}

} // namespace tactile