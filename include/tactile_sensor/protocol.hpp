#include <cstdint>
#include <cstddef>
#include <vector>

namespace tactile {
namespace protocol {

// CRC8-ITU 校验算法 (多项式 0x07, 结果异或 0x55)
inline uint8_t crc8_itu(const uint8_t* data, size_t len) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int b = 0; b < 8; ++b) {
            if (crc & 0x80) {
                crc = static_cast<uint8_t>((crc << 1) ^ 0x07);
            } else {
                crc = static_cast<uint8_t>(crc << 1);
            }
        }
    }
    return static_cast<uint8_t>(crc ^ 0x55);
}

inline uint8_t crc8_itu(const std::vector<uint8_t>& data) {
    return crc8_itu(data.data(), data.size());
}

// 构造请求帧:
//   HEADER(55 AA) + payload_len(=9,LE) + dev_addr + reserved(0) + func(0xFB)
//   + start_addr(LE32) + byte_count(LE16) + CRC8
inline std::vector<uint8_t> buildRequestFrame(uint8_t dev_addr,
                                              uint32_t start_addr,
                                              uint16_t byte_count) {
    std::vector<uint8_t> f;
    f.reserve(14);
    f.push_back(0x55);
    f.push_back(0xAA);

    const uint16_t payload_len = 9;
    f.push_back(static_cast<uint8_t>(payload_len & 0xFF));
    f.push_back(static_cast<uint8_t>((payload_len >> 8) & 0xFF));

    f.push_back(dev_addr);
    f.push_back(0x00);   // reserved
    f.push_back(0xFB);   // func code

    f.push_back(static_cast<uint8_t>(start_addr & 0xFF));
    f.push_back(static_cast<uint8_t>((start_addr >> 8) & 0xFF));
    f.push_back(static_cast<uint8_t>((start_addr >> 16) & 0xFF));
    f.push_back(static_cast<uint8_t>((start_addr >> 24) & 0xFF));

    f.push_back(static_cast<uint8_t>(byte_count & 0xFF));
    f.push_back(static_cast<uint8_t>((byte_count >> 8) & 0xFF));

    f.push_back(crc8_itu(f.data(), f.size()));
    return f;
}

// 流式帧解析器:
//   持续 feed() 收到的字节, 通过 nextFrame() 取出已校验通过的有效传感器数据段。
//   有效数据段 = 响应载荷跳过前 10 字节的协议头, 长度需等于 expected_byte_count。
class FrameParser {
public:
    explicit FrameParser(size_t expected_byte_count)
        : expected_byte_count_(expected_byte_count) {}

    void feed(const uint8_t* data, size_t len) {
        buffer_.insert(buffer_.end(), data, data + len);
        // 防止异常情况下缓冲区无限增长
        if (buffer_.size() > kMaxBuffer) {
            buffer_.erase(buffer_.begin(), buffer_.end() - kMaxBuffer / 2);
        }
    }

    void reset() { buffer_.clear(); }

    // 若成功提取到一个有效帧, 将传感器数据写入 out 并返回 true。
    bool nextFrame(std::vector<uint8_t>& out) {
        while (true) {
            const size_t hpos = findHeader();
            if (hpos == kNoPos) {
                // 未找到帧头, 保留最后一个字节 (可能是被截断的 0xAA)
                if (buffer_.size() > 1) {
                    buffer_.erase(buffer_.begin(), buffer_.end() - 1);
                }
                return false;
            }
            if (hpos > 0) {
                buffer_.erase(buffer_.begin(), buffer_.begin() + hpos);
            }
            // buffer_[0..1] == AA 55
            if (buffer_.size() < 4) return false;  // 等待长度字段

            const uint16_t payload_len =
                static_cast<uint16_t>(buffer_[2] | (buffer_[3] << 8));
            if (payload_len < 10 || payload_len > 512) {
                dropHeader();  // 长度非法, 跳过当前帧头继续搜索
                continue;
            }

            const size_t total = 4 + payload_len + 1;  // 头2 + 长度2 + 载荷 + CRC1
            if (buffer_.size() < total) return false;   // 等待更多数据

            const uint8_t calc = crc8_itu(buffer_.data(), 4 + payload_len);
            const uint8_t recv = buffer_[4 + payload_len];
            if (calc != recv) {
                dropHeader();
                continue;
            }

            const size_t sensor_off = 4 + 10;               // 跳过 10 字节协议头
            const size_t sensor_len = payload_len - 10;
            if (sensor_len != expected_byte_count_) {
                buffer_.erase(buffer_.begin(), buffer_.begin() + total);
                continue;
            }

            out.assign(buffer_.begin() + sensor_off,
                       buffer_.begin() + sensor_off + sensor_len);
            buffer_.erase(buffer_.begin(), buffer_.begin() + total);
            return true;
        }
    }

private:
    static constexpr size_t kNoPos = static_cast<size_t>(-1);
    static constexpr size_t kMaxBuffer = 8192;

    size_t findHeader() const {
        for (size_t i = 0; i + 1 < buffer_.size(); ++i) {
            if (buffer_[i] == 0xAA && buffer_[i + 1] == 0x55) return i;
        }
        return kNoPos;
    }

    void dropHeader() {
        if (buffer_.size() >= 2) {
            buffer_.erase(buffer_.begin(), buffer_.begin() + 2);
        } else {
            buffer_.clear();
        }
    }

    std::vector<uint8_t> buffer_;
    size_t expected_byte_count_;
};

}  // namespace protocol
}  // namespace tactile