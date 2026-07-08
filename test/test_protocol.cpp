#include "tactile_sensor/protocol.hpp"
#include <vector>
#include <cstdio>
#include <cstdint>

static int g_fail = 0;
#define CHECK(cond) do { \
    if (!(cond)) { std::printf("  [FAIL] %s (line %d)\n", #cond, __LINE__); ++g_fail; } \
    else         { std::printf("  [ OK ] %s\n", #cond); } } while(0)

using namespace tactile::protocol;

// 构造一个模拟响应帧: AA55 + len + (10字节头 + 传感器数据) + CRC
static std::vector<uint8_t> makeResponse(const std::vector<uint16_t>& pts) {
    std::vector<uint8_t> body(10, 0x00);            // 10 字节协议头(内容任意)
    for (uint16_t v : pts) { body.push_back(v & 0xFF); body.push_back((v >> 8) & 0xFF); }

    std::vector<uint8_t> f = {0xAA, 0x55};
    uint16_t len = (uint16_t)body.size();
    f.push_back(len & 0xFF); f.push_back((len >> 8) & 0xFF);
    f.insert(f.end(), body.begin(), body.end());
    f.push_back(crc8_itu(f.data(), f.size()));      // CRC 覆盖 头+长度+载荷
    return f;
}

int main() {
    std::printf("== CRC8-ITU ==\n");
    { uint8_t d[] = {0x01, 0x02, 0x03}; CHECK(crc8_itu(d, 3) == crc8_itu(d, 3)); }
    // 已知性质: 空数据 => 0x00 ^ 0x55 = 0x55
    CHECK(crc8_itu(nullptr, 0) == 0x55);

    std::printf("== 请求帧 ==\n");
    {
        auto req = buildRequestFrame(0x34, 0x00001C00, 160);
        CHECK(req.size() == 14);
        CHECK(req[0] == 0x55 && req[1] == 0xAA);
        CHECK(req[6] == 0xFB);                       // 功能码
        CHECK(req.back() == crc8_itu(req.data(), req.size() - 1));
    }

    std::printf("== 解析: 正常帧 ==\n");
    {
        const size_t pts = 24;                       // 6x4
        std::vector<uint16_t> data(pts);
        for (size_t i = 0; i < pts; ++i) data[i] = (uint16_t)(1000 + i);
        auto resp = makeResponse(data);

        FrameParser parser(pts * 2);
        parser.feed(resp.data(), resp.size());
        std::vector<uint8_t> out;
        CHECK(parser.nextFrame(out));
        CHECK(out.size() == pts * 2);
        CHECK((out[0] | (out[1] << 8)) == 1000);
    }

    std::printf("== 解析: 带垃圾前缀 + 分片喂入 ==\n");
    {
        const size_t pts = 24;
        std::vector<uint16_t> data(pts, 7);
        auto resp = makeResponse(data);
        std::vector<uint8_t> stream = {0x11, 0x22, 0xAA};  // 干扰字节
        stream.insert(stream.end(), resp.begin(), resp.end());

        FrameParser parser(pts * 2);
        std::vector<uint8_t> out;
        bool got = false;
        for (size_t i = 0; i < stream.size(); ++i) {       // 逐字节喂入
            parser.feed(&stream[i], 1);
            if (parser.nextFrame(out)) { got = true; break; }
        }
        CHECK(got);
        CHECK(out.size() == pts * 2);
    }

    std::printf("== 解析: CRC 错误应被拒绝 ==\n");
    {
        const size_t pts = 24;
        std::vector<uint16_t> data(pts, 3);
        auto resp = makeResponse(data);
        resp.back() ^= 0xFF;                          // 破坏 CRC
        FrameParser parser(pts * 2);
        parser.feed(resp.data(), resp.size());
        std::vector<uint8_t> out;
        CHECK(!parser.nextFrame(out));
    }

    std::printf(g_fail == 0 ? "\n全部通过 ✅\n" : "\n存在失败 ❌\n");
    return g_fail == 0 ? 0 : 1;
}