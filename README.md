# Tactile Sensor SDK

Linux 触觉传感器 C++ SDK，提供串口通信、协议帧解析、上电校准和实时数据获取功能。

## 目录结构

```
tactile_sdk/
├── CMakeLists.txt
├── include/tactile_sensor/
│   ├── protocol.hpp          # CRC8 / 请求帧构建 / 流式帧解析器
│   ├── serial_port.hpp       # Linux 串口封装
│   └── tactile_sensor.hpp    # 传感器高层接口 (Config, TactileFrame, TactileSensor)
├── src/
│   ├── serial_port.cpp
│   └── tactile_sensor.cpp
├── examples/
│   └── example_main.cpp      # 使用示例
└── test/
    └── test_protocol.cpp     # 协议层单元测试
```

## 依赖

- CMake ≥ 3.10
- C++14 编译器
- Linux 系统（依赖 termios / select）

## 构建

```bash
mkdir build && cd build
cmake ..
make -j$(nproc)
```

产物：

- `libtactile_sensor.a` — 静态库
- `tactile_example` — 示例程序
- `test_protocol` — 协议测试

## 快速开始

```bash
# 构建
mkdir build && cd build
cmake .. && make -j$(nproc)

# 运行示例（使用默认参数）
./tactile_example

# 自定义参数
./tactile_example --port /dev/ttyUSB0 --baud 921600 --rate 100 --threshold 80
```

| 参数            | 说明          | 默认值           |
| --------------- | ------------- | ---------------- |
| `--port`      | 串口设备路径  | `/dev/ttyUSB0` |
| `--baud`      | 波特率        | `921600`       |
| `--rate`      | 轮询频率 (Hz) | `200`          |
| `--threshold` | 触发阈值      | `100`          |

输出示例：

```
mean=152.3 max=780 sum=42100.1
阵列 [12x7]:
    0   120     0     0    505     0     0
    0    80   320   150     0     0    95
    ...
----------------------------------------
```

`Ctrl+C` 退出。示例源码见 [examples/example_main.cpp](examples/example_main.cpp)。

## API 参考

### Config 配置结构体

| 字段              | 类型            | 默认值             | 说明                                                |
| ----------------- | --------------- | ------------------ | --------------------------------------------------- |
| `port`          | `std::string` | `"/dev/ttyUSB0"` | 串口设备路径                                        |
| `baudrate`      | `int`         | `921600`         | 波特率                                              |
| `poll_rate`     | `double`      | `200.0`          | 轮询频率 (Hz)                                       |
| `threshold`     | `int`         | `100`            | 去基线后的阈值，低于此值归零                        |
| `rows`          | `int`         | `12`             | 传感器行数                                          |
| `cols`          | `int`         | `7`              | 传感器列数                                          |
| `single_sensor` | `bool`        | `true`           | `true`: 单片传感器；`false`: 双片（四象限统计） |
| `dev_addr`      | `uint8_t`     | `0x34`           | 设备地址                                            |
| `start_addr`    | `uint32_t`    | `0x00001C00`     | 读取起始地址                                        |
| `buffer_len`    | `int`         | `100`            | 上电校准所需帧数                                    |

### TactileFrame 数据结构

| 字段          | 类型                   | 说明                                           |
| ------------- | ---------------------- | ---------------------------------------------- |
| `raw_data`  | `std::vector<int>`   | 原始 uint16 值（未处理）                       |
| `fine_data` | `std::vector<int>`   | 去基线 + 阈值过滤后的值（size = rows × cols） |
| `dev_data`  | `std::vector<float>` | 统计量                                         |

`dev_data` 内容取决于 `single_sensor` 配置：

- **单片模式** (`single_sensor = true`)：`[mean, max, sum]` — 全局均值、最大值、总和
- **双片模式** (`single_sensor = false`)：`[mean1, max1, sum1, mean2, max2, sum2]` — 上下两半区域各自的统计量

### TactileSensor 类

#### `explicit TactileSensor(const Config& cfg)`

构造函数，传入配置。不会打开串口。

#### `bool start()`

打开串口并启动后台轮询线程。返回 `true` 表示启动成功。启动后自动进入上电校准阶段：

1. 采集 `buffer_len` 帧原始数据
2. 计算每个传感点的均值作为底噪基线
3. 校准完成后，后续每帧自动减去基线并应用阈值

可通过 `isCalibrated()` 查询校准是否完成，或调用 `recalibrate()` 重新校准。

#### `void stop()`

停止轮询线程并关闭串口。可安全重复调用。

#### `void setDataCallback(DataCallback cb)`

注册回调函数 `void(const TactileFrame&)`，每收到一帧有效数据时在后台线程中调用。**校准完成前不会触发回调**。

#### `bool getLatest(TactileFrame& out)`

线程安全地获取最新一帧数据的拷贝。返回 `false` 表示尚无数据。

#### `void recalibrate()`

清空当前基线，触发重新校准。

#### `bool isRunning()` / `bool isCalibrated()`

查询运行状态和校准状态。

---

### SerialPort 串口类

低层串口封装，使用 termios raw 模式（8N1，无流控）。

```cpp
tactile::SerialPort sp;
sp.open("/dev/ttyUSB0", 921600);  // 打开串口
sp.write(data);                     // 发送数据
int n = sp.read(buf, 1024, 100);   // 读取（超时 100ms，返回 0 表示超时）
sp.flushInput();                    // 清空输入缓冲
sp.close();                         // 关闭
```

支持的波特率：`9600`, `19200`, `38400`, `57600`, `115200`, `230400`, `460800`, `921600`。

---

### Protocol 协议层

`protocol.hpp` 为 header-only，提供三个核心工具：

#### CRC8-ITU 校验

```cpp
uint8_t crc = tactile::protocol::crc8_itu(data_ptr, length);
```

多项式 `0x07`，结果异或 `0x55`。

#### 构建请求帧

```cpp
auto frame = tactile::protocol::buildRequestFrame(
    0x34,             // 设备地址
    0x00001C00,       // 起始地址
    160               // 期望字节数
);
// → 14 字节: 55 AA + len(9,LE) + dev_addr + reserved + 0xFB + addr(LE32) + count(LE16) + CRC8
```

#### 流式帧解析器 FrameParser

```cpp
tactile::protocol::FrameParser parser(160);  // 期望的传感器数据字节数

// 持续喂入接收到的字节
parser.feed(rx_buffer, n);

// 循环提取已通过 CRC 校验的有效帧
std::vector<uint8_t> payload;
while (parser.nextFrame(payload)) {
    // payload 为传感器数据段（已跳过协议头）
}
```

帧格式：`AA 55` (帧头) + payload_len (LE16) + 载荷 + CRC8。

---

## 运行测试

```bash
cd build
./test_protocol
```

测试覆盖：

- CRC8-ITU 正确性
- 请求帧格式验证
- 正常帧解析
- 带垃圾前缀 + 逐字节分片喂入
- CRC 错误帧应被拒绝

---

## 注意事项

1. **权限**：串口设备需要读写权限，可将用户加入 `dialout` 组：`sudo usermod -a -G dialout $USER`
2. **校准**：启动后传感器需要静止放置，等待 `buffer_len` 帧采集完成。校准时无回调触发。
3. **线程安全**：`setDataCallback` 和 `getLatest` 是线程安全的。回调在后台线程执行，避免在回调中做耗时操作。
4. **阈值调优**：`threshold` 用于过滤底噪波动，根据实际传感器响应调整。
