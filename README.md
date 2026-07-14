# eSkin Tactile Sensor SDK v1.0

触觉传感器 C++ SDK — 串口通信、实时特征提取、切向力角度感知。

## 交付文件

```
eskin_sdk_v1.0/
├── lib/libtactile_sensor.so          # 核心动态库
├── include/tactile_sensor/
│   ├── tactile_sensor.hpp            # Config / TactileFrame / TactileSensor
│   ├── tangential_sensor.hpp         # TangentialConfig / TangentialSensor
│   └── api_export.h                  # 符号导出宏
├── example/example_main.cpp          # 完整示例
└── README.md
```

## 编译 & 运行

```bash
# 编译
g++ -std=c++14 -I./include example/example_main.cpp \
    -L./lib -ltactile_sensor -pthread \
    -Wl,-rpath,'$ORIGIN/lib' \
    -o tactile_example

# 运行
./tactile_example --port /dev/ttyUSB0 --baud 921600 --rate 200 --threshold 100
```

## API 参考

### Config — 全部可配置参数

| 分类             | 字段                      | 类型         | 默认值             | 说明                         |
| ---------------- | ------------------------- | ------------ | ------------------ | ---------------------------- |
| **连接**   | `port`                  | `string`   | `"/dev/ttyUSB0"` | 串口设备路径                 |
|                  | `baudrate`              | `int`      | `921600`         | 波特率                       |
|                  | `poll_rate`             | `double`   | `200.0`          | 轮询频率 (Hz)                |
| **几何**   | `rows`                  | `int`      | `12`             | 传感器行数                   |
|                  | `cols`                  | `int`      | `7`              | 传感器列数                   |
|                  | `single_sensor`         | `bool`     | `true`           | 单片 / 双片模式              |
| **协议**   | `dev_addr`              | `uint8_t`  | `0x34`           | 设备地址                     |
|                  | `start_addr`            | `uint32_t` | `0x00001C00`     | 读取起始地址                 |
| **校准**   | `threshold`             | `int`      | `100`            | 去基线后阈值，低于此值归零   |
|                  | `buffer_len`            | `int`      | `100`            | 上电校准帧数                 |
| **切向力** | `tang_threshold_factor` | `float`    | `5.0`            | 动态阈值倍数                 |
|                  | `tang_collect_frames`   | `int`      | `20`             | 阈值学习帧数 (≤0=阈值0)     |
|                  | `tang_stability_frames` | `int`      | `5`              | 连续低压帧数→自动复位原点   |
|                  | `tang_reset_at_frame`   | `int`      | `0`              | 第N帧自动复位 (0=禁用)       |
|                  | `tang_refine_cnt`       | `int`      | `10`             | 稳定帧数→二次精修 (0=禁用)  |
|                  | `tang_refine_distance`  | `float`    | `0.1`            | 稳定判定距离 (cells, 0=禁用) |

### TactileFrame — 每帧数据

| 字段                     | 类型            | 说明                                                          |
| ------------------------ | --------------- | ------------------------------------------------------------- |
| `rawArrayData`         | `vector<int>` | 原始 ADC 值 (rows×cols)                                      |
| `fineArrayData`        | `vector<int>` | 去基线+阈值后 (rows×cols)                                    |
| `minValue`             | `float`       | 最小值                                                        |
| `maxValue`             | `float`       | 最大值                                                        |
| `sumValue`             | `float`       | 总和                                                          |
| `meanValue`            | `float`       | 均值                                                          |
| `forceDerivative`      | `float`       | 力导数 df/dt                                                  |
| `copX`                 | `float`       | 压力中心 X (长边方向, cells)                                  |
| `copY`                 | `float`       | 压力中心 Y (短边方向, cells)                                  |
| `sensingArea`          | `int`         | 感应面积 (非零像素数)                                         |
| `tangentialForceAngle` | `float`       | 切向力角度 0~360°（指腹向上，指尖对外朝向为0度，顺时针方向） |

### TactileSensor 类

```cpp
class TactileSensor {
public:
    explicit TactileSensor(const Config& cfg);
    ~TactileSensor();

    bool start();                              // 打开串口, 启动后台轮询
    void stop();                               // 停止并关闭串口
    bool isRunning() const;                    // 运行状态
    bool isCalibrated() const;                 // 底噪校准状态

    void setDataCallback(DataCallback cb);     // 注册回调 (后台线程调用)
    bool getLatest(TactileFrame& out);         // 线程安全获取最新帧

    void recalibrate();                        // 重新底噪校准
    void recalibrateTangential();              // 手动复位切向力原点
};
```

**启动后自动流程**：

1. 采集 `buffer_len` 帧 → 计算底噪基线
2. 校准完成 → 开始触发回调
3. 切向力算法自动学习阈值、锁定原点、二次精修

### TangentialSensor 类 (可独立使用)

```cpp
#include "tactile_sensor/tangential_sensor.hpp"

tactile::TangentialConfig tc;
tc.rows = 12; tc.cols = 7;

tactile::TangentialSensor ts(tc);
float angle = ts.getAngle(adc_data);  // 0~360°
ts.resetOrigin();                      // 手动复位
```

---

## 切向力角度坐标系

```
12×7 阵列 (从传感器背部看):
  [0,0]=右上角, [11,6]=左下角

角度定义 (0~360°, 逆时针):
   -X 方向 (长边负向) =   0°
   +Y 方向 (短边正向) =  90°
   +X 方向 (长边正向) = 180°
   -Y 方向 (短边负向) = 270°

            0° (-X)
             ↑
   270° ←───┼───→ 90° (+Y)
             ↓
           180° (+X)
```

---

## 依赖

| 依赖      | 最低版本     |
| --------- | ------------ |
| Linux     | x86_64       |
| glibc     | 2.17         |
| libstdc++ | 6.0 (GCC 5+) |
| pthread   | 系统自带     |
