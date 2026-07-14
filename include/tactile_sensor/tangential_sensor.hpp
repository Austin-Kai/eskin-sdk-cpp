#pragma once
#include <vector>
#include <deque>

#include "tactile_sensor/api_export.h"

namespace tactile {

/// 切向力角度估计配置
struct TACTILE_API TangentialConfig {
    int   rows             = 12;   ///< 传感器阵列行数
    int   cols             = 7;    ///< 传感器阵列列数 (ADC 输入长度 = rows × cols)
    float threshold_factor = 5.0f; ///< 动态阈值倍数: thresh = factor × mean(前 collect_frames 帧总压力)
    int   collect_frames   = 20;   ///< 学习动态阈值的样本窗口 (≤0 则阈值为 0, 首帧非零即锁原点)
    int   stability_frames = 5;    ///< 连续低压帧数阈值, 超过则自动 resetOrigin
    int   reset_at_frame   = 0;    ///< 在第 N 帧自动 resetOrigin (0 = 禁用)
    int   refine_cnt       = 10;   ///< 原点锁定后 CoP 连续稳定帧数触发二次精修 (0 = 禁用)
    float refine_distance  = 0.1f; ///< 判定 CoP 稳定的欧氏距离阈值 (cells, 0 = 禁用)
};

/// 压阻传感器切向力角度估计 (CoP 位移 → 角度)
///
/// 输入 rows×cols 个 ADC 原始值, 输出基于压力中心 (CoP) 位移的切向力角度 (0~360°)。
/// 内置动态阈值学习、原点锁定、低压自动复位和二次精修。
class TACTILE_API TangentialSensor {
public:
    explicit TangentialSensor(const TangentialConfig& cfg);

    /// 从 ADC 原始数据计算切向力角度
    /// @param adc_data  长度必须等于 rows × cols 的 ADC 原始值
    /// @return 切向力角度 (0~360°)
    float getAngle(const std::vector<int>& adc_data);

    /// 重置原点及所有内部状态 (阈值保留)
    void resetOrigin();

private:
    std::pair<float, float> computeCoP(const std::vector<float>& frame2d,
                                       float total_pressure);
    void updateDynamicThreshold(float total_pressure);
    std::pair<float, float> computeDeltaCoP(const std::vector<int>& raw_frame);

    static float computeAngle(float x, float y);
    static float computeCopAngleCcW90(float px, float py);

    TangentialConfig cfg_;
    const bool       refine_enabled_;

    // 动态阈值
    std::deque<float> pressure_history_;
    float thresh_     = -1.0f;
    bool  thresh_set_ = false;

    // 原点
    float origin_x_ = 0.0f;
    float origin_y_ = 0.0f;
    bool  contact_init_ = false;

    // 低压检测
    int low_counter_ = 0;
    int frame_count_ = 0;

    // 二次精修
    float refine_cand_x_ = 0.0f;
    float refine_cand_y_ = 0.0f;
    int   refine_curr_   = 0;
    bool  refined_       = false;
};

} // namespace tactile
