#pragma once

#include "Motor.hpp"

class LINGZU : public Motor {
public:

    static constexpr float REDUCTION = 9.0f;       // 减速比 9:1
    static constexpr UnitFloat<> Kt = 0.94f * Nm_A; // 转矩常数 0.94 N.m/Arms
    static constexpr UnitFloat<> R = 0.0f * Ohm;    // 相电阻估算 (用于部分前馈)
    static constexpr UnitFloat<> MAX_TORQUE = 6.0f * Nm;  // 峰值负载
    static constexpr UnitFloat<A> MAX_CURRENT = 11.0f * A; // 最大允许相电流

    // 电机运行状态与错误代码
    enum mode_status_e {
        RESET_MODE = 0x00,    // 复位
        CALI_MODE  = 0x01,    // 标定
        MOTOR_MODE = 0x02     // 运行
    } mode_status = RESET_MODE;

    bool has_fault = false;   // 故障标志位
    bool has_warning = false; // 预警标志位

    LINGZU(const config_t& config);

    void SendCanCmd();

    // 暴露给外部调用的标准 MIT 控制指令
    void sendMitCtrl(float pos, float vel, float kp, float kd, float torque) const;

private:
    // MIT 模式参数范围限制 (严格依据手册第53页)
    constexpr static float P_MIN = -12.57f;
    constexpr static float P_MAX = 12.57f;
    constexpr static float V_MIN = -50.0f;
    constexpr static float V_MAX = 50.0f;
    constexpr static float KP_MIN = 0.0f;
    constexpr static float KP_MAX = 500.0f;
    constexpr static float KD_MIN = 0.0f;
    constexpr static float KD_MAX = 5.0f;
    constexpr static float T_MIN = -6.0f;
    constexpr static float T_MAX = 6.0f;

    uint32_t send_cnt = 0; // 用于定期发送使能命令

    // CAN接收函数
    void callback(uint8_t port, uint32_t id, const uint8_t data[8], uint8_t dlc);

    static float uint_to_float(int x_int, float x_min, float x_max, int bits);

    static uint16_t float_to_uint(float x, float x_min, float x_max, int bits);

    // 发送电机使能 (MIT 指令 1)
    void sendEnable() const;

    // 发送电机失能 (MIT 指令 2)
    void sendDisable() const;

    // 适配父类的纯力矩控制
    void sendTorque(const UnitFloat<>& torque) const;
};