#include "lingzu.hpp"
#include <algorithm>

LINGZU::LINGZU(const config_t& config) : Motor(config) {
    // 设置电机默认参数
    if (this->config.reduction == 0) this->config.reduction = REDUCTION;
    if (this->config.Kt == 0) this->config.Kt = Kt;
    if (this->config.R == 0) this->config.R = R;

    // 注册CAN回调
    auto callback = [this](const uint8_t port, const uint32_t id, const uint8_t data[8], const uint8_t dlc) {
        this->callback(port, id, data, dlc);
    };
    BSP::CAN::RegisterCallback(callback);
}

void LINGZU::SendCanCmd() {
    send_cnt++;
    if (is_connect && is_enable) {
        if (send_cnt % 100 == 0) { // 每100次调用重新发送使能防超时
            sendEnable();
        } else {
            // 父类PID计算的结果将作为前馈力矩发送，完全兼容已有架构
            sendTorque(torque.ref);
        }
    } else {
        sendDisable();
    }
}

void LINGZU::callback(const uint8_t port, const uint32_t id, const uint8_t data[8], const uint8_t dlc) {
    // 端口、ID、长度校验
    if (port != config.can_port) return;
    if (id != config.master_id) return;
    if (dlc != 8) return;

    // 检查电机ID
    if ((data[0] & 0x0F) != (config.slave_id & 0x0F)) return;

    // 数据拆包 (严格参照手册第52页：应答指令 1)
    const uint16_t angle_u16  = (data[1] << 8) | data[2];
    const uint16_t speed_u12  = (data[3] << 4) | (data[4] >> 4);
    const uint16_t torque_u12 = ((data[4] & 0x0F) << 8) | data[5];
    
    // 解析状态机和故障码 (Byte6)
    mode_status = static_cast<mode_status_e>((data[6] >> 6) & 0x03);
    has_fault   = (data[6] >> 5) & 0x01;
    has_warning = (data[6] >> 4) & 0x01;

    // 解析温度 (Byte6 低4位和 Byte7) -> 12位有符号数
    int16_t temp_u12 = ((data[6] & 0x0F) << 8) | data[7];
    if (temp_u12 & 0x0800) temp_u12 |= 0xF000; // 负数符号位扩展
    temperature.motor = (temp_u12 / 10.0f) * C;
    temperature.mos   = 0 * C; // 手册中只包含绕组温度

    // 单位标准化转换
    const UnitFloat torque = uint_to_float(torque_u12, T_MIN, T_MAX, 12) * Nm;
    const raw_t raw = {
        .current = torque / config.Kt,
        .torque  = torque,
        .speed   = uint_to_float(speed_u12, V_MIN, V_MAX, 12) * rad_s,
        .angle   = uint_to_float(angle_u16, P_MIN, P_MAX, 16) * rad
    };

    Motor::callback(raw);
}

float LINGZU::uint_to_float(const int x_int, const float x_min, const float x_max, const int bits) {
    const float span = x_max - x_min;
    const float offset = x_min;
    return (float)x_int * span / (float)((1 << bits) - 1) + offset;
}

uint16_t LINGZU::float_to_uint(const float x, const float x_min, const float x_max, const int bits) {
    const float span = x_max - x_min;
    const float offset = x_min;
    return (uint16_t)((x - offset) * (float)((1 << bits) - 1) / span);
}

void LINGZU::sendEnable() const {
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    BSP::CAN::Transmit(config.can_port, config.slave_id, data, 8);
}

void LINGZU::sendDisable() const {
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
    BSP::CAN::Transmit(config.can_port, config.slave_id, data, 8);
}

void LINGZU::sendTorque(const UnitFloat<>& torque) const {
    sendMitCtrl(0.0f, 0.0f, 0.0f, 0.0f, torque.toFloat(Nm));
}

void LINGZU::sendMitCtrl(float pos, float vel, float kp, float kd, float torque) const {
    // 处理坐标系反转与限幅
    float torque_limit = std::clamp(torque, T_MIN, T_MAX);
    if (config.is_invert) {
        torque_limit = -torque_limit;
        pos = -pos;
        vel = -vel;
    }
    
    pos = std::clamp(pos, P_MIN, P_MAX);
    vel = std::clamp(vel, V_MIN, V_MAX);
    kp  = std::clamp(kp, KP_MIN, KP_MAX);
    kd  = std::clamp(kd, KD_MIN, KD_MAX);

    // MIT 协议标准 16/12 浮点转整形运算
    uint16_t pos_u16  = float_to_uint(pos, P_MIN, P_MAX, 16);
    uint16_t vel_u12  = float_to_uint(vel, V_MIN, V_MAX, 12);
    uint16_t kp_u12   = float_to_uint(kp, KP_MIN, KP_MAX, 12);
    uint16_t kd_u12   = float_to_uint(kd, KD_MIN, KD_MAX, 12);
    uint16_t torq_u12 = float_to_uint(torque_limit, T_MIN, T_MAX, 12);

    // 标准 MIT 数据打包 (参考手册第53页：指令3)
    uint8_t data[8];
    data[0] = pos_u16 >> 8;
    data[1] = pos_u16 & 0xFF;
    data[2] = vel_u12 >> 4;
    data[3] = ((vel_u12 & 0x0F) << 4) | (kp_u12 >> 8);
    data[4] = kp_u12 & 0xFF;
    data[5] = kd_u12 >> 4;
    data[6] = ((kd_u12 & 0x0F) << 4) | (torq_u12 >> 8);
    data[7] = torq_u12 & 0xFF;

    // 发送标准帧 (第5个参数 bool is_ext 保持默认值 false)
    BSP::CAN::Transmit(config.can_port, config.slave_id, data, 8);
}