import numpy as np
import struct

class CommunicationType:
    """
    电机通信类型定义 (Bit28~24)
    参考说明书 4.1 章节
    
    通信 ID 结构 (29位扩展帧):
    | Bit 28-24 | Bit 23-8 | Bit 7-0 |
    | 通信类型   | 数据区2  | 目标地址 |
    """
    GET_DEVICE_ID       = 0   # 获取设备 ID 和 64 位 MCU 唯一标识符 (Type 0)
    OPERATION_CONTROL   = 1   # 运控模式电机控制指令 (MIT 模式) (Type 1)
    OPERATION_STATUS    = 2   # 电机反馈数据 (标准反馈帧) (Type 2)
    ENABLE              = 3   # 电机使能运行 (Type 3)
    DISABLE             = 4   # 电机停止运行 (可用于清除故障) (Type 4)
    SET_ZERO_POSITION   = 6   # 设置电机机械零位 (设置当前位置为零点) (Type 6)
    SET_CAN_ID          = 7   # 设置电机 CAN ID (立即生效，需保存) (Type 7)
    READ_PARAMETER      = 17  # 单个参数读取 (Type 17, 0x11)
    WRITE_PARAMETER     = 18  # 单个参数写入 (Type 18, 0x12, 掉电丢失)
    FAULT_REPORT        = 21  # 故障反馈帧 (Type 21, 0x15)
    SAVE_PARAMETERS     = 22  # 电机数据保存帧 (保存所有参数到 Flash) (Type 22)
    SET_BAUDRATE        = 23  # 电机波特率修改帧 (重新上电生效) (Type 23)
    ACTIVE_REPORT       = 24  # 电机主动上报设置帧 (开启/关闭主动上报) (Type 24)
    PROTOCOL_SWITCH     = 25  # 电机协议修改帧 (切换 Canopen/MIT/私有协议) (Type 25)
    READ_VERSION        = 26  # 版本号读取帧 (Type 26)

class RunMode:
    """
    电机运行模式 (参数索引 0x7005)
    参考说明书 4.3 章节
    """
    MIT                 = 0   # 运控模式 (默认): 适用于高动态响应控制
    POS_PP              = 1   # 位置模式 (PP): 梯形加减速位置控制
    SPEED               = 2   # 速度模式: 闭环速度控制
    CURRENT             = 3   # 电流模式: 闭环力矩(电流)控制
    POS_CSP             = 5   # 位置模式 (CSP): 循环同步位置模式 (适用于周期性指令)

class BaudRate:
    """
    电机波特率 (通信类型 23)
    参考说明书 4.1 通信类型 23
    注意: 修改后需重新上电生效
    """
    BAUD_1M             = 1   # 1 Mbps (默认)
    BAUD_500K           = 2   # 500 Kbps
    BAUD_250K           = 3   # 250 Kbps
    BAUD_125K           = 4   # 125 Kbps

class ActiveReportStatus:
    """
    电机主动上报状态 (通信类型 24)
    参考说明书 4.1 通信类型 24
    """
    DISABLE             = 0   # 关闭主动上报 (默认)
    ENABLE              = 1   # 开启主动上报 (默认间隔 10ms, 可通过 EP_SCAN_TIME 修改)

class ProtocolType:
    """
    电机协议类型 (通信类型 25)
    参考说明书 4.2.4 章节
    注意: 切换协议后需重新上电生效
    """
    PRIVATE             = 0   # 私有协议 (默认): 使用 29 位扩展帧
    CANOPEN             = 1   # CANopen 协议: 符合 CiA 402 标准
    MIT                 = 2   # MIT 协议 (标准帧): 使用 11 位标准帧

class ParamType:
    """
    参数数据类型定义
    
    - 私有协议 (Type 17/18) 参数表主要使用 UINT8/UINT16/UINT32/FLOAT
    - CANopen 对象字典会用到有符号类型 (INTEGER8/16/32)
    """
    UINT8               = 0   # 无符号 8 位整数
    UINT16              = 1   # 无符号 16 位整数
    UINT32              = 2   # 无符号 32 位整数
    FLOAT               = 3   # 32 位浮点数 (IEEE 754)
    INT8                = 4   # 有符号 8 位整数
    INT16               = 5   # 有符号 16 位整数
    INT32               = 6   # 有符号 32 位整数

class ErrorCode:
    """
    异常状态 fault 值位定义
    
    说明书位置:
    - 章节 6 (Mit) 的“异常状态应答帧”对 fault 值 bit 位做了明确描述
    - 私有协议 Type 21 故障反馈帧也会携带 fault/warning 值
    """
    OVER_TEMP           = 1 << 0  # bit0: 电机过温故障 (默认 >145°C)
    DRIVE_CHIP          = 1 << 1  # bit1: 驱动芯片故障 (DRV8353 等报告错误)
    UNDER_VOLTAGE       = 1 << 2  # bit2: 欠压故障 (电压 < 12V)
    OVER_VOLTAGE        = 1 << 3  # bit3: 过压故障 (电压 > 60V)
    CURRENT_B_OVER      = 1 << 4  # bit4: B 相电流采样过流
    CURRENT_C_OVER      = 1 << 5  # bit5: C 相电流采样过流
    ENCODER_NOT_CALIB   = 1 << 7  # bit7: 编码器未标定
    HARDWARE_ERR        = 1 << 8  # bit8: 硬件识别故障
    POS_INIT_ERR        = 1 << 9  # bit9: 位置初始化故障
    LOAD_BLOCK          = 1 << 14 # bit14: 堵转过载算法保护
    CURRENT_A_OVER      = 1 << 16 # bit16: A 相电流采样过流

class WarningCode:
    """
    预警状态 warning 值位定义 (Type 21 Byte 4-7)
    """
    OVER_TEMP_WARNING   = 1 << 0  # bit0: 电机过温预警 (默认 >135°C)

class DriveFault1:
    """
    驱动芯片故障码 1 (0x3024) - DRV8353 状态寄存器 1
    参考说明书 3.3.7 章节
    """
    VDS_LC              = 1 << 0  # VDS overcurrent on C low-side (C相下管VDS过流)
    VDS_HC              = 1 << 1  # VDS overcurrent on C high-side (C相上管VDS过流)
    VDS_LB              = 1 << 2  # VDS overcurrent on B low-side (B相下管VDS过流)
    VDS_HB              = 1 << 3  # VDS overcurrent on B high-side (B相上管VDS过流)
    VDS_LA              = 1 << 4  # VDS overcurrent on A low-side (A相下管VDS过流)
    VDS_HA              = 1 << 5  # VDS overcurrent on A high-side (A相上管VDS过流)
    OTSD                = 1 << 6  # Overtemperature shutdown (过温关断)
    UVLO                = 1 << 7  # Undervoltage lockout (欠压锁定)
    GDF                 = 1 << 8  # Gate drive fault (栅极驱动故障)
    VDS_OCP             = 1 << 9  # VDS monitor overcurrent (VDS 监控过流)
    FAULT               = 1 << 10 # Logic OR of FAULT status (故障状态逻辑或)

class DriveFault2:
    """
    驱动芯片故障码 2 (0x3025) - DRV8353 状态寄存器 2
    参考说明书 3.3.7 章节
    """
    VGS_LC              = 1 << 0  # Gate drive fault on C low-side (C相下管栅极故障)
    VGS_HC              = 1 << 1  # Gate drive fault on C high-side (C相上管栅极故障)
    VGS_LB              = 1 << 2  # Gate drive fault on B low-side (B相下管栅极故障)
    VGS_HB              = 1 << 3  # Gate drive fault on B high-side (B相上管栅极故障)
    VGS_LA              = 1 << 4  # Gate drive fault on A low-side (A相下管栅极故障)
    VGS_HA              = 1 << 5  # Gate drive fault on A high-side (A相上管栅极故障)
    GDUV                = 1 << 6  # VCP charge pump / VGLS undervoltage (电荷泵欠压)
    OTW                 = 1 << 7  # Overtemperature warning (过温预警)
    SC_OC               = 1 << 8  # Overcurrent on phase C sense amplifier (C相采样过流)
    SB_OC               = 1 << 9  # Overcurrent on phase B sense amplifier (B相采样过流)
    SA_OC               = 1 << 10 # Overcurrent on phase A sense amplifier (A相采样过流)

class MotorParams:
    """
    电机物理参数限制 (用于 MIT 模式数据压缩)
    参考说明书 4.1 通信类型 1
    
    注意:
    - P_MIN/MAX: 位置范围 (RS03: -12.57 ~ 12.57 rad)
    - V_MIN/MAX: 速度范围 (RS03: -20 ~ 20 rad/s)
    - T_MIN/MAX: 力矩范围 (RS03: -60 ~ 60 Nm)
    - KP/KD: 刚度和阻尼系数范围
    """
    def __init__(self,
                 p_min: float = -12.57,
                 p_max: float = 12.57,  # RS03: -12.57 ~ 12.57 rad (约 -4pi ~ 4pi)
                 v_min: float = -20.0,
                 v_max: float = 20.0,   # RS03: -20 ~ 20 rad/s
                 kp_min: float = 0.0,
                 kp_max: float = 5000.0, # RS03: 0 ~ 5000
                 kd_min: float = 0.0,
                 kd_max: float = 100.0,  # RS03: 0 ~ 100
                 t_min: float = -60.0,
                 t_max: float = 60.0):  # RS03: -60 ~ 60 Nm
        self.P_MIN = p_min
        self.P_MAX = p_max
        self.V_MIN = v_min
        self.V_MAX = v_max
        self.KP_MIN = kp_min
        self.KP_MAX = kp_max
        self.KD_MIN = kd_min
        self.KD_MAX = kd_max
        self.T_MIN = t_min
        self.T_MAX = t_max

class ParamIndex:
    """
    电机参数索引表 (Index)
    参考说明书 4.1 可读写单个参数列表
    """
    RUN_MODE            = 0x7005  # 运行模式: 0:运控, 1:PP, 2:速度, 3:电流, 5:CSP (W/R)
    IQ_REF              = 0x7006  # 电流模式 Iq 指令 (-43~43A) (W/R)
    SPD_REF             = 0x700A  # 转速模式转速指令 (-20~20rad/s) (W/R)
    LIMIT_TORQUE        = 0x700B  # 转矩限制 (0~60Nm) (W/R)
    CUR_KP              = 0x7010  # 电流 Kp (默认 0.17) (W/R)
    CUR_KI              = 0x7011  # 电流 Ki (默认 0.012) (W/R)
    CUR_FILT_GAIN       = 0x7014  # 电流滤波系数 (0~1.0, 默认 0.1) (W/R)
    LOC_REF             = 0x7016  # 位置模式角度指令 (rad) (W/R)
    LIMIT_SPD           = 0x7017  # 位置模式(CSP)速度限制 (0~20rad/s) (W/R)
    LIMIT_CUR           = 0x7018  # 速度/位置模式电流限制 (0~43A) (W/R)
    MECH_POS            = 0x7019  # 负载端计圈机械角度 (rad) (Read Only)
    IQF                 = 0x701A  # Iq 滤波值 (A) (Read Only)
    MECH_VEL            = 0x701B  # 负载端转速 (rad/s) (Read Only)
    VBUS                = 0x701C  # 母线电压 (V) (Read Only)
    LOC_KP              = 0x701E  # 位置环 Kp (默认 60) (W/R)
    SPD_KP              = 0x701F  # 速度环 Kp (默认 6) (W/R)
    SPD_KI              = 0x7020  # 速度环 Ki (默认 0.02) (W/R)
    SPD_FILT_GAIN       = 0x7021  # 速度滤波值 (默认 0.1) (W/R)
    ACC_RAD             = 0x7022  # 速度模式加速度 (默认 20rad/s^2) (W/R)
    VEL_MAX             = 0x7024  # 位置模式(PP)速度 (默认 10rad/s) (W/R)
    ACC_SET             = 0x7025  # 位置模式(PP)加速度 (默认 10rad/s^2) (W/R)
    EP_SCAN_TIME        = 0x7026  # 主动上报时间 (1=10ms, +1=+5ms) (W)
    CAN_TIMEOUT         = 0x7028  # CAN 超时阈值 (20000=1s, 0=禁用) (W)
    ZERO_STA            = 0x7029  # 零点标志位 (0: 0~2pi, 1: -pi~pi) (W)
    DAMPER              = 0x702A  # 阻尼开关 (1: 取消关机反驱保护) (W/R)
    ADD_OFFSET          = 0x702B  # 零位偏置 (rad) (W/R)

class CanopenIndex:
    """
    CANopen 对象字典常用索引
    参考说明书第 5 章 (Canopen)
    """
    ERROR_CODE                   = 0x603F  # 错误码
    CONTROLWORD                  = 0x6040  # 控制字
    STATUSWORD                   = 0x6041  # 状态字
    MODES_OF_OPERATION           = 0x6060  # 运行模式
    MODES_OF_OPERATION_DISPLAY   = 0x6061  # 当前运行模式显示
    POSITION_DEMAND_VALUE        = 0x6062  # 位置指令值
    POSITION_ACTUAL_VALUE        = 0x6064  # 位置实际值
    POSITION_WINDOW              = 0x6067  # 位置窗口
    POSITION_WINDOW_TIME         = 0x6068  # 位置窗口时间
    VELOCITY_DEMAND_VALUE        = 0x606B  # 速度指令值
    VELOCITY_ACTUAL_VALUE        = 0x606C  # 速度实际值
    TARGET_TORQUE                = 0x6071  # 目标力矩 (0.1% 额定力矩)
    TORQUE_ACTUAL_VALUE          = 0x6077  # 力矩实际值
    CURRENT_ACTUAL_VALUE         = 0x6078  # 电流实际值
    DC_LINK_CIRCUIT_VOLTAGE      = 0x6079  # 母线电压
    TARGET_POSITION              = 0x607A  # 目标位置
    PROFILE_VELOCITY             = 0x6081  # 轮廓速度
    PROFILE_ACCELERATION         = 0x6083  # 轮廓加速度
    TARGET_VELOCITY              = 0x60FF  # 目标速度

class CanopenModeOfOperation:
    """CANopen 模式 (6060)"""
    PP       = 1  # Profile Position Mode
    SPEED    = 3  # Profile Velocity Mode
    TORQUE   = 4  # Profile Torque Mode
    CSP      = 5  # Cyclic Synchronous Position Mode
    HOMING   = 6  # Homing Mode

class CanopenControlword:
    """CANopen 控制字 (6040) 常用值"""
    SHUTDOWN           = 0x0006  # Shutdown
    SWITCH_ON          = 0x0007  # Switch On
    ENABLE_OPERATION   = 0x000F  # Enable Operation
    DISABLE_VOLTAGE    = 0x0001  # Disable Voltage
    QUICK_STOP         = 0x000B  # Quick Stop

# CANopen 协议切换帧 (扩展帧)
# 说明书 5.10: 29 位 ID 为 0xFFF，数据区 Byte0~6 固定 01~06，Byte7=F_CMD(协议类型)
CANOPEN_PROTOCOL_SWITCH_EXT_ID = 0xFFF

class MitStdCommandType:
    """
    MIT 标准帧指令类型 (对应说明书第 6 章的指令 1~11)
    
    标准帧 ID (11位) 结构:
    | Bit 10-8 | Bit 7-0 |
    | 模式/指令 | 电机 ID |
    
    注意:
    - 指令 1~9: CAN ID 的 Bit10~8 为 0，通过数据区 Payload 区分功能
    - 指令 10:  CAN ID 的 Bit10~8 为 1 (位置模式)
    - 指令 11:  CAN ID 的 Bit10~8 为 2 (速度模式)
    """
    ENABLE                      = 1   # 指令 1: 电机使能运行
    STOP                        = 2   # 指令 2: 电机停止运行
    DYNAMIC_PARAM               = 3   # 指令 3: MIT 动态参数
    SET_ZERO                    = 4   # 指令 4: 设置零点 (非位置模式)
    CLEAR_ERROR_OR_READ_STATUS  = 5   # 指令 5: 清错 / 读取异常状态
    SET_RUN_MODE                = 6   # 指令 6: 设置运行模式
    SET_MOTOR_CAN_ID            = 7   # 指令 7: 修改电机 CANID
    SET_PROTOCOL                = 8   # 指令 8: 修改电机协议 (重新上电生效)
    SET_MASTER_CAN_ID           = 9   # 指令 9: 修改主机 CANID
    POS_CONTROL                 = 10  # 指令 10: 位置模式控制指令 (ID Bit10-8=1)
    SPEED_CONTROL               = 11  # 指令 11: 速度模式控制指令 (ID Bit10-8=2)

def get_mit_can_id_mode(cmd_type: int) -> int:
    """
    获取 MIT 标准帧 CAN ID 的 Bit10~8 值
    
    :param cmd_type: MitStdCommandType 枚举值
    :return: 模式位 (0, 1, 或 2)
    """
    if cmd_type in (MitStdCommandType.POS_CONTROL,):
        return 1
    elif cmd_type in (MitStdCommandType.SPEED_CONTROL,):
        return 2
    else:
        # 指令 1~9 (以及其他潜在指令) 默认为 0
        return 0

def build_mit_std_id(cmd_type: int, motor_id: int) -> int:
    """
    构建 MIT 标准帧 11 位 CAN ID
    
    :param cmd_type: MitStdCommandType 枚举值
    :param motor_id: 电机 ID (0~127)
    :return: 11 位 CAN ID
    """
    mode = get_mit_can_id_mode(cmd_type)
    return ((mode & 0x07) << 8) | (motor_id & 0xFF)

class MitPayloads:
    """
    MIT 协议特殊指令的固定 Payload 定义 (指令 1, 2, 4, 5, 6, 7, 8, 9)
    部分指令的 Payload 末尾字节需要根据参数动态修改
    """
    # 指令 1: FF FF FF FF FF FF FF FC
    ENABLE = b'\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFC'
    
    # 指令 2: FF FF FF FF FF FF FF FD
    STOP = b'\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFD'
    
    # 指令 3: 动态参数 (全 0 或根据参数设置)
    DYNAMIC_PARAM_ZERO = b'\x00\x00\x00\x00\x00\x00\x00\x00'
    
    # 指令 4: FF FF FF FF FF FF FF FE
    SET_ZERO = b'\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFE'
    
    # 指令 5: FF FF FF FF FF FF FF FB (清除错误)
    # 若 F_CMD (Byte6) 为 0xFF 则清除错误，否则为读取异常状态
    CLEAR_ERROR = b'\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFB'
    
    # 指令 6: FF FF FF FF FF FF [Mode] FC
    # Template, last 2 bytes are [Mode, FC]
    SET_RUN_MODE_PREFIX = b'\xFF\xFF\xFF\xFF\xFF\xFF'
    
    # 指令 7: FF FF FF FF FF FF [NewID] FA
    SET_MOTOR_CAN_ID_PREFIX = b'\xFF\xFF\xFF\xFF\xFF\xFF'
    
    # 指令 8: FF FF FF FF FF FF [Protocol] FD
    SET_PROTOCOL_PREFIX = b'\xFF\xFF\xFF\xFF\xFF\xFF'
    
    # 指令 9: FF FF FF FF FF FF [MasterID] 01
    SET_MASTER_CAN_ID_PREFIX = b'\xFF\xFF\xFF\xFF\xFF\xFF'


# 参数表配置: (参数名, 数据类型, 字节数)
PARAM_TABLE = {
    ParamIndex.RUN_MODE:      ("run_mode", ParamType.UINT8, 1),
    ParamIndex.IQ_REF:        ("iq_ref", ParamType.FLOAT, 4),
    ParamIndex.SPD_REF:       ("spd_ref", ParamType.FLOAT, 4),
    ParamIndex.LIMIT_TORQUE:  ("limit_torque", ParamType.FLOAT, 4),
    ParamIndex.CUR_KP:        ("cur_kp", ParamType.FLOAT, 4),
    ParamIndex.CUR_KI:        ("cur_ki", ParamType.FLOAT, 4),
    ParamIndex.CUR_FILT_GAIN: ("cur_filt_gain", ParamType.FLOAT, 4),
    ParamIndex.LOC_REF:       ("loc_ref", ParamType.FLOAT, 4),
    ParamIndex.LIMIT_SPD:     ("limit_spd", ParamType.FLOAT, 4),
    ParamIndex.LIMIT_CUR:     ("limit_cur", ParamType.FLOAT, 4),
    ParamIndex.MECH_POS:      ("mechPos", ParamType.FLOAT, 4),
    ParamIndex.IQF:           ("iqf", ParamType.FLOAT, 4),
    ParamIndex.MECH_VEL:      ("mechVel", ParamType.FLOAT, 4),
    ParamIndex.VBUS:          ("VBUS", ParamType.FLOAT, 4),
    ParamIndex.LOC_KP:        ("loc_kp", ParamType.FLOAT, 4),
    ParamIndex.SPD_KP:        ("spd_kp", ParamType.FLOAT, 4),
    ParamIndex.SPD_KI:        ("spd_ki", ParamType.FLOAT, 4),
    ParamIndex.SPD_FILT_GAIN: ("spd_filt_gain", ParamType.FLOAT, 4),
    ParamIndex.ACC_RAD:       ("acc_rad", ParamType.FLOAT, 4),
    ParamIndex.VEL_MAX:       ("vel_max", ParamType.FLOAT, 4),
    ParamIndex.ACC_SET:       ("acc_set", ParamType.FLOAT, 4),
    ParamIndex.EP_SCAN_TIME:  ("EPScan_time", ParamType.UINT16, 2),
    ParamIndex.CAN_TIMEOUT:   ("cantimeout", ParamType.UINT32, 4),
    ParamIndex.ZERO_STA:      ("zero_sta", ParamType.UINT8, 1),
    ParamIndex.DAMPER:        ("damper", ParamType.UINT8, 1),
    ParamIndex.ADD_OFFSET:    ("add_offset", ParamType.FLOAT, 4),
}

MODEL_MIT_POSITION_TABLE = {
    "rs-00": 4 * np.pi, "rs-01": 4 * np.pi, "rs-02": 4 * np.pi,
    "rs-03": 4 * np.pi, "rs-04": 4 * np.pi, "rs-05": 4 * np.pi, "rs-06": 4 * np.pi,
    "el-05": 4 * np.pi,
}

MODEL_MIT_VELOCITY_TABLE = {
    "rs-00": 50, "rs-01": 44, "rs-02": 44,
    "rs-03": 50, "rs-04": 15, "rs-05": 33, "rs-06": 20,
    "el-05": 50,
}

MODEL_MIT_TORQUE_TABLE = {
    "rs-00": 17, "rs-01": 17, "rs-02": 17,
    "rs-03": 60, "rs-04": 120, "rs-05": 17, "rs-06": 60,
    "el-05": 6,
}

MODEL_MIT_KP_TABLE = {
    "rs-00": 500.0, "rs-01": 500.0, "rs-02": 500.0,
    "rs-03": 5000.0, "rs-04": 5000.0, "rs-05": 500.0, "rs-06": 5000.0,
    "el-05": 500.0,
}

MODEL_MIT_KD_TABLE = {
    "rs-00": 5.0, "rs-01": 5.0, "rs-02": 5.0,
    "rs-03": 100.0, "rs-04": 100.0, "rs-05": 5.0, "rs-06": 100.0,
    "el-05": 5.0,
}

def get_pack_format(param_type):
    """
    获取 struct.pack 的格式字符串和字节大小

    说明:
    - Type 17/18 参数读写使用小端序
    - CANopen SDO 数据同样通常按小端序解释 (取决于实现)
    """
    if param_type == ParamType.UINT8:
        return '<B', 1
    elif param_type == ParamType.UINT16:
        return '<H', 2
    elif param_type == ParamType.UINT32:
        return '<I', 4
    elif param_type == ParamType.INT8:
        return '<b', 1
    elif param_type == ParamType.INT16:
        return '<h', 2
    elif param_type == ParamType.INT32:
        return '<i', 4
    elif param_type == ParamType.FLOAT:
        return '<f', 4
    return None, 0
