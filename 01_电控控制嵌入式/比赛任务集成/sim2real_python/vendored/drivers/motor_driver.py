import struct
import time
import queue
import numpy as np
from typing import Dict, Optional, Any, List
from dataclasses import dataclass

from drivers.usb_can_adapter import DmUsbAdapter
from drivers.motor_params import (
    CommunicationType, ParamIndex, ParamType, 
    MODEL_MIT_POSITION_TABLE, MODEL_MIT_VELOCITY_TABLE, 
    MODEL_MIT_TORQUE_TABLE, MODEL_MIT_KP_TABLE, MODEL_MIT_KD_TABLE,
    get_pack_format, PARAM_TABLE
)

@dataclass
class MotorState:
    position: float = 0.0
    velocity: float = 0.0
    torque: float = 0.0
    temperature: float = 0.0
    current: float = 0.0
    update_count: int = 0

class RobStrideMotor:
    def __init__(self, name: str, motor_id: int, model: str):
        """
        初始化电机对象。
        
        :param name: 电机名称 (例如 "knee")
        :param motor_id: 电机 ID
        :param model: 电机型号 (例如 "rs-06")
        """
        self.name = name
        self.id = motor_id
        self.model = model
        self.state = MotorState()
        
    def update_state(self, pos: float, vel: float, torque: float, temp: float, current: float = 0.0):
        """
        更新电机状态。
        """
        self.state.position = pos
        self.state.velocity = vel
        self.state.torque = torque
        self.state.temperature = temp
        self.state.update_count += 1
        if current != 0.0:
            self.state.current = current

class RobStrideDriver:
    def __init__(self, port: str, debug: bool = False):
        """
        初始化驱动器。
        
        :param port: 串口名称
        :param debug: 是否开启调试模式
        """
        self.adapter = DmUsbAdapter(port, debug=debug)
        self.motors: Dict[str, RobStrideMotor] = {}
        self.motors_by_id: Dict[int, RobStrideMotor] = {}
        self.host_id = 0xFD # 根据文档，主机 ID 默认为 0xFD
        
        self.parameter_values = {} # 读取参数缓存: (motor_id, param_index) -> value
        
    def connect(self):
        """连接到底层适配器。"""
        self.adapter.open()
        print(f"已连接到 RobStride 驱动器，端口: {self.adapter.serial.port}")
        # 设置 CAN 波特率为 1000kbps (Index 0)
        self.adapter.set_can_baudrate(0)
        
    def disconnect(self):
        """断开连接。"""
        self.adapter.close()
        print("已断开 RobStride 驱动器连接")
        
    def set_can_id(self, current_id: int, new_id: int):
        """
        设置电机 CAN ID。
        
        :param current_id: 当前电机 ID
        :param new_id: 新电机 ID
        """
        # Type 7: Set CAN ID
        # Bits 23-16: New ID (Preset ID)
        # Bits 15-8: Master ID
        # Bits 7-0: Target ID
        extra_data = (new_id << 8) | self.host_id
        self._send_command(CommunicationType.SET_CAN_ID, extra_data, current_id)
        print(f"已发送 ID 修改指令: {current_id} -> {new_id} (Master: {self.host_id})")

    def scan_motors(self, timeout: float = 0.1) -> List[int]:
        """
        快速扫描总线上的电机 (ID 1-127)。
        
        :param timeout: 等待响应的超时时间
        :return: 发现的电机 ID 列表
        """
        found_ids = []
        print("正在快速扫描所有电机 (ID 1-127)...")
        
        # 清空缓冲区
        while self.adapter.read_can_frame():
            pass
            
        # 快速发送查询指令
        for dev_id in range(1, 128):
            # 发送获取设备 ID 命令
            self._send_command(CommunicationType.GET_DEVICE_ID, self.host_id, dev_id)
            
        # 等待响应
        start_time = time.time()
        while time.time() - start_time < timeout:
            frame = self.adapter.read_can_frame()
            if frame:
                can_id, data, cmd, ide, rtr = frame
                if not ide: continue
                
                # 解析回复
                # 通信类型 0 (GET_DEVICE_ID/Status)
                comm_type = (can_id >> 24) & 0x1F
                
                if comm_type == CommunicationType.GET_DEVICE_ID: # Type 0
                    # Type 0 回复格式:
                    # Bits 23-8: Status info
                    # Bits 7-0: Motor ID
                    extra_data = (can_id >> 8) & 0xFFFF
                    motor_id = extra_data & 0xFF # Device ID
                    
                    if motor_id not in found_ids:
                        print(f"发现电机 ID: {motor_id}")
                        found_ids.append(motor_id)
                        
        return sorted(found_ids)

        
    def add_motor(self, name: str, motor_id: int, model: str):
        """
        添加电机到控制列表。
        
        :param name: 电机名称
        :param motor_id: 电机 ID
        :param model: 电机型号
        """
        motor = RobStrideMotor(name, motor_id, model)
        self.motors[name] = motor
        self.motors_by_id[motor_id] = motor
        
    def _send_command(self, comm_type: int, extra_data: int, device_id: int, data: bytes = b''):
        # 构建 29 位扩展 CAN ID
        # Bits 28-24: 通信类型 (Communication Type)
        # Bits 23-8:  额外数据 (Extra Data)
        # Bits 7-0:   设备 ID (Device ID)
        can_id = (comm_type << 24) | (extra_data << 8) | device_id
        
        # 通过适配器发送
        # RobStride 使用扩展帧
        self.adapter.send_can_frame(can_id, data, extended=True)
        
    def enable(self, motor_name: str):
        """使能电机。"""
        motor = self.motors[motor_name]
        self._send_command(CommunicationType.ENABLE, self.host_id, motor.id)
        
    def disable(self, motor_name: str):
        """失能电机 (Type 4: Stop)。"""
        motor = self.motors[motor_name]
        # Data: 全 0
        data = bytes([0x00]*8)
        self._send_command(CommunicationType.DISABLE, self.host_id, motor.id, data)

    def clear_warnings(self, motor_name: str):
        """
        清除警告/故障 (Type 4: Stop Motor with Byte0=1)。
        根据文档 Type 4: Byte[0]=1 时清除故障。
        """
        motor = self.motors[motor_name]
        data = bytes([0x01] + [0x00]*7)
        self._send_command(CommunicationType.DISABLE, self.host_id, motor.id, data)
        
    def set_zero_position(self, motor_name: str):
        """设置电机当前位置为零点。"""
        motor = self.motors[motor_name]
        # Type 6: Set Zero Position
        # Data: Byte0=1
        data = bytes([0x01] + [0x00]*7)
        self._send_command(CommunicationType.SET_ZERO_POSITION, self.host_id, motor.id, data)

    def control_mit(self, motor_name: str, 
                    position: float, velocity: float, 
                    kp: float, kd: float, torque: float):
        """
        发送 MIT 控制指令。
        
        :param motor_name: 电机名称
        :param position: 期望位置 (rad)
        :param velocity: 期望速度 (rad/s)
        :param kp: 位置增益
        :param kd: 速度增益
        :param torque: 前馈力矩 (Nm)
        """
        motor = self.motors[motor_name]
        model = motor.model
        
        # 获取限制值
        p_limit = MODEL_MIT_POSITION_TABLE.get(model, 12.5)
        v_limit = MODEL_MIT_VELOCITY_TABLE.get(model, 50.0)
        t_limit = MODEL_MIT_TORQUE_TABLE.get(model, 60.0)
        kp_limit = MODEL_MIT_KP_TABLE.get(model, 500.0)
        kd_limit = MODEL_MIT_KD_TABLE.get(model, 5.0)
        
        # 限幅
        position = np.clip(position, -p_limit, p_limit)
        velocity = np.clip(velocity, -v_limit, v_limit)
        kp = np.clip(kp, 0, kp_limit)
        kd = np.clip(kd, 0, kd_limit)
        torque = np.clip(torque, -t_limit, t_limit)
        
        # 转换为 uint16
        # Position: [-L, L] -> [0, 65535]
        p_u16 = int(((position / p_limit) + 1.0) * 32767.0)
        p_u16 = np.clip(p_u16, 0, 65535)
        
        # Velocity: [-L, L] -> [0, 65535]
        v_u16 = int(((velocity / v_limit) + 1.0) * 32767.0)
        v_u16 = np.clip(v_u16, 0, 65535)
        
        # Kp: [0, L] -> [0, 65535]
        kp_u16 = int((kp / kp_limit) * 65535.0)
        kp_u16 = np.clip(kp_u16, 0, 65535)
        
        # Kd: [0, L] -> [0, 65535]
        kd_u16 = int((kd / kd_limit) * 65535.0)
        kd_u16 = np.clip(kd_u16, 0, 65535)
        
        # Torque: [-L, L] -> [0, 65535] (发送在 Extra Data 域)
        t_u16 = int(((torque / t_limit) + 1.0) * 32767.0)
        t_u16 = np.clip(t_u16, 0, 65535)
        
        # 打包数据 (大端序)
        data = struct.pack('>HHHH', p_u16, v_u16, kp_u16, kd_u16)
        
        # 发送
        self._send_command(CommunicationType.OPERATION_CONTROL, t_u16, motor.id, data)

    def read_parameter(self, motor_id: int, param_index: int):
        """
        发送读取参数指令 (Type 17)。
        """
        # Type 17
        # Data: Index (2B) + 00 00 + 00 00 00 00
        data = struct.pack('<H', param_index) + b'\x00\x00\x00\x00\x00\x00'
        self._send_command(CommunicationType.READ_PARAMETER, self.host_id, motor_id, data)
        
    def write_parameter(self, motor_id: int, param_index: int, value: Any):
        """
        发送写入参数指令 (Type 18)。
        """
        param_info = PARAM_TABLE.get(param_index)
        if not param_info:
            print(f"未知参数索引: {param_index}")
            return
            
        # motor_params.py format: (name, p_type, size)
        name, p_type, size = param_info
        
        fmt, _ = get_pack_format(p_type)
        if not fmt:
            print(f"不支持的参数类型: {p_type}")
            return

        # 注意：不再进行范围检查，因为 motor_params.py 中没有定义范围
        
        # 打包数据
        val_bytes = struct.pack(fmt, value)
        # 填充 val_bytes 到 4 字节
        if len(val_bytes) < 4:
            val_bytes += b'\x00' * (4 - len(val_bytes))
            
        # Index (2B) + 00 00 + Value (4B)
        data = struct.pack('<H', param_index) + b'\x00\x00' + val_bytes
            
        self._send_command(CommunicationType.WRITE_PARAMETER, self.host_id, motor_id, data)

    def save_parameters(self, motor_id: int):
        """
        保存参数到 EEPROM (Type 22)。
        """
        data = bytes([0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08])
        self._send_command(CommunicationType.SAVE_PARAMETERS, self.host_id, motor_id, data)

    def process_messages(self, max_messages=50):
        """
        从 CAN 总线读取消息并更新电机状态。
        """
        count = 0
        while count < max_messages:
            frame = self.adapter.read_can_frame()
            if not frame:
                break
                
            can_id, data, cmd, ide, rtr = frame
            
            if not ide: 
                continue # 跳过标准帧
                
            # 解析扩展 ID
            comm_type = (can_id >> 24) & 0x1F
            
            if comm_type == CommunicationType.READ_PARAMETER:
                # 解析参数读取反馈 (Type 17)
                extra_data = (can_id >> 8) & 0xFFFF
                success_flag = (extra_data >> 8) & 0xFF
                motor_id = extra_data & 0xFF
                
                if success_flag == 0: # 0 表示成功
                    if len(data) >= 8:
                        param_index = struct.unpack('<H', data[0:2])[0]
                        raw_value = data[4:8]
                        
                        param_info = PARAM_TABLE.get(param_index)
                        if param_info:
                            name, p_type, size = param_info
                            fmt, _ = get_pack_format(p_type)
                            if fmt:
                                try:
                                    # 根据类型大小解包
                                    val_size = struct.calcsize(fmt)
                                    val = struct.unpack(fmt, raw_value[:val_size])[0]
                                    self.parameter_values[(motor_id, param_index)] = val
                                    
                                    # 如果是 IQF (电流)，更新电机状态
                                    if param_index == ParamIndex.IQF:
                                        if motor_id in self.motors_by_id:
                                            self.motors_by_id[motor_id].state.current = val
                                            
                                except Exception as e:
                                    print(f"解析参数失败: {e}")
                else:
                    print(f"读取参数失败，错误码: {success_flag}")

            elif comm_type == CommunicationType.OPERATION_STATUS:
                # 处理电机反馈
                extra_data = (can_id >> 8) & 0xFFFF
                motor_id = extra_data & 0xFF
                if motor_id in self.motors_by_id:
                    motor = self.motors_by_id[motor_id]
                    self._parse_feedback(motor, data)
            
            count += 1

    def _parse_feedback(self, motor: RobStrideMotor, data: bytes):
        if len(data) < 8:
            return
            
        # 解包大端序数据
        p_u16, v_u16, t_i16, temp_u16 = struct.unpack('>HHHH', data)
        
        model = motor.model
        p_limit = MODEL_MIT_POSITION_TABLE.get(model, 12.5)
        v_limit = MODEL_MIT_VELOCITY_TABLE.get(model, 50.0)
        t_limit = MODEL_MIT_TORQUE_TABLE.get(model, 60.0)
        
        # 转换回浮点数
        pos = (float(p_u16) / 32767.0 - 1.0) * p_limit
        vel = (float(v_u16) / 32767.0 - 1.0) * v_limit
        torque = (float(t_i16) / 32767.0 - 1.0) * t_limit
        temp = float(temp_u16) * 0.1
        
        motor.update_state(pos, vel, torque, temp)
