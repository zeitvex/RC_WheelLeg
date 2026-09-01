import serial
import struct
import time
from typing import Optional, Tuple

class DmUsbAdapter:
    """
    达妙 USB 转 CAN 适配器驱动。
    处理底层串口通信和帧的封装/解包。
    """
    
    # 帧常量
    SEND_HEADER = b'\x55\xAA'
    SEND_FRAME_LEN = 30
    RECV_HEADER = 0xAA
    RECV_TAIL = 0x55
    RECV_FRAME_LEN = 16
    
    def __init__(self, port: str, baudrate: int = 921600, timeout: float = 0.01, debug: bool = False):
        """
        初始化 USB 转 CAN 适配器。
        
        :param port: 串口名称 (例如 "COM3")
        :param baudrate: 串口波特率 (默认 921600)
        :param timeout: 读取超时时间 (秒)
        :param debug: 是否打印调试信息
        """
        self.serial = serial.Serial()
        self.serial.port = port
        self.serial.baudrate = baudrate
        self.serial.timeout = timeout
        self.data_buffer = bytearray()
        self.debug = debug

    def open(self):
        """打开串口连接。"""
        if not self.serial.is_open:
            try:
                self.serial.open()
                if self.debug:
                    print(f"[DEBUG] 串口 {self.serial.port} 已打开")
            except Exception as e:
                print(f"[ERROR] 无法打开串口 {self.serial.port}: {e}")
                raise

    def close(self):
        """关闭串口连接。"""
        if self.serial.is_open:
            self.serial.close()
            if self.debug:
                print(f"[DEBUG] 串口 {self.serial.port} 已关闭")

    def set_can_baudrate(self, index: int = 0):
        """
        设置 CAN 波特率。
        
        索引对照表:
        0: 1000 kbps
        1: 800 kbps
        2: 666 kbps
        3: 500 kbps
        ...
        63: 
        
        :param index: 波特率索引 (默认 0, 即 1000kbps)
        """
        # 构建设置波特率指令: 55 05 Index(1byte) AA 55
        cmd = bytearray([0x55, 0x05, index & 0xFF, 0xAA, 0x55])
        self.serial.write(cmd)
        if self.debug:
            print(f"[DEBUG] 发送设置波特率指令: {cmd.hex()}")
        time.sleep(0.1) # 等待生效

    def send_can_frame(self, can_id: int, data: bytes, 
                       extended: bool = True, remote: bool = False, 
                       feedback: bool = False) -> None:
        """
        发送 CAN 帧。
        
        :param can_id: CAN 标识符 (标准帧或扩展帧)
        :param data: 数据负载 (最多 8 字节)
        :param extended: True 为扩展帧 (29位), False 为标准帧 (11位)
        :param remote: True 为远程帧, False 为数据帧
        :param feedback: True 请求设备反馈 (CMD 0x01), False 不反馈 (CMD 0x03)
        """
        if len(data) > 8:
            raise ValueError("CAN 数据不能超过 8 字节")
            
        # 填充数据到 8 字节
        data_padded = data + b'\x00' * (8 - len(data))
        
        cmd = 0x01 if feedback else 0x03
        send_count = 1
        interval = 10 # 默认 10ms
        id_type = 1 if extended else 0
        frame_type = 1 if remote else 0
        data_len = len(data)
        
        # 构建帧 (30 字节)
        frame = bytearray(30)
        frame[0] = 0x55
        frame[1] = 0xAA
        frame[2] = 0x1E # 长度
        frame[3] = cmd
        
        # 发送次数 (4 字节, 小端序)
        frame[4:8] = struct.pack('<I', send_count)
        
        # 时间间隔 (4 字节, 小端序)
        frame[8:12] = struct.pack('<I', interval)
        
        frame[12] = id_type
        
        # CAN ID (4 字节, 小端序)
        frame[13:17] = struct.pack('<I', can_id)
        
        frame[17] = frame_type
        frame[18] = data_len
        # 19, 20 为保留位 0
        
        frame[21:29] = data_padded
        frame[29] = 0x00 # CRC (任意值)
        
        self.serial.write(frame)
        
        if self.debug:
            print(f"[DEBUG] 发送帧: ID=0x{can_id:08X} Data={data.hex()} Raw={frame.hex()}")

    def read_can_frame(self) -> Optional[Tuple[int, bytes, int, bool, bool]]:
        """
        如果缓冲区中有可用数据，读取一帧 CAN 数据。
        
        :return: 元组 (can_id, data, cmd, extended, remote) 或者 None (如果没有完整帧)
        """
        # 读取可用数据
        if self.serial.in_waiting:
            raw_data = self.serial.read(self.serial.in_waiting)
            self.data_buffer.extend(raw_data)
            
        # 检查完整帧 (16 字节)
        while len(self.data_buffer) >= self.RECV_FRAME_LEN:
            # 查找帧头
            try:
                header_idx = self.data_buffer.index(self.RECV_HEADER)
            except ValueError:
                # 没有找到帧头，清空缓冲区（保留最后几个字节以防截断）
                self.data_buffer = self.data_buffer[-(self.RECV_FRAME_LEN-1):]
                return None
                
            # 检查从帧头开始是否有足够字节
            if len(self.data_buffer) - header_idx < self.RECV_FRAME_LEN:
                # 保留从帧头开始的数据
                self.data_buffer = self.data_buffer[header_idx:]
                return None
                
            # 检查帧尾
            if self.data_buffer[header_idx + self.RECV_FRAME_LEN - 1] != self.RECV_TAIL:
                # 无效帧，跳过该帧头继续查找
                self.data_buffer = self.data_buffer[header_idx + 1:]
                continue
                
            # 提取有效帧
            frame = self.data_buffer[header_idx : header_idx + self.RECV_FRAME_LEN]
            self.data_buffer = self.data_buffer[header_idx + self.RECV_FRAME_LEN:]
            
            if self.debug:
                print(f"[DEBUG] 解析帧: {frame.hex()}")

            # 解析帧
            cmd = frame[1]
            format_byte = frame[2]
            
            data_len = format_byte & 0x3F
            ide = bool((format_byte >> 6) & 0x01)
            rtr = bool((format_byte >> 7) & 0x01)
            
            can_id = struct.unpack('<I', frame[3:7])[0]
            data = bytes(frame[7:15])
            
            if data_len < 8:
                data = data[:data_len]
                
            return (can_id, data, cmd, ide, rtr)
            
        return None
