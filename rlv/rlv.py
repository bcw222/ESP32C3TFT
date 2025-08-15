import struct
from typing import List
from dataclasses import dataclass
import numpy as np


@dataclass
class RLVHeader:
    """RLV 文件头"""
    width: int          # 视频宽度 (1 byte)
    height: int         # 视频高度 (1 byte)
    fps: int           # 帧率 (1 byte)
    frame_count: int   # 帧数 (2 bytes)
    frame_table_bits: int  # 帧表位宽 (5 bits)
    unit_bits: int     # 单元位宽 (3 bits)
    
    def __post_init__(self):
        """验证头部参数的有效性"""
        if not (0 <= self.width <= 255):
            raise ValueError(f"宽度必须在 0-255 范围内，当前值: {self.width}")
        if not (0 <= self.height <= 255):
            raise ValueError(f"高度必须在 0-255 范围内，当前值: {self.height}")
        if not (0 <= self.fps <= 255):
            raise ValueError(f"帧率必须在 0-255 范围内，当前值: {self.fps}")
        if not (0 <= self.frame_count <= 65535):
            raise ValueError(f"帧数必须在 0-65535 范围内，当前值: {self.frame_count}")
        if not (0 <= self.frame_table_bits <= 31):
            raise ValueError(f"帧表位宽必须在 0-31 范围内，当前值: {self.frame_table_bits}")
        if not (0 <= self.unit_bits <= 7):
            raise ValueError(f"单元位宽必须在 0-7 范围内，当前值: {self.unit_bits}")
    
    def to_bytes(self) -> bytes:
        """将头部转换为 6 字节的二进制数据"""
        # 前 4 个字节：宽、高、帧率、帧数
        header_bytes = bytearray()
        header_bytes.append(self.width)
        header_bytes.append(self.height)
        header_bytes.append(self.fps)
        header_bytes.extend(struct.pack('>H', self.frame_count))  # 大端序 2 字节
        
        # 第 6 字节：帧表位宽(5位) + 单元位宽(3位)
        last_byte = (self.frame_table_bits << 3) | self.unit_bits
        header_bytes.append(last_byte)
        
        return bytes(header_bytes)
    
    @classmethod
    def from_bytes(cls, data: bytes) -> 'RLVHeader':
        """从 6 字节的二进制数据解析头部"""
        if len(data) != 6:
            raise ValueError(f"头部数据必须是 6 字节，当前长度: {len(data)}")
        
        width = data[0]
        height = data[1]
        fps = data[2]
        frame_count = struct.unpack('>H', data[3:5])[0]
        
        # 解析最后一个字节
        last_byte = data[5]
        frame_table_bits = (last_byte >> 3) & 0x1F  # 取高 5 位
        unit_bits = last_byte & 0x07  # 取低 3 位
        
        return cls(width, height, fps, frame_count, frame_table_bits, unit_bits)


class BitStream:
    """位流工具类"""
    
    def __init__(self):
        self.bits = []
    
    def add_bits(self, value: int, bit_count: int):
        """添加指定位数的值到位流"""
        for i in range(bit_count):
            bit = (value >> (bit_count - 1 - i)) & 1
            self.bits.append(bit)
    
    def add_bit(self, bit: int):
        """添加单个位到位流"""
        self.bits.append(bit & 1)
    
    def to_bytes(self, align_to_8: bool = True) -> bytes:
        """将位流转换为字节数据"""
        bits = self.bits.copy()
        
        # 对齐到字节边界
        if align_to_8:
            while len(bits) % 8 != 0:
                bits.append(0)
        
        # 转换为字节
        result = bytearray()
        for i in range(0, len(bits), 8):
            byte_val = 0
            for j in range(8):
                if i + j < len(bits):
                    byte_val |= (bits[i + j] << (7 - j))
            result.append(byte_val)
        
        return bytes(result)
    
    @classmethod
    def from_bytes(cls, data: bytes) -> 'BitStream':
        """从字节数据创建位流"""
        stream = cls()
        for byte_val in data:
            for i in range(8):
                bit = (byte_val >> (7 - i)) & 1
                stream.add_bit(bit)
        return stream
    
    def read_bits(self, start_bit: int, bit_count: int) -> int:
        """从指定位置读取指定位数的值"""
        if start_bit + bit_count > len(self.bits):
            raise ValueError("读取位数超出范围")
        
        value = 0
        for i in range(bit_count):
            bit = self.bits[start_bit + i]
            value |= (bit << (bit_count - 1 - i))
        
        return value


class FrameTable:
    """帧表类"""
    
    def __init__(self, frame_table_bits: int):
        """
        初始化帧表
        
        Args:
            frame_table_bits: 帧表位宽
        """
        if frame_table_bits < 0 or frame_table_bits > 31:
            raise ValueError(f"帧表位宽必须在 0-31 范围内，当前值: {frame_table_bits}")
        
        self.frame_table_bits = frame_table_bits
        self.max_unit_count = (1 << frame_table_bits) if frame_table_bits > 0 else 1
        self.frame_starts = []
    
    def add_frame_start(self, unit_count: int):
        """添加帧的起始单元计数"""
        if unit_count >= self.max_unit_count:
            raise ValueError(f"单元计数 {unit_count} 超出帧表位宽 {self.frame_table_bits} 的表示范围")
        self.frame_starts.append(unit_count)
    
    def to_bytes(self) -> bytes:
        """将帧表转换为字节数据"""
        if self.frame_table_bits == 0:
            return b''
        
        stream = BitStream()
        for start in self.frame_starts:
            stream.add_bits(start, self.frame_table_bits)
        
        return stream.to_bytes()
    
    @classmethod
    def from_bytes(cls, data: bytes, frame_count: int, frame_table_bits: int) -> 'FrameTable':
        """从字节数据解析帧表"""
        table = cls(frame_table_bits)
        
        if frame_table_bits == 0 or frame_count == 0:
            return table
        
        stream = BitStream.from_bytes(data)
        bit_pos = 0
        
        for _ in range(frame_count):
            if bit_pos + frame_table_bits <= len(stream.bits):
                start = stream.read_bits(bit_pos, frame_table_bits)
                table.frame_starts.append(start)
                bit_pos += frame_table_bits
            else:
                break
        
        return table
    
    def get_frame_start(self, frame_index: int) -> int:
        """获取指定帧的起始单元计数"""
        if frame_index < 0 or frame_index >= len(self.frame_starts):
            raise ValueError(f"帧索引 {frame_index} 超出范围")
        return self.frame_starts[frame_index]


class RLEEncoder:
    """RLE 编码器"""
    
    def __init__(self, unit_bits: int):
        """
        初始化 RLE 编码器
        
        Args:
            unit_bits: 单元位宽（不包括颜色位）
        """
        if unit_bits < 0 or unit_bits > 7:
            raise ValueError(f"单元位宽必须在 0-7 范围内，当前值: {unit_bits}")
        
        self.unit_bits = unit_bits
        self.total_unit_bits = unit_bits + 1  # 包括颜色位
        self.max_length = (1 << unit_bits) if unit_bits > 0 else 1
    
    def encode_frame(self, frame_data: np.ndarray) -> bytes:
        """
        编码单帧数据
        
        Args:
            frame_data: 二值化的帧数据
            
        Returns:
            编码后的字节数据
        """
        if self.unit_bits == 0:
            # 禁用游程编码，直接返回像素数据
            return self._encode_raw_pixels(frame_data.flatten())
        
        return self._encode_rle(frame_data.flatten())
    
    def _encode_raw_pixels(self, pixels: np.ndarray) -> bytes:
        """编码原始像素（禁用游程编码时）"""
        stream = BitStream()
        for pixel in pixels:
            stream.add_bit(int(pixel))
        return stream.to_bytes()
    
    def _encode_rle(self, pixels: np.ndarray) -> bytes:
        """执行 RLE 编码"""
        if len(pixels) == 0:
            return b''
        
        stream = BitStream()
        current_color = int(pixels[0])
        run_length = 0
        
        for pixel in pixels:
            pixel = int(pixel)
            if pixel == current_color and run_length < self.max_length:
                run_length += 1
            else:
                # 输出当前游程
                self._add_run_to_stream(stream, current_color, run_length)
                current_color = pixel
                run_length = 1
        
        # 输出最后一个游程
        self._add_run_to_stream(stream, current_color, run_length)
        
        return stream.to_bytes()
    
    def _add_run_to_stream(self, stream: BitStream, color: int, length: int):
        """将游程添加到位流中"""
        while length > 0:
            current_length = min(length, self.max_length)
            # 创建单元：颜色位 + 长度位
            # 长度使用 0 到 max_length-1 的范围
            unit_value = (color << self.unit_bits) | (current_length - 1)
            stream.add_bits(unit_value, self.total_unit_bits)
            length -= current_length


class RLEDecoder:
    """RLE 解码器"""
    
    def __init__(self, unit_bits: int):
        """
        初始化 RLE 解码器
        
        Args:
            unit_bits: 单元位宽（不包括颜色位）
        """
        if unit_bits < 0 or unit_bits > 7:
            raise ValueError(f"单元位宽必须在 0-7 范围内，当前值: {unit_bits}")
        
        self.unit_bits = unit_bits
        self.total_unit_bits = unit_bits + 1  # 包括颜色位
        self.max_length = (1 << unit_bits) if unit_bits > 0 else 1
    
    def decode_frame(self, data: bytes, frame_size: int) -> np.ndarray:
        """
        解码单帧数据
        
        Args:
            data: 编码后的字节数据
            frame_size: 帧的像素总数
            
        Returns:
            解码后的帧数据
        """
        if self.unit_bits == 0:
            # 禁用游程编码，直接读取像素数据
            return self._decode_raw_pixels(data, frame_size)
        
        return self._decode_rle(data, frame_size)
    
    def _decode_raw_pixels(self, data: bytes, frame_size: int) -> np.ndarray:
        """解码原始像素（禁用游程编码时）"""
        stream = BitStream.from_bytes(data)
        pixels = []
        
        for i in range(min(frame_size, len(stream.bits))):
            pixels.append(stream.bits[i])
        
        # 如果数据不足，用 0 填充
        while len(pixels) < frame_size:
            pixels.append(0)
        
        return np.array(pixels[:frame_size], dtype=np.uint8)
    
    def _decode_rle(self, data: bytes, frame_size: int) -> np.ndarray:
        """执行 RLE 解码"""
        stream = BitStream.from_bytes(data)
        pixels = []
        bit_pos = 0
        
        while len(pixels) < frame_size and bit_pos + self.total_unit_bits <= len(stream.bits):
            # 读取一个单元
            unit_value = stream.read_bits(bit_pos, self.total_unit_bits)
            
            # 解析颜色和长度
            color = (unit_value >> self.unit_bits) & 1
            length = (unit_value & ((1 << self.unit_bits) - 1)) + 1
            
            # 添加像素
            for _ in range(length):
                if len(pixels) < frame_size:
                    pixels.append(color)
                else:
                    break
            
            bit_pos += self.total_unit_bits
        
        # 如果数据不足，用 0 填充
        while len(pixels) < frame_size:
            pixels.append(0)
        
        return np.array(pixels[:frame_size], dtype=np.uint8)


class RLVFile:
    """RLV 文件处理类"""
    
    def __init__(self, header: RLVHeader):
        """
        初始化 RLV 文件
        
        Args:
            header: RLV 文件头部
        """
        self.header = header
        self.frame_table = FrameTable(header.frame_table_bits)
        self.frame_data = []  # 存储每帧的编码数据
        self.encoder = RLEEncoder(header.unit_bits)
        self.decoder = RLEDecoder(header.unit_bits)
    
    def add_frame(self, frame: np.ndarray):
        """
        添加一帧到 RLV 文件
        
        Args:
            frame: 二值化的帧数据，形状应为 (height, width)
        """
        if frame.shape != (self.header.height, self.header.width):
            raise ValueError(f"帧尺寸 {frame.shape} 与头部定义的尺寸 ({self.header.height}, {self.header.width}) 不匹配")
        
        # 计算当前的单元计数（用于帧表）
        current_unit_count = self._calculate_current_unit_count()
        self.frame_table.add_frame_start(current_unit_count)
        
        # 编码帧数据
        encoded_frame = self.encoder.encode_frame(frame)
        self.frame_data.append(encoded_frame)
    
    def _calculate_current_unit_count(self) -> int:
        """计算当前已编码数据的单元总数"""
        if self.header.unit_bits == 0:
            # 禁用游程编码时，每个像素就是一个单元
            total_pixels = sum(len(data) * 8 for data in self.frame_data)
            return total_pixels
        else:
            # 启用游程编码时，计算单元数
            total_bits = sum(len(data) * 8 for data in self.frame_data)
            return total_bits // (self.header.unit_bits + 1)
    
    def to_bytes(self) -> bytes:
        """将 RLV 文件转换为字节数据"""
        result = bytearray()
        
        # 1. 添加头部 (6 bytes)
        result.extend(self.header.to_bytes())
        
        # 2. 添加帧表
        frame_table_data = self.frame_table.to_bytes()
        result.extend(frame_table_data)
        
        # 3. 添加帧数据
        for frame_data in self.frame_data:
            result.extend(frame_data)
        
        # 4. 添加填充以对齐到 8 字节
        while len(result) % 8 != 0:
            result.append(0)
        
        return bytes(result)
    
    def save_to_file(self, filepath: str):
        """保存 RLV 文件到磁盘"""
        with open(filepath, 'wb') as f:
            f.write(self.to_bytes())
    
    @classmethod
    def from_bytes(cls, data: bytes) -> 'RLVFile':
        """从字节数据解析 RLV 文件"""
        if len(data) < 6:
            raise ValueError("数据太短，无法包含有效的 RLV 头部")
        
        # 1. 解析头部
        header = RLVHeader.from_bytes(data[:6])
        rlv_file = cls(header)
        
        # 2. 计算帧表大小
        frame_table_bits_total = header.frame_count * header.frame_table_bits
        frame_table_bytes = (frame_table_bits_total + 7) // 8  # 向上取整
        
        if len(data) < 6 + frame_table_bytes:
            raise ValueError("数据太短，无法包含完整的帧表")
        
        # 3. 解析帧表
        frame_table_data = data[6:6 + frame_table_bytes]
        rlv_file.frame_table = FrameTable.from_bytes(
            frame_table_data, header.frame_count, header.frame_table_bits
        )
        
        # 4. 解析帧数据
        data_start = 6 + frame_table_bytes
        remaining_data = data[data_start:]
        
        rlv_file._parse_frame_data(remaining_data)
        
        return rlv_file
    
    def _parse_frame_data(self, data: bytes):
        """解析帧数据部分"""
        frame_size = self.header.width * self.header.height
        
        if self.header.unit_bits == 0:
            # 禁用游程编码，直接按像素解析
            self._parse_raw_frame_data(data, frame_size)
        else:
            # 启用游程编码，按单元解析
            self._parse_rle_frame_data(data, frame_size)
    
    def _parse_raw_frame_data(self, data: bytes, frame_size: int):
        """解析原始像素数据（禁用游程编码时）"""
        stream = BitStream.from_bytes(data)
        bit_pos = 0
        
        for frame_idx in range(self.header.frame_count):
            frame_bits = stream.bits[bit_pos:bit_pos + frame_size]
            if len(frame_bits) < frame_size:
                # 数据不足，用 0 填充
                frame_bits.extend([0] * (frame_size - len(frame_bits)))
            
            # 将位转换为字节数据
            frame_bytes = bytearray()
            for i in range(0, len(frame_bits), 8):
                byte_val = 0
                for j in range(8):
                    if i + j < len(frame_bits):
                        byte_val |= (frame_bits[i + j] << (7 - j))
                frame_bytes.append(byte_val)
            
            self.frame_data.append(bytes(frame_bytes))
            bit_pos += frame_size
    
    def _parse_rle_frame_data(self, data: bytes, frame_size: int):
        """解析 RLE 编码的帧数据"""
        stream = BitStream.from_bytes(data)
        unit_bits_total = self.header.unit_bits + 1
        
        for frame_idx in range(self.header.frame_count):
            # 获取当前帧的起始单元位置
            if frame_idx < len(self.frame_table.frame_starts):
                start_unit = self.frame_table.frame_starts[frame_idx]
            else:
                break
            
            # 计算下一帧的起始位置
            if frame_idx + 1 < len(self.frame_table.frame_starts):
                end_unit = self.frame_table.frame_starts[frame_idx + 1]
            else:
                # 最后一帧，计算到数据末尾
                end_unit = len(stream.bits) // unit_bits_total
            
            # 提取当前帧的数据
            start_bit = start_unit * unit_bits_total
            end_bit = end_unit * unit_bits_total
            
            frame_bits = stream.bits[start_bit:end_bit]
            
            # 将位转换为字节数据
            frame_bytes = bytearray()
            for i in range(0, len(frame_bits), 8):
                byte_val = 0
                for j in range(8):
                    if i + j < len(frame_bits):
                        byte_val |= (frame_bits[i + j] << (7 - j))
                frame_bytes.append(byte_val)
            
            self.frame_data.append(bytes(frame_bytes))
    
    @classmethod
    def load_from_file(cls, filepath: str) -> 'RLVFile':
        """从文件加载 RLV 数据"""
        with open(filepath, 'rb') as f:
            data = f.read()
        return cls.from_bytes(data)
    
    def get_frame(self, frame_index: int) -> np.ndarray:
        """
        获取指定索引的帧
        
        Args:
            frame_index: 帧索引
            
        Returns:
            解码后的帧数据，形状为 (height, width)
        """
        if frame_index < 0 or frame_index >= len(self.frame_data):
            raise ValueError(f"帧索引 {frame_index} 超出范围 [0, {len(self.frame_data)})")
        
        frame_size = self.header.width * self.header.height
        frame_data = self.decoder.decode_frame(self.frame_data[frame_index], frame_size)
        
        return frame_data.reshape(self.header.height, self.header.width)
    
    def get_all_frames(self) -> List[np.ndarray]:
        """获取所有帧"""
        frames = []
        for i in range(len(self.frame_data)):
            frames.append(self.get_frame(i))
        return frames
    
    def get_info(self) -> dict:
        """获取 RLV 文件信息"""
        return {
            'width': self.header.width,
            'height': self.header.height,
            'fps': self.header.fps,
            'frame_count': self.header.frame_count,
            'frame_table_bits': self.header.frame_table_bits,
            'unit_bits': self.header.unit_bits,
            'total_unit_bits': self.header.unit_bits + 1,
            'actual_frame_count': len(self.frame_data),
            'file_size_estimate': len(self.to_bytes())
        }


class RLVBuilder:
    """RLV 文件构建器，用于快速创建 RLV 文件"""
    
    def __init__(self, width: int, height: int, fps: int, unit_bits: int):
        """
        初始化 RLV 构建器
        
        Args:
            width: 视频宽度
            height: 视频高度
            fps: 帧率
            unit_bits: 单元位宽
        """
        self.width = width
        self.height = height
        self.fps = fps
        self.unit_bits = unit_bits
        self.frames = []
    
    def add_frame(self, frame: np.ndarray):
        """添加一帧"""
        if frame.shape != (self.height, self.width):
            raise ValueError(f"帧尺寸 {frame.shape} 与期望尺寸 ({self.height}, {self.width}) 不匹配")
        
        # 确保是二值化数据
        binary_frame = (frame > 0).astype(np.uint8)
        self.frames.append(binary_frame)
    
    def build(self) -> RLVFile:
        """构建 RLV 文件"""
        if not self.frames:
            raise ValueError("没有添加任何帧")
        
        frame_count = len(self.frames)
        
        # 估算需要的帧表位宽
        frame_table_bits = self._estimate_frame_table_bits()
        
        # 创建头部
        header = RLVHeader(
            width=self.width,
            height=self.height,
            fps=self.fps,
            frame_count=frame_count,
            frame_table_bits=frame_table_bits,
            unit_bits=self.unit_bits
        )
        
        # 创建 RLV 文件并添加帧
        rlv_file = RLVFile(header)
        for frame in self.frames:
            rlv_file.add_frame(frame)
        
        return rlv_file
    
    def _estimate_frame_table_bits(self) -> int:
        """估算需要的帧表位宽"""
        if not self.frames:
            return 1
        
        # 实际计算每帧的单元数来获得准确的估算
        total_units = 0
        encoder = RLEEncoder(self.unit_bits)
        
        for frame in self.frames:
            if self.unit_bits == 0:
                # 禁用游程编码，每个像素一个单元
                frame_units = self.width * self.height
            else:
                # 启用游程编码，实际编码来计算单元数
                encoded = encoder.encode_frame(frame)
                frame_units = len(encoded) * 8 // (self.unit_bits + 1)
            
            total_units += frame_units
        
        # 计算需要的位数，确保能表示最大的单元计数
        if total_units <= 1:
            return 1
        
        import math
        # 添加一些余量以确保安全
        required_bits = math.ceil(math.log2(total_units + 1))
        return min(31, max(1, required_bits))
    
    def save_to_file(self, filepath: str):
        """直接保存到文件"""
        rlv_file = self.build()
        rlv_file.save_to_file(filepath)