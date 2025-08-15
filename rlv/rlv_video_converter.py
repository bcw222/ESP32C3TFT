import cv2
import numpy as np
import argparse
import sys
import os
import time
from typing import List, Tuple, Dict, Optional
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import dataclass
from rlv import RLVBuilder, RLVFile


@dataclass
class ConversionResult:
    """转换结果数据类"""
    unit_bits: int
    file_size: int
    compression_ratio: float
    processing_time: float
    output_path: str
    rlv_file: Optional[RLVFile] = None


class VideoConverter:
    """视频到 RLV 格式转换器"""
    
    def __init__(self, target_width: int = 240, target_height: int = 240):
        """
        初始化视频转换器
        
        Args:
            target_width: 目标视频宽度
            target_height: 目标视频高度
        """
        self.target_width = target_width
        self.target_height = target_height
    
    def convert_video_to_rlv(
        self, 
        video_path: str, 
        output_path: str, 
        fps: int = 8, 
        unit_bits: int = 7,
        threshold: int = 128,
        verbose: bool = True
    ) -> RLVFile:
        """
        将视频文件转换为 RLV 格式
        
        Args:
            video_path: 输入视频文件路径
            output_path: 输出 RLV 文件路径
            fps: 目标帧率
            unit_bits: RLE 单元位宽
            threshold: 二值化阈值
            verbose: 是否显示详细信息
            
        Returns:
            生成的 RLV 文件对象
        """
        if verbose:
            print(f"开始转换视频: {video_path}")
            print(f"目标尺寸: {self.target_width}x{self.target_height}")
            print(f"目标帧率: {fps} FPS")
            print(f"单元位宽: {unit_bits}")
            print(f"二值化阈值: {threshold}")
        
        # 提取视频帧
        frames = self._extract_frames(video_path, fps, threshold, verbose)
        
        if not frames:
            raise ValueError("未能从视频中提取到任何帧")
        
        # 创建 RLV 构建器
        builder = RLVBuilder(
            width=self.target_width,
            height=self.target_height,
            fps=fps,
            unit_bits=unit_bits
        )
        
        # 添加所有帧
        if verbose:
            print("正在编码帧数据...")
        
        for i, frame in enumerate(frames):
            builder.add_frame(frame)
            if verbose and (i + 1) % 100 == 0:
                print(f"已编码 {i + 1}/{len(frames)} 帧")
        print(f"编码完成: 编码了 {len(frames)} 帧")
        
        # 构建并保存 RLV 文件
        rlv_file = builder.build()
        rlv_file.save_to_file(output_path)
        
        if verbose:
            print(f"RLV 文件已保存到: {output_path}")
            info = rlv_file.get_info()
            print(f"文件大小: {info['file_size_estimate']} 字节")
            print(f"实际帧数: {info['actual_frame_count']}")
        
        return rlv_file
    
    def _extract_frames(
        self, 
        video_path: str, 
        target_fps: int, 
        threshold: int,
        verbose: bool = True
    ) -> List[np.ndarray]:
        """
        从视频文件中提取并处理帧
        
        Args:
            video_path: 视频文件路径
            target_fps: 目标帧率
            threshold: 二值化阈值
            verbose: 是否显示详细信息
            
        Returns:
            处理后的帧列表
        """
        cap = cv2.VideoCapture(video_path)
        if not cap.isOpened():
            raise ValueError(f"无法打开视频文件: {video_path}")
        
        try:
            original_fps = cap.get(cv2.CAP_PROP_FPS)
            total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
            frame_interval = max(1, int(original_fps / target_fps))
            
            frames = []
            frame_count = 0
            
            if verbose:
                print(f"原始帧率: {original_fps:.2f} FPS")
                print(f"总帧数: {total_frames}")
                print(f"帧间隔: {frame_interval}")
                print("正在提取帧...")
            
            while True:
                ret, frame = cap.read()
                if not ret:
                    break
                
                # 按间隔采样帧
                if frame_count % frame_interval == 0:
                    processed_frame = self._process_frame(frame, threshold)
                    frames.append(processed_frame)
                
                frame_count += 1
                
                # 显示进度
                if verbose and frame_count % 500 == 0:
                    progress = (frame_count / total_frames) * 100 if total_frames > 0 else 0
                    print(f"进度: {progress:.1f}% ({frame_count}/{total_frames}), 已提取: {len(frames)} 帧")
            
            if verbose:
                print(f"提取完成: 处理了 {frame_count} 帧，提取了 {len(frames)} 帧")
            
            return frames
            
        finally:
            cap.release()
    
    def _process_frame(self, frame: np.ndarray, threshold: int) -> np.ndarray:
        """
        处理单个帧：灰度化、缩放、二值化
        
        Args:
            frame: 原始帧
            threshold: 二值化阈值
            
        Returns:
            处理后的二值化帧
        """
        # 转换为灰度图
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # 缩放到目标尺寸
        resized = cv2.resize(
            gray, 
            (self.target_width, self.target_height), 
            interpolation=cv2.INTER_AREA
        )
        
        # 二值化
        _, binary = cv2.threshold(resized, threshold, 1, cv2.THRESH_BINARY)
        
        return binary.astype(np.uint8)


def convert_with_unit_bits(args_tuple) -> ConversionResult:
    """
    使用指定位宽转换视频的工作函数（用于并行处理）
    
    Args:
        args_tuple: 包含转换参数的元组
        
    Returns:
        转换结果
    """
    (video_path, output_dir, width, height, fps, unit_bits, threshold, base_name) = args_tuple
    
    start_time = time.time()
    
    # 创建输出文件名
    output_path = os.path.join(output_dir, f"{base_name}_bits{unit_bits}.rlv")
    
    try:
        # 创建转换器
        converter = VideoConverter(width, height)
        
        # 转换视频
        rlv_file = converter.convert_video_to_rlv(
            video_path=video_path,
            output_path=output_path,
            fps=fps,
            unit_bits=unit_bits,
            threshold=threshold,
            verbose=False  # 在并行处理中关闭详细输出
        )
        
        # 计算文件大小和压缩比
        file_size = os.path.getsize(output_path)
        original_size = width * height * len(rlv_file.get_all_frames())  # 原始像素数
        compression_ratio = original_size / file_size if file_size > 0 else 0
        
        processing_time = time.time() - start_time
        
        return ConversionResult(
            unit_bits=unit_bits,
            file_size=file_size,
            compression_ratio=compression_ratio,
            processing_time=processing_time,
            output_path=output_path,
            rlv_file=rlv_file
        )
        
    except Exception as e:
        processing_time = time.time() - start_time
        print(f"位宽 {unit_bits} 转换失败: {e}")
        return ConversionResult(
            unit_bits=unit_bits,
            file_size=0,
            compression_ratio=0,
            processing_time=processing_time,
            output_path=output_path,
            rlv_file=None
        )


class ParallelVideoConverter:
    """并行视频转换器"""
    
    def __init__(self, max_workers: Optional[int] = None):
        """
        初始化并行转换器
        
        Args:
            max_workers: 最大工作进程数，None 表示使用 CPU 核心数
        """
        self.max_workers = max_workers
    
    def convert_with_multiple_unit_bits(
        self,
        video_path: str,
        output_dir: str,
        unit_bits_range: List[int],
        width: int = 240,
        height: int = 240,
        fps: int = 8,
        threshold: int = 128,
        auto_select_best: bool = True
    ) -> Dict[int, ConversionResult]:
        """
        使用多个位宽并行转换视频
        
        Args:
            video_path: 输入视频路径
            output_dir: 输出目录
            unit_bits_range: 要测试的位宽列表
            width: 目标宽度
            height: 目标高度
            fps: 目标帧率
            threshold: 二值化阈值
            auto_select_best: 是否自动选择最佳结果
            
        Returns:
            位宽到转换结果的映射
        """
        # 确保输出目录存在
        os.makedirs(output_dir, exist_ok=True)
        
        # 获取基础文件名
        base_name = os.path.splitext(os.path.basename(video_path))[0]
        
        print(f"开始并行转换视频: {video_path}")
        print(f"测试位宽: {unit_bits_range}")
        print(f"目标尺寸: {width}x{height}")
        print(f"使用 {self.max_workers or '自动'} 个工作进程")
        print("-" * 60)
        
        # 准备参数
        args_list = [
            (video_path, output_dir, width, height, fps, unit_bits, threshold, base_name)
            for unit_bits in unit_bits_range
        ]
        
        results = {}
        
        # 并行处理
        with ProcessPoolExecutor(max_workers=self.max_workers) as executor:
            # 提交所有任务
            future_to_bits = {
                executor.submit(convert_with_unit_bits, args): args[5] 
                for args in args_list
            }
            
            # 收集结果
            for future in as_completed(future_to_bits):
                unit_bits = future_to_bits[future]
                try:
                    result = future.result()
                    results[unit_bits] = result
                    
                    if result.rlv_file is not None:
                        print(f"✓ 位宽 {unit_bits:2d}: "
                              f"文件大小 {result.file_size:8d} 字节, "
                              f"压缩比 {result.compression_ratio:6.2f}x, "
                              f"用时 {result.processing_time:5.1f}s")
                    else:
                        print(f"✗ 位宽 {unit_bits:2d}: 转换失败")
                        
                except Exception as e:
                    print(f"✗ 位宽 {unit_bits:2d}: 处理异常 - {e}")
        
        print("-" * 60)
        
        # 显示结果摘要
        self._print_results_summary(results)
        
        # 自动选择最佳结果
        if auto_select_best and results:
            best_result = self._select_best_result(results)
            if best_result:
                self._create_best_result_copy(best_result, output_dir, base_name)
        
        return results
    
    def _print_results_summary(self, results: Dict[int, ConversionResult]):
        """打印结果摘要"""
        successful_results = {k: v for k, v in results.items() if v.rlv_file is not None}
        
        if not successful_results:
            print("没有成功的转换结果")
            return
        
        print("转换结果摘要:")
        print(f"{'位宽':<6} {'文件大小':<12} {'压缩比':<10} {'处理时间':<10}")
        print("-" * 50)
        
        # 按文件大小排序
        sorted_results = sorted(successful_results.items(), key=lambda x: x[1].file_size)
        
        for unit_bits, result in sorted_results:
            print(f"{unit_bits:<6} {result.file_size:<12} {result.compression_ratio:<10.2f} {result.processing_time:<10.1f}s")
    
    def _select_best_result(self, results: Dict[int, ConversionResult]) -> Optional[ConversionResult]:
        """
        选择最佳结果（文件大小最小的成功结果）
        
        Args:
            results: 转换结果字典
            
        Returns:
            最佳结果，如果没有成功结果则返回 None
        """
        successful_results = [r for r in results.values() if r.rlv_file is not None]
        
        if not successful_results:
            return None
        
        # 选择文件大小最小的结果
        best_result = min(successful_results, key=lambda x: x.file_size)
        
        print(f"\n推荐使用位宽 {best_result.unit_bits} (文件大小: {best_result.file_size} 字节)")
        
        return best_result
    
    def _create_best_result_copy(self, best_result: ConversionResult, output_dir: str, base_name: str):
        """创建最佳结果的副本"""
        best_output_path = os.path.join(output_dir, f"{base_name}_best.rlv")
        
        try:
            import shutil
            shutil.copy2(best_result.output_path, best_output_path)
            print(f"最佳结果已复制到: {best_output_path}")
        except Exception as e:
            print(f"复制最佳结果失败: {e}")


class RLVValidator:
    """RLV 文件验证器"""
    
    @staticmethod
    def validate_rlv_file(rlv_file: RLVFile, original_frames: List[np.ndarray]) -> bool:
        """
        验证 RLV 文件的正确性
        
        Args:
            rlv_file: RLV 文件对象
            original_frames: 原始帧列表
            
        Returns:
            验证是否通过
        """
        try:
            # 获取解码后的帧
            decoded_frames = rlv_file.get_all_frames()
            
            # 检查帧数
            if len(decoded_frames) != len(original_frames):
                print(f"帧数不匹配: 原始 {len(original_frames)}, 解码 {len(decoded_frames)}")
                return False
            
            # 逐帧比较
            for i, (orig, decoded) in enumerate(zip(original_frames, decoded_frames)):
                if not np.array_equal(orig, decoded):
                    print(f"第 {i} 帧不匹配")
                    return False
            
            print("RLV 文件验证通过：所有帧都与原始帧一致")
            return True
            
        except Exception as e:
            print(f"验证过程中出现错误: {e}")
            return False


class VideoReconstructor:
    """视频重建器，将 RLV 文件转换回视频"""
    
    @staticmethod
    def rlv_to_video(
        rlv_file: RLVFile, 
        output_video_path: str, 
        scale_factor: int = 1
    ):
        """
        将 RLV 文件转换为视频文件
        
        Args:
            rlv_file: RLV 文件对象
            output_video_path: 输出视频路径
            scale_factor: 缩放因子
        """
        frames = rlv_file.get_all_frames()
        if not frames:
            raise ValueError("RLV 文件中没有帧数据")
        
        # 获取视频参数
        height, width = frames[0].shape
        fps = rlv_file.header.fps
        
        # 应用缩放因子
        if scale_factor > 1:
            width *= scale_factor
            height *= scale_factor
        
        # 创建视频写入器
        fourcc = cv2.VideoWriter.fourcc(*'MJPG')
        out = cv2.VideoWriter(output_video_path, fourcc, fps, (width, height), isColor=False)
        
        try:
            for frame in frames:
                # 将二值图像转换为 0-255 范围的灰度图像
                gray_frame = (frame * 255).astype(np.uint8)
                
                # 应用缩放
                if scale_factor > 1:
                    gray_frame = cv2.resize(
                        gray_frame, 
                        (width, height), 
                        interpolation=cv2.INTER_NEAREST
                    )
                
                out.write(gray_frame)
            
            print(f"视频已保存到: {output_video_path}")
            
        finally:
            out.release()


def main():
    """命令行入口点"""
    parser = argparse.ArgumentParser(
        description="RLV 视频转换器 - 将视频转换为 RLV 格式",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例用法:
  # 基本转换
  python rlv_video_converter.py input.mp4 --width 240 --height 240 --fps 10 -o output.rlv
  
  # 并行测试多个位宽
  python rlv_video_converter.py input.mp4 --width 240 --height 240 --fps 10 -o output_dir --parallel-bits 0 1 2 3 4 5 6 7
  
  # 转换回视频
  python rlv_video_converter.py --rlv-to-video input.rlv -o output.avi
        """
    )
    
    # 输入文件
    parser.add_argument("input", help="输入视频文件路径或 RLV 文件路径")
    
    # 输出选项
    parser.add_argument("-o", "--output", required=True, 
                       help="输出文件路径或目录（并行模式时）")
    
    # 转换模式
    parser.add_argument("--rlv-to-video", action="store_true",
                       help="将 RLV 文件转换为视频")
    
    # 并行处理选项
    parser.add_argument("--parallel-bits", type=int, nargs="+", metavar="BITS",
                       help="启用并行处理并指定要测试的单元位宽列表 (例如: --parallel-bits 3 4 5 6 7)")
    
    parser.add_argument("--unit-bits", type=int, default=7,
                       help="单一转换模式下的单元位宽 (默认: 7)")
    
    parser.add_argument("--workers", type=int, default=None,
                       help="并行工作进程数 (默认: CPU 核心数)")
    
    parser.add_argument("--no-auto-select", action="store_true",
                       help="不自动选择最佳结果")
    
    # 视频参数
    parser.add_argument("--width", type=int, default=240,
                       help="目标视频宽度 (默认: 240)")
    
    parser.add_argument("--height", type=int, default=240,
                       help="目标视频高度 (默认: 240)")
    
    parser.add_argument("--fps", type=int, default=8,
                       help="目标帧率 (默认: 8)")
    
    parser.add_argument("--threshold", type=int, default=128,
                       help="二值化阈值 (默认: 128)")
    
    # 其他选项
    parser.add_argument("--scale", type=int, default=1,
                       help="视频重建时的缩放因子 (默认: 1)")
    
    parser.add_argument("--validate", action="store_true",
                       help="验证转换结果的正确性")
    
    args = parser.parse_args()
    
    try:
        # RLV 转视频模式
        if args.rlv_to_video:
            print(f"将 RLV 文件转换为视频: {args.input} -> {args.output}")
            rlv_file = RLVFile.load_from_file(args.input)
            VideoReconstructor.rlv_to_video(rlv_file, args.output, args.scale)
            return
        
        # 检查输入文件
        if not os.path.exists(args.input):
            print(f"错误: 输入文件不存在: {args.input}")
            sys.exit(1)
        
        # 并行处理模式
        if args.parallel_bits:
            converter = ParallelVideoConverter(max_workers=args.workers)
            results = converter.convert_with_multiple_unit_bits(
                video_path=args.input,
                output_dir=args.output,
                unit_bits_range=args.parallel_bits,
                width=args.width,
                height=args.height,
                fps=args.fps,
                threshold=args.threshold,
                auto_select_best=not args.no_auto_select
            )
            
            # 显示最终统计
            successful_count = sum(1 for r in results.values() if r.rlv_file is not None)
            print(f"\n转换完成: {successful_count}/{len(results)} 个位宽转换成功")
            
        else:
            # 单一转换模式
            unit_bits = args.unit_bits
            
            converter = VideoConverter(args.width, args.height)
            rlv_file = converter.convert_video_to_rlv(
                video_path=args.input,
                output_path=args.output,
                fps=args.fps,
                unit_bits=unit_bits,
                threshold=args.threshold
            )
            
            # 验证结果
            if args.validate:
                print("\n正在验证转换结果...")
                # 重新提取原始帧进行验证
                original_frames = converter._extract_frames(args.input, args.fps, args.threshold, False)
                is_valid = RLVValidator.validate_rlv_file(rlv_file, original_frames)
                if not is_valid:
                    print("警告: 验证失败，转换结果可能有问题")
    
    except KeyboardInterrupt:
        print("\n用户中断操作")
        sys.exit(1)
    except Exception as e:
        print(f"错误: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
