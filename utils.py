def join_path(dir_path, file_name):
    # 简单的路径拼接处理，适用于 MicroPython 环境
    if dir_path.endswith('/'):
        return dir_path + file_name
    else:
        return dir_path + '/' + file_name

import os
def find_files(ext, directory):
    files = []
    
    try:
        # 列出目录中的所有文件和目录
        entries = os.listdir(directory)
        
        for entry in entries:
            path = join_path(directory, entry)
            
            # 如果是目录，递归查找
            if os.stat(path)[0] & 0x4000:  # 0x4000 是目录的文件属性标志
                files.extend(find_files(ext, path))
            # 如果是文件，检查扩展名
            elif entry.lower().endswith(ext):
                files.append(path)
    
    except OSError as e:
        print(f"Error accessing {directory}: {e}")
    
    return files


from micropython import schedule
from machine import Pin, Timer
import time
class ClickDetector:
    def __init__(self, pin, timer_id, callback=None, max_clicks=3, click_timeout=300, debounce_time=0, scheduled=True):
        # 初始配置
        self.pin = pin
        self.timer = Timer(timer_id)
        self.callback = callback if callback else lambda x: print(f"{x} clicks detected on {self.pin}")
        self.click_timeout = click_timeout
        self.debounce_time = debounce_time
        self.max_clicks = max_clicks
        self.scheduled = scheduled
        
        # 状态变量
        if self.debounce_time > 0:
            self.last_press_time = 0
        self.current_clicks = 0
        self.pending = False
        
        # 配置中断
        self.pin.irq(trigger=Pin.IRQ_FALLING, handler=self._irq_handler)
    
    def __del__(self):
        self.timer.deinit()
        self.pin.irq(handler=None)
        
    def _irq_handler(self, pin):
        if self.scheduled:
            schedule(self._process_press, None)
        else:
            self._process_press(None)
    
    def _process_press(self, _):
        if self.debounce_time > 0:  # 开启消抖
            current_time = time.ticks_ms()
            if time.ticks_diff(current_time, self.last_press_time) <= self.debounce_time:  # 消抖
                return
            self.last_press_time = current_time
        
        self.timer.deinit()  # 停止定时器, 防止误触发
        
        if self.pending:  # 是连击
            self.current_clicks += 1
            if self.current_clicks >= self.max_clicks:  # 达到最大连击数, 结束连击
                self._clicks_done(None)
        else:  # 已离开上一个连击，开始新连击
            self.pending = True
            self.current_clicks = 1
        
        if self.pending:
            self.timer.init(period=self.click_timeout, mode=Timer.ONE_SHOT, callback=self._clicks_done)
    
    def _clicks_done(self, _):  # 结束连击, 回调并重置
        self.pending = False
        current_clicks = self.current_clicks
        self.current_clicks = 0
        self.timer.deinit()  # 释放定时器
        if self.scheduled:
            schedule(self.callback, current_clicks)
        else:
            self.callback(current_clicks)