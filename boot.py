# 系统入口（固件按文件名执行 boot.py，必须保持源码形态）。
# 本文件负责全部设备初始化：无线电静默 → ADXL345 上电配置 →
# 共享 SPI 总线 + 显示屏 → 组装 Board 句柄 → 进入应用。
import machine

import lib.board as board

# ---- 无线电下电静默（应用需要时再激活）----
import network
wlan = network.WLAN(network.STA_IF)
wlan.active(False)
from ubluetooth import BLE
BLE().active(False)

# ---- ADXL345：自建 5MHz 总线完成上电配置，随后释放给共享总线 ----
from lib.ADXL345_spi import ADXL345
acc = ADXL345(cs_pin=8, scl_pin=6, sda_pin=2, sdo_pin=7,
              spi_freq=5_000_000).init_spi()
acc.set_sampling_rate(6.25)
acc.set_g_range(2)
acc.set_threshold_activity(0.75)
acc.set_threshold_inactivity(0.5)
acc.set_act_inact_ctl(*((True,)*8))
acc.set_time_inactivity(255)
acc.set_int_enable(activity=True, inactivity=True)
acc.set_measure_mode(True)

# 清掉上电期间可能滞留的锁存中断（读 INT_SOURCE 即清）
int1 = machine.Pin(3, machine.Pin.IN)
while int1.value():
    acc.clear_fifo()
    acc.get_int_source()

acc.deinit_spi()        # 释放自建总线，改挂共享总线

# ---- 共享 SPI 总线（显示参数常驻 40MHz mode0）+ 显示屏 ----
import st7789
display_cs = machine.Pin(10, machine.Pin.OUT, value=1)
spi = machine.SPI(1, baudrate=40_000_000,
                  sck=machine.Pin(6), mosi=machine.Pin(2),
                  miso=machine.Pin(7))
display = st7789.ST7789(
    spi, 240, 240, cs=display_cs,
    reset=machine.Pin(20, machine.Pin.OUT), dc=machine.Pin(4, machine.Pin.OUT),
    backlight=machine.Pin(5, machine.Pin.OUT), rotation=2)
display.init()

# ---- 组装 Board（acc 挂共享总线 + 上锁）并进入应用 ----
import main
main.run(board.Board(spi, display_cs, display, int1, acc, wlan))
