from machine import Pin
from utils import find_files, ClickDetector

import gc
import time
import st7789
import lib.vga2_8x16 as font

def main(display_spi, display_cs, sleep_pin, wlan, ble, wlan_cfg):
    rlv = find_files('.rlv', '.')[0]
    
    ## initialize display
    display = st7789.ST7789(display_spi, 240, 240, cs=display_cs, reset=Pin(20, Pin.OUT), dc=Pin(4, Pin.OUT), backlight=Pin(5, Pin.OUT), rotation=2)
    display.init()
    display.fill(st7789.BLACK)
    
    ## state variables
    status = False
    play = False
    
    def pin_handler(clicktimes):
        nonlocal play
        play = True
    
    boot_pin = ClickDetector(Pin(9, Pin.IN, Pin.PULL_UP), 0, pin_handler, 1)
    func_pin = ClickDetector(Pin(21, Pin.IN, Pin.PULL_UP), 1, pin_handler, 1)
    
    while True:
        if not play and not status:
            display.text(font, rlv, 0, 0)
            rlv_info = display.rlv_info(rlv)
            for i, (k, v) in enumerate(rlv_info.items()):
                display.text(font, f"{k}: {v}", 0, font.HEIGHT * (i + 1))
            display.text(font, f"video_duration: {rlv_info['frame_count'] / rlv_info['fps']} s", 0, font.HEIGHT * (i + 2))
            status = True
            continue
        if play:
            status = False
            display.rlv_play(rlv, 0, 0, st7789.WHITE, st7789.BLACK, cache_size=4096)
            play = False
            continue
        if sleep_pin.value():
            display.off()
            break
        time.sleep_ms(200)