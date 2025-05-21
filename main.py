from micropython import schedule
from machine import Pin, Timer
from esp32 import NVS
from utils import find_files, ClickDetector
from wlan_cfg import wlan_cfg, host2queryip

import gc
import time
import st7789
import lib.vga2_8x16 as font

def main(display_spi, display_cs, sleep_pin, wlan, ble):
    display = st7789.ST7789(display_spi, 240, 240, cs=display_cs, reset=Pin(20, Pin.OUT), dc=Pin(4, Pin.OUT), backlight=Pin(5, Pin.OUT), rotation=2)
    display.init()
    
    imgs = find_files('.png', 'imgs')
    
    nvstatus = NVS('ESP32C3TFT')
    
    ## state variables
    try:
        cursor = nvstatus.get_i32('entrypoint')
        imgs[cursor]
        print(f"read entrypoint at {cursor}: {imgs[cursor]}")
    except:
        cursor = 0
        print(f"failed reading entrypoint, default to 0")
    displayed = -1
    show_name = False
    name_shown = False
    setting_entrypoint = False
    sleep_mode = False
    query_ip = False
    ip = ''
    
    def boot_pin_handler(clicktimes):
        nonlocal cursor
        nonlocal show_name
        nonlocal query_ip
        if clicktimes == 1:
            cursor = (cursor + 1) % len(imgs)
        elif clicktimes == 2:
            show_name = True
        elif clicktimes == 3:
            query_ip = True
    
    def func_pin_handler(clicktimes):
        nonlocal cursor
        nonlocal setting_entrypoint
        nonlocal sleep_mode
        if clicktimes == 1:
            cursor = (cursor - 1) % len(imgs)
        elif clicktimes == 2:
            setting_entrypoint = True
        elif clicktimes == 3:
            sleep_mode = True
    
    boot_pin = ClickDetector(Pin(9, Pin.IN, Pin.PULL_UP), 0, boot_pin_handler, 3)
    func_pin = ClickDetector(Pin(21, Pin.IN, Pin.PULL_UP), 1, func_pin_handler, 3, debounce_time=50)
    
    while True:
            if displayed != cursor:
                show_name = False
                name_shown = False
                displayed = cursor
                display.fill(st7789.BLACK)
                print(f"displaying {displayed}: {imgs[displayed]}")
                gc.collect() # Render image takes a lot of ram so collect garbage to avoid oom
                display.png(imgs[displayed], 0, 0)
                continue
            if show_name and not name_shown:
                name_shown = True
                display.text(font, imgs[displayed], 0, 0)
                continue
            if setting_entrypoint:
                nvstatus.set_i32('entrypoint', cursor)
                nvstatus.commit()
                setting_entrypoint = False
                print(f"set entrypoint {displayed}: {imgs[displayed]}")
                display.text(font, f"set entrypoint {displayed}: ", 0, 240 - font.HEIGHT * 2)
                display.text(font, imgs[displayed], 0, 240 - font.HEIGHT)
                continue
            if query_ip:
                display.text(font, f"IP for {host2queryip}:", 0, font.HEIGHT)
                if not ip:
                    import socket
                    wlan.active(True)
                    wlan.connect(*wlan_cfg)
                    display.text(font, f"SSID: {wlan_cfg[0]}", 0, font.HEIGHT * 3)
                    display.text(font, f"password: {wlan_cfg[1]}", 0, font.HEIGHT * 4)
                    dots = 0
                    prompt = "WLAN connecting"
                    display.text(font, prompt, 0, font.HEIGHT * 5)
                    while not wlan.isconnected():
                        display.text(font, '.' * (dots + 1) + ' ' * (3 - dots), len(prompt) * font.WIDTH, font.HEIGHT * 5)
                        dots = (dots + 1) % 3
                        time.sleep_ms(300)
                    ip = socket.getaddrinfo(host2queryip, 53)[0][4][0]
                    wlan.active(False)
                    del socket, prompt, dots
                display.text(font, ip, 0, font.HEIGHT * 2)
                query_ip = False
                continue
            if sleep_mode or sleep_pin.value():
                display.off()
                break
            time.sleep_ms(200)

