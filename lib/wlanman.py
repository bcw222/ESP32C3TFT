"""WiFi 多 AP 管理。

配置格式（wlan_cfg.py）::

    networks = {
        'ssid1': 'password1',
        'ssid2': 'password2',
    }

按书写顺序尝试（靠前的优先）；单个 AP 超时（默认 15s）后换下一个。
连接成功（开机首连或运行中断线重连）会把该 AP 的条目序号存入 NVS
（key: wifi_ap_idx，经 lib/board.py 的 NVS 函数），下次连接优先试它；
序号 index out of range（如改配置删了 AP）或记住的那个连不上时，
直接从第一个遍历。进度通过 status_cb(ssid, state) 上报，state
∈ {'try', 'ok', 'fail'}，供屏幕显示。

NVS 记忆统一在 lib/board.py（设备级持久 KV）：本模块只存取
wifi_ap_idx；usageapp 的记住页（usage_page）由应用侧直接用 board
函数存取，不经本模块。
"""
import time

from lib.board import nvs_get_i32, nvs_set_i32

_NVS_KEY = 'wifi_ap_idx'


class WlanManager:
    def __init__(self, wlan, networks, status_cb=None, per_ap_timeout=15,
                 tick_cb=None):
        self.wlan = wlan
        self.networks = networks or {}
        self.status_cb = status_cb
        self.tick_cb = tick_cb    # 连接等待期每 200ms 一次（动画等）
        self.timeout = int(per_ap_timeout)

    def _report(self, ssid, state):
        if self.status_cb:
            try:
                self.status_cb(ssid, state)
            except Exception:  # 显示回调绝不能打断连接流程
                pass

    def _tick(self):
        if self.tick_cb:
            try:
                self.tick_cb()
            except Exception:
                pass

    def isconnected(self):
        return self.wlan.isconnected()

    def ensure(self, remember=True):
        """已连接直接 True；否则按配置顺序逐个尝试，全部失败返回 False。

        remember=True：连接成功后把 AP 序号写入 NVS，下次连接（含下次
        启动）优先试它；序号越界（index out of range）或优先者连不上
        时，直接从第一个遍历。
        """
        if self.isconnected():
            return True
        self.wlan.active(True)
        items = list(self.networks.items())
        order = list(range(len(items)))
        if remember:
            last = nvs_get_i32(_NVS_KEY)
            if last is not None and 0 <= last < len(items):
                # 越界（index out of range）则不重排，保持原序从第一个遍历
                order.remove(last)
                order.insert(0, last)   # 优先上次成功的，其余仍按原顺序
        for i in order:
            ssid, pswd = items[i]
            self._report(ssid, 'try')
            try:
                self.wlan.disconnect()
            except Exception:  # 未连接时 disconnect 可能抛错，忽略
                pass
            try:
                self.wlan.connect(ssid, pswd)
            except OSError:
                self._report(ssid, 'fail')
                continue
            deadline = time.ticks_add(time.ticks_ms(), self.timeout * 1000)
            while time.ticks_diff(deadline, time.ticks_ms()) > 0:
                if self.wlan.isconnected():
                    self._report(ssid, 'ok')
                    if remember:
                        nvs_set_i32(_NVS_KEY, i)
                    return True
                self._tick()
                time.sleep_ms(200)
            self._report(ssid, 'fail')
        return False
