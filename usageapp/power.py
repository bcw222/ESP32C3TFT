"""背光电源管理：PWM 亮度 2×2 矩阵（日夜 × 亮暗）。

调暗只是降占空比，数据与时间条照常刷新（轮询间隔由 app 按
dim_poll_interval 拉长）。设备的电源开关由硬件负责，本模块不管睡眠。

亮度四档（usage_cfg.brightness 可配置，0-65535 PWM 占空比）：
- day_bright / day_dim      白天亮屏 / 白天无操作调暗
- night_bright / night_dim  夜间亮屏 / 夜间调暗
（白底屏夜间刺眼，night_bright 默认压得比白天低很多。）
"""
from machine import Pin, PWM

# 2×2 默认占空比
DEFAULT_LEVELS = {
    'day_bright': 65535,
    'day_dim': 20000,
    'night_bright': 24000,
    'night_dim': 2500,
}


def resolve_levels(cfg):
    """usage_cfg.brightness（部分覆盖）⊕ 默认 → 全量四档 dict。

    cfg 为模块形态也兼容；值钳到 0-65535，非法项用默认。
    """
    user = None
    if isinstance(cfg, dict):
        user = cfg.get('brightness')
    elif cfg is not None:
        user = getattr(cfg, 'brightness', None)
    out = {}
    for k, dv in DEFAULT_LEVELS.items():
        v = None
        if isinstance(user, dict) and k in user:
            try:
                v = int(user[k])
            except (TypeError, ValueError):
                v = None
            if v is not None and not 0 <= v <= 65535:
                v = None
        out[k] = v if v is not None else dv
    return out


class Backlight:
    def __init__(self, pin=5, levels=None, freq=200):
        self.pwm = PWM(Pin(pin), freq=freq)
        self.levels = dict(levels or DEFAULT_LEVELS)
        self._dim = False
        self._night = False
        self.pwm.duty_u16(self._duty())

    def _duty(self):
        """按 (日夜, 亮暗) 2×2 取当前档位。"""
        key = ('night_' if self._night else 'day_') + \
            ('dim' if self._dim else 'bright')
        return self.levels[key]

    def set_dim(self, dim, night=None):
        """切亮/暗档；night 传 None 保持当前日夜态。档位变化才写
        PWM（duty 写入有毫秒级阻塞，别每帧刷）。"""
        if night is not None and night != self._night:
            self._night = night
        if dim == self._dim:
            return
        self._dim = dim
        self.pwm.duty_u16(self._duty())

    def set_night(self, night):
        """日夜切换（app 夜间检测调）：重取当前 (日夜,亮暗) 档。"""
        if night == self._night:
            return
        self._night = night
        self.pwm.duty_u16(self._duty())

    def set_levels(self, levels):
        """运行中换四档表：立即按当前状态重取档位。"""
        self.levels = dict(levels)
        self.pwm.duty_u16(self._duty())
