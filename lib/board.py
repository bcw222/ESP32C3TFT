"""板级包装：把 boot 初始化好的硬件统一成易用的 Board 句柄。

设备初始化全在 boot.py；本模块只做三件包装：

1. 句柄集合——应用从 Board 拿现成的 spi/display/int1/acc/wlan，
   不再各自 Pin(...) 构造；
2. 共享 SPI 总线仲裁——显示与加速度计共用一条总线
   （display CS=10 @40MHz mode0 / acc CS=8 @5MHz mode3），
   运行期借用/归还/互斥都在这：

   - 平时总线常驻显示参数，display 随便用；
   - acc 只能在 `with board.acc_bus():` 里用——进出借还区自动切
     5MHz mode3 / 还原显示参数；
   - 借还区外加锁校验：acc.spi 平时挂的是只会 raise 的哨兵对象，
     借还区内重复借用直接抛 BusConflictError——「一个设备使用期间
     另一个设备不被使用」这条应用层假设一旦被打破立即暴露。
     （display 是冻结 C 模块、直握总线对象，无法在这拦截，唯一
     约束：`with acc_bus():` 块内不要画屏——单线程主循环天然满足。）
3. NVS 记忆函数——设备级持久 KV（nvs_get/set_i32、nvs_get/set_str、
   nvs_erase）：WiFi AP 序号（wlanman 用）与记住页（usageapp 用）
   都经这存取，业务模块不再各自握 NVS 细节。
"""

from machine import Pin

_ACC_HZ = 5_000_000
_DISP_HZ = 40_000_000


class BusConflictError(RuntimeError):
    """共享 SPI 总线的互斥假设被打破（借用期间又借用/借还区外偷用）。"""


class _SpiGuard:
    """挂在 acc.spi 上的哨兵：借还区外任何总线操作都抛错。"""

    def _deny(self, *a, **kw):
        raise BusConflictError(
            'acc used the shared SPI outside board.acc_bus()')

    write = read = readinto = init = deinit = _deny


class _AccBorrow:
    """借还区（board.acc_bus() 返回值）。__enter__ 切 acc 总线参数，
    __exit__ 还原显示参数并重新上锁。预创建复用，进出零分配。"""

    def __init__(self, board):
        self._board = board

    def __enter__(self):
        b = self._board
        if b._owner is not None:
            raise BusConflictError('shared SPI busy: ' + b._owner)
        b._owner = 'acc'
        b.acc.spi = b.spi
        b.spi.init(baudrate=_ACC_HZ, polarity=1, phase=1)
        return b.acc

    def __exit__(self, *exc):
        b = self._board
        b.spi.init(baudrate=_DISP_HZ, polarity=0, phase=0)
        b.acc.spi = b._guard
        b._owner = None
        return False            # 不吞异常，只保证总线归还


class Board:
    """硬件句柄集合（由 boot.py 初始化好并传入）。

    属性：
    - spi        共享硬件 SPI 总线对象（常驻显示参数 40MHz mode0；
                 只有 display 直用它，acc 一律走 acc_bus()）
    - display_cs 显示片选
    - display    已 init 的 st7789 屏对象
    - int1       ADXL345 INT1 输入引脚（活动检测中断/电平哨兵）
    - acc        已配置好的 ADXL345 对象（acc_bus() 借还区内可用）
    - wlan       STA 接口对象（boot 已 active(False)，应用自行激活）
    """

    def __init__(self, spi, display_cs, display, int1, acc, wlan):
        self.spi = spi
        self.display_cs = display_cs
        self.display = display
        self.int1 = int1
        self.wlan = wlan
        # acc 挂共享总线：只重绑对象，绝不再 init/deinit——
        # deinit_spi 会把共享总线关掉。平时 acc.spi 挂哨兵（上锁），
        # 借还区内由 _AccBorrow 换成真总线。
        self.acc = acc
        acc.cs = Pin(8, Pin.OUT, value=1)
        acc.spi = _SpiGuard()
        self._guard = acc.spi
        self._owner = None               # 当前借用人（None=显示态）
        self._borrow = _AccBorrow(self)  # 预创建，进出借还区零分配

    def acc_bus(self):
        """acc 借还区：`with board.acc_bus():` 内可正常读写 acc
        寄存器，进出自动切/还总线参数。重复借用/借还区外使用
        抛 BusConflictError。"""
        return self._borrow


# ---- NVS 记忆（设备级持久 KV；写失败静默——记忆是优化不是依赖，
# 失败不影响本次功能，下次退默认/旧值）----

try:
    from esp32 import NVS
except ImportError:              # 宿主冒烟/无 NVS 平台
    NVS = None

_NVS_NS = 'ESP32C3TFT'


def nvs_get_i32(key):
    """读 i32；无记录/读失败/无 NVS 返回 None（负值视为无记录）。"""
    if NVS is None:
        return None
    try:
        v = NVS(_NVS_NS).get_i32(key)
        return v if v >= 0 else None
    except Exception:
        return None


def nvs_set_i32(key, value):
    if NVS is None:
        return
    try:
        nvs = NVS(_NVS_NS)
        nvs.set_i32(key, int(value))
        nvs.commit()
    except Exception:
        pass


def nvs_get_str(key):
    """读 str；无记录/空串/读失败返回 None。"""
    if NVS is None:
        return None
    try:
        v = NVS(_NVS_NS).get_str(key)
        return v if v else None
    except Exception:
        return None


def nvs_set_str(key, value):
    if NVS is None:
        return
    try:
        nvs = NVS(_NVS_NS)
        nvs.set_str(key, value or '')
        nvs.commit()
    except Exception:
        pass


def nvs_erase(key):
    """删除 key；不存在/失败静默。"""
    if NVS is None:
        return
    try:
        nvs = NVS(_NVS_NS)
        nvs.erase_key(key)
        nvs.commit()
    except Exception:
        pass
