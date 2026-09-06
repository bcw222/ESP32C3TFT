"""ADXL345 活动检测：INT1 硬件中断 + 引脚电平哨兵。

显示与加速度计共享 SPI 总线（CS=10/8，参数互异）。总线借用/归还
与互斥锁都在 board 层（`with board.acc_bus():`），本模块对共享无感，
只做事件路径：

- INT1 上升沿（activity/inactivity 任一）→ ISR 只 micropython.schedule
  排队（零分配）→ scheduled worker 在主上下文只置一个布尔标志。
  ADXL345 中断是锁存式：INT1 保持高直到读 INT_SOURCE 清除，期间不再
  产生新边沿 → 并发自然上限为 1，无风暴风险。
- 引脚电平哨兵：锁存一旦未读清，INT1 停在高电平（含暗屏期
  TIME_INACT 到点的 inactivity）。主循环每 tick 看
  `flags.acc_wake or int1.value()`，见线高才借总线读——
  死锁滞留 ≤1 tick，无定时器，静置时总线借用次数为 0。

activity 阈值/采样率沿用 board.py 的配置（THRESH_ACT=0.75g 等），
本模块只读不写寄存器。
"""
from machine import Pin
from micropython import schedule


class ActivityMonitor:
    """INT1 边沿 + 引脚电平哨兵的混合检测器（无定时器）。"""

    def __init__(self, board):
        self.board = board
        self.acc = board.acc
        self.flag = False               # worker 与主循环间的唯一信箱
        # 硬件 ISR 里用的绑定方法预存好（避免在 ISR 中临时建对象）
        self._worker = self._wakeup
        self._handler = self._irq

    def bind(self, pin):
        """挂 INT1 中断。board 配置已把 activity/inactivity 都映射上
        INT1 且锁存；硬件 ISR 里绝不碰 SPI，只排队。"""
        pin.irq(trigger=Pin.IRQ_RISING, handler=self._handler)

    def _irq(self, _):
        # 硬件 ISR：schedule 把 worker 排进待办，在主线程安全检查点执行；
        # 队列满等异常吞掉（丢一次唤醒无实质影响，电平哨兵兑底）。
        try:
            schedule(self._worker, None)
        except RuntimeError:
            pass

    def _wakeup(self, _):
        # scheduled 上下文（仍是主线程 VM）：只写预分配 bool，零分配。
        self.flag = True

    def poll(self):
        """主循环调用（仅当 flags.acc_wake 或引脚电平高时）：
        进 board 的 acc 借还区读 INT_SOURCE，区分 activity/inactivity
        并清锁存。返回本次是否观察到 activity。"""
        try:
            with self.board.acc_bus():
                source = self.acc.get_int_source()
        except OSError:
            source = {}
        return bool(source.get('activity', False))
