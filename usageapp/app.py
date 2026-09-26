"""usageapp 主状态机。

职责：
- WiFi 连接/断线重连（wlan 对象来自 board.Board）
- 调度 fetch：恒定周期（亮屏 poll_interval / 调暗 dim_poll_interval），
  下键/唤醒/换页立即发起（服务器无抓取缓存，每个请求都是实时抓取）
- 页面序列完全由响应驱动（无占位）：overview（若有）+ providers[]；
  按 id 复用页对象保留增量渲染状态；失败按「本次失败页是否已有
  数据」分流：该页无数据（首次拉取失败，含换页目标页）→ 独立
  错误页（retry Ns + 连续失败计数 fail #N，证明循环活着在重试）；
  该页有旧数据 → 全部页标 stale 保留旧数据 + 页头下 ASCII 单行
  错误（当前页保持显示，不跳页）。错误页也是完整页：有页表时
  上键可翻页离开（以 err_from 为起点前移一页，跳过坏页）。
  服务端无状态化（2026-09-08）：provider 页失败一律 503 + error.code
  ——端侧非 200 走此分流，stale 徽标/ERR_Y/wait 红三件同步出现消失
- 诊断：任何失败串口 [usage] 前缀打印（fail #N 计数、SSR 下载失败、
  意外异常 traceback、wifi failed、recovered）；do_fetch 有
  except Exception 兑底——意外异常不穿透杀死主循环
- 渲染：当前页（上键翻页）+ 底部时间条（每帧重画）
  + 页头 RSSI 角标（固定槽 x112，5s 采样；全量重画后补画）
- SSR 贴图：主响应只带 hash，先渲染 ASCII UI，缺失贴图排队下载
  （同 socket 切片，灰条照算，mark_fetch_done 推迟到队列清空）；
  下载段实测毫秒 = 时间条 SSR 分段（端侧自测，不取服务端 serve）。
  命中本地缓存零请求（render.RenderStore，rcache/）
- 电源：无操作 dim_after 秒调暗；按键或加速度计 activity 唤醒，
  从调暗唤醒时立即强刷一次（拿起就能看到新数据）
- 配色：day/night 两套全量色板（usageapp/theme.py，usage_cfg.palette
  可按昼夜分别覆盖），夜间检测切套——page 落色/时间条换色/整页重画/
  SSR bg+theme 参数随夜态（贴图 hash 换套，双套缓存共存）
- 时钟：无 NTP，用响应里的 server_time 校准偏移，倒计时/夜间小时
  据此计算（tz_offset 折算本地时）
- 帧节奏：计时器结算式动态 sleep（rlvplayer 固件同款）：每帧记起点，
  干完活算已用时，补睡 max(target−elapsed, 1)——渲染耗时不吃掉
  周期，帧率由 target_frame_ms 锚定（usage_cfg 可配置）
"""
import gc
import sys
import time

import st7789
import lib.vga2_8x16 as font
from machine import Pin

from . import client, page, render, theme
from .activity import ActivityMonitor
from .page import ErrorPage, OverviewPage, ProviderPage, NAME_MAX
from .power import Backlight, resolve_levels
from .timeline import Timeline
from lib.board import nvs_erase, nvs_get_i32, nvs_set_i32
from lib.utils import ClickDetector
from lib.wlanman import WlanManager

_TICK_MS = 1     # 动态补睡下限：帧耗时超过 target 时至少让 1ms
# 错误页游标（2026-09-02）：error_page 是独立兑底态不进 pages，
# cursor == ERR_CURSOR 时显示错误页；正常页游标恒 >= 0
ERR_CURSOR = -1
# 记住页 NVS key（存取统一走 lib/board.py 的 NVS 函数）。
# 2026-09-09：**存 i32 不存 str**——真机实测本固件 NVS.set_str 写入
# 后 get_str 读回 None（set_str 内部异常被静默吞，页码从未落盘），
# i32 读写回路正常；页码是非负整数，语义上 i32 更贴合。
_NVS_PAGE = 'usage_page'


class _Flags:
    """中断回调（schedule 上下文）只置标志，主循环消费。"""

    def __init__(self):
        self.next_page = False
        self.force_refresh = False
        self.acc_wake = False   # ActivityMonitor worker 置位：INT1 边沿待消费


def _read_version():
    """读 dist 的版本标识（gendist.sh 写入 version.txt）；找不到 → 'not found'。"""
    try:
        with open('version.txt') as f:
            v = f.read().strip()
        return v[:10] if v else 'not found'
    except OSError:
        return 'not found'


def _read_mac(wlan):
    """本机 STA MAC（冒号小写 hex）；wlan 未激活/异常时返回 ''。"""
    try:
        return ':'.join('{:02x}'.format(b) for b in wlan.config('mac'))
    except (OSError, ValueError, TypeError):
        return ''


def _title_screen(display, status, wlan=None):
    display.fill(page.BG())
    # 设备标识 + git 版本 + 本机 MAC（首次加载/WiFi 连接期间可见，
    # 之后被数据页覆盖）。MAC 由调用方传 wlan 对象读取，拿不到就跳过。
    display.text(font, 'ESP32C3TFT-usage', 4, 6, page.FG(), page.BG())
    display.text(font, _read_version(), 4, font.HEIGHT + 8,
                 page._col(page._TEXT_DIM_RGB), page.BG())
    if wlan is not None:
        mac = _read_mac(wlan)
        if mac:
            display.text(font, mac, 4, (font.HEIGHT + 8) * 2,
                         page._col(page._TEXT_DIM_RGB), page.BG())
    _status_line(display, status)


_WIFI_ANIM_MS = 350          # WiFi 等待点动画步进周期
_STATUS_ROWS = (240 - 60) // (font.HEIGHT + 2)     # 状态区可容纳整行数


def _row_y(row):
    """标题页状态区第 row 行的 y（从 60 起步，给设备名/版本留头部）；
    超过容量的行叠在最底一行复用。"""
    return 60 + min(row, _STATUS_ROWS - 1) * (font.HEIGHT + 2)


def _status_line(display, text, row=0):
    """状态区第 row 行整行重写。只用于一次性内容变化，逐帧动画
    禁止走这里（整行反复重写在 SPI 屏上肉眼可见地闪）。"""
    y = _row_y(row)
    display.fill_rect(0, y, 240, font.HEIGHT, page.BG())
    display.text(font, text, 4, y, page.FG(), page.BG())


class WifiAnim:
    """WiFi 连接等待显示：每个 AP 一行，ok/fail 定格后换行。
    等待期只对尾部点区做「涂背景色 → 画新点」的局部更新，不整行
    重写（整行刷会闪）。只要往屏上写过连接字迹就置 dirtied；主循环
    在确认 WiFi 可用后消费它并强制当前页全量重画，否则断线重连留下
    的日志会一直压在数据页上（增量渲染不会主动去抹别处写的字）。"""

    def __init__(self, display):
        self.display = display
        self.ssid = ''
        self.row = 0
        self.last = time.ticks_ms()
        self.n = 0
        self.dirtied = False       # 屏上有待清理的连接过程字迹

    def _paint_dots(self):
        """点区局部更新：恒定三字符宽先盖背景色（清掉上一帧的所有点）
        再画当前帧；擦除宽度固定、不随点数缩放，天然无残影。"""
        x = 4 + (len(self.ssid) + 1) * font.WIDTH
        y = _row_y(self.row)
        self.display.fill_rect(x, y, 3 * font.WIDTH, font.HEIGHT,
                               page.BG())
        if self.n:
            self.display.text(font, '.' * self.n, x, y,
                              page.FG(), page.BG())

    def begin(self, ssid):
        """新 AP 开始尝试：换新一行，整行只清这一次并画 'SSID' 与首帧。"""
        self.ssid = ssid[:20]
        self.n = 1
        self.last = time.ticks_ms()
        self.dirtied = True
        y = _row_y(self.row)
        self.display.fill_rect(0, y, 240, font.HEIGHT, page.BG())
        self.display.text(font, self.ssid, 4, y, page.FG(), page.BG())
        self._paint_dots()

    def poll(self):
        """到帧则推进点动画（1→2→3 循环），只动点区不碰 SSID 文本。"""
        if time.ticks_diff(time.ticks_ms(), self.last) < _WIFI_ANIM_MS:
            return
        self.last = time.ticks_ms()
        self.n = self.n % 3 + 1
        self._paint_dots()

    def result(self, state):
        """本 AP 出结果：点区一次性覆写为 'ok'/'fail'（顺带清掉
        残留点），下一 AP 用下一行。"""
        x = 4 + (len(self.ssid) + 1) * font.WIDTH
        y = _row_y(self.row)
        self.display.fill_rect(x, y, 5 * font.WIDTH, font.HEIGHT,
                               page.BG())
        self.display.text(font, 'ok' if state == 'ok' else 'fail', x, y,
                          page.FG(), page.BG())
        self.row += 1

    def take_dirtied(self):
        """取出并复位 dirtied；返回 True 表示屏幕被连接过程写过字。"""
        d = self.dirtied
        self.dirtied = False
        return d


def _cfg_dict(cfg):
    """usage_cfg 以模块形态从 boot.py 传入（dict 形态也兼容），统一成 dict。"""
    if isinstance(cfg, dict):
        return cfg
    return {k: getattr(cfg, k) for k in dir(cfg) if not k.startswith('_')}


def _is_night(epoch, start, end, tz_off):
    """夜间小时判定；start/end 任一为 None 即关闭。支持跨零点区间。"""
    if start is None or end is None:
        return False
    hour = (epoch + tz_off * 3600) % 86400 // 3600
    if start <= end:
        return start <= hour < end
    return not (end <= hour < start)


def _read_rssi(wlan, now_ms):
    """RSSI 角标文本（'RSSI:-42'，大写前缀标明含义）；
    拿不到返回 ''（槽清空）。"""
    try:
        v = min(abs(int(wlan.status('rssi'))), 99)
    except (OSError, ValueError, TypeError):
        return ''
    return 'RSSI:{}'.format(-v)


def run(board, networks, cfg):
    """应用主体（常亮循环，不返回）。board 为 board.Board 硬件句柄。"""
    cfg = _cfg_dict(cfg)
    # theme.apply 先于任何显示：解析 usage_cfg.palette（day/night 双层
    # 或旧单层）；启动夜态默认夜间（无 RTC，断电不记忆——开机
    # time.time()≈0，_is_night 不可信，时钟校准前不按小时判）
    theme.apply(cfg)
    page_night = True
    page.NIGHT = page_night
    tl_palette0, tl_bg0 = theme.set_night(page_night)
    display = board.display          # st7789 已由 board.init() 建好
    int1 = board.int1
    wlan = board.wlan
    backlight = Backlight(levels=resolve_levels(cfg))   # 2×2：日夜×亮暗
    _title_screen(display, 'wifi ...', wlan)

    poll_ms = int(cfg.get('poll_interval', 60)) * 1000
    dim_poll_ms = int(cfg.get('dim_poll_interval', 300)) * 1000
    dim_after_ms = int(cfg.get('dim_after', 60)) * 1000
    wifi_timeout = int(cfg.get('wifi_timeout', 15))
    # 目标帧时长（ms）：帧率锚点。主循环每帧按 target−已用时 动态补睡
    # （rlvplayer 固件同款算法），渲染慢时帧率自动降、不会累积漂移。
    target_frame_ms = max(int(cfg.get('target_frame_ms', 50)), 2)
    night_start = cfg.get('night_start')
    night_end = cfg.get('night_end')
    tz_off = int(cfg.get('tz_offset', 8))
    server_cfg = cfg['server']
    # 贴图端点与主 API 同 base：{base}/api/render/<hash>
    # server_cfg 顺路传入：贴图下载重试（ssr_retry/ssr_retry_delay_ms）
    # 与 rcache 容量（rcache_max_files）——与主 API 共用同一份 server
    # 配置
    store = render.RenderStore(
        client.server_base(server_cfg['url']),
        str(server_cfg.get('key') or server_cfg.get('token', '')),
        server_cfg=server_cfg)

    wifi_anim = WifiAnim(display)

    def wifi_status(ssid, state):
        # WlanManager 的状态回调：try=新 AP 开试（换行+动画），ok/fail 定格
        if state == 'try':
            wifi_anim.begin(ssid)
        else:
            wifi_anim.result(state)

    wm = WlanManager(wlan, networks, status_cb=wifi_status,
                     per_ap_timeout=wifi_timeout,
                     tick_cb=wifi_anim.poll)
    wifi_ok = wm.ensure()   # 成功者序号入 NVS，下次启动优先试它
    if wifi_ok:
        time.sleep_ms(700)      # "ok"停留一拍可读，再整屏切数据
        display.fill(page.BG())   # 全屏刷新进入抓取阶段
        wifi_anim.take_dirtied()     # 开机连接痕迹已被整屏清屏带走
    else:
        _status_line(display, 'wifi failed')
        print('[usage] wifi failed: no AP reachable')

    # 页面模型（2026-09-02 单端点 /api/page）：端侧只维护页码游标 + 本
    # 地页对象缓存（按响应 type/id 分层，跨周期保留增量渲染状态）。
    # pages 为定长列表（长度 = 最近一次响应 total，None = 尚未拉过）：
    # pages[0] 是 overview 页（服务端 total>1 时存在），其余是 provider
    # 页。error_page 是独立兜底态，不占页码。
    pages = []
    overview_page = None
    error_page = ErrorPage()
    cursor = 0
    net_ok = True
    loaded = False     # 首次成功加载标志：False 期失败→错误页；True 后→stale
    fails = 0          # 连续失败计数（错误页 fail #N 屏显；成功归零）
    err_from = None    # 错误页代表的页码（无数据页游标）；None=无页表可翻

    # y=222：标注行 y204，给 4 行紧凑档让位（旣往 y216 时与第三行打架）。
    # fetch 段三色分段（用户定稿）：响应到达后按服务端 timing 把暗段
    # 重涂 network/upstream/ssr，分段数据随数据响应顺路回传。
    # 四色+底色由 usageapp/theme.py 昼夜两套色板出（usage_cfg.palette
    # 可按 day/night 分别覆盖）；夜间切换时 app 调 timeline.set_palette。
    timeline = Timeline(0, 222, 240, 12,
                        *tl_palette0,
                        bg=st7789.color565(*tl_bg0), font=font,
                        err_rgb=theme.err_rgb(page_night))
    timeline.set_period(poll_ms)
    monitor = ActivityMonitor(board)   # 总线借用/互斥都在 board 层
    # INT1 上升沿 → schedule 置 flag；锁存式中断天然并发上限 1。
    # 锁存滞留由引脚电平哨兵兑底（见主循环）：线高即读，无定时器。
    monitor.bind(int1)

    flags = _Flags()

    # 记住页（NVS）：同页连续手动刷新（中途无 dim、无换页）达
    # refresh_remember_n 次 → 记住当前页（页码 i32），下次开机直接转
    # 到它。restore_page = 启动时从 NVS 读到的页码；restoring = 本次
    # 启动待"加载记忆页"：成功加载保留记忆（每次开机都恢复），加载
    # 失败才清除。
    remember_n = max(int(cfg.get('refresh_remember_n', 3)), 1)
    restore_page = nvs_get_i32(_NVS_PAGE)
    restoring = restore_page is not None
    refresh_count = 0        # 同页连续手动刷新计数
    manual_pending = False   # 本帧手动刷新键已按下、待 do_fetch 结算

    def on_boot_click(_):
        flags.next_page = True

    def on_func_click(_):
        flags.force_refresh = True

    # 防抖 80ms：按键按下沿触发（ClickDetector 默认 IRQ_FALLING）
    ClickDetector(Pin(9, Pin.IN, Pin.PULL_UP), 0, on_boot_click, 1,
                  debounce_time=80)
    ClickDetector(Pin(21, Pin.IN, Pin.PULL_UP), 1, on_func_click, 1,
                  debounce_time=80)

    clock_offset = 0
    last_timing = [None]       # 最近一次响应的 timing（列表绕 nonlocal）
    last_activity = time.ticks_ms()
    next_night_check = 0
    next_rssi_poll = 0
    rssi_value = None            # 最近一次采集的 RSSI 文本（5s 一拍）
    rssi_drawn = None            # 当前屏幕上已画的 RSSI（None=槽被整屏刷新抹掉）

    def draw_rssi():
        """RSSI 角标写入（值与屏上一致时零操作）。擦除区收紧到角标
        实际文本带（'RSSI:-99' 8 字符=64px，右缘 228）——左侧留空给
        同行错误行（x4..156，2026-09-06 同行分置定稿）。"""
        nonlocal rssi_drawn
        if rssi_value == rssi_drawn:
            return
        display.fill_rect(page.DOT_X - 66, 22, 68, font.HEIGHT,
                          page.BG())
        if rssi_value:
            display.text(font, rssi_value, page.DOT_X + 2
                         - len(rssi_value) * 8, 22,
                         page._col(page._CAPTION_RGB), page.BG())
        rssi_drawn = rssi_value

    def rssi_poll(now):
        """RSSI 5s 采集一拍 + 差异重画（主循环与 fetch 帧回调共用）。"""
        nonlocal next_rssi_poll, rssi_value
        if time.ticks_diff(now, next_rssi_poll) >= 0:
            next_rssi_poll = time.ticks_add(now, 5_000)
            rssi_value = _read_rssi(wlan, now)
        draw_rssi()

    def tl_frame():
        """fetch/SSR 期间的帧回调：时间条动画 + RSSI 采集/重画。
        主循环阻塞在 socket 切片读/PNG 解码期间只有这里被执行——
        此前 RSSI 采集与重画只在主循环做，一轮 fetch 数秒~十几秒
        期间角标数值冻结不刷新（2026-09-07 修）。"""
        rssi_poll(time.ticks_ms())
        timeline.draw(display)

    force_refresh = True            # 开机立即抓一轮
    was_dim = False

    def now_epoch():
        return time.time() + clock_offset

    def _sync_clock(data):
        """只在 server_time 合理（正 unix 秒 >1e9）时校准 clock_offset。
        旧/异常响应可能缺 server_time（=0）——盲目相减会把本地时钟打成
        0（导致时间行恒"8 点"、倒计时几十万小时）。仅接受真实 unix 秒。
        顺路：摘走 timing + 通知时间条"主响应已到"——右标注 network/
        upstream 两段展开冻结，ssr 段从此刻从 0 现场计时（贴图下载段
        端侧实测）。"""
        nonlocal clock_offset
        last_timing[0] = (data.get('timing')
                          if isinstance(data, dict) else None)
        timeline.mark_response(last_timing[0])
        st = data.get('server_time', 0) if isinstance(data, dict) else 0
        try:
            st = int(st)
        except (TypeError, ValueError):
            return
        if st > 1_000_000_000:
            clock_offset = st - time.time()

    def page_at(idx):
        """页码 → 页对象：pages 按最近 total 定长占位（None=未拉过）。"""
        if not pages or idx < 0:
            return None
        return pages[idx % len(pages)]

    def enter_error_page(message):
        """失败页无数据（含 loaded=True 时单页首拉失败）：独立错误页
        兑底（不进 pages），cursor 置 ERR_CURSOR；重试时 target_page
        回 0 拉 total。进入前记下所在页码 err_from——错误页也是完整页
        （2026-09-26 owner 定），上键翻页以它为起点前移一页，从而
        跳过持续无数据的坏页（此前错误页吞掉上键，坏页永远拦住后续页）。"""
        nonlocal net_ok, cursor, err_from
        net_ok = False
        error_page.set(message)
        err_from = cursor if (pages and 0 <= cursor < len(pages)) else None
        cursor = ERR_CURSOR

    def degrade_stale(message):
        """已加载过数据后的失败（loaded=True）：全部页（含总览）标
        stale 保留旧数据 + 页头下 ASCII 单行错误，当前页保持显示
        （不跳错误页）；服务端下次正常返回自会复原（status 换 ok →
        sig 变 → 全量重画）。"""
        nonlocal net_ok
        net_ok = False
        for p in pages:
            if p is not None:
                p.set_error_keep(message)

    def _note_fail(message):
        """统一失败入口：连续失败计数 +1（错误页 fail #N 屏显，成功
        归零）、串口详情打印、按「本次失败页是否已有数据」分流——
        该页从未拉到过数据（首次拉取失败，含换页目标页）→独立错误页；
        该页已有数据（轮询失败 / 上游 503）→全页 stale 保留旧数据。

        （2026-09-26 owner 定 B：此前只看全局 loaded，只要成功加载
        过任一页，之后单页首拉失败也走 degrade_stale，屏上只剩叠加的
        单行 ASCII，没有整页错误页——与「首次错误有整页」的预期不符。）

        判据用 cursor 所在页对象：换页失败已由 _fail_switch 把 cursor
        落到目标页，故 page_at(cursor) is None 即「目标页无数据」。"""
        nonlocal fails
        fails += 1
        print('[usage] fetch fail #{}: {}'.format(fails, message))
        error_page.set_fails(fails)
        if loaded and page_at(cursor) is not None:
            degrade_stale(message)
        else:
            enter_error_page(message)

    def _paint_current():
        """立即渲染当前页（do_fetch 中先出 ASCII UI 用；含错误页分支）。
        全量重画时顺带失效 RSSI 角标与时间条（页 fill 抹掉了它们）。"""
        nonlocal rssi_drawn
        if cursor == ERR_CURSOR:
            full = error_page.render(display, font, now_epoch(),
                                     retry_left_s=timeline.left_s())
        else:
            cur = page_at(cursor) if pages else None
            if cur is None:
                return False
            full = cur.render(display, font, now_epoch(), store)
        if full:
            # 咽喉点：页整屏 fill 抹掉了 RSSI 角标与时间条。时间条
            # 失效下次 draw 全量重画；RSSI 立即原地补画——不等下一帧
            # 主循环（fetch/SSR 下载期间 _paint_current 被反复调用，
            # 角标缺位曾达秒级可见，2026-09-06 修）。
            rssi_drawn = None
            draw_rssi()
            timeline.invalidate()
        return full

    # 换页 pending：目标页码（尚未拉取到详情前不真正切页）。
    # None = 无换页；其余 = 目标页序号（端侧本地回卷，服务端兜底）。
    pending_page = None

    def do_switch(idx):
        """数据到达后切到对应页。换页（挂过 pending_page）与错误恢复
        （cursor==ERR_CURSOR 后的首拉）都走这里——拉到的数据对应
        tgt（target_page 的返回值），成功即显示它。必须在
        _download_missing 之前调——先把新页切成当前页，_download_missing
        的首个 _paint_current 才会渲染新页的 ASCII 骨架（缺图槽留空），
        下载段再逐张补贴；否则新页要等全部贴图下载完才首次上屏。"""
        nonlocal cursor, pending_page
        if not pages:
            return
        pending_page = None
        cursor = idx % len(pages) if idx >= 0 else 0
        if pages[cursor] is not None:
            pages[cursor].invalidate()   # 时间条随全量渲染自动失效

    def target_page():
        """当前应拉取数据的页码：pending_page（换页目标）优先；
        错误页/无页 → None（表示先拉任意页定 total）。"""
        if pending_page is not None:
            return pending_page
        if not pages:
            return None
        if cursor == ERR_CURSOR:
            return 0            # 错误页重试：从首页拉，拿 total 定序列
        return cursor

    def _fail_switch():
        """换页 fetch 失败：目标页照常落为当前页（cursor 前移）。
        否则目标被丢弃、cursor 不动，下一记上键 (cursor+1)%total 又
        指回同一页——该页持续 erroring 时永远翻不过去（2026-09-26
        owner 报「一页卡住没法翻页」）。仅 loaded（成功加载过）生效：
        未加载过时 cursor 不动，失败由 _note_fail 按「目标页无数据」
        判据进独立错误页（cursor 归 ERR_CURSOR）。落位后本页
        照常进轮询（target_page 回 cursor），上游恢复即自愈。"""
        nonlocal cursor
        if pending_page is not None and loaded and pages:
            cursor = pending_page % len(pages)

    def _resize(total):
        """响应 total → pages 定长列表（新增 None 占位、超出回收）。
        页对象在 do_fetch 按响应 type 挂载——overview 单例/新建挂
        page 位、provider 按 id 复用挂 page 位。这里只保证长度与
        占位连续性。"""
        nonlocal pages, overview_page, cursor
        if total is None or total < 0:
            return
        if total == 0:
            pages = []
            return
        old = pages
        new = [None] * total
        for i in range(min(len(old), total)):
            new[i] = old[i]
        pages = new
        if cursor >= total:
            cursor = total - 1

    def _download_missing(want):
        """下载本响应缺失的贴图；先渲染当前页（缺失位留空）再逐张下载，
        每张落地立即重画（贴图 + 时间条，防整屏刷新抹掉的区段缺帧）。
        want：render.collect 输出的 [(hash, slot_w), ...]，slot_w 是
        该 hash 所在槽位的实际宽度，随请求 ?w= 上报服务端只读校验。
        返回 (下载段实测毫秒, 是否有下载失败)——SSR 分段用（端侧自测，
        不取服务端 timing.serve；无缺失即 ≈0）；下载失败置 err（时间条
        wait 段涂红提示，2026-09-02 用户定稿，不弹错误页）。"""
        widths = dict(want)
        missing = store.missing(list(widths))
        _paint_current()
        tl_frame()
        t0 = time.ticks_ms()
        had_fail = False
        for h in missing:
            try:
                store.download(h, on_wait=tl_frame,
                               slot_w=widths[h],
                               on_retry=timeline.set_retrying)
                _paint_current()
                tl_frame()
            except client.FetchError as exc:
                had_fail = True     # 单张失败不中断，槽空等下轮；wait 红
                print('[usage] ssr {} fail: {}'.format(h, exc))
                continue
        return time.ticks_diff(time.ticks_ms(), t0), had_fail

    def do_fetch(anchor_ms):
        nonlocal force_refresh, clock_offset, pending_page, cursor
        nonlocal overview_page, manual_pending, refresh_count, restoring
        nonlocal loaded, fails, net_ok
        # 整页重画的两个触发源：上轮失败过的恢复（broken）、断线重连
        # 期间往屏上写过连接日志（dirty）。统一在成功尾部处理，
        # 不在路径中途零散打补丁。
        broken = not net_ok
        dirty = False
        ssr_ms = 0
        ssr_fail = False
        manual = manual_pending     # 手动刷新意图本帧消费（成败都复位）
        manual_pending = False
        timeline.begin_cycle(anchor_ms)
        gc.collect()
        try:
            if not wm.ensure():
                raise client.FetchError('wifi down')
            dirty = wifi_anim.take_dirtied()

            # 单端点 /api/page：只按页码拉数据，响应 type 决定分发。
            # 换页中：当前页保持原样显示（不标 stale），时间条走灰段；
            # 拉到目标页数据后再真正切过去渲染。stale 只在请求失败时才标。
            tgt = target_page()
            if tgt is None:
                # 首次/无 total：优先拉记忆页（restoring），否则从首页
                # 起（拿 total 定序列；服务端对越界页码自会回卷）。
                tgt = (restore_page if restoring and restore_page is not None
                       else 0)
            data = client.fetch_page(
                server_cfg, tgt, on_wait=tl_frame,
                night=page_night)
            _sync_clock(data)
            render.check_alt(data)   # 双 hash 契约：缺 alt 即报错（旧服务器不兼容）
            # 200 + 顶层 status=error：无状态化（2026-09-08）后仅剩
            # total=0 的空 overview（无 provider，配置错误）会走这里——
            # total=0 下方即 raise，实际到不了渲染。overview 正常聚合
            # 页面级恒 ok、单 provider 失败只标行级 error（红 err），
            # provider 页失败已是 503 走 FetchError 失败路径（degrade_stale
            # 三件同步）。此处仅串口留痕作防御。
            if str(data.get('status') or '') == 'error':
                err = data.get('error')
                code = (err.get('code') if isinstance(err, dict)
                        else None) or 'no code'
                print('[usage] page status=error: {}'.format(code))
            typ = data.get('type')
            total = data.get('total')
            try:
                total = int(total) if total is not None else None
            except (TypeError, ValueError):
                total = None
            _resize(total)          # pages 定长对齐服务端页表
            if total == 0:
                raise client.FetchError('no pages')
            want = render.collect(data)
            cur_page = data.get('page', tgt)
            try:
                cur_page = int(cur_page)
            except (TypeError, ValueError):
                cur_page = tgt % total
            if typ == 'overview':
                if overview_page is None:
                    overview_page = OverviewPage()
                overview_page.update(data, night=page_night)
                if pages and 0 <= cur_page < len(pages):
                    pages[cur_page] = overview_page
            else:
                # type=provider：页对象按响应 id 复用（增量渲染状态保留）
                pid = str(data.get('id') or '')
                p = None
                if pages and 0 <= cur_page < len(pages):
                    p = pages[cur_page]
                if p is None or p is overview_page:
                    p = ProviderPage(pid)
                p.pid = pid if pid else p.pid
                # dark：暗屏轮 Δ 增量累计、不推进基准；night：渲染套
                # 登记（page._alt 双套记忆，昼夜切换回填用）
                p.update(data, now_epoch(), dark=dim, night=page_night)
                if pages and 0 <= cur_page < len(pages):
                    pages[cur_page] = p
            data = None
            gc.collect()
            do_switch(cur_page)     # 先切页再下载（骨架先行）
            ssr_ms, ssr_fail = _download_missing(want)

            if broken or dirty:
                # 恢复/写脏统一整页重画：错误页整屏 fill、WiFi 连接日志、
                # stale 残留都是增量渲染的盲区，恢复时全部推倒重画一次。
                for pg in pages:
                    if pg is not None:
                        pg.invalidate()
                timeline.invalidate()
            timeline.mark_fetch_done(last_timing[0], ssr_ms, error=ssr_fail)
            # 成功：loaded 置位（此后失败走 degrade_stale 保留数据）、
            # 连续失败计数归零（错误页 fail #N 消失）；上轮失败过则报捷
            if broken:
                print('[usage] recovered after {} fails'.format(fails))
            loaded = True
            net_ok = True   # 复位（失败置 False 后成功路径从不回 True：
                            # 此前一旦失败过，之后每轮含换页左标注恒
                            # retrying...，2026-09-05 修复）
            fails = 0
            error_page.set_fails(0)
            # 手动刷新计数结算：本帧手动刷新（非 dim、非换页）已成功
            # 拉取 → 同页连续计数 +1，达到 refresh_remember_n 次即记住
            # 当前页（页码 i32，见 _NVS_PAGE 注释——set_str 真机写不
            # 进，改存 i32），下次开机直接转到它。换页/进入调暗会把
            # 计数清零（见输入/调暗段）。
            if manual and not dim and cursor != ERR_CURSOR:
                refresh_count += 1
                if refresh_count >= remember_n:
                    nvs_set_i32(_NVS_PAGE, cursor)
            # 开机恢复记忆页落地：restoring 时本次 do_fetch 成功——记忆
            # 页是页码，_resize 已按 total 对齐、do_switch 已切过去
            # （能走到成功尾部即恢复完成）。记忆**保留**：每次开机都
            # 恢复它，仅开机加载失败才清除（见下失败分支）。
            if restoring:
                restoring = False
            # 注意：这里**不**清 flags.force_refresh（曾经的"重叠去重"，
            # 2026-09-07 删）：do_fetch 阻塞数秒~十几秒，期间按下的刷新
            # 键被尾部无条件清掉即被静默吞——既不刷新也进不了
            # manual_pending，连续手动刷新计数永远凑不满 refresh_remember_n，
            # 记住页功能形同虚设（坑 3"标志只有一份"的违例）。保留标志
            # 让它在下一帧输入段正常消费：刷新 + 计数两不误；fetch 串行
            # 执行，连按只是排队逐轮拉，无风暴。
        except client.FetchError as exc:
            timeline.mark_fetch_done(error=True)
            _fail_switch()   # 换页失败也落位（跳过持续 err 的页）
            pending_page = None
            if restoring:
                restoring = False
                nvs_erase(_NVS_PAGE)         # 开机加载记忆页失败 → 清除记忆
            _note_fail(str(exc))
        except Exception as exc:        # 意外异常兑底：不穿透杀死主循环
            timeline.mark_fetch_done(error=True)
            _fail_switch()   # 换页失败也落位（跳过持续 err 的页）
            pending_page = None
            if restoring:
                restoring = False
                nvs_erase(_NVS_PAGE)
            sys.print_exception(exc)    # 串口完整 traceback（诊断）
            _note_fail('unexpected: {}'.format(exc))
        force_refresh = False
        gc.collect()

    while True:
        frame_start = time.ticks_ms()      # 本帧起点：动态补睡的计时基准
        now = time.ticks_ms()

        # 暗屏判定（按 last_activity）：暗屏中按键 = 仅唤醒，不解释为
        # 换页/强刷——撤暗由本帧稍后的调暗段完成，且 was_dim 过渡会
        # 自动补一次强刷（与拿起唤醒同语义，数据不陈旧）
        dim_key = time.ticks_diff(now, last_activity) >= dim_after_ms

        # ---- 输入：上键换页 ----
        if flags.next_page:
            flags.next_page = False
            refresh_count = 0        # 换页打断连续手动刷新计数
            if dim_key:
                last_activity = now          # 暗屏中：仅唤醒
            elif len(pages) > 1:
                # 不立即切页：算目标页码（本地回卷 (n+1)%total），挂
                # pending_page；fetch 完成后才真正切过去。错误页
                # （ERR_CURSOR 兜底态）不作为翻页目的地，但自身可作
                # 起点离开：以 err_from（错误页代表的页码，缺省首页）
                # 为基准前移一页，跳过持续无数据的坏页（2026-09-26
                # owner 定「错误页也是一个完整的页」）。
                base = cursor if cursor != ERR_CURSOR else (
                    err_from if err_from is not None else -1)
                pending_page = (base + 1) % len(pages)
                force_refresh = True    # 立即发起换页 fetch
                # 注意：换页只清零连续计数（见本段开头），不清除已记住
                # 页——记忆是长期书签，仅开机加载失败才清除。
                last_activity = now

        # ---- 手动刷新键：读到 flags 再落到局部状态（否则失联） ----
        if flags.force_refresh:
            flags.force_refresh = False
            if dim_key:
                last_activity = now          # 暗屏中：仅唤醒
            else:
                force_refresh = True
                manual_pending = True

        # ---- 加速度计：INT1 边沿（实时） + 引脚电平哨兵 ----
        # 边沿标志由 scheduled worker 置位；消费时借总线读 INT_SOURCE
        # 区分 activity/inactivity 并清锁存。哨兵看引脚电平：锁存式
        # 中断一旦未读清，线会停在高电平（含暗屏期 TIME_INACT 到点
        # 的 inactivity）——见线高就读一次，死锁滞留 ≤1 tick 且零
        # 定时器。两条路都只刷 last_activity，消费端依旧按
        # dim_after 钟化，不受事件频率影响。
        if flags.acc_wake or int1.value():
            flags.acc_wake = False
            if monitor.poll():
                last_activity = now

        # ---- 调暗 / 唤醒 ----
        idle_ms = time.ticks_diff(now, last_activity)
        dim = idle_ms >= dim_after_ms
        backlight.set_dim(dim)
        timeline.set_period(dim_poll_ms if dim else poll_ms)
        if dim and not was_dim:
            refresh_count = 0        # 进入调暗 → 连续手动刷新计数清零
        if was_dim and not dim:
            force_refresh = True     # 拿起来立即刷新
        was_dim = dim

        # ---- 调度 fetch（恒定周期 + 强刷 + 换页 pending）----
        if force_refresh or timeline.due() or pending_page is not None:
            do_fetch(timeline.next_anchor()
                     if (not force_refresh and pending_page is None) else now)

        # ---- 夜间模式（分钟粒度检测；昼夜两套全量配色整体切换：
        # page 落色 + 时间条换四色/底色 + 当前页贴图槽换套 +
        # backlight 换夜间档。**仅 SSR，零数据请求**（2026-09-07
        # 定稿）：主响应每槽双发两套 hash（*_render + *_render_alt），
        # swap_renders 回填另一套后渲染直接读 rcache 双套共存缓存；
        # 缺失贴图直接走 /api/render/<hash> 按需补下（时间条在当前
        # 相位后追加一段 ssr 增量走字，右标注 ssr 位继续走字），不碰
        # /api/page、不触发上游——数据刷新仍归常规轮询。未成功加载过
        # （错误页/无页）无数据无贴图，仅换色 ----
        if time.ticks_diff(now, next_night_check) >= 0:
            next_night_check = time.ticks_add(now, 60_000)
            # 时钟未校准（无 RTC，time.time()≈0）时 _is_night 不可信：
            # 保持启动默认夜态，校准后第一个检测点才切真值
            night = (_is_night(now_epoch(), night_start, night_end, tz_off)
                     if clock_offset else page_night)
            if night != page_night:
                page_night = night
                page.NIGHT = night
                tl4, tlbg = theme.set_night(night)
                timeline.set_palette(tl4, st7789.color565(*tlbg),
                                     err_rgb=theme.err_rgb(night))
                timeline.set_night(night)
                backlight.set_night(night)   # 背光换夜间档（2×2）
                cur = page_at(cursor)
                if cur is not None:
                    cur.swap_renders(night)  # 贴图槽换另一套 hash
                    cur.invalidate()         # 全量重画（换色+换图）
                    # 按需补下：枚举槽位（带 ?w= 槽宽）查盘，缺哪张
                    # 下载哪张；下载失败槽空下轮自愈（[usage] 留痕）
                    widths = {}
                    for h, w in cur.render_slots():
                        if h and w > widths.get(h, 0):
                            widths[h] = w
                    missing = store.missing(list(widths))
                    if missing:
                        timeline.begin_extra()
                        for h in missing:
                            try:
                                store.download(
                                    h,
                                    on_wait=lambda: timeline.draw(display),
                                    slot_w=widths[h])
                                _paint_current()
                            except client.FetchError as exc:
                                print('[usage] ssr {} fail: {}'.format(
                                    h, exc))
                        timeline.extra_done()
                        _paint_current()
                elif cursor == ERR_CURSOR:
                    error_page.invalidate()  # 错误页整屏 fill 换夜底

        # ---- RSSI 角标：5s 采集一拍；写屏统一走 draw_rssi（值未变
        # 零操作；页 full 重画后已由 _paint_current 立即补画；
        # fetch/SSR 期间由 tl_frame 帧回调接手，不再冻结）----
        rssi_poll(now)

        # ---- 渲染（增量：无变化时零绘制操作）----
        _paint_current()
        timeline.draw(display)

        # ---- 帧节奏：计时器结算式动态 sleep（rlvplayer 固件同款）----
        # 本帧已用时 = 渲染+网络+调度全部开销；补睡 target−已用时。
        # 帧重超 target 时只让 1ms（保底喂看门狗/让按键中断进来），
        # 帧率自动降但不会累积漂移——每帧都从真实时钟重新结算。
        elapsed = time.ticks_diff(time.ticks_ms(), frame_start)
        time.sleep_ms(max(target_frame_ms - elapsed, _TICK_MS))
