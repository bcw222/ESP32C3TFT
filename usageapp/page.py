"""provider 页（动态 1..4 行配额）+ 总览页 + 服务器错误页。

增量渲染：布局签名（页名 + 行标签序列 + 昼夜）不变时走 diff 路径——
每行记录上次绘制的 (百分比文本, 填充宽度, 颜色, 倒计时, 数额, Δ)，
只重画变化元素；签名变化（翻页回来、配额结构变化、昼夜切换）才
全量重画。fetch 刷新数值、倒计时走秒都不会整屏闪。

动态布局（quotas 1..4 档，查表 _LAYOUTS；>4 由 client 判无效响应）：
- 1 行大条 / 2、3 行数额在条下方 / 4 行紧凑（数额与标签同行右对齐）
- 行区避开底部时间条标注区（timeline 挪到 y=222，标注 y=204）

阈值变色（用户批准）：percent>=80 琥珀、>=95 红；>=95 的百分比文本
500ms 闪烁；reset_at 前 RESET_NEAR_S 秒倒计时文本闪烁。

Δ 显示：内存态（页对象里），同窗口 percent 较上次 fetch 变化 >=0.05
时在数额行右侧显示 "+0.3"，60s 后自动消失；不写 flash；4 行紧凑档
无处可放，不显示。

stale 年龄：status=stale 时页头状态走字 "stale Nm"（用 updated_at）。

错误模型（2026-09-05 定稿）：状态徽标两态 ok/stale——error 徽标删除，
错误详情一律页头下 ERR_Y ASCII 单行（status≠ok 时）或独立 ErrorPage
（带连续失败计数 fail #N，证明主循环活着在重试）。总览行级
items[].status=error：屏蔽服务端 percent=0.0 兜底——该行不画条/"0%"/
倒计时，percent 槽画红 'err'；status=stale 行是真实旧值照常渲染。

页头固定槽位：名字 x4 宽 13 字符 | RSSI 角标 x160 宽 8 字符（app 画，
page 不碰）| 状态文字右对齐 x226 最多 9 字符 | 状态点 x226。

SSR 贴图（title_render/label_render/overview 贴图）：
主响应只带 8hex hash，本体由 app 经 render.RenderStore 按需下载到
本地缓存；页面渲染时 store.usable(hash, 槽宽) 命中→display.png 直接
贴文件（固件方法，流式解码），未命中/超槽宽→槽位置空
（不 ASCII 回退，串口打诊断，等下轮平补刷新）
（name/label）。下载完成后的下一帧 diff 自动补贴
（槽位记录已画 hash，与期望不一致才重画）。message 不渲染。
"""
import gc
import time

import st7789

try:
    import lib.digits as bigfont
except ImportError:              # 兑底：未烧 digits.mpy 的旧 dist
    bigfont = None

W, H = 240, 240
BAR_X, BAR_W = 8, 160
ERR_Y = 22      # 错误提示行：与 RSSI 角标同一行（y22..38）左右分置
                # ——错误居左 x4..156，角标居右 x164..228，不叠不擦
ERR_MAX_CHARS = 19   # 错误单行最大字符数（19×8=152px，x4..156<x160）

# ---- 配色方案（usageapp/theme.py 集中定义：day/night 两套全量色板
# 整体切换，usage_cfg.palette 可按昼夜分别覆盖；set_palette() 由
# theme.apply/set_night 落色）----
_ROW_RGB = ((14, 111, 196), (0, 137, 123), (108, 63, 194), (95, 102, 112))
_WARN_RGB = (200, 120, 0)          # >=80% 琥珀
_CRIT_RGB = (200, 0, 0)            # >=95% 红
TRACK_RGB = (236, 236, 239)
_OK_RGB = (0, 140, 0)
_STALE_RGB = (138, 145, 153)
_ERR_RGB = (200, 0, 0)
_TEXT_DIM_RGB = (64, 64, 64)
_PEAK_BIG_RGB = (200, 0, 0)        # 峰谷红（余额大字/总览余额行）
_OFFPEAK_BIG_RGB = (0, 140, 0)     # 峰谷绿
_CAPTION_RGB = (85, 90, 97)        # 灰标签/倒计时/时间条标注
_BG_RGB = (255, 255, 255)          # 昼夜套整体切换（theme.set_night 落色）
_FG_RGB = (0, 0, 0)
_TL_RGBS = None                    # 时间条四色（app 启动时写入）

BADGE_H = 32                        # 大字插图固定高（客户端默认，实际按 PNG 头）

PCT_WARN = 80.0
PCT_CRIT = 95.0
RESET_NEAR_S = 600                  # reset_at 前十分钟：倒计时闪烁
DELTA_SHOW_S = 60                   # Δ 显示时长
DELTA_MIN = 0.05                    # 小于该变化的 percent 抖动不显示
# 暗屏累计：暗屏轮只把增量并入挂起 Δ 显示（基准不推进），退出暗屏
# 的强刷一次结算——+1,+2,+1（入暗）+2,+0.1,+0.5（亮）→ 显示 +3.6

# display.png 单次解码需 ~44KB 连续堆块（pngdec inflate 32KB 窗口
# +行缓冲+w*h*2 输出）。真机诊断（diag_png v2）结论：正常路径与
# OOM 失败路径均零泄漏；开机裸堆 maxblock 仅 ~61KB（固件基线碎片化），
# fetch 后 JSON 树会进一步切碎堆。故不预检（mem_free 测的是总量，
# 判错量），直接 try/except：失败无害，本周期退避、下轮堆况好了重试

# 页头槽位（见模块 docstring）：名字 x4..x108，状态文字右对齐 x226，
# 状态点 x226；RSSI 角标在状态点正下方 y22（app 负责画，page 不碰）
NAME_X, NAME_MAX = 4, 13
STATUS_SLOT_X, STATUS_SLOT_W = 146, 80
DOT_X = 226

# 贴图槽宽度（display.png 不裁剪，超宽不贴、槽留空 + 串口诊断）
TITLE_SLOT_W = 140        # 页头名字槽：x4..x144（状态文字槽 x146 前）
ERR_SLOT_W = 232          # 错误行槽
LABEL_SLOT_W = 160        # 行标签槽：x8..x168（百分比文本 x176 起）
OV_ITEM_SLOT_W = 80       # 总览行名槽：x8..x88（迷你条 x96 起）

# 总览行右侧数值区（bundle 剩余 / balance 余额）：金额大字（端侧位图
# 字体 lib/digits 现场画，含 ¥/$ 符号——字符集已并入字体子集），
# 右对齐 x232。金额每轮变化 → 端侧现场（不贴图，内容寻址下贴图会
# 每次变化都触发新 hash 下载）。
AMT_RIGHT = 232
_AMT_CHAR_N = 11                          # 数字最多 11 字符

# 总览行内布局：条 x96 宽 64（收窄给右侧行内元素让位），百分比紧贴
# 条尾（用户定稿"条后面紧跟百分比，百分比后面跟时间"）
OV_PCT_X = 96 + 64 + 6                    # =166

# 档位表 {行数: (y0, step, bar_h, bar_dy, amount_inline)}
# 条下数额行高需求 = bar_dy + bar_h + 2 + 16；step 须 ≥ 需求，否则
# 下行标签贴图会盖住上行条下数额（2026-09-09 修：不用 inline——
# SSR 贴图清除矩形整槽宽会把同行右对齐数字盖掉，用户定稿保留条下
# 数额原布局，仅微调 y0/step：普通 3 行 (54,50) 恰好相接；峰谷 3 行
# (64,46,12,16) 压缩条高 2px + 条上移 2px 换出间距；峰谷 2 行 step 56)。
_LAYOUTS = {
    1: (96, 64, 28, 22, False),
    2: (70, 68, 18, 20, False),
    3: (54, 50, 14, 18, False),
    4: (54, 38, 12, 18, True),
}
# 密集 + 峰谷两小行：quota 区整体下移，步距压缩（换出峰谷 ~36px 空档）。
# 1 档 = 单窗 plan（qwen 谷价形态）：大条布局与普通 1 行档相同，只是
# 峰谷两小行占掉 title 下的 ~16px，配额区从 y0=96 起（2026-09-02 补）。
# 2 档 step 54→56：行高需求 56，原 54 差 2px 下行标签擦到上行数额。
# 3 档 (76,40,14,18)→(64,46,12,16)：原 40 差 10px 数额被下一行标签盖；
# 可用区 62..204 只有 142px，3 行需求 50 放不下——条高 14→12、条上移
# bar_dy 18→16 使需求降到 46，y0 76→64 贴峰谷行下缘换出底部空间。
_LAYOUTS_PEAK = {
    1: (96, 64, 28, 22, False),
    2: (86, 56, 18, 20, False),
    3: (64, 46, 12, 16, False),
    4: (70, 32, 12, 18, True),
}

# 稀疏型（≤1 quota：余额型/pack）纵向排布游标起点与间距
SPARSE_Y0 = 52
SPARSE_CAP_H = 18       # caption 标签行高（贴图随 ?h= 16 + 留 2 间隙）
SPARSE_GAP = 10         # 大字插图与下一元素间隙
BADGE_SLOT_W = 232      # 大字插图槽宽（"梁文峰" 3 字贴图 ~96px，留余量）
CAPTION_SLOT_W = 160    # caption 灰标签槽宽（x 左起，右半屏留给倒计时数字）

# 密集 + 峰谷行位置（页头之下，quota 区之上）：一行 = caption 贴图
# （"距切换计价还剩："）+ 两色倒计时（peak 红 / offpeak 绿）。
# 2026-09-04/06：无 second 行（badge_small_render 已停发）
PEAK_SMALL_Y = 46       # 峰谷行（caption 贴图 + 两色倒计时）

_CHAR_W = 8        # vga2_8x16 等宽字体字宽


def fmt_hms(seconds):
    """剩余秒 -> 倒计时文本（峰谷/时钟）。
    <24h：HH:MM:SS（固定 8 字符）；>=24h：`Nd HH:MM`（峰谷跨周末
    场景真实剩余 40+h，纯 HH:MM:SS 读不出天数——用户定稿加天位）；
    异常（负/超 99 天=时钟未同步）返回 '--:--:--'。"""
    if seconds is None or seconds < 0 or seconds > 99 * 86400:
        return '--:--:--'
    seconds = int(seconds)
    if seconds >= 24 * 3600:
        return '{}d {:02d}:{:02d}'.format(seconds // 86400,
                                          seconds // 3600 % 24,
                                          seconds // 60 % 60)
    return '{:02d}:{:02d}:{:02d}'.format(seconds // 3600,
                                         seconds // 60 % 60, seconds % 60)


# 徽标两态（2026-09-05）：error 徽标删除——失败一律 stale 或错误页，
# 错误详情走页头下 ASCII 单行 / 错误页，不上徽标
STATUS_TEXT = {'ok': 'ok'}
UNIT_SHORT = {'tokens': 'tok', 'requests': 'req', 'credits': 'crd', 'cny': ''}

# 夜间模式（app 按配置小时切换并 invalidate 全部页）：昼夜两套全量
# 配色方案整体切换——day 白底深字 / night 深底亮字（用户定稿 2026-08-29）
NIGHT = False


def BG():
    """背景色 565（当前昼夜套的页底，theme.set_night 切套）。"""
    return st7789.color565(*_BG_RGB)


def FG():
    """默认前景 565（与 BG 配对，随昼夜套切换）。"""
    return st7789.color565(*_FG_RGB)


def set_palette(c):
    """落色板（theme.apply 落 day 套 / theme.set_night 落对应套）：
    c 是 theme._one 产出的全量 RGB dict。"""
    global _ROW_RGB, _WARN_RGB, _CRIT_RGB, TRACK_RGB, _OK_RGB
    global _STALE_RGB, _ERR_RGB, _TEXT_DIM_RGB, _PEAK_BIG_RGB
    global _OFFPEAK_BIG_RGB, _CAPTION_RGB, _BG_RGB, _FG_RGB
    _ROW_RGB = c['rows']
    _WARN_RGB = c['warn']
    _CRIT_RGB = c['crit']
    TRACK_RGB = c['track']
    _OK_RGB = c['ok']
    _STALE_RGB = c['stale']
    _ERR_RGB = c['err']
    _TEXT_DIM_RGB = c['dim']
    _PEAK_BIG_RGB = c['peak']
    _OFFPEAK_BIG_RGB = c['offpeak']
    _CAPTION_RGB = c['caption']
    _BG_RGB = c['bg']
    _FG_RGB = c['fg']


def _col(rgb):
    return st7789.color565(*rgb)


def _status_rgb(status):
    """徽标两态：ok 绿 / 其余（stale）灰——error 红徽标已删除。"""
    return _OK_RGB if status == 'ok' else _STALE_RGB


def _bar_rgb(percent):
    """阈值变色：>=95 红 / >=80 琥珀 / None=用行本色。"""
    if percent >= PCT_CRIT:
        return _CRIT_RGB
    if percent >= PCT_WARN:
        return _WARN_RGB
    return None


def _blink_on(now_epoch):
    """500ms 闪烁相位（用秒时钟，免额外传 ticks）。"""
    return int(now_epoch * 2) % 2 == 0


def _fill_w(percent, bar_w=BAR_W):
    w = int(bar_w * min(max(percent, 0.0), 100.0) / 100)
    if 0 < w < 1:
        w = 1
    return w


def fmt_countdown(seconds):
    """reset_at 剩余时间 -> 短文本（quota 行槽 <=8 字符）。

    冒号只表示 mm:ss（语义单一化，2026-08-31 定稿）：
    - <1h：MM:SS（如 11:05）——唯一冒号形态
    - 1h..24h：XhYm（0d0h0m0s 取最高非零起两截，如 2h3m）
    - >=24h：XdYh（如 1d16h）
    - None/负：'--'
    """
    if seconds is None or seconds < 0:
        return '--'
    seconds = int(seconds)
    if seconds < 3600:
        return '{:02d}:{:02d}'.format(seconds // 60, seconds % 60)
    if seconds < 86400:
        return '{}h{}m'.format(seconds // 3600, seconds // 60 % 60)
    return '{}d{}h'.format(seconds // 86400, seconds // 3600 % 24)


def fmt_amount(n):
    """used/limit 数字 -> 短文本（k/m 缩写，<=7 字符）。"""
    if n is None:
        return '?'
    if n >= 1_000_000:
        return '{:.1f}m'.format(n / 1_000_000)
    if n >= 10_000:
        return '{:.0f}k'.format(n / 1_000)
    if n >= 1_000:
        return '{:.1f}k'.format(n / 1_000)
    return '{}'.format(n)


def _erase_text(display, x, y, n_chars, h):
    display.fill_rect(x, y, _CHAR_W * n_chars, h, BG())


def _swap_text(display, font, old, new, x, y, color=None):
    """擦旧串区域（按新旧较长者）再写新串。返回新串。"""
    if old == new:
        return old
    _erase_text(display, x, y, max(len(old), len(new)), font.HEIGHT)
    display.text(font, new, x, y, FG() if color is None else _col(color),
                 BG())
    return new


def _ascii(s):
    """vga2_8x16 只能画 ASCII；服务端下发的中文名（总览/GLM 余额…）
    走 display.text 就是乱码。非纯 ASCII 的 fallback 一律不留字
    （槽位空着等 SSR 贴图，而不是闪一帧乱码）。"""
    if s and all(32 <= ord(ch) < 127 for ch in s):
        return s
    return None


# ---- 大数字（端侧位图字体 lib/digits，display.write 直推 SPI）----
# 用于余额大字：金额数值频繁变化，贴图会每次变化都触发新 hash 下载
# （违背内容寻址零传输），位图字体把数字绘制成本降到纯 SPI 写。
# 字符集含 ¥/$/0-9/.（字体子集），货币符号与大数字同字体一次画出。


def _digits_only(s):
    """过滤出 digits 字体字符集内的字符（¥/$/0-9/. 与字母等字符集外
    字符丢弃——字符集外字符固件会静默跳过导致右对齐漂移）。"""
    if not s:
        return ''
    if bigfont is None:
        return ''.join(c for c in str(s) if 32 <= ord(c) < 127)
    allowed = set(bigfont.MAP)
    return ''.join(c for c in str(s) if c in allowed)


# 币种码 → TrueType 符号字形（lib/digits 字符集内：¥/$）。符号并入
# 金额大字串同色同字体（用户定稿：余额页大字 = 彩色货币符号+数字）；
# 未列出的币种码无字形，只画数字。
_CURRENCY_GLYPH = {'CNY': '¥', 'JPY': '¥', 'USD': '$'}


def _currency_code(currency):
    """币种码 → 总览行 ASCII 文本（CNY/USD…）；空/非 ASCII 字母数字
    返回 ''。（MicroPython str 无 isascii/isalnum，只能逐字符 ord 判）"""
    cur = str(currency or '').strip().upper()
    if not cur:
        return ''
    for ch in cur:
        o = ord(ch)
        if not (48 <= o <= 57 or 65 <= o <= 90):   # 0-9 A-Z
            return ''
    return cur


def _currency_glyph(currency):
    """币种码 → 符号字形（'¥'/'$'）；无字形币种返回 ''。"""
    cur = str(currency or '').strip().upper()
    if not cur:
        return ''
    sym = _CURRENCY_GLYPH.get(cur)
    if sym is None:
        # 兼容直接下发符号（如 '¥'）而非币种码
        sym = cur if cur in _CURRENCY_GLYPH.values() \
            and (bigfont is None or cur in set(bigfont.MAP)) else ''
    return sym


def _draw_big(display, font, text, right, y, color,
              fallback_color=None, currency=''):
    """右对齐画余额大字组（lib/digits TrueType 位图字体）：
    货币符号（¥/$，有字形币种）+ 金额数字，**同色同字体一次 write**
    （用户定稿：彩色货币符号+数字，色随峰谷红/绿/灰）。

    right = 文本右缘 x。先按宽度清矩形再写；text 应已过 _digits_only。
    digits 字体缺失时退回 vga 灰 ASCII（符号字形丢失则只画数字）。
    返回实际绘制文本（None=没画）。"""
    sym = _currency_glyph(currency)
    text = sym + text if sym else text
    if not text:
        return None
    if bigfont is not None:
        try:
            w = display.write_len(bigfont, text)
            display.fill_rect(right - w, y, w, bigfont.HEIGHT, BG())
            display.write(bigfont, text, right - w, y, _col(color), BG())
            return text
        except AttributeError:
            pass                    # 宿主 stub display：走灰字兑底
    fb = text if _ascii(text) else None
    if fb:
        _erase_text(display, right - len(fb) * _CHAR_W, y, len(fb),
                    font.HEIGHT)
        display.text(font, fb, right - len(fb) * _CHAR_W, y,
                     _col(fallback_color or _TEXT_DIM_RGB), BG())
    return fb or None


def _erase_big(display, font, text, right, y, currency=''):
    """擦掉 _draw_big 画的内容（按记录的旧文本与右缘）。"""
    if not text:
        return
    sym = _currency_glyph(currency)
    text = sym + text if sym and not text.startswith(sym) else text
    if bigfont is not None:
        try:
            w = display.write_len(bigfont, text)
            display.fill_rect(right - w, y, w, bigfont.HEIGHT, BG())
            return
        except AttributeError:
            pass
    _erase_text(display, right - len(text) * _CHAR_W, y, len(text),
                font.HEIGHT)


def _paint_img(display, store, h, max_w, x, y, clear_h=None,
               clear_w=None):
    """贴图槽：缓存命中且宽度合格 → display.png 贴文件；否则槽位置
    空等下轮平补刷新（不 ASCII 回退——用户定稿），同步串口诊断由
    store.note_miss/note_oom 去重打印。返回实际画到的 hash（没贴图
    为 None）。

    clear_h：清除矩形高度。文本类贴图默认 font.HEIGHT；大字插图
    （峰谷/余额固定高）传入 store.height(h) 或 BADGE_H，避免只擦
    掉半截旧图。

    clear_w：清除矩形宽度（缺省整槽 max_w）。总览货币符号按位图
    实宽窄清除，避免宽清除矩形抹到紧邻右侧的金额数字。

    display.png 解码要一次性申请大缓冲（实测 ~44KB），堆紧时抛
    MemoryError——先 gc.collect 挤一挤，还失败就登记 OOM 本周期
    退避（下轮 diff 会再试贴，缓存命中后成本为零）。"""
    if clear_h is None:
        clear_h = 16             # vga2_8x16 行高（文本槽缺省清除高度）
    display.fill_rect(x, y, max_w if clear_w is None else clear_w,
                      clear_h, BG())
    p = store.usable(h, max_w) if (h and store) else None
    if p:
        gc.collect()
        try:
            display.png(p, x, y)    # 不透明 PNG（服务端已与 bg 合成）
            return h
        except MemoryError:
            # 堆碎片化下拿不到 ~44KB 连续块（诊断已证失败路径无泄漏；
            # 每周期重试成本低且堆况会变，串口诊断由 store 去重只打首次）
            if store:
                store.note_oom(h)   # 本周期退避；下轮 missing() 重置重试
    elif h and store:
        store.note_miss(h)          # 未下载/下载失败/超宽：留空 + 串口报错
    return None


class ProviderPage:
    def __init__(self, name):
        self.pid = None             # 稳定 id（type=provider 响应的 id 回显）
        self.name = name
        self.status = 'ok'          # 徽标两态（ok/stale）；失败由 app 标
        self.error_text = ''
        self.updated_at = None
        self.title_hash = None      # SSR 贴图 hash（未下载/非法→ASCII）
        self.rows = []              # [(label, pct, reset_at, amount, rhash)]
        self.balance = None         # {'currency','render','caption_render'}
        self.peak = None            # {'state','badge_render','caption_render',
        #                             'ends_at'}
        #   - 密集（plan/bundle，≥1 quota）：caption_render（"距切换计价还剩："）
        #     + state（两色倒计时红/绿）+ ends_at
        #   - 稀疏（balance，0 quota）：state + badge_render + caption_render
        self._sparse = False
        # 增量渲染状态
        self._sig = None            # 布局签名：(name, hashes, night, 块)
        self._lay = _LAYOUTS[4]     # 上次使用的档位
        self._last = []             # 每行 [pct, fill_w, color, cd, amt, delta]
        self._dot_rgb = None
        self._status_text = None    # 已画出的状态文字
        self._err_drawn = None      # 已画出的错误行文本（None=没画）
        self._title_img = None      # 已贴的标题/标签贴图 hash
        self._label_imgs = []
        # 端侧 ASCII 现场（每秒变化，不贴图）
        self._peak_cd_drawn = None  # 峰谷倒计时数字（两色：peak 红/offpeak 绿）
        self._peak_cd_color = None
        # 新增块贴图补贴状态（稀疏大字 / 峰谷/余额 caption）
        self._badge_img = None      # peak.badge_render（稀疏大字）
        self._peak_cap_img = None   # peak.caption_render（峰谷灰标签）
        self._bal_cap_img = None    # balance.caption_render
        self._bal_txt_drawn = None  # 已画的余额大数字文本（_draw_big 返回）
        self._bal_big_right = 232   # 大数字右缘 x（diff 擦除基准）
        # Δ 显示（内存态，不写 flash）
        self._prev_pct = {}         # label -> 上次 fetch 的 percent
        self._deltas = {}           # label -> ('+0.3', until_epoch)
        # 双套贴图 hash 记忆（'tD'/'tN'…：槽位+昼夜 tag → hash）：fetch
        # 时顺路登记本套 hash；昼夜切换 swap_renders 回填另一套，渲染
        # 直接读 rcache（双套共存）——零网络零下载（2026-09-06 定稿）
        self._alt = {}

    # ---- 数据更新（只改数据，不触碰显示）----

    def set_error_keep(self, message=''):
        """失败统一入口（2026-09-05）：标 stale 保留旧数据 + 页头下
        ASCII 单行错误（app 按 loaded 分流——未加载过直接错误页，
        清空式 set_error 已删除）。文本 ASCII 防护：非 ASCII 不留字；
        限宽 ERR_MAX_CHARS（止步 RSSI 角标左缘，不叠字）。"""
        self.status = 'stale'
        self.error_text = _ascii((message or '')[:ERR_MAX_CHARS]) or ''

    def update(self, pdata, now_epoch, dark=False, night=False):
        """pdata：type=provider 的当前页数据（GET /api/page）。
        dark：本响应来自暗屏期轮询（app 传 dim 态）——percent 增量
        累计进挂起 Δ 显示，不推进基准；亮屏轮一次性结算累计。
        night：本次响应的渲染套（hash 含 theme）——顺路登记进 _alt
        记忆库，昼夜切换 swap_renders 回填另一套时用。"""
        # 徽标两态：status 只收 ok/stale（服务器 status=error 由 app
        # 拦截转 FetchError，这里归一 stale 纯防御）
        st = str(pdata.get('status') or 'ok')
        self.status = st if st in ('ok', 'stale') else 'stale'
        self.updated_at = pdata.get('updated_at')
        self.title_hash = pdata.get('title_render')
        err = pdata.get('error')
        if isinstance(err, dict):    # 结构化 error：只取 machine code
            # 2026-09-02：error.render 停发——错误统一端侧 ASCII 单行
            err = err.get('code') or 'error'
        self.error_text = _ascii(str(err or '')[:ERR_MAX_CHARS]) or ''

        # balance 块（余额大字插图 + 灰标签 + 金额纯文本降级）
        bal = pdata.get('balance')
        if isinstance(bal, dict):
            self.balance = {'currency': str(bal.get('currency', '')),
                            'render': bal.get('render'),
                            'caption_render': bal.get('caption_render'),
                            'amount_text': (str(bal.get('amount_text') or '')
                                            [:14])}
        else:
            self.balance = None

        # peak 块（state + caption_render + ends_at；badge_render 仅稀疏余额
        # 型下发；badge_small_render 已停发——2026-09-04/06）
        pk = pdata.get('peak')
        if isinstance(pk, dict):
            self.peak = {'state': str(pk.get('state', '') or ''),
                         'badge_render': pk.get('badge_render'),
                         'caption_render': pk.get('caption_render'),
                         'ends_at': pk.get('ends_at')}
        else:
            self.peak = None

        rows = []
        rows_alt = []
        for quota in pdata.get('quotas') or []:
            window = str(quota.get('id', '?')).split(':')[-1]
            unit = UNIT_SHORT.get(quota.get('unit'), '')
            label = '{} {}'.format(window, unit).rstrip()
            try:
                percent = float(quota.get('percent', 0.0))
            except (TypeError, ValueError):
                percent = 0.0
            # 已用/总量小字：优先服务端格式化文本（SCHEMA amount_text，
            # 2026-08-31 起 k/m 缩写归服务端）；缺失（旧服务器）回退
            # 端侧 fmt_amount 自算——回退式样与服务端缺省逐字相同
            amount = str(quota.get('amount_text') or '')
            if not amount:
                amount = '{} / {}'.format(
                    fmt_amount(quota.get('used')),
                    fmt_amount(quota.get('limit')))
            reset_at = quota.get('reset_at')
            rows.append((label, percent, reset_at, amount,
                         quota.get('label_render')))
            rows_alt.append(quota.get('label_render_alt'))
        self._apply_deltas(rows, now_epoch, dark)
        self.rows = rows

        # 稀疏型判定：quota 为空（纯余额型）且（有余额或有峰谷）→ 大字插图竖排；
        # pack（1 quota，无峰谷大字的度量型）走既有普通布局——与 plan 单条一致。
        # 有 quota 且有峰谷 → 密集型：峰谷退化为 title 下两小行。
        self._sparse = (len(rows) == 0 and (self.balance is not None
                                           or self.peak is not None))
        # 双套记忆登记（槽位+tag → hash）：主 hash 归本套 tag，alt
        # hash 归对偶 tag——双发契约（render.check_alt 已拦缺 alt 的
        # 旧服务器）下每轮 fetch 两套都齐，不靠跨轮历史积累
        tag = 'N' if night else 'D'
        atag = 'D' if night else 'N'
        alt = self._alt
        if self.title_hash:
            alt['t' + tag] = self.title_hash
        if pdata.get('title_render_alt'):
            alt['t' + atag] = pdata['title_render_alt']
        for i, r in enumerate(rows):
            if r[4]:
                alt['r{}{}'.format(i, tag)] = r[4]
                if i < len(rows_alt) and rows_alt[i]:
                    alt['r{}{}'.format(i, atag)] = rows_alt[i]
        if self.balance is not None:
            if self.balance.get('caption_render'):
                alt['bc' + tag] = self.balance['caption_render']
            if isinstance(bal, dict) and bal.get('caption_render_alt'):
                alt['bc' + atag] = bal['caption_render_alt']
        if self.peak is not None:
            if self.peak.get('badge_render'):
                alt['pb' + tag] = self.peak['badge_render']
            if isinstance(pk, dict) and pk.get('badge_render_alt'):
                alt['pb' + atag] = pk['badge_render_alt']
            if self.peak.get('caption_render'):
                alt['pc' + tag] = self.peak['caption_render']
            if isinstance(pk, dict) and pk.get('caption_render_alt'):
                alt['pc' + atag] = pk['caption_render_alt']

    def _apply_deltas(self, rows, now_epoch, dark=False):
        """记录同窗口 percent 的变化；未过期的旧 Δ 保留继续显示。
        dark=True：暗屏累计——增量并入挂起 Δ（含自增的门槛、过期续期
        到下一个亮屏轮），不推进基准；退出暗屏的亮屏轮（dark=False）
        一次性结算挂起值（+2,+0.1,+0.5 → +3.6），其后与普通轮相同。"""
        deltas = {}
        labels = set()
        for label, percent, _r, _a, _h in rows:
            labels.add(label)
            old = self._prev_pct.get(label)
            if old is None:
                continue
            diff = percent - old
            if abs(diff) < DELTA_MIN:
                continue
            if dark:
                entry = self._deltas.get(label)
                if entry is not None and entry[1] > now_epoch:
                    try:                    # 挂起值折回 float 再自增
                        acc = float(entry[0])
                    except (TypeError, ValueError):
                        acc = 0.0
                else:
                    acc = 0.0               # 新一轮累计
                acc += diff
                deltas[label] = ('{}{:.1f}'.format(
                    '+' if acc >= 0 else '-', abs(acc)),
                    now_epoch + DELTA_SHOW_S)
            else:
                sign = '+' if diff > 0 else '-'
                deltas[label] = ('{}{:.1f}'.format(sign, abs(diff)),
                                 now_epoch + DELTA_SHOW_S)
        for label, entry in self._deltas.items():
            if label not in deltas and label in labels and entry[1] > now_epoch:
                deltas[label] = entry
        self._deltas = deltas
        if not dark:
            self._prev_pct = {label: pct
                              for label, pct, _r, _a, _h in rows}

    # ---- 渲染 ----

    def invalidate(self):
        """强制下次全量重画（翻页回来、外部动了屏幕、昼夜切换）。"""
        self._sig = None

    def swap_renders(self, night):
        """昼夜切换：贴图槽整体换到另一套 hash（_alt 记忆库回填）。
        hash 含 theme/bg——两套图内容不同 hash 不同，但 rcache 双套
        共存，命中即零网络直接贴（服务端也不重渲染：同内容 hash 缓存
        命中）。记忆缺失的槽置空（绝不留旧套图贴新底色）；app 切换
        后经 render_slots 检查，任一槽缺失即立即 fetch 补齐（见
        app 夜间切换段）。数据值（percent/倒计时/金额）不变。"""
        tag = 'N' if night else 'D'
        alt = self._alt
        self.title_hash = alt.get('t' + tag)
        self.rows = [(l, p, r, a, alt.get('r{}{}'.format(i, tag)))
                     for i, (l, p, r, a, _h) in enumerate(self.rows)]
        if self.balance is not None:
            self.balance['caption_render'] = alt.get('bc' + tag)
        if self.peak is not None:
            self.peak['badge_render'] = alt.get('pb' + tag)
            self.peak['caption_render'] = alt.get('pc' + tag)

    def render_slots(self):
        """当前页应占的贴图槽位 [(hash|None, slot_w), ...]（含 None=
        该槽无 hash）。app 昼夜切换 swap 后据此判缺失并带 ?w= 下载：
        全槽在 rcache → 零网络；否则仅补缺失贴图，不碰数据端点。"""
        out = [(self.title_hash, TITLE_SLOT_W)]
        out += [(r[4], LABEL_SLOT_W) for r in self.rows]
        if self.balance is not None:
            out.append((self.balance.get('caption_render'),
                        CAPTION_SLOT_W))
        if self.peak is not None:
            out.append((self.peak.get('caption_render'), CAPTION_SLOT_W))
            if self._sparse:
                out.append((self.peak.get('badge_render'), BADGE_SLOT_W))
        return out

    def render(self, display, font, now_epoch, store=None):
        """返回 True 表示发生了全量重画（app 据此重画 RSSI 角标）。"""
        bal = (self.balance or {})
        pk = (self.peak or {})
        # sig 含 status/error_text：stale 恢复 ok 时（服务端正常返回，
        # 不走 broken 全量重画）必须全量重画擦掉残留错误行/状态点。
        # updated_at 不进 sig（每次 fetch 都变，且走 diff 状态年龄）。
        # 峰谷签名：state（两色倒计时红色变化）+ caption_render + 大字
        # badge（仅稀疏）+ ends_at。badge_small_render 已停发。
        sig = (self.name, self.title_hash, NIGHT, self.status,
               self.error_text,
               tuple((r[0], r[4]) for r in self.rows),
               bal.get('caption_render'), bal.get('amount_text'),
               pk.get('state'), pk.get('caption_render'),
               (pk.get('badge_render') if self._sparse else None),
               pk.get('ends_at'), self._sparse)
        if sig != self._sig:
            self._render_full(display, font, sig, now_epoch, store)
            return True
        self._render_diff(display, font, now_epoch, store)
        return False

    def _status_str(self, now_epoch):
        if self.status == 'stale' and isinstance(self.updated_at,
                                                 (int, float)):
            age = int(now_epoch - self.updated_at)
            return 'stale {}m'.format(min(max(age // 60, 0), 99))
        return STATUS_TEXT.get(self.status, 'stale')

    def _draw_status(self, display, font, text, rgb):
        display.fill_rect(STATUS_SLOT_X, 6, STATUS_SLOT_W, font.HEIGHT,
                          BG())
        display.text(font, text, DOT_X - (len(text) + 1) * _CHAR_W, 6,
                     _col(rgb), BG())

    def _render_full(self, display, font, sig, now_epoch, store=None):
        self._sig = sig
        display.fill(BG())

        # 页头：标题贴图（缺失/失败留空，不 ASCII 回退）+ 状态点 + 状态文字
        self._title_img = _paint_img(display, store, self.title_hash,
                                     TITLE_SLOT_W, NAME_X, 6)
        rgb = _status_rgb(self.status)
        self._dot_rgb = rgb
        display.fill_rect(DOT_X, 8, 10, 10, _col(rgb))
        self._status_text = self._status_str(now_epoch)
        self._draw_status(display, font, self._status_text, rgb)

        # 错误行（status≠ok）：统一 ASCII 单行 error.code（可截断；2026-09-02
        # 定稿——error 不入 SSR 贴图，动态非预期不占 rcache）
        self._err_drawn = None
        if self.status != 'ok' and self.error_text:
            display.text(font, self.error_text, 4, ERR_Y, FG(), BG())
            self._err_drawn = self.error_text

        # 数据块：稀疏型（余额/峰谷大字插图）或密集型（配额行）
        if self._sparse:
            self._render_sparse_full(display, font, now_epoch, store)
        else:
            self._render_rows_full(display, font, now_epoch, store)

    def _render_rows_full(self, display, font, now_epoch, store=None):
        """密集型：配额行（多 quota plan）或 pack/plan 单条（与 plan 一致）。
        >=1 quota 且有 peak 时，title 下先插峰谷行（caption 贴图 + 两色
        倒计时）并压缩配额区（2026-09-06：一行，无 badge_small 第二行）。"""
        has_small_peak = (len(self.rows) >= 1 and self.peak is not None)

        # 峰谷行（密集 + peak）：caption_render 灰标签 + 两色倒计时
        if has_small_peak:
            self._render_peak_small(display, font, now_epoch, store)
            self._lay = _LAYOUTS_PEAK.get(len(self.rows), _LAYOUTS_PEAK[4])
        else:
            self._lay = _LAYOUTS.get(len(self.rows), _LAYOUTS[4])

        y0, step, bar_h, bar_dy, inline = self._lay
        self._last = []
        self._label_imgs = []
        for i in range(len(self.rows)):
            y = y0 + i * step
            ybar = y + bar_dy
            label, percent, reset_at, amount, rhash = self.rows[i]
            color = _col(_bar_rgb(percent) or _ROW_RGB[i % 4])
            display.fill_rect(BAR_X, ybar, BAR_W, bar_h, _col(TRACK_RGB))
            self._label_imgs.append(
                _paint_img(display, store, rhash, LABEL_SLOT_W, BAR_X, y))
            pct = self._pct_str(percent, now_epoch)
            if pct:
                display.text(font, pct, BAR_X + 168, y, color, BG())
            fill_w = _fill_w(percent)
            if fill_w > 0:
                display.fill_rect(BAR_X, ybar, fill_w, bar_h, color)
            cd = self._cd_str(reset_at, now_epoch)
            if cd:
                display.text(font, cd, BAR_X + BAR_W + 8, ybar,
                             color, BG())
            if inline:              # 4 行紧凑档：数额与标签同行右对齐
                display.text(font, amount, 168 - len(amount) * _CHAR_W, y,
                             _col(_TEXT_DIM_RGB), BG())
                delta = ''
            else:
                display.text(font, amount, BAR_X, ybar + bar_h + 2,
                             _col(_TEXT_DIM_RGB), BG())
                delta = self._delta_str(label, now_epoch)
                if delta:
                    display.text(font, delta, BAR_X + 168, ybar + bar_h + 2,
                                 color, BG())
            self._last.append([pct, fill_w, color, cd, amount, delta])

    def _peak_cd(self, now_epoch):
        """峰谷倒计时秒（ends_at - now）；无 peak 返回 None。"""
        if not self.peak or not self.peak.get('ends_at'):
            return None
        return self.peak['ends_at'] - now_epoch

    def _peak_cd_str(self, now_epoch):
        cd = self._peak_cd(now_epoch)
        return None if cd is None else fmt_hms(cd)

    def _bal_rgb(self):
        """余额大字语义色（RGB，_draw_big 内部再转 565）：带峰谷随
        峰谷红/绿，无峰谷灰（用户定稿）。"""
        if self.peak is None:
            return _TEXT_DIM_RGB
        return _PEAK_BIG_RGB if self.peak.get('state') == 'peak' \
            else _OFFPEAK_BIG_RGB

    def _peak_cd_rgb(self):
        """峰谷倒计时语义色（RGB）：peak 红 / offpeak 绿（两色时间，
        2026-09-06 定稿——倒计时随当前状态染色）。"""
        if self.peak is None:
            return _CAPTION_RGB
        return _PEAK_BIG_RGB if self.peak.get('state') == 'peak' \
            else _OFFPEAK_BIG_RGB

    def _render_peak_small(self, display, font, now_epoch, store=None):
        """密集 + peak（plan/bundle）：**一行** = caption_render 灰标签
        贴图（"距切换计价还剩："，SSR——中文端侧画不了）左置 + 两色
        倒计时（peak 红 / offpeak 绿）右对齐（2026-09-06 用户定稿：
        非余额类峰谷不占大字 SSR，只此一行）。caption 缺失留空等下载，
        不 ASCII 回退。"""
        cap = self.peak.get('caption_render')
        self._peak_cap_img = _paint_img(display, store, cap,
                                        CAPTION_SLOT_W, 8, PEAK_SMALL_Y)
        cd = self._peak_cd_str(now_epoch)
        rgb = self._peak_cd_rgb()
        self._peak_cd_drawn = cd
        self._peak_cd_color = rgb
        if cd:
            display.text(font, cd, 232 - len(cd) * _CHAR_W, PEAK_SMALL_Y,
                         _col(rgb), BG())

    def _render_sparse_full(self, display, font, now_epoch, store=None):
        """稀疏型（余额型 provider）：峰谷 caption+大字 / 余额 caption+大字
        自上而下竖排。峰谷大字 = SSR 插图（固定高，按 PNG 头，缺省 BADGE_H）；
        余额大数字 = 端侧位图字体 lib/digits 现场画（数字每轮变化，贴图
        会每次触发新 hash 下载；符号 ¥ 走 SSR 灰色小图贴在左侧）。

        大字峰谷不做 ASCII 退避（用户定稿）：贴图缺失一律留空等下轮补贴。
        余额大数字的金额文本是数字（端侧可画），始终现场画不依赖 SSR；
        digits 字体缺失时退灰 ASCII。语义色：带峰谷的余额数字随峰谷
        红绿；无峰谷灰色（用户定稿）。
        """
        y = SPARSE_Y0

        # 峰谷：灰标签 caption_render + 红/绿大字 badge_render
        self._peak_cap_img = None
        self._badge_img = None
        self._peak_cd_drawn = None
        self._peak_cd_color = None
        self._bal_cap_img = None
        self._bal_txt_drawn = None

        if self.peak is not None:
            cap = self.peak.get('caption_render')
            # caption 缺失留空（状态语义由红绿大字贴图承载）
            self._peak_cap_img = _paint_img(
                display, store, cap, CAPTION_SLOT_W, 8, y)
            y += SPARSE_CAP_H
            # 倒计时数字放右侧（caption 左，数字右对齐，随状态两色）
            cd = self._peak_cd_str(now_epoch)
            rgb = self._peak_cd_rgb()
            if cd:
                self._peak_cd_drawn = cd
                self._peak_cd_color = rgb
                display.text(font, cd, 232 - len(cd) * _CHAR_W,
                             y - SPARSE_CAP_H, _col(rgb), BG())
            self._badge_img = _paint_img(
                display, store, self.peak.get('badge_render'), BADGE_SLOT_W,
                8, y,
                clear_h=(store.height(self.peak.get('badge_render'))
                         if store else None) or BADGE_H)
            y += (store.height(self.peak.get('badge_render'))
                  if store else None) or BADGE_H
            y += SPARSE_GAP

        # 余额：灰标签 caption_render + 端侧大字组（彩色货币符号+数字，
        # TrueType 子集整串同色——用户定稿）。
        # （峰谷语义色：peak 红 / offpeak 绿 / 无峰谷灰——用户定稿）
        if self.balance is not None:
            self._bal_cap_img = _paint_img(
                display, store, self.balance.get('caption_render'),
                CAPTION_SLOT_W, 8, y)
            y += SPARSE_CAP_H
            amt = _digits_only(self.balance.get('amount_text') or '')
            cur = self.balance.get('currency')
            self._bal_txt_drawn = (amt, cur)   # 存渲染输入意图（diff 键）
            _draw_big(display, font, amt, 232, y, self._bal_rgb(),
                      fallback_color=_TEXT_DIM_RGB, currency=cur)

    def _pct_str(self, percent, now_epoch):
        if percent >= PCT_CRIT and not _blink_on(now_epoch):
            return ''               # 临界闪烁：灭相
        return '{:.1f}%'.format(percent)

    def _cd_str(self, reset_at, now_epoch):
        if not reset_at:
            return None
        left = reset_at - now_epoch
        if 0 < left <= RESET_NEAR_S and not _blink_on(now_epoch):
            return None             # 临近重置：闪烁灭相
        return fmt_countdown(left)

    def _delta_str(self, label, now_epoch):
        entry = self._deltas.get(label)
        if entry and entry[1] > now_epoch:
            return entry[0]
        return ''

    def _render_diff(self, display, font, now_epoch, store=None):
        # 页头标题贴图（下载完成后补贴）
        if self.title_hash and store:
            now_img = (self.title_hash
                       if store.usable(self.title_hash, TITLE_SLOT_W)
                       else None)
            if now_img != self._title_img:
                self._title_img = _paint_img(display, store,
                                             self.title_hash, TITLE_SLOT_W,
                                             NAME_X, 6)

        # 页头状态（stale 年龄走字也走这里）
        rgb = _status_rgb(self.status)
        stext = self._status_str(now_epoch)
        if stext != self._status_text or rgb != self._dot_rgb:
            self._draw_status(display, font, stext, rgb)
            display.fill_rect(DOT_X, 8, 10, 10, _col(rgb))
            self._status_text = stext
            self._dot_rgb = rgb

        # 错误行（status≠ok）：统一 ASCII 单行 error.code（内容变化才重画）
        if self.status != 'ok':
            want_err = self.error_text
            if want_err != self._err_drawn:
                if self._err_drawn:
                    _erase_text(display, 4, ERR_Y, len(self._err_drawn),
                                font.HEIGHT)
                if want_err:
                    display.text(font, want_err, 4, ERR_Y, FG(), BG())
                self._err_drawn = want_err

        if self._sparse:
            self._render_sparse_diff(display, font, now_epoch, store)
        else:
            self._render_rows_diff(display, font, now_epoch, store)

    def _render_rows_diff(self, display, font, now_epoch, store=None):
        # 密集 + peak 峰谷行（caption 补贴 + 两色倒计时走字）；
        # 单窗 plan（1 条 quota + 峰谷，qwen 谷价形态）同样走这里
        if len(self.rows) >= 1 and self.peak is not None:
            cap = self.peak.get('caption_render')
            if cap and store:
                now_cap = cap if store.usable(cap, CAPTION_SLOT_W) else None
                if now_cap != self._peak_cap_img:
                    self._peak_cap_img = _paint_img(
                        display, store, cap, CAPTION_SLOT_W, 8,
                        PEAK_SMALL_Y)
            cd = self._peak_cd_str(now_epoch)
            rgb = self._peak_cd_rgb()
            if cd != self._peak_cd_drawn or rgb != self._peak_cd_color:
                _swap_text(display, font, self._peak_cd_drawn or '', cd,
                           232 - len(cd) * _CHAR_W, PEAK_SMALL_Y, rgb)
                self._peak_cd_drawn = cd
                self._peak_cd_color = rgb

        # 配额行
        y0, step, bar_h, bar_dy, inline = self._lay
        for i in range(len(self.rows)):
            y = y0 + i * step
            ybar = y + bar_dy
            label, percent, reset_at, amount, rhash = self.rows[i]
            pct_old, fill_old, color_old, cd_old, amt_old, dlt_old = \
                self._last[i]
            color = _col(_bar_rgb(percent) or _ROW_RGB[i % 4])

            if rhash and store:      # 标签贴图补贴
                now_img = (rhash if store.usable(rhash, LABEL_SLOT_W)
                           else None)
                if now_img != self._label_imgs[i]:
                    self._label_imgs[i] = _paint_img(
                        display, store, rhash, LABEL_SLOT_W, BAR_X, y)

            pct_new = self._pct_str(percent, now_epoch)
            if pct_new != pct_old:
                _swap_text(display, font, pct_old, pct_new,
                           BAR_X + 168, y, _bar_rgb(percent)
                           or _ROW_RGB[i % 4])

            fill_new = _fill_w(percent)
            if fill_new > fill_old:            # 增长：补色
                display.fill_rect(BAR_X + fill_old, ybar,
                                  fill_new - fill_old, bar_h, color)
            elif fill_new < fill_old:          # 回落：擦回轨道色
                display.fill_rect(BAR_X + fill_new, ybar,
                                  fill_old - fill_new, bar_h, _col(TRACK_RGB))
            elif color != color_old and fill_new > 0:
                display.fill_rect(BAR_X, ybar, fill_new, bar_h, color)

            cd_new = self._cd_str(reset_at, now_epoch)
            if cd_new != cd_old:
                x = BAR_X + BAR_W + 8
                if cd_old:
                    _erase_text(display, x, ybar, len(cd_old), font.HEIGHT)
                if cd_new:
                    display.text(font, cd_new, x, ybar, color, BG())

            if amount != amt_old:
                if inline:          # 右对齐文本长度会变：精确擦旧新最大宽
                    # （仅 4 行紧凑档；2026-09-09：旧固定区 x64..168 会
                    # 擦掉标签贴图右半，改按文本实宽擦除）
                    old_w = len(amt_old) * _CHAR_W
                    new_w = len(amount) * _CHAR_W
                    w = max(old_w, new_w)
                    display.fill_rect(168 - w, y, w, font.HEIGHT, BG())
                    display.text(font, amount,
                                 168 - new_w, y,
                                 _col(_TEXT_DIM_RGB), BG())
                else:
                    _swap_text(display, font, amt_old, amount,
                               BAR_X, ybar + bar_h + 2, _TEXT_DIM_RGB)

            dlt_new = dlt_old
            if not inline:
                dlt_new = self._delta_str(label, now_epoch)
                if dlt_new != dlt_old:
                    _swap_text(display, font, dlt_old, dlt_new,
                               BAR_X + 168, ybar + bar_h + 2,
                               _bar_rgb(percent) or _ROW_RGB[i % 4])

            self._last[i] = [pct_new, fill_new, color, cd_new, amount,
                             dlt_new]

    def _peak_badge_y(self):
        """稀疏型峰谷大字插图 y 坐标（含峰谷 caption/大字块）。"""
        return SPARSE_Y0 + SPARSE_CAP_H

    def _peak_badge_h(self, store):
        return ((store.height(self.peak.get('badge_render'))
                 if store else None) or BADGE_H)

    def _balance_block_y(self, store):
        """稀疏型余额块起始 y（峰谷块之下）。"""
        y = SPARSE_Y0
        if self.peak is not None:
            y += SPARSE_CAP_H + self._peak_badge_h(store) + SPARSE_GAP
        return y

    def _render_sparse_diff(self, display, font, now_epoch, store=None):
        # 峰谷：caption 补贴 + 倒计时走字 + 大字插图补贴
        if self.peak is not None:
            cap = self.peak.get('caption_render')
            if cap and store:
                now_img = cap if store.usable(cap, CAPTION_SLOT_W) else None
                if now_img != self._peak_cap_img:
                    self._peak_cap_img = _paint_img(
                        display, store, cap, CAPTION_SLOT_W, 8, SPARSE_Y0)

            cd = self._peak_cd_str(now_epoch)
            rgb = self._peak_cd_rgb()
            if cd != self._peak_cd_drawn or rgb != self._peak_cd_color:
                self._peak_cd_drawn = _swap_text(
                    display, font, self._peak_cd_drawn or '', cd,
                    232 - len(cd) * _CHAR_W, SPARSE_Y0, rgb)
                self._peak_cd_color = rgb

            badge_y = self._peak_badge_y()
            badge = self.peak.get('badge_render')
            if badge and store:
                now_img = badge if store.usable(badge, BADGE_SLOT_W) else None
                if now_img != self._badge_img:
                    self._badge_img = _paint_img(
                        display, store, badge, BADGE_SLOT_W, 8, badge_y,
                        clear_h=self._peak_badge_h(store))

        # 余额：caption 补贴 + 大数字增量重画（含 ¥/$ 符号，同字体）
        if self.balance is not None:
            by = self._balance_block_y(store)
            cap = self.balance.get('caption_render')
            if cap and store:
                now_img = cap if store.usable(cap, CAPTION_SLOT_W) else None
                if now_img != self._bal_cap_img:
                    self._bal_cap_img = _paint_img(
                        display, store, cap, CAPTION_SLOT_W, 8, by)
            big_y = by + SPARSE_CAP_H
            amt = _digits_only(self.balance.get('amount_text') or '')
            cur = self.balance.get('currency')
            if (amt, cur) != self._bal_txt_drawn:
                # 金额/币种变化才擦写（键=渲染输入意图；_bal_txt_drawn
                # 存 (amt, cur)，与上轮 diff 比较键一致——此前存输出文本
                # （含符号）导致恒不等每帧重画闪动）
                old = self._bal_txt_drawn
                old_amt = old[0] if isinstance(old, tuple) else ''
                old_cur = old[1] if isinstance(old, tuple) else ''
                _erase_big(display, font, old_amt,
                           self._bal_big_right, big_y, currency=old_cur)
                self._bal_txt_drawn = (amt, cur)
                _draw_big(display, font, amt, 232, big_y, self._bal_rgb(),
                          fallback_color=_TEXT_DIM_RGB, currency=cur)


class OverviewPage:
    """总览页：overview 字段由服务器聚合（取 max + 状态合并），端侧零计算。

    每 provider 一行，行内自左向右：行名贴图 → 进度条 → 百分比
    （紧贴条尾）→ 距重置倒计时（行末，极短格式 Xh/Xm/Xd）。
    - plan/bundle ：迷你条 + 百分比 + 倒计时
    - balance     ：金额大数字（lib/digits 位图字体，含 ¥/$ 符号，
                    峰谷语义色：state peak 红 / offpeak 绿 / 无峰谷灰）；
                    无条无百分比
    贴图缺失一律留空不占位（不画 '--'/id）；数字/百分比/倒计时端侧现场。
    最多 6 行。
    """

    Y0, STEP = 56, 24
    BAR_X, BAR_W, BAR_H = 96, 64, 10
    MAX_ROWS = 6

    def __init__(self):
        self.pid = None             # overview 页无稳定 id（用 None 标识）
        self.name = 'Overview'
        self.status = 'ok'          # 服务端聚合状态（徽标两态 ok/stale）
        self.error_text = ''        # degrade_stale 写入的 ASCII 单行错误
        self.updated_at = None      # 服务端暂不下发；防御预留
        self.title_hash = None
        self.rows = []              # [(id, pct|None, kind, amount,
        #                           currency, rhash, reset_at, status,
        #                           state)]
        self._sig = None
        self._title_img = None
        self._dot_rgb = None
        self._status_text = None
        self._err_drawn = None      # 已画出的错误行文本（None=没画）
        # 每行 [fill, color, pct, img, cd, big_drawn]（big_drawn=已画
        # 的大字文本，避免 diff 每帧擦写同内容闪烁）
        self._last = []
        # 双套贴图 hash 记忆（同 ProviderPage._alt；昼夜切换换套用）
        self._alt = {}

    def update(self, ov, night=False):
        # 契约定稿：响应无 name（中文不上端侧），标题全靠 title_render；
        # self.name 仅作 PIL 降级时的 ASCII 回退（overview 标题无回退则留空）
        # night：本次响应的渲染套——顺路登记进 _alt（昼夜切换回填用）
        self.title_hash = ov.get('title_render')
        self.updated_at = ov.get('updated_at')   # 服务端暂不下发；防御预留
        # 徽标两态：只收 ok/stale（status=error 由 app 拦截转 FetchError，
        # 这里归一 stale 纯防御）
        st = str(ov.get('status') or 'ok')
        self.status = st if st in ('ok', 'stale') else 'stale'
        rows = []
        items_alt = []
        for item in (ov.get('items') or [])[:self.MAX_ROWS]:
            try:
                percent = float(item['percent'])
            except (KeyError, TypeError, ValueError):
                percent = None      # 余额行：无百分比语义
            amt = item.get('amount')
            amt = '' if amt is None else str(amt)[:_AMT_CHAR_N]
            stat = str(item.get('status') or '')
            if stat == 'error':
                # 行级上游失败且无历史（服务端对无数据 plan/bundle 下发
                # percent=0.0 兜底靠 status 显异常）：端侧屏蔽为无数据，
                # 该行画红 'err'，绝不渲染虚假 0%（2026-09-05）
                percent = None
                amt = ''
            rst = item.get('reset_at')
            state = str(item.get('state') or '')
            rows.append((str(item.get('id') or '?'),
                         percent,
                         str(item.get('kind') or ('plan' if percent is not None
                                                  else 'balance')),
                         amt,
                         str(item.get('currency') or ''),
                         item.get('render'),
                         rst if isinstance(rst, (int, float)) else None,
                         stat or self.status,
                         state))
            items_alt.append(item.get('render_alt'))
        self.rows = rows
        # 双套记忆登记（同 ProviderPage：主 hash 归本套、alt 归对偶套）
        tag = 'N' if night else 'D'
        atag = 'D' if night else 'N'
        if self.title_hash:
            self._alt['t' + tag] = self.title_hash
        if ov.get('title_render_alt'):
            self._alt['t' + atag] = ov['title_render_alt']
        for i, r in enumerate(rows):
            if r[5]:
                self._alt['r{}{}'.format(i, tag)] = r[5]
                if i < len(items_alt) and items_alt[i]:
                    self._alt['r{}{}'.format(i, atag)] = items_alt[i]

    def invalidate(self):
        self._sig = None

    def swap_renders(self, night):
        """昼夜切换：贴图槽换另一套 hash（_alt 回填，rcache 直读；
        语义同 ProviderPage.swap_renders；缺失槽由 app 经
        render_slots 判断后立即 fetch 补齐）。"""
        tag = 'N' if night else 'D'
        alt = self._alt
        self.title_hash = alt.get('t' + tag)
        self.rows = [r[:5] + (alt.get('r{}{}'.format(i, tag)),) + r[6:]
                     for i, r in enumerate(self.rows)]

    def render_slots(self):
        """当前页应占的贴图槽位 [(hash|None, slot_w), ...]（语义同
        ProviderPage.render_slots）。"""
        return ([(self.title_hash, TITLE_SLOT_W)]
                + [(r[5], OV_ITEM_SLOT_W) for r in self.rows])

    def set_error_keep(self, message=''):
        """网络失败但保留旧数据：标 stale + 页头下 ASCII 单行错误
        （app degrade_stale 统一调用；服务端恢复后 update 换回 ok）。
        限宽 ERR_MAX_CHARS（止步 RSSI 角标左缘）。"""
        self.status = 'stale'
        self.error_text = _ascii((message or '')[:ERR_MAX_CHARS]) or ''

    def _status_str(self, now_epoch):
        """徽标文本：stale 带年龄（有 updated_at 时；overview 服务端
        暂不下发 updated_at → 纯 'stale'）。"""
        if (self.status == 'stale'
                and isinstance(self.updated_at, (int, float))):
            age = int(now_epoch - self.updated_at)
            return 'stale {}m'.format(min(max(age // 60, 0), 99))
        return STATUS_TEXT.get(self.status, 'stale')

    def render(self, display, font, now_epoch, store=None):
        """返回 True 表示全量重画（app 据此重画 RSSI 角标）。"""
        sig = (self.name, self.title_hash, self.status, self.error_text,
               NIGHT,
               tuple((r[0], r[1], r[2], r[3], r[5], r[7], r[8])
                     for r in self.rows))
        if sig != self._sig:
            self._render_full(display, font, sig, now_epoch, store)
            return True
        self._render_diff(display, font, now_epoch, store)
        return False

    def _pct_str(self, percent, now_epoch):
        if percent is None:
            return None
        if percent >= PCT_CRIT and not _blink_on(now_epoch):
            return ''
        return '{:.0f}%'.format(percent)

    @staticmethod
    def _row_rgb(percent, state):
        """行色：plan/bundle 走阈值色；余额行按峰谷 state 上语义色
        （peak 红 / offpeak 绿 / 无峰谷灰——服务端 overview.state 同源）。"""
        if percent is not None:
            return _col(_bar_rgb(percent) or (120, 140, 170))
        if state == 'peak':
            return _col(_PEAK_BIG_RGB)
        if state == 'offpeak':
            return _col(_OFFPEAK_BIG_RGB)
        return _col(_TEXT_DIM_RGB)

    def _cd_str(self, reset_at, now_epoch):
        """距重置剩余时间的极短格式（<=3 字符）：行内空间只有
        百分比之后的一段，fmt_countdown 的 HH:MM 放不下。"""
        if not reset_at:
            return None
        left = reset_at - now_epoch
        if left <= 0:
            return None
        if left < 60:
            return '<1m'
        if left < 3600:
            return '{}m'.format(left // 60)
        if left < 48 * 3600:
            return '{}h'.format((left + 1800) // 3600)
        return '{}d'.format(left // 86400)

    def _draw_status(self, display, font, text, rgb):
        """页头状态：方形状态点（10x10，与 ProviderPage 同规格同 y）
        + 文字基线对齐紧贴其左。"""
        display.fill_rect(STATUS_SLOT_X, 6, STATUS_SLOT_W, font.HEIGHT,
                          BG())
        display.text(font, text,
                     DOT_X - 3 - len(text) * _CHAR_W, 6, _col(rgb), BG())
        display.fill_rect(DOT_X, 8, 10, 10, _col(rgb))

    def _render_full(self, display, font, sig, now_epoch, store=None):
        self._sig = sig
        display.fill(BG())
        self._title_img = _paint_img(display, store, self.title_hash,
                                     TITLE_SLOT_W, NAME_X, 6)
        rgb = _status_rgb(self.status)
        stext = self._status_str(now_epoch)
        self._draw_status(display, font, stext, rgb)
        self._dot_rgb = rgb
        self._status_text = stext
        # 错误行（status≠ok）：ASCII 单行（degrade_stale 写入）
        self._err_drawn = None
        if self.status != 'ok' and self.error_text:
            display.text(font, self.error_text, 4, ERR_Y, FG(), BG())
            self._err_drawn = self.error_text
        self._last = []
        for i, row in enumerate(self.rows):
            (ident, percent, kind, amount, currency, rhash, reset_at,
             stat, state) = row
            y = self.Y0 + i * self.STEP
            # 贴图缺失留空（fallback=None 不画任何占位）；契约无 name，
            # 行名全靠 render 贴图，ASCII id 不再上屏
            img = _paint_img(display, store, rhash, OV_ITEM_SLOT_W, 8, y)
            color = self._row_rgb(percent, state)
            fill = 0
            big = None
            if stat == 'error':
                # 行级失败（update 已屏蔽 percent/amount）：不画条/
                # 倒计时，percent 槽画红 'err'——绝不渲染虚假 0%
                pct = 'err'
                display.text(font, pct, OV_PCT_X, y, _col(_ERR_RGB), BG())
            else:
                pct = self._pct_str(percent, now_epoch)
                if percent is not None:
                    display.fill_rect(self.BAR_X, y + 3, self.BAR_W,
                                      self.BAR_H, _col(TRACK_RGB))
                    fill = _fill_w(percent, self.BAR_W)
                    if fill:
                        display.fill_rect(self.BAR_X, y + 3, fill,
                                          self.BAR_H, color)
                    if pct:
                        display.text(font, pct, OV_PCT_X, y, color, BG())
                elif amount:
                    # 余额行：**ASCII 直画**（用户定稿，不用大字也不用
                    # 贴图）：币种码 + 空格 + 金额，vga 右对齐（CNY 110.5）
                    txt = (_currency_code(currency) + ' ' + amount) \
                        if _currency_code(currency) else amount
                    self._erase_row_right(display, font, y)
                    display.text(font, txt,
                                 AMT_RIGHT - len(txt) * _CHAR_W, y,
                                 color, BG())
                    big = txt
            cd = None if stat == 'error' else self._cd_str(reset_at,
                                                           now_epoch)
            if cd:
                display.text(font, cd, AMT_RIGHT - len(cd) * _CHAR_W, y,
                             _col(_CAPTION_RGB), BG())
            self._last.append([fill, color, pct, img, cd, big])

    def _erase_row_right(self, display, font, y):
        """擦总览行右端数值区（币种码+空格+金额最多 ~17 字符）。"""
        display.fill_rect(AMT_RIGHT - 17 * _CHAR_W, y, 17 * _CHAR_W,
                          font.HEIGHT, BG())

    def _render_diff(self, display, font, now_epoch, store=None):
        # 标题贴图补贴
        if self.title_hash and store:
            now_img = (self.title_hash
                       if store.usable(self.title_hash, TITLE_SLOT_W)
                       else None)
            if now_img != self._title_img:
                self._title_img = _paint_img(display, store,
                                             self.title_hash, TITLE_SLOT_W,
                                             NAME_X, 6)
        # 页头状态（聚合态变化时更新框+字；徽标两态 ok/stale）
        rgb = _status_rgb(self.status)
        stext = self._status_str(now_epoch)
        if stext != self._status_text or rgb != self._dot_rgb:
            self._draw_status(display, font, stext, rgb)
            self._status_text = stext
            self._dot_rgb = rgb

        # 错误行（status≠ok）：内容变化才重画（恢复 ok 由 sig 变化触发
        # 全量重画擦除）
        if self.status != 'ok':
            want_err = self.error_text
            if want_err != self._err_drawn:
                if self._err_drawn:
                    _erase_text(display, 4, ERR_Y, len(self._err_drawn),
                                font.HEIGHT)
                if want_err:
                    display.text(font, want_err, 4, ERR_Y, FG(), BG())
                self._err_drawn = want_err

        for i, row in enumerate(self.rows):
            (ident, percent, kind, amount, currency, rhash, reset_at,
             stat, state) = row
            y = self.Y0 + i * self.STEP
            (fill_old, color_old, pct_old_val, img_old, cd_old,
             big_old) = self._last[i]
            img = img_old
            if rhash and store:      # 行名贴图补贴（缺失时留空）
                now_img = (rhash if store.usable(rhash, OV_ITEM_SLOT_W)
                           else None)
                if now_img != img_old:
                    img = _paint_img(display, store, rhash, OV_ITEM_SLOT_W,
                                     8, y)
            color = self._row_rgb(percent, state)
            fill = _fill_w(percent, self.BAR_W) if percent is not None else 0
            if fill > fill_old:
                display.fill_rect(self.BAR_X + fill_old, y + 3,
                                  fill - fill_old, self.BAR_H, color)
            elif fill < fill_old:
                display.fill_rect(self.BAR_X + fill, y + 3,
                                  fill_old - fill, self.BAR_H,
                                  _col(TRACK_RGB))
            elif percent is not None and color != color_old and fill > 0:
                display.fill_rect(self.BAR_X, y + 3, fill, self.BAR_H, color)
            # 余额行 ASCII：仅内容变化才重画（amount/state 均在 sig
            # 内，正常不会走到；防御后补画）。不变则零 SPI 写。
            if percent is None and amount:
                txt = (_currency_code(currency) + ' ' + amount) \
                    if _currency_code(currency) else amount
                if txt != big_old:
                    self._erase_row_right(display, font, y)
                    display.text(font, txt,
                                 AMT_RIGHT - len(txt) * _CHAR_W, y,
                                 color, BG())
                    big_old = txt
            else:
                big_old = None
            # 百分比：紧贴条尾固定槽；行级 error 恒红 'err'（不闪不重算）
            pct = 'err' if stat == 'error' \
                else self._pct_str(percent, now_epoch)
            if pct != pct_old_val:
                n = max(len(pct_old_val or ''), len(pct or ''))
                _erase_text(display, OV_PCT_X, y, n, font.HEIGHT)
                if pct:
                    display.text(font, pct, OV_PCT_X, y,
                                 _col(_ERR_RGB) if stat == 'error'
                                 else color, BG())
            # 距重置倒计时：行末右对齐（走字；error 行不画）
            cd = None if stat == 'error' else self._cd_str(reset_at,
                                                           now_epoch)
            if cd != cd_old:
                n = max(len(cd_old), len(cd or ''))
                if cd_old:
                    _erase_text(display, AMT_RIGHT - n * _CHAR_W, y, n,
                                font.HEIGHT)
                if cd:
                    display.text(font, cd, AMT_RIGHT - len(cd) * _CHAR_W, y,
                                 _col(_CAPTION_RGB), BG())
            self._last[i] = [fill, color, pct, img, cd, big_old]


class ErrorPage:
    """服务器不可达/响应无效的独立页面态（init 失败兑底，loaded=False
    期间）：错误消息（单行可截断）+ "重试 Ns" 倒计时间歇显示
    "retrying..." 点闪动（350ms 相位）+ 连续失败计数 "fail #N"
    （2026-09-05：单调递增证明主循环活着在重试——同内容重试屏显无
    变化看不出；成功归零后行消失）。"""

    _DOT_MS = 350            # retrying 点动画步进周期
    _NAME = 'server'

    def __init__(self):
        self.name = self._NAME
        self.message = ''
        self._drawn = False
        self._msg_drawn = None
        self._retry_drawn = None
        self._fails = 0          # 连续失败计数（app 每轮失败 +1）
        self._fails_drawn = None
        self._dots = 0       # 当前点相位 0..3（0=不显点）
        self._last_dot_t = 0

    def set(self, message):
        msg = (message or 'error')[:28]
        # 错误页直接 display.text：只画 ASCII（乱码防护）；可截断
        self.message = msg if _ascii(msg) else 'error'
        self._drawn = False

    def set_fails(self, n):
        """连续失败计数（app 统一失败入口每轮 +1；成功归零）。"""
        try:
            self._fails = max(int(n), 0)
        except (TypeError, ValueError):
            self._fails = 0

    def invalidate(self):
        self._drawn = False
        self._retry_drawn = None

    def _retry_line(self, retry_left_s, anim):
        """重试行文本：retrying...（点滴相位 anim=1..3）或 retry Ns。"""
        if retry_left_s is None:
            return 'retry --'
        if retry_left_s <= 0:
            # 到点（重试发起的瞬间）：显示 retrying...
            return 'retrying ' + '.' * max(anim, 1)
        return 'retry {}s'.format(retry_left_s)

    def render(self, display, font, now_epoch, retry_left_s=None):
        # 相位推进：140ms 一发；本页活跃时（retry_left_s<=0 即等待发
        # 起重试窗口）才动画
        if time.ticks_diff(time.ticks_ms(), self._last_dot_t) >= self._DOT_MS:
            self._last_dot_t = time.ticks_ms()
            self._dots = self._dots % 3 + 1
        full = False
        if not self._drawn or self.message != self._msg_drawn:
            display.fill(BG())
            display.text(font, self.name, NAME_X, 6, _col(_ERR_RGB), BG())
            # 单行错误消息（允许截断；非 ASCII 任何字 → 空）
            display.text(font, self.message or 'error', 8, 104,
                         FG(), BG())
            self._msg_drawn = self.message
            self._retry_drawn = None
            self._fails_drawn = None
            self._drawn = True
            full = True
        anim = 0
        if retry_left_s is not None and retry_left_s <= 0:
            anim = self._dots
        retry = self._retry_line(retry_left_s, anim)
        if retry != self._retry_drawn:
            _swap_text(display, font, self._retry_drawn or '', retry,
                       8, 130, _WARN_RGB)
            self._retry_drawn = retry
        # 连续失败计数行（fail #N）：变化才重画（计数不动零 SPI 写）
        fails = 'fail #{}'.format(self._fails) if self._fails else ''
        if fails != self._fails_drawn:
            _swap_text(display, font, self._fails_drawn or '', fails,
                       8, 152, _ERR_RGB)
            self._fails_drawn = fails
        return full
