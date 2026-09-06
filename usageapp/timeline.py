"""底部时间条：相位型，满条恒等于当前轮询周期。

语义（用户定稿 2026-08-31 四次修订）：
- 条底 = 页面 bg（未消费区域与屏底同色，不可见）
- 主请求进行中：整段 network 色增长（network 语义色独立键）
- 主响应到达：network/upstream 两段按服务端 timing 占比切分 [0,resp)
  冻结重涂，其后 SSR 段（端侧实测贴图下载，蓝色）实时增长
- SSR 冻结 → ssr 定格（蓝），其后 wait 灰随时间延伸到周期末
  （ssr 与 wait 异色）；满条 = 轮询周期（相位契约不变）
- 周期起点固定为"上一轮理想起点 + 周期"，相邻刷新间隔恒定
- 一 fetch 一重置；不保留历史；周期切换（调暗拉长轮询）相位按新周期
  重算压缩

时间标注（条上方一行）：左 = 距下次刷新倒计时（每秒走字），
fetch 进行中为 'fetching ...' 动画点（350ms 步进 1→2→3 循环）；
右 = 恒三段带 ' s' 尾与条上三色分段同源：主请求进行中后两段
置 0、全部时间实时算在第一项 network；响应到达展开
network/upstream——upstream 取服务端 timing.upstream，
network = 端侧实测主请求段总时 − upstream（钳 0；服务端 wait
只收包到鉴权、不含网络传输，直接用会出现"等 1s 显示 0.1/0.3"），
ssr 从 0 实时走字（贴图下载段端侧实测）；下载完成三段全冻结
（'/' 分隔灰）。文本可能超 10s：按左标注长度算宽度预算，
超了先降小数位再降整数秒防重叠。
**失败也保持三段**（用户定稿 2026-08-31）：响应没到就失败时，
主请求段实测时长冻结在 network 段、后两段 0.0——数字说真话
（这轮确实只消耗了主请求段），不回退单段灰；仅当总耗时为 0
（瞬时拒绝）才显示灰单段 0.0s。
"""
import time

import st7789

_CHAR_W = 8
_FETCH_ANIM_MS = 350          # fetching 动画点步进周期


def _label_colors():
    """标注文字色（随昼夜反相）：左倒计时用前景、分隔/失败灰。
    延迟 import 避免与 page 循环依赖。"""
    from . import page
    return page.FG(), page._col(page._CAPTION_RGB), page.BG()


class Timeline:
    def __init__(self, x, y, w, h, wait_rgb, fetch_rgb, up_rgb, sv_rgb,
                 bg, font=None, err_rgb=None):
        self.x, self.y, self.w, self.h = x, y, w, h
        # 四色 RGB：wait=等待下一轮的条底色 / network 段（fetch 进行中
        # 整段增长）/ upstream 段 / SSR 段（端侧实测贴图下载段）。
        # wait 段与 ssr 段异色：ssr 冻结后剩余段重涂 wait 色到周期末。
        # err_rgb：请求出错（主请求失败/SSR 下载失败）时 wait 段改涂
        # 错误红作 UI 提示（2026-09-02 用户定稿）。
        self._rgbs = (wait_rgb, fetch_rgb, up_rgb, sv_rgb)
        self._err_rgb = err_rgb
        self.night = False
        self._apply_colors()
        self.bg = bg
        self.font = font            # 传字体则在条上方画时间标注
        self.period_ms = 60_000
        self.anchor = time.ticks_ms()   # 本周期理想起点
        self._t0 = self.anchor          # 本轮 fetch 实际开始
        self.fetch_ms = None            # 整轮完成后冻结的总耗时
        self._timing = None             # (network,upstream)ms——upstream
        # 取服务端 timing，network=端侧实测响应段−upstream（见 mark_response）
        self._resp_ms = 0               # 主请求段毫秒（响应到达时刻冻结）
        self._t_ssr0 = 0                # SSR 计时起点（响应到达时刻）
        self._ssr_ms = 0                # SSR 段实测 ms（完成后冻结）
        self._seg_painted = True        # 三段布局是否已画出（响应到达/全清后重画一次）
        self._err = False               # 本轮请求出错（wait 段涂红）
        self._retrying = False          # 重试中（左标注 retrying...）
        # 增量渲染：只画差异段，记录上次已画到的像素边界
        self._last_d = 0    # 暗色段已画到（两段式）
        self._last_c = 0    # 彩色段右缘（两段式=暗段，三段式=ssr 段末）
        self._need_reset = True
        self._lab_left = ''      # 左标注已画文本（全串）
        self._lab_fixed = None   # 左标注已画的固定前缀（None=未知，强制全画）
        self._lab_right = ''     # 右标注已画文本
        self._lab_cols = None    # 右标注已画颜色组（同文本不同色也要重画）
        self._lab_parts = None   # 右标注已画分段（None=单段形态）

    def _apply_colors(self):
        """四色直通（昼夜两套色板的 RGB 已按各自底色调好对比度，
        app 换套时经 set_palette 传入）。"""
        self.c_wait, self.c_fetch, self.c_up, self.c_sv = \
            (st7789.color565(*rgb) for rgb in self._rgbs)
        # 错误红：未传时退 wait 色（无错误语义的构造方可省略参数）
        if self._err_rgb is not None:
            self.c_err = st7789.color565(*self._err_rgb)
        else:
            self.c_err = self.c_wait

    def set_palette(self, rgbs, bg, err_rgb=None):
        """换四色板与条/标注底色（昼夜两套配色整体切换时由 app 调）。
        err_rgb 随色板更新（theme.err 红），不传则维持现有值。"""
        self._rgbs = rgbs
        if err_rgb is not None:
            self._err_rgb = err_rgb
        self.bg = bg
        self._apply_colors()

    def set_period(self, period_ms):
        """切换周期：已过部分按新周期压缩（全量重画一次换算边界）。"""
        if period_ms != self.period_ms:
            self.period_ms = period_ms
            self._need_reset = True

    def begin_cycle(self, anchor_ms=None):
        """开始新一轮 fetch。anchor 传理想起点以消除调度抖动。一 fetch 一重置。
        左标注恒从 'fetching ...' 起步——上轮失败后的延时重发不算重试
        （2026-09-06 用户定稿：'retrying' 仅保留给 SSR 贴图重试退避，
        见 set_retrying）。"""
        self.anchor = anchor_ms if anchor_ms is not None else time.ticks_ms()
        self._t0 = time.ticks_ms()
        self.fetch_ms = None
        self._timing = None
        self._resp_ms = 0
        self._t_ssr0 = 0
        self._ssr_ms = 0
        self._seg_painted = True
        self._err = False
        self._retrying = False
        self._lab_left = ''     # 标注区可能被错误页/连接日志写过：强制重画
        self._lab_fixed = None
        self._lab_right = ''
        self._lab_cols = None
        self._lab_parts = None
        self._need_reset = True

    def set_retrying(self, on):
        """SSR 贴图抓取失败过 → 置位（左标注改 'retrying ...'，点闪动
        同款节奏）。**sticky**：置位后本轮 fetch 内不再翻回 fetching
        （含退避后的下个尝试），仅本轮完成（mark_fetch_done）或下一轮
        begin_cycle 复位。主请求失败后的延时重发不置位（仍 fetching，
        2026-09-06 用户定稿：retrying = 次要 fetch（SSR）失败过）。"""
        self._retrying = bool(on)

    def mark_response(self, timing=None):
        """主响应到达（文本 payload 已收到）：右标注 network/upstream
        两段展开——upstream 取服务端 timing.upstream，network = 端侧
        实测主请求段总时 − upstream（钳 0；服务端 wait 只计到鉴权
        完、不含网络传输与端侧收包，直接用会"等 1s 显示 0.1/0.3"），
        SSR 段从 0 现场计时（贴图下载进行中实时走字）；时间条上已
        增长的暗段下帧重排为 network/up 两色、其后继续长 ssr 亮蓝段。
        timing 缺失（旧版服务端）→ 不展开，整段暗色到底。"""
        self._resp_ms = max(time.ticks_diff(time.ticks_ms(), self._t0), 0)
        t = timing if isinstance(timing, dict) else None
        try:
            up = max(int((t or {}).get('upstream', 0)), 0)
        except (TypeError, ValueError):
            up = 0
        net = max(self._resp_ms - up, 0) if up > 0 else 0
        self._timing = (net, up) if (net + up) > 0 else None
        if self._timing is not None:
            self._t_ssr0 = time.ticks_ms()
            self._seg_painted = False    # 下帧 draw 全量重排三段

    def mark_fetch_done(self, timing=None, ssr_ms=0, error=False):
        """整轮完成：冻结 SSR 段与总耗时。mark_response 未走过的
        路径（直接失败/旧版服务端）在此兜底解析 timing——**失败也
        保持三段**（用户定稿）：主请求段实测时长进 network、后两段
        0.0，数字不说谎；仅总耗时 0（瞬时拒绝/无实测段）才回退单段。
        error：请求出错（主请求失败 / SSR 下载失败）→ wait 段涂错误红
        作 UI 提示（2026-09-02 用户定稿），到下一轮 begin_cycle 复位。"""
        self.fetch_ms = max(time.ticks_diff(time.ticks_ms(), self._t0), 0)
        if self._timing is None:
            # 失败兜底：无服务端 timing 可拆，主请求段整体记 network、
            # upstream 0——三段布局成立（0/0 后两段），条上不出单段灰。
            if self.fetch_ms > 0:
                self._timing = (self.fetch_ms, 0)
                self._resp_ms = self.fetch_ms
                self._seg_painted = False   # 下帧重排三段（ssr 段 0 宽）
        try:
            ssr = max(int(ssr_ms or 0), 0)
        except (TypeError, ValueError):
            ssr = 0
        self._ssr_ms = ssr if self._timing is not None else 0
        self._err = bool(error)
        self._retrying = False      # 完成（含失败）即脱离重试用例

    def elapsed_ms(self):
        return time.ticks_diff(time.ticks_ms(), self.anchor)

    def due(self):
        return self.elapsed_ms() >= self.period_ms

    def next_anchor(self):
        """下一轮理想起点：锚点 + 周期；落后太多则从现在重新起算。"""
        nxt = time.ticks_add(self.anchor, self.period_ms)
        if time.ticks_diff(nxt, time.ticks_ms()) < -1000:
            return time.ticks_ms()
        return nxt

    def left_s(self):
        """距下次刷新的秒数（错误页"重试 Ns"倒计时用）。"""
        return max(self.period_ms - self.elapsed_ms(), 0) // 1000

    def set_night(self, night):
        """夜间模式：色板由 app 经 set_palette 换套（day/night 两套
        全量配色整体切换），这里触发全量重画（标注色动态取）。"""
        if night != self.night:
            self.night = night
            self._apply_colors()
        self.invalidate()

    def invalidate(self):
        """通知时间条：屏幕可能被页面整屏 fill 抹掉，下次 draw 全量重画
        （wait 底铺满 + 实段重铺）。fetch 已完成且有 timing 时同时撤销
        三色重涂标记——全量重画必须再走一次分段重涂。"""
        self._need_reset = True
        self._lab_left = ''
        self._lab_fixed = None
        self._lab_right = ''
        self._lab_cols = None
        self._lab_parts = None
        self._last_d = 0
        self._last_c = 0
        if self._timing is not None:
            self._seg_painted = False   # 全清后三段布局需重画

    def _left_parts(self):
        """左标注拆 (固定前缀, 动态尾)：fetch 进行中 = ('fetching'|
        'retrying', 动画点 1..3)；空闲 = ('next Ns', '')。前缀不变时
        draw 只擦重画动态尾——整行擦写每次走字都闪一遍，违背"尽量
        减少重绘部分"（2026-09-06 用户定稿）。"""
        if self.fetch_ms is None:
            verb = 'retrying' if self._retrying else 'fetching'
            n = (time.ticks_diff(time.ticks_ms(), self._t0)
                 // _FETCH_ANIM_MS) % 3 + 1
            return (verb, '.' * n)
        return ('next {}s'.format(
            max(self.period_ms - self.elapsed_ms(), 0) // 1000), '')

    def _right_parts(self):
        """右标注内容：(parts|None, color, freeze)。parts=None 走
        单段文本；freeze=True 表示分段数值已冻结（不变则不重画）。
        恒三段与条上分段同源：主请求进行中 = network 实时 + 0/0；
        响应到达 = network/upstream 冻结 + ssr 实时；完成（含失败
        兜底）= 全冻结——失败时主请求段实测时长进 network、后两段
        0.0（用户定稿：数字说真话，不回退单段灰）。"""
        now = time.ticks_ms()
        if self.fetch_ms is not None:
            # 完成/失败：三段全冻结（带 ' s' 尾标单位）
            parts = [self._fmt_secs(v) for v in self._timing] \
                if self._timing else []
            if self._timing:
                parts.append(self._fmt_secs(self._ssr_ms))
                return (parts, None, True)
            # 总耗时 0（瞬时拒绝）：无可拆实测段，回退灰单段
            return (None, _label_colors()[1], True)
        if self._timing is not None:
            # 下载中：前两段冻结，ssr 现场走字（贴图下载段实测）
            parts = [self._fmt_secs(v) for v in self._timing]
            parts.append(self._fmt_secs(
                max(time.ticks_diff(now, self._t_ssr0), 0)))
            return (parts, None, False)
        # 主请求进行中：后两段置 0，全部时间实时算在 network
        live = self._fmt_secs(max(time.ticks_diff(now, self._t0), 0))
        return ([live, '0.0', '0.0'], None, False)

    def _fmt_secs(self, ms):
        """毫秒 → 文本：默认 0.1s 精度；≥10s 去 1 位小数防字符溢出。"""
        if ms >= 10_000:
            return '{}'.format(ms // 1000)
        return '{:.1f}'.format(ms / 1000)

    def _draw_right(self, display):
        """右标注（恒右对齐 self.x + self.w）。宽度预算：可宽
        240−右缘左的左标注（左标注右端+2 间距起算）——超了逐段降
        精度（0.1s→整秒）再截段数。

        增量策略（尽量减少重绘部分，2026-09-06 用户定稿）：布局不变
        （段数/各段宽/配色/总宽都同）时只重画文本变化的段——走字时
        通常仅 1 段，斜杠与 ' s' 尾不动；布局变（阶段切换：实时→响应
        到达→冻结，每轮仅两三次）才整块擦写一次。整块擦写每次走字
        都闪一遍，真机上肉眼可见。"""
        y = self.y - self.font.HEIGHT - 2
        fg, gray, bg = _label_colors()
        parts, color, freeze = self._right_parts()
        if parts is None:
            txt = '{:.1f}s'.format(self.fetch_ms / 1000)
            if txt == self._lab_right and color == self._lab_cols:
                return
            n = max(len(txt), len(self._lab_right))
            display.fill_rect(self.x + self.w - _CHAR_W * n, y,
                              _CHAR_W * n, self.font.HEIGHT, bg)
            display.text(self.font, txt,
                         self.x + self.w - _CHAR_W * len(txt), y, color,
                         bg)
            self._lab_right = txt
            self._lab_cols = color
            self._lab_parts = None
            return
        # 分段文本（' s' 尾标单位）：总宽超预算时先把每个 >10s 的段
        # 降整秒，再不够就截尾段（丢 ssr 段保主请求段）
        budget = (self.w - (len(self._lab_left) + 1) * _CHAR_W
                  if self.font else self.w)
        txt = '/'.join(parts) + ' s'
        while len(txt) * _CHAR_W > budget and len(parts) > 2:
            parts = parts[:2]
            txt = '/'.join(parts) + ' s'
        cols = (self.c_fetch, self.c_up, self.c_sv)
        cols = cols[:len(parts)]
        stable = (self._lab_parts is not None
                  and len(parts) == len(self._lab_parts)
                  and len(txt) == len(self._lab_right)
                  and cols == self._lab_cols
                  and all(len(p) == len(q)
                          for p, q in zip(parts, self._lab_parts)))
        if stable and txt == self._lab_right:
            return
        if not stable:
            # 布局变：整块擦写一次（阶段切换时才发生）
            n = max(len(txt), len(self._lab_right))
            display.fill_rect(self.x + self.w - _CHAR_W * n, y,
                              _CHAR_W * n, self.font.HEIGHT, bg)
            cx = self.x + self.w - _CHAR_W * len(txt)
            for i, p in enumerate(parts):
                display.text(self.font, p, cx, y, cols[i], bg)
                cx += _CHAR_W * len(p)
                if i < len(parts) - 1:
                    display.text(self.font, '/', cx, y, gray, bg)
                    cx += _CHAR_W
            display.text(self.font, ' s', cx, y, gray, bg)   # 尾部单位
        else:
            # 走字：只重画文本变化的段（右对齐 + 各段等宽 → 位置不变）
            cx = self.x + self.w - _CHAR_W * len(txt)
            for i, p in enumerate(parts):
                if p != self._lab_parts[i]:
                    display.fill_rect(cx, y, _CHAR_W * len(p),
                                      self.font.HEIGHT, bg)
                    display.text(self.font, p, cx, y, cols[i], bg)
                cx += _CHAR_W * len(p)
                if i < len(parts) - 1:
                    cx += _CHAR_W
        self._lab_right = txt
        self._lab_cols = cols
        self._lab_parts = list(parts)

    def _draw_left(self, display):
        """左标注增量绘制：固定前缀（'fetching'/'retrying'/'next Ns'）
        不变时只擦重画动态尾（动画点最多 3 字符），前缀不重画——
        整行擦写在真机上肉眼可见地闪（2026-09-06 用户定稿）。"""
        y = self.y - self.font.HEIGHT - 2
        fg, _g, bg = _label_colors()
        fixed, dots = self._left_parts()
        new = fixed + ' ' + dots if dots else fixed
        if new == self._lab_left:
            return
        if fixed == self._lab_fixed and self._lab_left:
            # 前缀未变：只动尾部小块（如 'fetching ...' 的 ' ...'）
            old_tail = self._lab_left[len(self._lab_fixed):]
            new_tail = new[len(fixed):]
            tx = self.x + len(self._lab_fixed) * _CHAR_W
            display.fill_rect(tx, y, _CHAR_W * max(len(old_tail),
                                                   len(new_tail)),
                              self.font.HEIGHT, bg)
            display.text(self.font, new_tail, tx, y, fg, bg)
        else:
            # 前缀变/首次/被整屏抹过：整块重画一次
            n = max(len(self._lab_left), len(new))
            display.fill_rect(self.x, y, _CHAR_W * n, self.font.HEIGHT, bg)
            display.text(self.font, new, self.x, y, fg, bg)
            self._lab_fixed = fixed
        self._lab_left = new

    def draw(self, display):
        """增量绘制。周期内单调不减只延伸新增像素；边界回退（异常
        时序/周期切换/新周期）或两段→三段布局切换时全量重画一次。
        分段（用户定稿 2026-08-31 四次修订）：**底 = 页面 bg**（未消费
        区域不可见）；主请求中整段 network 色增长；响应到达后
        network/upstream 冻结占 [0,resp)、ssr 蓝实时增长；ssr 冻结后
        ssr 定格（蓝）、其后 wait 灰随时间延伸到周期末——ssr 与 wait
        异色、底色归 bg。"""
        total = self.period_ms
        live = min(time.ticks_diff(time.ticks_ms(), self._t0), total)
        seg = self._timing is not None
        if seg:
            resp_px = min(self._resp_ms * self.w // total, self.w)
            live_px = live * self.w // total
        else:
            resp_px = live_px = live * self.w // total

        if self.font:
            self._draw_left(display)
            self._draw_right(display)

        if self._need_reset or live_px < self._last_c \
                or resp_px < self._last_d \
                or (seg and not self._seg_painted):
            # 全量重画：底 = 页面 bg（未消费区域与屏底同色不可见）
            # → 铺实段（net/up/ssr/wait 按当前布局）
            display.fill_rect(self.x, self.y, self.w, self.h, self.bg)
            self._last_d = self._last_c = 0
            self._need_reset = False
            if seg:
                w, u = self._timing
                b1 = resp_px * w // (w + u) if w + u > 0 else 0
                display.fill_rect(self.x, self.y, b1, self.h, self.c_fetch)
                if resp_px > b1:
                    display.fill_rect(self.x + b1, self.y,
                                      resp_px - b1, self.h, self.c_up)
                self._last_d = resp_px
                self._seg_painted = True
        if not seg:
            # 两段式（瞬时失败）：整段 network 延伸（底=bg）
            if live_px > self._last_c:
                display.fill_rect(self.x + self._last_c, self.y,
                                  live_px - self._last_c, self.h,
                                  self.c_fetch)
                self._last_c = live_px
            self._last_d = live_px
        elif self.fetch_ms is None:
            # 三段式·下载中：[0,resp) 冻结不动，ssr 实时增长到 live
            if live_px > self._last_c:
                start = max(self._last_c, resp_px)
                if live_px > start:
                    display.fill_rect(self.x + start, self.y,
                                      live_px - start, self.h, self.c_sv)
                self._last_c = live_px
        else:
            # 三段式·冻结后：ssr 定格到 ssr_px（蓝），其后 wait 色随
            # live 延伸到周期末——ssr 与 wait 异色（用户定稿：底=bg、
            # ssr=蓝、wait=灰）；**请求出错（主请求失败 / SSR 下载
            # 失败）时 wait 段改涂错误红**作 UI 提示（2026-09-02 用户
            # 定稿），下一轮 begin_cycle 复位。
            wait_c = self.c_err if self._err else self.c_wait
            seg_end_ms = self._resp_ms + self._ssr_ms
            ssr_px = min(seg_end_ms * self.w // total, self.w) \
                if seg_end_ms > 0 else resp_px
            if live_px > self._last_c:
                if self._last_c < ssr_px:
                    start = max(self._last_c, resp_px)
                    end = min(live_px, ssr_px)
                    if end > start:
                        display.fill_rect(self.x + start, self.y,
                                          end - start, self.h, self.c_sv)
                if live_px > ssr_px:
                    display.fill_rect(self.x + ssr_px, self.y,
                                      live_px - ssr_px, self.h,
                                      wait_c)
                self._last_c = live_px
