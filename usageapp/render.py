"""SSR 贴图：hash 引用 + 按需下载 + 文件缓存。

协议（../llm-usage-server/SCHEMA.md「贴图」节）：主响应只带 8hex
hash；贴图本体走 GET {base}/api/render/<hash>，认证同主 API。
命中本地缓存的 hash 零请求；缺失的由 app 在渲染完 ASCII UI 后
排队下载（on_wait 切片回调照常驱动时间条灰段）。

缓存即渲染源：display.png(path, x, y)（固件方法）直接从文件流式
解码——下载落盘的文件就是渲染源，零内存整图解码。hash 即文件名，
上限 max_files（默认 50，可用 server['rcache_max_files'] 配置）、
**LRU 淘汰**（内存记"最近使用"，常看的图不被挤掉）；下载先写
.tmp，校验 PNG 魔数与 IHDR 宽度后原子改名。hash 严格 8 位小写
hex（防路径穿越）。

下载失败按 server['ssr_retry'] 次（默认 3）重试、间隔
server['ssr_retry_delay_ms']（默认 2000ms）——瞬时抖动自愈，
次数用尽才抛 FetchError → 槽空置下轮平补刷新（原退避语义保留）。
"""
import os
import time

from . import client
from .page import (BADGE_SLOT_W, CAPTION_SLOT_W, LABEL_SLOT_W,
                   OV_ITEM_SLOT_W, TITLE_SLOT_W)

FetchError = client.FetchError
_HEX = set('0123456789abcdef')
_PNG_SIG = b'\x89PNG\r\n\x1a\n'
_MAX_W = 240                     # 屏宽即上限，超了判坏图
_RETRY_ATTEMPTS = 3              # SSR 单张贴图下载尝试次数（含首次）
_RETRY_DELAY_MS = 2000           # 尝试间隔（固定延迟，ms）
_MAX_FILES = 50                  # rcache 默认上限：2x 昼夜双套 hash 共存


def _cfg_int(cfg, key, dflt, lo, hi):
    """cfg[key] 取 int，越界/非法回默认（端侧不信任配置形态）。"""
    try:
        v = int(cfg.get(key, dflt))
    except (TypeError, ValueError):
        return dflt
    return v if lo <= v <= hi else dflt


def _backoff(ms, on_wait):
    """重试间隔：切片睡 + 每片回调 on_wait，时间条照常走字。"""
    step = 100
    while ms > 0:
        t = step if ms > step else ms
        time.sleep_ms(t)
        ms -= t
        if on_wait:
            on_wait()


def valid_hash(h):
    return (isinstance(h, str) and len(h) == 8
            and all(c in _HEX for c in h))


def png_dims(path):
    """读 IHDR 宽度与高度（文件头 24 字节，零解码）。坏文件返回 None。"""
    try:
        with open(path, 'rb') as f:
            head = f.read(24)
    except OSError:
        return None
    if len(head) < 24 or head[:8] != _PNG_SIG or head[12:16] != b'IHDR':
        return None
    return (int.from_bytes(head[16:20], 'big'),
            int.from_bytes(head[20:24], 'big'))


def png_width(path):
    d = png_dims(path)
    return d[0] if d else None


def collect(data):
    """收集响应里全部贴图 hash 及其所在槽位的实际宽度。

    返回 [(hash, slot_w), ...]（去重、只留合法形态）；同一 hash 出现
    在多个槽位时取最大宽度（宽槽能过窄槽必然能过）。slot_w 供下载时
    拼 ?w= 上报（client 能力声明，服务端只读校验超宽警告）。

    单端点各只回一页：overview（type=overview）有 title_render +
    items[].render；provider 页（type=provider）有 title_render /
    quotas[].label_render / balance.{render,caption_render} /
    peak.{badge_render,caption_render}（badge_render 大字仅余额类稀疏
    型带；caption_render 两种形态都带——余额类“现在是：”/非余额类
    "距切换计价还剩："。注意：badge_small_render 已停发）。
    金额大字（含 ¥/$ 符号）由端侧位图字体现场画，不走贴图。
    error 不走贴图（2026-09-02 定稿——详情见 SCHEMA.md provider
    详情节：错误统一端侧 ASCII 单行）。
    """
    out = []

    def add(h, w):
        if not valid_hash(h):
            return
        for i, oh in enumerate(out):
            if oh[0] == h:
                if w > oh[1]:
                    out[i] = (h, w)
                return
        out.append((h, w))

    if not isinstance(data, dict):
        return out
    add(data.get('title_render'), TITLE_SLOT_W)
    for quota in data.get('quotas') or []:
        if isinstance(quota, dict):
            add(quota.get('label_render'), LABEL_SLOT_W)
    for item in data.get('items') or []:
        if isinstance(item, dict):
            add(item.get('render'), OV_ITEM_SLOT_W)
    bal = data.get('balance')
    if isinstance(bal, dict):
        add(bal.get('render'), BADGE_SLOT_W)
        add(bal.get('caption_render'), CAPTION_SLOT_W)
    pk = data.get('peak')
    if isinstance(pk, dict):
        add(pk.get('badge_render'), BADGE_SLOT_W)
        add(pk.get('caption_render'), CAPTION_SLOT_W)
    return out


def _remove(path):
    try:
        os.remove(path)
    except OSError:
        pass


class RenderStore:
    # 容量按最坏页集算：N provider × 昼夜双套 hash 共存（同文本不同
    # 底色/主题 = 两个 hash 各占一份）——默认 50 覆盖 7 provider；
    # 可由 server['rcache_max_files'] 配置（越界/非法回默认）。
    def __init__(self, base_url, key, cache_dir='rcache', max_files=None,
                 server_cfg=None):
        self.base = str(base_url or '').rstrip('/')
        self.key = str(key or '')
        self.srv = server_cfg          # usage_cfg.server：重试/容量配置源
        self.dir = cache_dir
        if max_files is None:
            max_files = _cfg_int(server_cfg or {}, 'rcache_max_files',
                                 _MAX_FILES, 8, 256)
        self.max_files = max_files
        self._dims = {}            # hash -> (宽, 高)（首次读盘后缓存）
        self._used = {}            # hash -> 最近使用序号（LRU 依据，内存态）
        self._clock = 0            # 单调递增使用序号（避开 ticks 回绕）
        self._oom = set()          # 本周期解码 OOM 的 hash（退避，新周期重试）
        self._oom_logged = set()   # 已打过串口诊断的 hash（去重防周期刷屏）
        self._miss_logged = set()  # 同上（槽空置诊断）
        try:
            os.mkdir(cache_dir)
        except OSError:
            pass

    def path(self, h):
        return self.dir + '/' + h if valid_hash(h) else None

    def _touch(self, h):
        """记录一次使用（LRU 依据）。序号纯内存态；每帧命中只是
        dict 值更新，无堆增长、无 flash 写。"""
        self._used[h] = self._clock
        self._clock += 1

    def has(self, h):
        p = self.path(h)
        if p is None:
            return False
        try:
            os.stat(p)
            return True
        except OSError:
            return False

    def usable(self, h, max_w):
        """命中缓存且宽度不超槽 → 文件路径；否则 None（走 ASCII fallback）。
        OOM 退避过的 hash 本周期直接 None，避免每帧重试 png 闪烁。

        缓存命中后必须 stat 确认文件还在盘上：_evict() 淘汰只删文件，
        若内存 (宽,高) 缓存不随之失效，这里会返回已删除文件的路径，
        display.png() 抛 OSError ENOENT——fetch 后偶发硬崩（真机已复现）
        就出自这条链。stat 成本一条 VFS 调用，换稳定性。"""
        p = self.path(h)
        if p is None or h in self._oom:
            return None
        d = self._dims.get(h)
        if d is None:
            d = png_dims(p)
            if d is None:
                return None        # 坏文件：不缓存结果，下轮重读
            self._dims[h] = d
        if d[0] <= 0 or d[0] > max_w:
            return None
        try:
            os.stat(p)
        except OSError:
            self._forget(h)        # 文件已被淘汰（见 _forget 注释）：降级未缓存
            return None
        self._touch(h)
        return p

    def _dims_checked(self, h):
        """命中缓存的 (宽, 高)，文件在盘上已 stat 校验；坏/未下载返回 None。"""
        p = self.path(h)
        if p is None:
            return None
        d = self._dims.get(h)
        if d is None:
            d = png_dims(p)
            if d is not None:
                self._dims[h] = d
        if d is None:
            return None
        try:
            os.stat(p)
        except OSError:
            self._forget(h)
            return None
        self._touch(h)
        return d

    def height(self, h):
        """命中缓存的贴图高度（首次读盘读取 IHDR 高）；坏/未下载返回 None。
        大字插图（峰谷/余额）高度固定——排版按 PNG 自带高度而非 ?h=。"""
        d = self._dims_checked(h)
        return d[1] if d else None

    def width(self, h):
        """命中缓存的贴图宽度（同 height()，读 IHDR）。坏/未下载返回 None。
        总览余额行用位图实宽把货币符号右对齐到金额左侧。"""
        d = self._dims_checked(h)
        return d[0] if d else None

    def _forget(self, h):
        """贴图文件已不在盘上（被淘汰/被删）：清掉内存里的尺寸缓存。
        不清的话 usable() 会命中 (宽,高) 缓存直接返回已删除文件的路径，
        display.png() 打不开文件 → OSError ENOENT（真机硬崩源）。"""
        try:
            del self._dims[h]
        except KeyError:
            pass
        self._used.pop(h, None)    # LRU 记录同步失效

    def missing(self, hashes):
        self._oom = set()          # 新周期：给上轮 OOM 的贴图重试机会
        # 淘汰可能发生在别处（本进程之外），对内存认为"有"的 hash 逐一
        # stat 一次太贵；文件系统不保证时用此兜底（开机首个周期必经）。
        return [h for h in hashes if not self.has(h)]

    def note_oom(self, h):
        """display.png 解码 MemoryError 后登记：本周期不再尝试该 hash。
        失败会每周期重试（成本低、堆况会变），串口诊断只打首次。"""
        if valid_hash(h):
            self._oom.add(h)
            if h not in self._oom_logged:
                self._oom_logged.add(h)
                self._log_oom(h)

    def note_miss(self, h):
        """贴图槽空置诊断：hash 合法但当前不可贴（未下载/下载失败/
        超槽宽）。槽位留空等下轮平补刷新，串口报错只打首次（新
        hash 自然会再打；OOM 登记过的由 note_oom 负责去重，不双报）。"""
        if not valid_hash(h) or h in self._oom:
            return
        if h not in self._miss_logged:
            self._miss_logged.add(h)
            print('[usage] ssr miss {} (slot blank, retry next cycle)'
                  .format(h))

    def _log_oom(self, h):
        import gc
        print('[usage] png oom free={} {} (后续静默重试)'
              .format(gc.mem_free(), h))

    def download(self, h, on_wait=None, slot_w=None, server_cfg=None,
                 on_retry=None):
        """下载一张贴图入缓存，返回宽度。失败抛 FetchError（本周期放弃）。

        重试：server['ssr_retry']（次数，默认 3，含首次）次内对
        DNS/连接/HTTP 状态/坏 PNG 统一重试，间隔 server
        ['ssr_retry_delay_ms']（默认 2000ms，切片睡保时间条动画）；
        次数用尽才抛错 → 槽空置，下轮平补刷新。server_cfg 缺省用
        构造时传入的（app 处 usage_cfg.server 同源）。

        slot_w：槽位预期最大宽度，拼 ?w= 让服务端做只读校验（超宽照发，
        服务端控制台警告配置不合理）。None 不带该参数。

        on_retry(bool)：SSR 贴图抓取失败过的 sticky 通知（退避/放弃均
        置位，本轮 fetch 内不回 False——端侧左标注保持 'retrying'，
        下轮 begin_cycle 才复位；'retrying' 仅次要 fetch 失败过用，
        主请求失败后的延时重发不算，2026-09-06 用户定稿）。
        None 不通知。"""
        if not valid_hash(h):
            raise FetchError('bad hash')
        srv = server_cfg or self.srv or {}
        host, port, base_path = client._parse_url(self.base)
        path = base_path.rstrip('/') + '/api/render/' + h
        if slot_w:
            path += '?w={}'.format(int(slot_w))
        tmp = self.dir + '/.tmp'
        attempts = _cfg_int(srv, 'ssr_retry', _RETRY_ATTEMPTS, 1, 10)
        delay = _cfg_int(srv, 'ssr_retry_delay_ms', _RETRY_DELAY_MS,
                         0, 60_000)

        f = None                 # 供 sink/attempt 共享当前打开的 .tmp 句柄
        def sink(chunk):
            f.write(chunk)

        def attempt():
            """单次尝试：开 .tmp → http_get → 校验 → 返回尺寸。
            失败抛 FetchError，重试层负责删半截文件与间隔。"""
            nonlocal f
            try:
                f = open(tmp, 'wb')
            except OSError as exc:
                raise FetchError('cache: {}'.format(exc))
            try:
                code, _body = client.http_get(host, port, path, on_wait,
                                              sink=sink, auth_key=self.key)
            finally:
                f.close()
            if code != 200:
                raise FetchError('render HTTP {}'.format(code))
            d = png_dims(tmp)
            if d is None or d[0] <= 0 or d[0] > _MAX_W:
                raise FetchError('bad png')
            return d

        last = None
        for i in range(attempts):
            try:
                d = attempt()
                break
            except FetchError as exc:
                last = exc
                _remove(tmp)            # 半截文件不留到下个尝试
                if on_retry:
                    on_retry(True)      # SSR 失败过：sticky（本轮内不翻回）
                if i + 1 < attempts:
                    _backoff(delay, on_wait)
        else:
            raise FetchError('render retry x{}: {}'.format(attempts, last))
        final = self.dir + '/' + h
        _remove(final)             # 部分 VFS 的 rename 不覆盖已有目标
        try:
            os.rename(tmp, final)
        except OSError as exc:
            _remove(tmp)
            raise FetchError('rename: {}'.format(exc))
        self._dims[h] = d
        self._touch(h)             # 新下载即"最近使用"（最不易被淘汰）
        self._evict()
        return d[0]

    def _evict(self):
        try:
            names = os.listdir(self.dir)
        except OSError:
            return
        entries = []
        for n in names:
            if not valid_hash(n):
                continue
            try:
                mt = os.stat(self.dir + '/' + n)[8]
            except OSError:
                continue
            # LRU：内存"最近使用序号"优先；开机后从未用过的（含上次
            # 开机遗留文件）记 0 最先淘汰。
            entries.append((self._used.get(n, 0), mt, n))
        if len(entries) <= self.max_files:
            return
        # 最久未用淘汰。文件删掉的同时必须失效内存 (宽,高) 缓存——否则
        # usable() 会把已删除的路径当作可贴图返回 → display.png ENOENT
        # （GLM 资源包页每次刷到必崩就是这条链，真机 traceback 已定位）。
        for *_k, n in sorted(entries)[:len(entries) - self.max_files]:
            _remove(self.dir + '/' + n)
            self._forget(n)
