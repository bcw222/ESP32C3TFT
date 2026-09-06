"""统一 schema 客户端：GET usage-server（client=1 协议），阻塞但切片读。

为什么不用 urequests：fetch 期间要驱动时间条动画（暗色段实时增长），
所以手写 socket，切片读回调 on_wait()。切片用 uselect.poll 窗口驱动
而**不依赖 sock.settimeout 的超时语义**：部分固件上 stream read 不按
超时抛 OSError，会整段阻塞在 C read 里——主循环冻结、时间条动画
全停（2026-09-05 真机症状：fetch 时全屏凝固、fetching 不闪）。poll
无数据 150ms 即返回 → 回调动画 → 再 poll，与固件超时行为无关。
设备→服务器是明文 http
（key 会明文过网，部署公网时请自行加 TLS 反代或接受该风险）。

协议（../llm-usage-server/SCHEMA.md 为唯一事实源）：单端点按页码查表，
「看哪页拉哪页」——fetch_page(N) 拉当前页数据，响应带 type
（overview/provider）决定解析分发；服务器无抓取缓存，每个请求实时
打上游，新鲜度节奏全由端侧控制：
- GET {base}/api/page?page=N&client=1&h=16&bg=<页底>&theme=<昼夜套>
  认证 `Authorization: Bearer <key>`（key 不进 URL/访问日志）；
  h=16 行高、bg=页底色、theme=昼夜（day/night 两套全量配色整体切换，
  夜间深底亮字）均计入贴图 hash——bg/theme 由 theme.py 按当前夜态给
- 响应公共字段：type（'overview'/'provider'——端侧按此解析）、
  total（页数，端侧本地 (n+1)%total 回卷）、page（回显）；
  越界回卷由服务端兜底（page % total），翻页永不 404
- 贴图为服务端与 bg 合成后的不透明 PNG（无 alpha）：抗锯齿边缘
  服务端已合成完毕，端侧逐像素直写，不做也不需要透明处理
- 昼夜双套贴图各自缓存（服务端按 hash、端侧 rcache/），
  切换时零计算只换 hash 引用
- 响应无版本字段：声明 client=1 即收到 client=1 schema
- 服务端输入可信：结构校验/控制全部在服务端，端侧不做校验
- http_get 为通用底层（render.py 贴图下载复用，sink 流式写文件）
"""
import gc
import socket
import time
import ujson

try:
    import uselect as _select     # MicroPython
except ImportError:
    import select as _select      # 宿主冒烟（虽不走真 socket，import 需成）

from . import theme

_SLICE_MS = 150                 # poll 切片窗口：无数据即回调动画一次


class FetchError(Exception):
    pass


def _parse_url(url):
    if not url.startswith('http://'):
        raise FetchError('only plain http url supported')
    hostport, _, path = url[7:].partition('/')
    host, _, port = hostport.partition(':')
    return host, int(port or 80), '/' + path


def _read(sock, n, on_wait, deadline, poller):
    """切片读：poll 150ms 窗口等数据，无数据回调动画后重试；总超时
    兑底防死循环。poll 对「无数据」的判定与固件 sock 超时语义无关
    （部分固件 stream read 不按 settimeout 抛 OSError，会整段阻塞在
    C read 里——动画全停、主循环冻结，2026-09-05 真机症状），服务器
    上游抓得再久，时间条/标注动画也照常走。"""
    while True:
        if time.ticks_diff(deadline, time.ticks_ms()) < 0:
            raise FetchError('read timeout')
        if not poller.poll(_SLICE_MS):       # 窗口内无数据：动画一帧
            if on_wait:
                on_wait()
            continue
        try:
            return sock.read(n)              # 有数据才读（不盲等阻塞）
        except OSError:
            if on_wait:                      # 连接异常：照旧切片重试，
                on_wait()                    # deadline 到点统一报错


def _read_line(sock, on_wait, deadline, poller):
    line = b''
    while True:
        c = _read(sock, 1, on_wait, deadline, poller)
        if not c:                      # None 或 b''：连接关闭
            raise FetchError('connection closed')
        line += c
        if line.endswith(b'\r\n'):
            return line[:-2]


def http_get(host, port, path, on_wait=None, sink=None, timeout_ms=30_000,
             auth_key=''):
    """底层 GET：手写 socket 切片读。返回 (code, body)。

    sink(chunk) 提供时 body 流式写出、返回 (code, None)——贴图下载
    用，不在堆上累积。auth_key 非空时带 `Authorization: Bearer` 头
    （key 不进 URL，不进访问日志）。失败统一抛 FetchError。
    """
    deadline = time.ticks_add(time.ticks_ms(), timeout_ms)
    if on_wait:
        on_wait()                     # DNS/连接阶段阻塞无切片，边界补画
    try:
        ai = socket.getaddrinfo(host, port)[0][-1]
        if on_wait:
            on_wait()
        sock = socket.socket()
    except OSError as exc:
        raise FetchError('dns/socket: {}'.format(exc))

    try:
        sock.settimeout(10)
        sock.connect(ai)
        if on_wait:
            on_wait()                 # 请求发出前再补一帧
        if auth_key:
            sock.write(b'GET ' + path.encode() + b' HTTP/1.1\r\n'
                       b'Host: ' + host.encode() + b'\r\n'
                       b'Authorization: Bearer ' + auth_key.encode()
                       + b'\r\n'
                       b'Connection: close\r\n\r\n')
        else:
            sock.write(b'GET ' + path.encode() + b' HTTP/1.1\r\n'
                       b'Host: ' + host.encode() + b'\r\n'
                       b'Connection: close\r\n\r\n')
        gc.collect()

        sock.settimeout(10)            # 读写兑底（读节奏由 poll 接管）
        poller = _select.poll()
        poller.register(sock, _select.POLLIN)
        status_line = _read_line(sock, on_wait, deadline, poller)
        parts = status_line.split()
        code = int(parts[1]) if len(parts) > 1 else 0

        clen = None
        while True:
            header = _read_line(sock, on_wait, deadline, poller)
            if not header:
                break
            hkey, _, value = header.partition(b':')
            if hkey.strip().lower() == b'content-length':
                clen = int(value.strip())

        buf = bytearray()
        total = 0
        while clen is None or total < clen:
            chunk = _read(sock, 256, on_wait, deadline, poller)
            if not chunk:
                if clen is not None:
                    raise FetchError('short body')
                break
            total += len(chunk)
            if sink:
                sink(chunk)
            else:
                buf += chunk
    except FetchError:
        raise
    except OSError as exc:
        raise FetchError('network: {}'.format(exc))
    finally:
        sock.close()

    gc.collect()
    return code, (None if sink else bytes(buf))


_H = 16
# SSR 贴图底色/主题随昼夜套走（theme.current_bg_hex/current_theme）：
# 贴图与页底同色合成、前景随昼夜套，夜态切换后 hash 自动换套
# （服务端双套各自缓存、端侧 rcache 按 hash 共存，切换零计算）。

def server_base(url):
    """从配置 url 推导 {base}（scheme://host[:port]）。

    兼容两种填法：纯 base（http://host:8765）或带 /api/ 路径
    （http://host:8765/api/xx）——后者截掉 /api/... 得 base。
    """
    host, port, path = _parse_url(url)
    p = path.split('?')[0]
    idx = p.find('/api/')
    if idx > 0:
        p = p[:idx]
    p = p.rstrip('/')
    base = 'http://' + host
    if port != 80:
        base += ':' + str(port)
    return base + p


def _endpoint_get(server_cfg, endpoint, page=None, on_wait=None, night=False):
    """对 {base}{endpoint}?client=1&h&bg&theme&fg&caption&peak&offpeak
    做一次 GET；page（页码）可选，前置拼在 client 之前。200 → 解析
    json；非 200/坏 json → FetchError。bg/theme/前景色覆盖均由
    theme.py 按当前夜态与 palette 给出（SSR 颜色全部走 palette——
    服务端按请求色渲染贴图）。"""
    full = server_base(server_cfg['url']) + endpoint
    host, port, path = _parse_url(full)
    c = theme.ssr_colors(night)
    if page is not None:
        path += '?page={}'.format(int(page)) + '&'
    else:
        path += '?'
    path += ('client=1&h={}&bg={}&theme={}'
             '&fg={}&caption={}&peak={}&offpeak={}').format(
        _H, theme.current_bg_hex(night), theme.current_theme(night),
        c['fg'], c['caption'], c['peak'], c['offpeak'])
    key = str(server_cfg.get('key') or server_cfg.get('token', ''))
    code, body = http_get(host, port, path, on_wait, auth_key=key)
    if code != 200:
        raw = body[:48]            # 可读错误片段（如 400 参数/版本协商失败）
        snippet = ''.join(chr(b) if 32 <= b < 127 else ' '
                          for b in raw).strip()
        raise FetchError('HTTP {} {}'.format(code, snippet)[:40])
    try:
        data = ujson.loads(body)
    except ValueError:
        raise FetchError('bad json')
    del body
    gc.collect()
    return data


def fetch_page(server_cfg, page, on_wait=None, night=False):
    """GET /api/page?page=N：按页码查表返回当前页（响应带
    type/total/page——overview 聚合或 provider 单页详情，按 type
    分发解析；越界回卷永不 404）。"""
    return _endpoint_get(server_cfg, '/api/page', page,
                         on_wait, night)
