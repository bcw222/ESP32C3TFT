"""usageapp 端侧逻辑的宿主冒烟测试（无需真机/MicroPython）。

stub 掉 st7789/ujson 后直接驱动 page.py 的渲染路径（含 SSR 贴图
槽）、client 的结构校验、render 的缓存/下载——只测纯逻辑，
不测显示效果。（demo 假数据已移到服务端 llm-usage-server，
端侧无 demo 模块。）
运行：python3 tools/smoke_usageapp.py（从仓库根）
"""
import json
import os
import sys
import tempfile
import types
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent

# ---- MicroPython 模块 stub ----
st7789 = types.ModuleType('st7789')
st7789.BLACK = 0
st7789.WHITE = 0xFFFF      # 真机常量：白底黑字模式用


def color565(r, g, b):
    return (r << 16) | (g << 8) | b      # 数值只需稳定，无需真彩


st7789.color565 = color565
sys.modules['st7789'] = st7789

ujson = types.ModuleType('ujson')
ujson.dumps = json.dumps
ujson.loads = json.loads
sys.modules['ujson'] = ujson

# gc.mem_free/mem_alloc 是 MicroPython 专属，宿主垫上（诊断打印用）
import gc as _host_gc  # noqa: E402
_host_gc.mem_free = lambda: 65536
_host_gc.mem_alloc = lambda: 32768
# time.sleep_ms 是 MicroPython 专属（render._backoff 重试间隔用）；
# 宿主垫成真实 sleep（秒），毫秒换算。真机不受影响。
import time as _host_time  # noqa: E402
_host_time.sleep_ms = lambda ms: _host_time.sleep(ms / 1000)
# ticks_ms/ticks_diff：MicroPython 单调时钟语义（ticks_diff 带环绕）——
# 宿主直接用真实毫秒即可（page.ErrorPage 的点动画相位、timeline 分段用）
_host_time.ticks_ms = lambda: int(_host_time.time() * 1000)
_host_time.ticks_diff = lambda a, b: a - b

sys.path.insert(0, str(ROOT))
# 以包形式导入（usageapp 内部用的是 `from . import x` 相对导入，
# 设备上由入口链 `import usageapp.app` 建包上下文，宿主同理）
from usageapp import client, page, render   # noqa: E402


class FakeFont:
    HEIGHT = 16


class FakeDisplay:
    def __init__(self):
        self.calls = 0
        self.pngs = []
        self.texts = []

    def fill(self, c):
        self.calls += 1

    def fill_rect(self, x, y, w, h, c):
        self.calls += 1

    def text(self, font, s, x, y, c=None, bg=None):
        assert isinstance(s, str), 'text() 收到非字符串: {!r}'.format(s)
        self.texts.append(s)
        self.calls += 1

    def write(self, font, s, x, y, c=None, bg=None):
        assert isinstance(s, str), 'write() 收到非字符串: {!r}'.format(s)
        self.calls += 1

    def write_len(self, font, s):
        return len(s) * 18       # digits 字体近满宽（大字擦除矩形计算用）

    def png(self, path, x, y, transparency=False):
        # 真机固件语义：文件打不开 → OSError（ENOENT 崩溃源）。
        # 宿主等价复刻，保证测试能抓住"把已删除路径当可用"这类 bug。
        if not os.path.exists(path):
            raise OSError(2, 'ENOENT')
        self.pngs.append((path, x, y))
        self.calls += 1


def main():
    d, f = FakeDisplay(), FakeFont()
    now = 1_800_000_000

    # client=1 schema 形态的样例数据（契约样例；服务端 demo 假数据
    # 由 llm-usage-server 自测覆盖，端侧只认结构）
    glm_data = {
        'server_time': now, 'id': 'glm', 'status': 'ok',
        'updated_at': now - 5,
        'quotas': [
            {'id': 'tokens:5h', 'unit': 'tokens', 'percent': 51.6,
             'used': 12_345, 'limit': 24_000, 'remaining': 11_655,
             'reset_at': now + 18_000},
            {'id': 'credits:1w', 'unit': 'credits', 'percent': 42.1,
             'used': 42_100, 'limit': 100_000, 'remaining': 57_900,
             'reset_at': now + 604_800},
            {'id': 'tokens:1mo', 'unit': 'tokens', 'percent': 8.2,
             'used': 1_640, 'limit': 20_000, 'remaining': 18_360,
             'reset_at': now + 2_592_000},
        ]}
    zen_data = {
        'server_time': now, 'id': 'zen', 'status': 'ok',
        'updated_at': now - 5,
        'quotas': [{'id': 'credits:1mo', 'unit': 'credits', 'percent': 55.0,
                    'used': 49_500, 'limit': 90_000, 'remaining': 40_500,
                    'reset_at': now + 2_592_000}]}
    ov_data = {'server_time': now, 'items': [
        {'id': 'glm', 'percent': max(q['percent']
                                     for q in glm_data['quotas'])},
        {'id': 'zen', 'percent': zen_data['quotas'][0]['percent']}]}
    by_id = {'glm': glm_data, 'zen': zen_data}

    # provider 页：3 行档全量 + diff + 1 行档
    pg = page.ProviderPage('GLM')
    pg.update(by_id['glm'], now)
    assert pg.render(d, f, now) is True           # 全量重画
    pg.render(d, f, now + 1)                      # diff
    assert pg.render(d, f, now + 2) is False

    zen = page.ProviderPage('zen')
    zen.update(by_id['zen'], now)
    assert zen.render(d, f, now) is True          # 1 行档

    # 4 行紧凑档
    four = dict(by_id['glm'])
    four['quotas'] = by_id['glm']['quotas'] + [
        {'id': 'x:1y', 'unit': 'tokens', 'percent': 1.0,
         'used': 1, 'limit': 100, 'reset_at': now + 100}]
    pg4 = page.ProviderPage('four')
    pg4.update(four, now)
    assert pg4.render(d, f, now) is True
    pg4.render(d, f, now + 1)

    # 总览页：plan 条目（迷你条+百分比）
    ov = page.OverviewPage()
    ov.update(ov_data)
    assert ov.render(d, f, now) is True
    ov.render(d, f, now + 1)
    assert ov.render(d, f, now + 2) is False

    # 总览页 kind 分形态（SCHEMA.md「kind 分类」）：balance 行无百分比、
    # 金额大字端侧自绘（含符号）；bundle 行剩余量 + 距重置倒计时；
    # plan 行最危险窗口 percent；行名缺失 → '--' 不画 id
    ovk = page.OverviewPage()
    ovk.update({
        'name': '总览', 'status': 'ok', 'items': [
            {'id': 'glm', 'kind': 'plan', 'percent': 66.0,
             'reset_at': now + 3600},
            {'id': 'deepseek', 'kind': 'balance', 'currency': 'CNY',
             'amount': '110', 'symbol_render': None},
            {'id': 'glm_pack', 'kind': 'bundle', 'currency': '',
             'amount': '612786tok', 'reset_at': now + 7200},
        ]})
    assert ovk.rows[0][1] == 66.0 and ovk.rows[0][6] == now + 3600
    # symbol_render 已从端侧消费面移除（金额大字端侧自绘），下发也须忽略；
    # 符号由端侧按 currency 币种码拼进大字（_with_symbol）
    assert ovk.rows[1][1] is None and ovk.rows[1][3] == '110'
    assert ovk.rows[1][4] == 'CNY'
    assert ovk.rows[2][3] == '612786tok' and ovk.rows[2][6] == now + 7200
    assert ovk.render(d, f, now) is True
    ovk.render(d, f, now + 61)               # diff：倒计时走字不炸
    assert ovk.render(d, f, now + 62) is False

    # 稀疏型（余额型 provider）：balance + peak 大字插图竖排
    sparse = page.ProviderPage('ds')
    sparse.update({'id': 'deepseek', 'name': 'deepseek', 'status': 'ok',
                   'updated_at': now, 'quotas': [],
                   'peak': {'state': 'peak', 'badge_render': None,
                            'badge_small_render': None,
                            'caption_render': None,
                            'ends_at': now + 300},
                   'balance': {'currency': 'CNY', 'render': None,
                               'caption_render': None}}, now)
    assert sparse._sparse is True
    assert sparse.render(d, f, now) is True
    sparse.render(d, f, now + 1)               # diff：倒计时走字

    # 密集 + peak：多 quota plan 峰谷行（不稀疏）——一行：caption 贴图 +
    # 两色倒计时（2026-09-06；badge_small_render 已停发）
    dense_peak = page.ProviderPage('glm')
    dp = dict(by_id['glm'])
    dp['peak'] = {'state': 'offpeak', 'caption_render': None,
                  'ends_at': now + 500}
    dense_peak.update(dp, now)
    assert dense_peak._sparse is False
    assert dense_peak.render(d, f, now) is True
    dense_peak.render(d, f, now + 1)
    # caption 缺失 → 槽留空；倒计时按 state 上绿色（offpeak）
    assert dense_peak._peak_cap_img is None
    assert dense_peak._peak_cd_color == page._OFFPEAK_BIG_RGB

    # 单窗 plan + 峰谷（qwen 谷价形态，2026-09-02）：1 条 quota 也走
    # 密集峰谷行（两色倒计时），不走稀疏大字；配额大条照画
    qwen = page.ProviderPage('qwen')
    qwen.update({'id': 'qwen', 'name': 'qwen', 'status': 'ok',
                 'updated_at': now,
                 'quotas': [{'id': 'credits:1w', 'unit': 'credits',
                             'percent': 42.0, 'used': 42_000,
                             'limit': 100_000, 'remaining': 58_000,
                             'reset_at': now + 604_800}],
                 'peak': {'state': 'peak', 'caption_render': None,
                          'badge_render': None,
                          'ends_at': now + 300}}, now)
    assert qwen._sparse is False            # 1 条 quota → 密集峰谷行
    assert qwen.render(d, f, now) is True
    qwen.render(d, f, now + 1)              # diff：倒计时走字
    # 峰谷行渲染所需 _LAYOUTS_PEAK[1] 档位存在且配额区避开峰谷区
    assert page._LAYOUTS_PEAK[1][0] >= 46 + 16

    # 错误页 + 重试倒计时
    ep = page.ErrorPage()
    ep.set('HTTP 400 bad client')
    assert ep.render(d, f, now, retry_left_s=30) is True
    ep.render(d, f, now, retry_left_s=29)

    # 夜间切换触发全量重画
    page.NIGHT = True
    assert pg.render(d, f, now + 3) is True
    page.NIGHT = False

    # Δ：同窗口 percent 变化被记录（内存态）
    bumped = dict(by_id['glm'])
    bumped['quotas'] = [dict(q) for q in by_id['glm']['quotas']]
    bumped['quotas'][0]['percent'] += 0.3
    pg.update(bumped, now + 60)
    pg.render(d, f, now + 60)
    assert pg._deltas, 'Δ 未记录'

    # ---- SSR 贴图 ----
    tmpcache = tempfile.mkdtemp()
    rstore = render.RenderStore('http://x:1', 'k',
                                cache_dir=tmpcache + '/rc', max_files=3)

    def fake_png_bytes(w):
        return (b'\x89PNG\r\n\x1a\n\x00\x00\x00\x0dIHDR'
                + w.to_bytes(4, 'big') + b'\x00\x00\x00\x10'
                + b'\x00' * 9)

    h_ok, h_wide = 'aabbcc01', '11223344'
    with open(rstore.path(h_ok), 'wb') as fh:
        fh.write(fake_png_bytes(60))
    with open(rstore.path(h_wide), 'wb') as fh:
        fh.write(fake_png_bytes(300))

    data_r = json.loads(json.dumps(glm_data))
    data_r['title_render'] = h_ok
    data_r['quotas'][0]['label_render'] = h_wide
    wanted = render.collect(data_r)
    # collect 返回 [(hash, slot_w), ...]：槽位实际宽度随 hash 走
    assert (h_ok, page.TITLE_SLOT_W) in wanted
    assert (h_wide, page.LABEL_SLOT_W) in wanted
    # error 不入 SSR（2026-09-02 定稿）：错误统一端侧 ASCII 单行，
    # error.render 停发——collect 必须忽略 error 字段
    data_r['error'] = {'code': 'x', 'render': h_ok}
    assert (h_ok, page.ERR_SLOT_W) not in render.collect(data_r)
    # overview 行名贴图（symbol_render 路径已删：金额大字端侧自绘，
    # 字符集 ¥$0-9. 并入 lib/digits）
    ov_want = render.collect({'items': [{'render': h_ok}]})
    assert (h_ok, page.OV_ITEM_SLOT_W) in ov_want

    png_calls = len(d.pngs)
    tpg = page.ProviderPage('T')
    tpg.update(data_r, now)
    tpg.render(d, f, now, rstore)
    assert len(d.pngs) == png_calls + 1     # 标题贴上；超宽标签 fallback
    assert d.pngs[-1] == (rstore.path(h_ok), 4, 6)

    # 缺失 hash → fallback 不炸；缓存后下一帧 diff 补贴
    data_r['title_render'] = 'ffeeddcc'
    tpg.update(data_r, now)
    tpg.render(d, f, now, rstore)           # hash 变 → 全量，无 png
    assert len(d.pngs) == png_calls + 1
    with open(rstore.path('ffeeddcc'), 'wb') as fh:
        fh.write(fake_png_bytes(70))
    tpg.render(d, f, now, rstore)           # diff 补贴
    assert len(d.pngs) == png_calls + 2

    # 结构化 error → 纯 ASCII 单行（2026-09-02 定稿：error 不入 SSR）；
    # 徽标两态（2026-09-05）：status='error' 归一 stale，徽标无红 err
    ep2 = page.ProviderPage('E')
    ep2.update({'id': 'e', 'status': 'error', 'updated_at': now,
                'error': {'code': 'upstream_http_401', 'render': h_ok},
                'quotas': []}, now)
    assert ep2.status == 'stale'
    png_before = len(d.pngs)
    ep2.render(d, f, now, rstore)
    assert len(d.pngs) == png_before          # error 不再贴图
    assert 'upstream_http_401' in d.texts     # 错误行 ASCII 直画
    assert 'stale 0m' in d.texts              # 徽标 stale（带年龄）
    assert 'err' not in d.texts               # 徽标两态：无 'err' 元素

    # 非 ASCII 错误文本不留字（乱码防护）
    ep2.set_error_keep('上游 502 错误')
    assert ep2.error_text == ''

    # 限宽：错误行止步 RSSI 角标左缘（19 字符×8=152px，x4..156<x160）
    ep2.set_error_keep('A' * 30)
    assert ep2.error_text == 'A' * 19

    # 总览页级 stale + 单行错误（OverviewPage 补齐 set_error_keep）
    ov2 = page.OverviewPage()
    ov2.update({'title_render': None, 'status': 'ok',
                'items': [{'id': 'a', 'kind': 'plan', 'percent': 10.0}]})
    ov2.render(d, f, now, rstore)
    ov2.set_error_keep('HTTP 502 x')
    assert ov2.status == 'stale' and ov2.error_text == 'HTTP 502 x'
    ov2.render(d, f, now + 1, rstore)         # diff：徽标 stale + 错误行
    assert 'HTTP 502 x' in d.texts
    assert 'stale' in d.texts

    # 总览行级 error（2026-09-05 + 2026-09-09 定稿：行级只两态
    # ok/error，stale 永远是整页级）：屏蔽服务端 percent=0.0 兑底——
    # 画红 'err'，绝不渲染虚假 0%；行级 stale（服务端已不产，纯防御）
    # 归一 ok 照常渲染旧值
    d2 = FakeDisplay()
    ov3 = page.OverviewPage()
    ov3.update({'status': 'ok', 'items': [
        {'id': 'bad', 'kind': 'plan', 'percent': 0.0, 'status': 'error'},
        {'id': 'old', 'kind': 'plan', 'percent': 33.0, 'status': 'stale'},
    ]})
    assert ov3.rows[0][1] is None and ov3.rows[0][3] == ''    # error 已屏蔽
    assert ov3.rows[1][1] == 33.0             # 行级 stale 归一 ok：旧值保留
    assert ov3.rows[1][7] == 'ok'             # 行级 status 只剩 error；空则
    #                                           回退页面级（ok/stale 传播）
    ov3.render(d2, f, now, rstore)
    assert '0%' not in d2.texts
    assert 'err' in d2.texts
    assert '33%' in d2.texts
    # 整页级 stale 传播（degrade_stale）不受影响：行级 status 空 →
    # 回退页面级 stale，行照常画旧值（上面的 ov2 已验徽标 + ERR_Y）

    # 错误页连续失败计数（2026-09-05）：fail #N 单调递增（证明在重试）
    ep3 = page.ErrorPage()
    ep3.set('HTTP 502')
    ep3.set_fails(3)
    assert ep3.render(d2, f, now, retry_left_s=10) is True
    assert 'fail #3' in d2.texts
    ep3.set_fails(4)
    ep3.render(d2, f, now, retry_left_s=9)
    assert 'fail #4' in d2.texts
    ep3.set_fails(0)
    ep3.render(d2, f, now, retry_left_s=8)
    assert ep3._fails_drawn == ''             # 归零：计数行擦除

    # 总览贴图（标题 + 行 render；金额大字端侧自绘不走贴图）
    ovp = page.OverviewPage()
    ovp.update({'name': '总览', 'title_render': h_ok,
                'status': 'ok',
                'items': [{'id': 'glm', 'kind': 'plan', 'percent': 50.0,
                           'render': h_ok},
                          {'id': 'deepseek', 'kind': 'balance',
                           'currency': '¥', 'amount': '110'}]})
    ovp.render(d, f, now, rstore)
    assert d.pngs[-2:] == [(rstore.path(h_ok), 4, 6),
                           (rstore.path(h_ok), 8, 56)]

    # ---- 淘汰后 _dims 失效（真机 ENOENT 崩溃回归）----
    # max_files 极小 → 下一张下载即触发淘汰；被淘汰 hash 的内存 (宽,高)
    # 缓存若不失效，usable() 会把已删除路径当可用返回 → display.png
    # OSError ENOENT。FakeDisplay.png 已复刻"文件不存在必抛"的真机语义。
    orig_get = client.http_get
    png_blob = fake_png_bytes(50)

    def fake_get(host, port, path, on_wait=None, sink=None, timeout_ms=0,
                 auth_key=''):
        assert sink is not None and '/api/render/' in path
        for i in range(0, len(png_blob), 16):
            sink(png_blob[i:i + 16])
        return 200, None

    tiny = render.RenderStore('http://x:1', 'k',
                              cache_dir=tmpcache + '/rtiny', max_files=2)
    seq = ['aa000001', 'bb000002', 'cc000003']
    for i, hh in enumerate(seq):
        with open(tiny.path(hh), 'wb') as fh:
            fh.write(fake_png_bytes(60))
        assert tiny.usable(hh, 100) == tiny.path(hh)   # 先读出尺寸入缓存
        os.utime(tiny.path(hh), (2000 + i, 2000 + i))
    # 下载第 4 张触发淘汰：max=2，最旧的 aa000001 / bb000002 被删
    client.http_get = fake_get
    try:
        tiny.download('dd000004')
    finally:
        client.http_get = orig_get
    assert not os.path.exists(tiny.path('aa000001'))
    assert tiny.usable('aa000001', 100) is None     # 死路径必须判不可用
    assert tiny.height('aa000001') is None          # 同理高度缓存也要失效

    # download：monkeypatch http_get 流式 sink → 落盘、宽度、淘汰
    client.http_get = fake_get
    try:
        seq = ('deadbeef', 'cafef00d', '0badc0de', '12345678')
        for i, hh in enumerate(seq):
            assert rstore.download(hh) == 50
            # 同秒下载的 mtime 并列会让淘汰排序不稳定，人为递增
            os.utime(rstore.path(hh), (1000 + i, 1000 + i))
        assert rstore.has('12345678') and not rstore.has('deadbeef')
        names = [n for n in os.listdir(tmpcache + '/rc')
                 if render.valid_hash(n)]
        assert len(names) <= 3             # max_files=3 淘汰最旧
    finally:
        client.http_get = orig_get

    # 404 → FetchError，不留垃圾（单次尝试：显式 ssr_retry=1 不重试）
    def fake_404(host, port, path, on_wait=None, sink=None, timeout_ms=0,
                 auth_key=''):
        return 404, None

    client.http_get = fake_404
    try:
        try:
            rstore.download('9999aaaa',
                            server_cfg={'ssr_retry': 1,
                                        'ssr_retry_delay_ms': 0})
            raise AssertionError('404 应抛 FetchError')
        except client.FetchError:
            pass
    finally:
        client.http_get = orig_get

    # SSR 重试：默认 3 次（含首次），前 2 次失败第 3 次成功 → 落盘；
    # 失败间不残留 .tmp；全部失败抛 'render retry x3'。
    # 显式 server_cfg 把延迟设 0，宿主测试不真睡 2000ms。
    retry_cfg = {'ssr_retry': 3, 'ssr_retry_delay_ms': 0}
    flaky = {'n': 0}

    def fake_flaky(host, port, path, on_wait=None, sink=None, timeout_ms=0,
                   auth_key=''):
        flaky['n'] += 1
        if flaky['n'] < 3:                 # 前 2 次网络失败
            raise client.FetchError('boom')
        assert sink is not None
        for i in range(0, len(png_blob), 16):
            sink(png_blob[i:i + 16])
        return 200, None

    client.http_get = fake_flaky
    try:
        assert rstore.download('f1a2b3c4', server_cfg=retry_cfg) == 50
        assert flaky['n'] == 3             # 恰好第 3 次成功
        assert rstore.has('f1a2b3c4')
    finally:
        client.http_get = orig_get
    # 不残留 .tmp（第 1、2 次失败后已删，第 3 次成功后 rename 走掉）
    assert not os.path.exists(rstore.dir + '/.tmp')

    flaky2 = {'n': 0}
    seq2 = []

    def fake_alldead(host, port, path, on_wait=None, sink=None,
                     timeout_ms=0, auth_key=''):
        flaky2['n'] += 1
        seq2.append(on_wait is not None)
        raise client.FetchError('boom')

    client.http_get = fake_alldead
    try:
        try:
            rstore.download('0d1e2f3a', on_wait=lambda: None,
                            server_cfg=retry_cfg)
            raise AssertionError('重试 3 次全败应抛 FetchError')
        except client.FetchError as exc:
            assert 'retry x3' in str(exc)
        assert flaky2['n'] == 3
        assert all(seq2)                   # 每次尝试都带 on_wait（时间条走字）
        assert not rstore.has('0d1e2f3a')  # 全败不落盘
    finally:
        client.http_get = orig_get

    # OOM 退避：png 抛 MemoryError → fallback，本周期 usable 返回 None
    h_oom = 'abcdef99'
    with open(rstore.path(h_oom), 'wb') as fh:
        fh.write(fake_png_bytes(60))
    boom = {'n': 0}

    def png_oom(path, x, y, transparency=False):
        boom['n'] += 1
        raise MemoryError()

    d.png, orig_png = png_oom, d.png
    try:
        opg = page.ProviderPage('O')
        opg.update({'id': 'o', 'status': 'ok', 'updated_at': now,
                    'title_render': h_oom, 'quotas': []}, now)
        opg.render(d, f, now, rstore)
        opg.render(d, f, now, rstore)       # diff 不重试（退避）
        assert boom['n'] == 1
        assert rstore.usable(h_oom, 104) is None
    finally:
        d.png = orig_png
    rstore.missing([])                      # 新周期清退避
    assert rstore.usable(h_oom, 104) is not None

    print('smoke OK: {} 次绘制调用, {} 张贴图'
          .format(d.calls, len(d.pngs)))


if __name__ == '__main__':
    main()
