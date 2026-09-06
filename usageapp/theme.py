"""配色方案：day/night 两套全量色板 + usage_cfg.palette 覆盖。

用户定稿（2026-08-29 二次）：**昼夜按两套配色方案整体切换**——day =
白底深色字，night = 深底亮色字；夜间不只是压背光，整个 UI 换肤。
usage_cfg.py 里加 `palette = {'day': {...}, 'night': {...}}` 可覆盖
任意子集（未给的键用内置默认；只写 day 侧则 night 侧仍用默认夜间
色板，反之亦然）；颜色值接受 'RRGGBB' hex 串（推荐）或 (r,g,b) 元组，
非法值忽略用默认。

app.run() 开头调 theme.apply(cfg) 落两套色；夜间检测切态时调
theme.set_night(night)——把对应那套色落进 page 模块（页面渲染处处
动态读 page 的模块常量）并返回该套色，app 据此给 Timeline 换四色/
底色。SSR 请求的 bg/theme 参数由 client 从 theme.current_bg_hex()/
current_theme() 取（贴图与页底同色合成、前景随昼夜），夜态切换后
贴图 hash 自动换套（服务端双套各自缓存，端侧 rcache 按 hash 共存）。"""
import st7789

from . import page

# ---- 默认色板：day（白底深色系，键名即语义，改值即换肤）----
_DAY = {
    'bg': 'FFFFFF',         # 全屏底色
    'fg': '000000',         # 默认前景（标题回退/错误行/状态文字格）
    'rows': ('0E6FC4',      # 配额行色 ×4（避开红/琥珀——阈值色专用）
             '00897B',
             '6C3FC2',
             '5F6670'),
    'warn': 'C87800',       # percent >= 80 琥珀
    'crit': 'C80000',       # percent >= 95 红
    'track': 'ECECEF',      # 进度条轨道
    'ok': '008C00',         # 状态点 ok / ok 行名
    'stale': '8A9199',      # 状态点 stale
    'err': 'C80000',        # 状态点 error / 错误提示
    'dim': '404040',        # 弱化文本（数额/币种码/无峰谷大字）
    'caption': '555A61',    # 灰标签/倒计时/时间条标注灰
    'peak': 'C80000',       # 峰谷红（大字/总览余额行）
    'offpeak': '008C00',    # 峰谷绿
    # 时间条四色：wait(等待下一轮条底)/fetch(network 主请求段)/
    # up(upstream 段)/ssr(SSR 贴图下载段)
    'timeline': ('DADDE2', '008C00', 'C87800', '0E6FC4'),
}

# ---- 默认色板：night（深底亮色系，同一组键——两套必须键位对齐）----
_NIGHT = {
    'bg': '000000',
    'fg': 'FFFFFF',
    'rows': ('42A5F5', '2FD6B0', 'B388FF', '9AA1AC'),
    'warn': 'FFB340',
    'crit': 'FF5A5A',
    'track': '262B33',
    'ok': '3DDC84',
    'stale': '5A616B',
    'err': 'FF5A5A',
    'dim': 'B0B0B0',
    'caption': '8A919C',
    'peak': 'FF5A5A',
    'offpeak': '3DDC84',
    'timeline': ('20242C', '3DDC84', 'FFB340', '42A5F5'),
}

# apply() 落下的两套全量 RGB（set_night/_current 读；None=未 apply）。
_DAY_PAL = None
_NIGHT_PAL = None


def _rgb(v):
    """'RRGGBB'、(r,g,b) 或多色元组（每项同上）→ RGB 元组；非法 None。

    多色形态（rows/timeline）逐项解析，返回元组的元组。
    """
    if isinstance(v, str):
        if len(v) == 6:
            try:
                return (int(v[0:2], 16), int(v[2:4], 16), int(v[4:6], 16))
            except ValueError:
                return None
        return None
    if isinstance(v, (tuple, list)):
        if len(v) == 3 and all(isinstance(x, int) and not
                               isinstance(x, bool) for x in v):
            return (int(v[0]), int(v[1]), int(v[2]))    # 单色 (r,g,b)
        parsed = [_rgb(item) for item in v]             # 多色组
        if parsed and all(c is not None for c in parsed):
            return tuple(parsed)
        return None
    return None


def _base(cfg):
    """usage_cfg.palette（分昼夜覆盖）⊕ 内置默认 → {day, night} 全量。

    两种覆盖形态都收：旧单层 `palette = {...}`（等价于只覆盖 day 套，
    夜间继续用内置夜间默认——白底屏用户的常见自定义是调 day 色）与
    新双层 `palette = {'day': {...}, 'night': {...}}`。键名对齐
    _DAY/_NIGHT；非法值/整组非法回默认。
    """
    user = None
    if isinstance(cfg, dict):
        user = cfg.get('palette')
    elif cfg is not None:
        user = getattr(cfg, 'palette', None)
    # 形态判定：{'day': {...}} / {'night': {...}} → 双层；否则视为
    # 单层（等价于只覆盖 day 套，夜间用内置默认）。
    two_layer = isinstance(user, dict) and any(
        k in user and isinstance(user[k], dict) for k in ('day', 'night'))
    day_user = user.get('day') if two_layer else user
    night_user = user.get('night') if two_layer else None
    out = {'day': _one(_DAY, day_user),
           'night': _one(_NIGHT, night_user)}
    return out


def _one(defaults, user):
    """单套色板合并：user（部分覆盖 dict，键同 defaults）⊕ defaults
    → 全量 RGB dict；非法值回默认。"""
    out = {}
    for k, dv in defaults.items():
        v = None
        if isinstance(user, dict) and k in user:
            v = _rgb(user[k])
        out[k] = v if v is not None else _rgb(dv)
    return out


def set_night(night):
    """切昼夜套：对应那套全量色落进 page 模块；返回 (四色 RGB 元组,
    底色 RGB)——app 据此给 Timeline 换色。apply() 之前调用无害
    （色板退内置默认）。"""
    pal = (_NIGHT_PAL if night else _DAY_PAL) or _one(
        _NIGHT if night else _DAY, None)
    page.set_palette(pal)
    page._TL_RGBS = pal['timeline']
    return pal['timeline'], pal['bg']


def current_bg_hex(night):
    """夜态 → SSR 请求的 bg hex（贴图与页底同色合成）。"""
    return '{:02X}{:02X}{:02X}'.format(
        *(_current(night)['bg']))


def ssr_colors(night):
    """夜态 → SSR 请求的前景色覆盖（RRGGBB hex dict）。

    fg=标题前景、caption=灰标签、peak/offpeak=峰谷语义色——贴图
    颜色跟随 usage_cfg.palette 自定义（服务端按请求色渲染，色入
    像素即入 hash，缓存机制不受影响；不带参数时服务端用其内置
    _THEME_FG/THEME_SEMANTIC 默认色）。"""
    pal = _current(night)

    def hx(rgb):
        return '{:02X}{:02X}{:02X}'.format(*rgb)

    return {'fg': hx(pal['fg']), 'caption': hx(pal['caption']),
            'peak': hx(pal['peak']), 'offpeak': hx(pal['offpeak'])}


def err_rgb(night):
    """夜态 → 错误红 RGB（时间条 wait 段请求出错时改涂；theme.err 键）。"""
    return _current(night)['err']


def current_theme(night):
    """夜态 → SSR 请求的 theme 参数（'day'/'night'）。"""
    return 'night' if night else 'day'


def _current(night):
    """当前夜态应生效的那套 RGB dict（set_night 未走过时退内置默认）。"""
    pal = _NIGHT_PAL if night else _DAY_PAL
    if pal is None:
        pal = _one(_NIGHT if night else _DAY, None)
    return pal


def apply(cfg=None):
    """解析 usage_cfg.palette 落两套色（app 启动时调一次）；并把
    日间那套立即落进 page（返回 (四色 RGB 元组, 底色 RGB)，app 转
    565 后传 Timeline）。夜间套此后由 set_night() 切换时落。"""
    global _DAY_PAL, _NIGHT_PAL
    merged = _base(cfg)
    _DAY_PAL = merged['day']
    _NIGHT_PAL = merged['night']
    page.set_palette(_DAY_PAL)
    page._TL_RGBS = _DAY_PAL['timeline']
    return _DAY_PAL['timeline'], _DAY_PAL['bg']


def timeline_colors():
    """时间条 (四色 RGB 元组, 底色 RGB)——读 apply 已落色的 page 状态。"""
    return page._TL_RGBS, page._BG_RGB
