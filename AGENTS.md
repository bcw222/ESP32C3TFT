# AGENTS.md — ESP32C3TFT 端侧（usageapp）开发备忘

> 本文件是换会话后的「唯一入口」：架构、契约、设计约束、踩过的坑都在这里。
> 归档对话前读它；做完非平凡改动后更新它。

---

## 1. 这是什么

电池供电的 ESP32-C3 + 240×240 ST7789 徽章。本项目在其上跑
**AI 用量显示**应用（`usageapp/`），常亮显示 AI 订阅用量
（出厂的图片浏览器/launcher 已移除，见 git 历史）。

配套服务端是**另一个独立仓库** `../llm-usage-server`（电脑/VPS 上跑）。
两者的唯一耦合点是**契约**，定义在：

```
../llm-usage-server/SCHEMA.md   ← 唯一事实源（双方都以它为准）
```

端侧不持有任何 provider 凭据、不做 schema 校验、不做 provider 特判。
加 provider / 换 provider 只改服务端配置，端侧零改动。

---

## 2. 目录与分发形态

| 位置 | 内容 | 分发 | 理由 |
|---|---|---|---|
| 根 `boot.py` | 固件入口 + **全部设备初始化**（无线电/acc 配置/共享 SPI/显示屏）→ 组装 Board → main.run | 源码 `.py` | 系统按文件名找 `boot.py`，`.mpy` 不认 |
| `lib/board.py` | **板级包装**：Board 句柄（spi/display_cs/display/int1/acc/wlan）+ 共享 SPI 借还区 acc_bus/互斥锁 + **NVS 记忆函数**（nvs_get/set_i32、nvs_get/set_str、nvs_erase；ns `ESP32C3TFT`） | 编译 `.mpy` | 框架层，与应用无关；NVS 统一入口，业务模块不再各自握 NVS 细节 |
| 根 `main.py` | **应用入口**：读配置 → usageapp | 源码 `.py` | 入口链 |
| 根 `wlan_cfg.py` `usage_cfg.py` | 用户自定义点 | 源码模板 | 复制后填写 |
| `lib/` | 共享 helper（board/ADXL345/uftpd/字体/wlanman/utils） | 编译 `.mpy` | 用户不改 |
| `usageapp/` | 用量应用 | 编译 `.mpy` | 行为参数全外置到 `usage_cfg.py` |
| `tools/` | 诊断小工具 + **冒烟测试** | 不进 dist | 宿主运行 |

**分界原则**：框架（board）与应用（main+usageapp）互不感知，仅通过
`board.Board` 句柄（spi/display_cs/display/int1/acc/wlan）衔接；
helper / usageapp 编译分发；用户可改的只有两个配置文件。

热链路：`boot.py`（初始化全部硬件 → `board.Board(...)`）→ `main.run(board)`
（读 wlan_cfg/usage_cfg）→ `usageapp.app.run(board, networks, cfg)`
（常亮循环，不返回）。

---

## 3. 命令速查

```bash
./gendist.sh                      # 编译 lib/ usageapp/ → dist/，组装可烧录目录
python3 tools/smoke_usageapp.py   # 端侧纯逻辑冒烟测试（宿主跑，stub st7789/ujson）
mpy-cross -o dst.mpy src.py       # 单文件交叉编译
```

- 修改 `usageapp/` 或 `lib/` 后必须重跑 `gendist.sh`，否则真机仍跑旧 `.mpy`。
- 冒烟测试是**逻辑正确性的第一道门**；改 page/client/render 后先跑它。
- 服务端联调：`USAGE_SERVER_AUTH_KEY=testkey python3 main.py -c config.yaml`
  端侧 `usage_cfg.url` 填 base（如 `http://192.168.1.10:8765`——注意用局域网 IP，不是回环）。
  服务端 config `demo: true` = 不实抓上游、合成假数据 + 注入模拟延迟
  （upstream_ms 进 timing.upstream；network_ms 进 wait；ssr_ms 在
  render 端点发图前 sleep 模拟大图下载——SSR 分段仍由端侧实测，
  服务端不回传该值），用于本地/无凭据调端侧界面；端侧无假数据开关。

---

## 4. 硬件约束（不可违背）

- **无 NTP / 无 RTC**：`time.time()` 开机≈0，只能靠响应的 `server_time` 校准偏移。
- **无中文字库**：`vga2_8x16` 只画 ASCII，中文/全名/数字必须由服务端渲染成 PNG 贴图。
- **内存紧张**：`display.png` 解码需 ~44KB 连续堆块；开机裸堆 maxblock 仅 ~61KB，
  fetch 后 JSON 树会进一步切碎堆。**不预检**，直接 try/except 退避（见 §7 坑 1）。
- 显示与加速度计 **共享 SPI 总线**：display CS=10 @40MHz mode0 / acc CS=8 @5MHz mode3。
  运行期借用/归还/互斥全部在 `board.py`：acc 只能在
  `with board.acc_bus():` 里读写（进出自动切 5MHz mode3/还原显示参数），
  借还区外 acc.spi 挂只 raise 的哨兵对象、重复借用抛 `BusConflictError`
  ——「一个设备使用期间另一设备不被使用」的假设被打破立即暴露。
  约束：借还区块内不要画屏（display 是冻结 C 模块直握总线，board
  拦不住；单线程主循环天然满足）。
- 双按键：`boot`=pin9（上键翻页）、`func`=pin21（下键强刷），中断只置标志。
- 中断回调（`schedule` 上下文）**只能置标志**，主循环消费（`_Flags`）。

---

## 5. 契约要点（协议 = 单端点 /api/page，已取代三端点）

端侧「看哪页拉哪页」：只按页码拉，响应 `type` 决定解析分发：

| 端点 | 作用 | 认证 | 渲染参数 |
|---|---|---|---|
| `GET /api/page?page=N` | 当前页数据：`type`(overview/provider) + `total`/`page` + 页内容 | Bearer | h/bg/theme |
| `GET /api/render/<hash>` | 贴图本体 PNG | Bearer | —（可带 ?w= 只读校验） |
| `GET /healthz` | 存活 | 无 | — |

- **页表在服务端**：`page=0` = overview 页（仅多 provider 时存在），
  其余 = providers 配置顺序逐页；`total` = 页数。**越界回卷**（服务端
  `page % total` 兜底），端侧本地 `(n+1) % total` 翻页——**换页永不 404**。
  `total=0`（无 provider）→ 带 error 的空 overview 结构，端侧按
  `total<=0` 进错误页。端侧不建表、不校验序列（type/id 自适应）。
- 请求参数 `client`/`page`/`h`/`bg`/`theme`/`fg`/`caption`/`peak`/`offpeak` 全
  可选；非法 400。**前景色覆盖（fg/caption/peak/offpeak）由端侧
  palette 随请求传出**——SSR 颜色全部走 palette（服务端按请求色渲染，
  缺省用其内置 _THEME_FG/THEME_SEMANTIC；色入像素即入 hash）。
- **新鲜度归客户端 + 服务端无状态（2026-09-08 定稿）**：服务器无抓取
  缓存、**无 last-known-good**，每个数据请求实时打上游；轮询周期/换页/
  强刷节奏全由端侧定，协议无 refresh 参数。**历史快照在端侧**——服务端
  随时可重启，端侧数据不丢。上游失败：provider 页返回 **503** +
  `{"error": {"code": ...}}`（端侧按非 200 走统一失败路径）；overview
  聚合页行级 `items[].status=error`（红 `err`）、页面级恒 ok + 下发
  `updated_at`（聚合时刻，端侧 degrade_stale 时 stale 徽标带年龄）。
- `client=1` 声明端侧能力；**响应无 version 字段**（端侧声明啥收啥）。
- 认证走 `Authorization: Bearer <key>` 头，**key 不落 URL/日志**。
- `server_time` 是 unix **秒**，数据端点都回，任一端点都能校准时钟。
- `timing`={wait,upstream,serve}ms 搭车字段，数据端点都回；端侧据此
  把时间条 fetch 段重涂三色（upstream 直接用；wait 退居遥测）；必须容忍缺失（旧版可无）。
  详见 llm-usage-server/specs/003（demo）与 ESP32C3TFT/specs/002（时间条三态）spec。
- `?w=<px>`：贴图请求槽宽只读校验——超宽服务端照发 200 仅控制台警告；
  不参与 hash、不参与响应。
- **字段语义**：`percent` 必有；其余数值可选；消费端**必须忽略未知字段**。
- 时间一律 unix 秒；`error.message` 只进日志，端侧**不得渲染**。
- **错误统一 ASCII（2026-09-02 定稿）**：`error.render` 已停发（错误是
  动态非预期的，不入 SSR 不占 rcache）；任何错误都是端侧 ASCII 单行
  （可截断）——`error.code` 优先、无则 FetchError 文本。请求出错时
  时间条 wait 段改涂错误红作 UI 提示（主请求失败 + SSR 下载失败都涂）。
- **错误模型（2026-09-05 定稿 + 2026-09-08 无状态化修订）**：状态徽标
  只有两态 ok/stale（error 徽标删除——`STATUS_TEXT` 只剩 ok、
  `_status_rgb` 缺省灰）。失败按 `loaded`（首次成功加载标志）分流：
  **从未成功** → 独立错误页（`retry Ns` 倒计时 + **连续失败计数
  `fail #N`**，单调递增证明循环活着在重试——同内容重试屏显无变化
  看不出；成功归零）；**成功过** → `degrade_stale`：全部页（含总览）
  标 stale 保留旧数据 + 页头下 ERR_Y ASCII 单行错误，当前页保持显示
  不跳页。**服务端无状态化后 provider 页失败一律 503**（不再 200 +
  status=error）：端侧非 200 → 解析 `error.code` 机器码作 FetchError
  文本（client.py）→ 走上述统一失败路径——stale 徽标（用端侧上次
  成功响应存的 `updated_at` 带年龄）+ ERR_Y 单行 + 时间条 wait 段
  错误红**同步出现、同步消失**（恢复 200 → update 换 ok → sig 变 →
  全量重画涂掉全部错误痕迹）。200 + `status=error` 仅剩 total=0 空
  overview 防御（串口 `[usage] page status=error: <code>` 留痕）。
  `do_fetch` 有 `except Exception` 兑底（`sys.print_exception` 串口
  traceback）——意外异常不再穿透杀死主循环。任何失败串口 `[usage]`
  前缀打印详情（fetch fail #N / ssr 下载失败 / ssr miss / png oom /
  wifi failed / recovered）。错误单行与 RSSI 角标**同行分置**
  （ERR_Y=22：错误 x4..156 限 19 字符 / 角标 x164..228，左右不叠不互擦
  ——曾错位叠字，2026-09-06 修）；RSSI 写屏统一 `draw_rssi`，页 full
  重画后**立即原地补画**（刷新期不再缺位秒级）；**fetch/SSR 期间由
  `tl_frame` 帧回调接手**采集+重画（主循环阻塞在 socket/PNG 期间唯一
  执行点就是 on_wait 切片回调——此前 RSSI 只在主循环采，fetching
  数秒~十几秒角标冻结，2026-09-07 修）。
- **总览行级失败（2026-09-05 + 2026-09-08 + 2026-09-09 定稿）**：
  行级状态**只两态 ok/error**——stale 永远是整页级的（服务端无状态化
  后行级不产 stale，端侧收到也归一 ok 防御、不屏蔽数据）。
  `items[].status=error` 的行（服务端对无数据 plan/bundle 下发
  `percent=0.0` 兑底）端侧屏蔽为无数据——不画条/"0%"/倒计时，
  percent 槽（OV_PCT_X）画红 `err`。**overview 页面级恒 ok**（单
  provider 失败只标该行，不整页降级）；OverviewPage 页面级 stale 徽标
  + ERR_Y 单行错误只由端侧 `degrade_stale`（整页网络失败/503）触发
  （行渲染靠行级 status 回退页面级保留旧值），且因服务端下发
  `updated_at` 带年龄（2026-09-08）。

### 渲染责任边界（关键划分）

- **服务端 SSR（贴图）**：一切用户可见文本——标题/行名/标签/峰谷大字
  （**仅余额类**）/***错误提示除外***，`*_render` 都是 **8 位小写 hex hash**。
  **2026-09-04 定稿 + 2026-09-06 修订：非余额类（plan/bundle）峰谷不占
  大字 SSR**——密集形态 = 一行「caption 灰标签贴图（`距切换计价还剩：`）
  + 两色倒计时（随 state 红/绿）」，badge_small_render 停发。
- **端侧现场**：每秒变化的数字——峰谷倒计时（**两色：peak 红 / offpeak
  绿**）、时钟条；以及**余额大字**（金额每轮变化，贴图会每次触发新 hash
  下载）。余额大字用 TrueType 提取的**位图字体子集** `lib/digits`
  （字符集 `¥$0-9.`，生成脚本 `tools/make_digits_font.py`，宿主跑一次
  产出 `lib/digits.py`）现场画，`display.write` 直推 SPI。
  **贴图失败/未下载/超宽一律槽位置空等下轮平补刷新（用户多次定稿，
  绝不 ASCII 回退），串口 print 报错去重**（store.note_miss/note_oom；
  例外：`error.code` 机器码无 err_hash 时可画 ASCII）。
- 例外（端侧可画）：机器标识 `id`/`unit`/`error.code`。

### 贴图下载模型

- hash 按**位图内容寻址**（sha256(宽,高,RGB) 前 8 hex）：同 (文本,h,bg,theme)
  → 同 hash。端侧按 hash 做文件缓存 → 内容不变零请求。
- 下载走 `GET /api/render/<hash>`，`Content-Length` 必有；404 → 下轮拿新 hash 自愈。
- 输出为**不透明 RGB PNG**（无 alpha），端侧只贴图、不做也做不了 alpha 合成。
- `?h=` 只作用于**文本类**贴图；余额大字/峰谷大字是**插图，固定高（约 32px）**，
  排版按 PNG 自带高度，不随 `?h=`。

### 三块（balance / peak / pack）

- **balance 块**（余额型 provider，如 DeepSeek）：`quotas=[]`，只有 `balance`。
  端侧消费 `currency` + `caption_render`(灰标签) + `amount_text`(金额纯文本)；
  大字不再走 SSR：`render`/`symbol_render` 端侧忽略（未知字段容忍）。
  provider 页金额大字 = `lib/digits` 现场画（峰谷语义色随 state 红/绿/灰），
  **货币符号（¥/$）并入大字串同色同字体**（`page._currency_glyph`：
  CNY/JPY→¥、USD→$；无字形币种只画数字）——用户定稿：余额页大字 =
  彩色货币符号+数字。
  **总览余额行 = 彩色 ASCII**（用户定稿）：`CNY 110.5` 币种码+空格+金额
  vga 直画右对齐（`page._currency_code`），不用大字；色随峰谷。
- **peak 块**（分时计价）：`state`(peak/offpeak) + `ends_at`(倒计时基准)；
  `caption_render` 灰标签**两种形态都带**——余额类`现在是：` /
  非余额类`距切换计价还剩：`（2026-09-06）；`badge_render` 大字插图
  **仅余额类稀疏型**（2026-09-04 定稿：**非余额类峰谷不占大字 SSR**，
  badge_small_render 停发）。
  - **稀疏型**（quotas 空，纯 balance 页）：大字插图竖排；倒计时右置
    两色（随 state）。**密集型**（**≥1 条 quota**，2026-09-02 放宽——
    单窗 plan 如 qwen 谷价形态也走）：**一行 = caption 灰标签贴图
    （`距切换计价还剩：`）+ 两色倒计时**（peak 红 / offpeak 绿；
    `_LAYOUTS_PEAK` 有 1 档）。
  - **档位布局防重叠（2026-09-09 定稿）**：**保留条下数额原布局**（数额
    与标签同行右对齐的 inline 方案被否——SSR 贴图清除矩形整槽宽会把
    同行数字盖掉）。行高需求 = bar_dy+bar_h+2+16，step 须 ≥ 需求；不足
    只微调纵向几何（y0/step/条高/条上移），不用 inline：普通 3 行档
    (54,50) 恰好相接；峰谷 3 行档 (64,46,12,16) 条高 14→12 + 条上移
    bar_dy 18→16 换出间距（可用区 62..204 只 142px，3 行 50 放不下）；
    峰谷 2 行档 step 54→56 补足。diff 路径 inline（4 行档）擦除按文本
    实宽（旧固定区 x64..168 会擦掉标签贴图右半）。
  - **时段规则随 provider 驱动配置**（服务端 config `peak:` 段）：
    deepseek 缺省工作日 09:00–12:00、14:00–18:00；**qwen（通义灵码
    Token Plan Solo）谷价 = 每晚 22:00–次日 08:00**（北京时间 UTC+8
    恒定，**天天生效无周末差别**；peak 窗口配置 08:00–22:00 补集，
    见驱动 docstring）。
  - **倒计时格式**（2026-08-29）：`fmt_hms` ≥24h 显示 `Nd HH:MM`（如跨周末
    `1d 16:37`）——峰谷仅工作日，周六起看下一峰真实剩 40+h，纯 HH:MM:SS
    读不出天数；>99 天（时钟未同步）仍 `--:--:--`。
- **pack 资源包**：**不新增块**，复用 `quotas` 放 1 条，`reset_at` 当有效期，
  与 plan 单条走同一布局。

### 配色方案（2026-08-31 三次定稿：v3 高对比色板回写为内置默认）

**昼夜 = 两套全量配色整体切换**（不只是压背光）：day = 白底黑字、
night = 黑底白字（bg/fg 纯 `FFFFFF`/`000000` 拉满对比）。**背光亮度 2×2
保留**：`usage_cfg.brightness` 四档——`day_bright/day_dim/
night_bright/night_dim`（PWM 占空比 0-65535），`power.resolve_levels(cfg)`
合并（非法/越界回默认），`Backlight` 按 (日夜, 亮暗) 取档。
色板集中定义在 **`usageapp/theme.py` 的 `_DAY`/`_NIGHT`**（两套键位
必须对齐；键名语义：bg/fg/rows×4/warn/crit/track/ok/stale/err/dim/
caption/peak/offpeak/timeline×4——`name` 键已删：总览行名全走
SSR 贴图、无 ASCII 回退，`_NAME_RGB` 死代码一并移除）。`usage_cfg.py` 加
`palette = {'day': {...}, 'night': {...}}` 分别覆盖任意子集
（'RRGGBB' hex 或 (r,g,b)；非法值/不完整组忽略用默认；**旧单层
`palette = {...}` 兼容 = 只覆盖 day 套**，夜间用内置默认），注释掉
即用内置默认。链路：
`app.run()` 开头 `theme.apply(cfg)` → 两套全量 RGB 落模块态 + day 套
`page.set_palette()` 落模块常量（渲染处处动态读）+ 返回时间条
(四色, 底色)；夜间检测切态时 `theme.set_night(night)` → 对应套落
page + 返回新 (四色, 底色)，app 顺路 `timeline.set_palette()` 换条色
page + 返回新 (四色, 底色)，app 顺路 `timeline.set_palette()` 换条色
+ **当前页 `swap_renders(night)`+`invalidate()` 重绘**（2026-09-07
定稿：**双 hash 契约 + 切套仅 SSR**——主响应每槽双发两套 hash
（`*_render` 当前套 + `*_render_alt` 对偶套，请求带 `alt*` 渲染参数；
`render.check_alt` 拦缺 alt 的旧服务器，直接报错不静默兼容），
`update(night=)` 同轮双登记进 `_alt`，切态回填另一套后渲染读
rcache 双套共存缓存；**缺失贴图直接走 `/api/render/<hash>` 按需
补下**（时间条当前相位后追加一段 ssr 增量走字、右标注 ssr 位
继续走字完成后并入冻结），不碰 `/api/page` 不触发上游——数据刷新
仍归常规轮询。非当前页不管——翻到必先 fetch 拿新夜态 hash 覆盖。
未成功加载过（错误页/无页）无数据无贴图仅换色，错误页
`invalidate()` 整屏 fill 换夜底）。
**SSR 参数随夜态**：client.py `bg`/`theme` 从
`theme.current_bg_hex(night)`/`current_theme(night)` 动态取
（day `FFFFFF&theme=day` / night `000000&theme=night`）——贴图与页底
同色合成、hash 含 theme，切换后贴图 hash 换套自动重下，端侧 rcache/
与服务端缓存双套共存（切换零计算）。**前景色覆盖**：client.py 还随
请求带 `&fg=&caption=&peak=&offpeak=`（`theme.ssr_colors(night)` 出
hex），服务端按请求色渲染——色板自定义不再受服务端内置色限制。
服务端缺省色权威：`usage/render.py _THEME_FG`（标题/标签灰阶）+
`THEME_SEMANTIC`（峰谷红/绿按 theme 适配；仅当请求未带色覆盖时生效）。
day 色板值（v3 定稿，theme.py/page.py 默认）：行色 `0E6FC4`/`00897B`/
`6C3FC2`/`5F6670`，阈值琥珀 `(200,120,0)`、红 `(200,0,0)`，轨道
`(236,236,239)`，caption 灰 `(85,90,97)`；峰谷红 `(200,0,0)`/绿
`(0,140,0)`；时间条四色 (wait `DADDE2`, network `008C00`, up
`C87800`, ssr `0E6FC4`)。night 套 bg/fg 纯黑白，timeline
(`20242C`,`3DDC84`,`FFB340`,`42A5F5`)。峰谷大字文案：
peak 红`梁文峰`、offpeak 绿`梁文谷`（服务端 SSR，色随 theme）。
**夜态默认夜间（无 NVS 记忆）**：设备无 RTC，断电丢态——启动夜态
恒为夜间默认 True；时钟校准前不按小时判（同夜态）。夜间检测切态时
不再回写。
**记住页（NVS，2026-09-01/02/09）**：同页连续手动刷新（中途无调暗、无换页）
达 `refresh_remember_n`（默认 3，`usage_cfg` 可配）次 → 记住当前页
（**页码 i32**，key `usage_page`，存取统一走
`lib/board.py` 的 NVS 函数——**必须 i32：真机实测本固件 NVS.set_str
写入后 get_str 读回 None，str 从没落盘，功能形同虚设**，2026-09-09
Thonny REPL 验证 `set_str→get_str=None`、`set_i32→get_i32=1`，故改存
i32；i32 语义上也更贴合页码），下次开机 `restore_page`
直接拉该页（fetch_page(序号)，服务端越界自动回卷）。记忆是**持久书签**：
成功加载保留、每次开机都恢复；**清除时机**=开机加载失败（`do_fetch`
失败分支）。连续计数在换页/进入调暗时清零（不清除已记书签）；计数
只针对"非 dim、无换页"的手动刷新成功拉取（do_fetch 成功尾部结算，
`manual_pending` 标记本帧手动刷新意图）。**fetch 期间到达的刷新键不清不吞**（2026-09-07
修：曾按"重叠去重"在 do_fetch 成功尾部无条件清 flags.force_refresh，
阻塞数秒的 fetch 窗口内按键被静默吞、计数永远凑不满 N，记住页功能
形同虚设——坑 3"标志只有一份/读到 flags 再落局部"的违例；现保留
标志到下一帧输入段正常消费，刷新+计数两不误）。

### overview 行名（易踩坑）

`overview.items[].render` 用的是 **`display_name` 专属短名**，与 provider 详情页
`title_render`（全名）是**不同文本、不同 hash**。端侧切不可把两者当「同图」复用
缓存——各按 hash 下载即可（端侧当前实现天然就是各自独立下载，无跨端点复用逻辑）。

---

## 6. usageapp 模块职责

| 模块 | 职责 |
|---|---|
| `client.py` | 手写 socket 切片读 HTTP（替代 urequests，为驱动时间条动画）；`fetch_page(N)` 单端点；请求随夜态带双套渲染参数（`bg/theme/fg…` + `alt*` 对偶套，双 hash 契约）；`server_base()` 推导 base |
| `render.py` | `RenderStore`：hash→文件缓存、`png_dims`（宽+高）、`usable()`/`height()`、下载、上限淘汰、OOM 退避（error 不入 SSR）；`check_alt()` 双 hash 契约校验（缺 alt 即 FetchError） |
| `page.py` | `ProviderPage`（配额 1..4 档 + balance/peak 稀疏/密集布局）、`OverviewPage`（含页面级 stale/错误行 + 行级 error 画红 err）、`ErrorPage`（单行 ASCII err + retrying... 点闪动 + fail #N 连续失败计数）；徽标两态 ok/stale；余额大字 = `lib/digits` 位图字体现场画（¥/$ 符号并入大字串同色）；总览余额行彩色 ASCII（币种码+金额）；Δ 显示（内存态）——`update(dark=)` 暗屏轮增量并入挂起 Δ（+2,+0.1,+0.5 → +3.6）不推进基准，退出暗屏强刷一次结算；`update(night=)` 同轮双登记双套 hash 进 `_alt` 记忆库，`swap_renders(night)` 昼夜切槽回填另一套 + `render_slots()` 枚举槽位（含槽宽，供切套按需补图） |
| `app.py` | 主状态机：WiFi、调度、页码游标翻页/换页 pending、夜景（启动态=默认夜间，无 NVS 记忆；切态=当前页 `swap_renders`+按需补图，不碰数据端点，见 §5）、RSSI、SSR 队列；按响应 `type` 分发解析（overview/provider）；失败按 `loaded` 分流（未加载→错误页 / 已加载→`degrade_stale` 全页 stale）+ 连续失败计数 + `[usage]` 串口诊断 + `except Exception` 兑底 |
| `timeline.py` | 底部时间条（相位型，满条=轮询周期；分段三态：主请求中整段 network 色增长，响应到达重排 network+upstream（upstream=服务端 timing，network=端侧实测主请求段−upstream 钳0——wait 不含传输不能直接用）+ssr 实时增长（端侧实测），**ssr 冻结后剩余段重涂 wait 色（等待下一轮，条底色）；请求出错 wait 段改涂错误红**；右标注恒三段带' s'尾同态流转（进行中=实时+0/0，下载中=前两段冻结+ssr 走字，完成全冻结），>10s 段自动降整秒防溢出；**失败也保持三段**（主请求段实测时长进 network、后两段 0.0——2026-08-31 定稿不再回退单段灰；仅总耗时 0 才灰单段）；左标注 fetching... 点闪动，**'retrying' 仅 SSR（次要 fetch）失败过才用**——sticky 置位、本轮内不翻回（render.download 任一尝试失败即 on_retry(True)，复位只在本轮完成 mark_fetch_done/下轮 begin_cycle；主请求失败后的延时重发仍显示 fetching——2026-09-06 定稿）；**标注行增量重绘**（2026-09-06 定稿：左=固定前缀不动只擦动态尾（动画点最多 3 字符），右=布局不变时只重画文本变化的段、斜杠与 ' s' 尾不动——整行擦写每次走字都闪，真机肉眼可见；布局变（阶段切换）才整块擦写一次）；**切套补下载追加段**（2026-09-07：`begin_extra()/extra_done()`——冻结 wait 相位中途追加一段 ssr 增量走字，wait 起点后移/从段尾续铺，相位锚点不动；右标注 ssr 位=冻结值+追加实时，完成后并入重新冻结） |
| （已移除）demo.py | 端侧假数据源已删除——demo 改在服务端（`llm-usage-server/usage/demo.py`：按 providers[] 合成 + 延迟注入） |
| `activity.py` / `power.py` | 加速度计唤醒（INT1 中断 + 引脚电平哨兵）/ 背光调暗 |

**加速度计唤醒模型**：INT1 上升沿 → 硬件 ISR 只 `micropython.schedule` 排队
（handler/worker 预存绑定方法，ISR 内零分配）→ scheduled worker 只置
`monitor.flag` → 主循环消费时才进 `board.acc_bus()` 借还区读 INT_SOURCE（区分 activity/
inactivity 并清锁存）。锁存滞留由**引脚电平哨兵**兑底：每 tick 看
`flags.acc_wake or int1.value()`，见线高才借总线——死锁滞留 ≤1 tick，
无定时器，静置时总线借用次数为 0（暗屏期 TIME_INACT 到点的 inactivity
也走这条路被读清）。ADXL345 中断是锁存式，INT1 保持高直到被读清 →
并发天然上限 1，无事件风暴。`activity_poll` 配置项已退役删除。
**换页语义（重要）**：上键不立即切页——先把目标页码挂 `pending_page`、
`force_refresh=True`，fetch 期间**当前页保持原样显示 + 时间条走灰段**，
数据拿到后才真正切 `cursor` + `invalidate()`。**stale 只在请求真正失败时标**
（`set_net_error` 路径），换页不标 stale。

**页面序列由响应 page/total/type 驱动（2026-09-02）**：端侧只维护页码
游标 + 定长 `pages` 占位（None=未拉过）；`total` 变化时 `_resize` 对齐；
错误页是独立兜底态（`ERR_CURSOR=-1`，不进 pages）；provider 页对象
按 `pid` 复用（保留增量渲染状态）；总览页单例仅在服务端 `type=overview`
时存在。

---

## 7. 踩过的坑（换会话必读）

1. **png OOM（44KB 连续块）**：根因是 pngle 内联 tinfl(11KB)+lz_buf(32KB)≈44KB，
   每次从 GC 堆整块 malloc，碎片化下拿不到。固件侧修复（pngle_static 静态实例进 BSS，
   见 `st7789_mpy` 仓库）。端侧不预检（`mem_free` 测总量判错量），直接 try/except，
   MemoryError → `store.note_oom(h)` 本周期退避，下轮 `missing()` 重试。

2. **时间恒"8点" + 倒计时几十万小时**：ESP32 无 RTC，`time.time()`≈0。旧代码
   `clock_offset = server_time - time.time()` 在响应缺 `server_time`（=0）时把偏移打成 0。
   **修复**：`_sync_clock()` 只在 `server_time > 1e9`（真实 unix 秒）时校准；`fmt_hms`
   加 99 小时上界，超界返回 `--:--:--`。

3. **手动刷新键失效**：中断回调设了 `flags.force_refresh`，主循环读的是另一个
   局部 `force_refresh`（两个独立变量失联）。**原则**：标志只有一份，读到 falls 再落局部。

4. **换页后时间条/标注消失**：换页全量重画 `fill(BLACK)` 抹掉时间条，但 timeline
   增量渲染不知道被抹 → 必须 `timeline.invalidate()` 通知全量重画。
   **约定（2026-08 定稿）**：失效通知不走各路径零散特判，统一两个咽喉点——
   ① 页面全量渲染返回 True 时在 `_paint_current()` 里顺带失效时间条与 RSSI；
   ② `do_fetch` 成功尾部按 `broken(上轮失败) or dirty(重连日志写过屏)` 统一
   整页重画。新增任何"整屏 fill"的代码路径都必须经由这两个咽喉点之一。

5. **activity 唤醒失效**：`set_int_enable(inactivity=True)` 里 `activity` 默认 False，
   抖动唤醒位没置位。修复为 `set_int_enable(activity=True, inactivity=True)`。

6. **闭包变量 `nonlocal` 漏声明**：`do_fetch` 里 `overview_page = OverviewPage()`
   重绑外层变量但没加 `nonlocal` → `NameError: local variable referenced before assignment`。
   任何在闭包里「重新赋值」的外层变量都必须进 `nonlocal` 声明。

7. **alpha 二值化显示怪**：固件无 alpha 合成（0 跳过 / >0 全画），必须服务端与 bg
   合成后输出**不透明 RGB PNG**，端侧不处理透明。

8. **key 进 URL/日志**：历史上 `?key=` 明文过网且会进访问日志，已改为
   `Authorization: Bearer` 头。

9. **中断里不能分配/不能阻塞**：ClickDetector 的 handler 里只 `flags.xxx = True`。

10. **`st7789` 是固件冻结模块**：`lib/board.py` 初始化背光引脚方式稳妥，Backlight 用 PWM
    接管同一引脚。display 构造需传 reset/dc/backlight 引脚 + `rotation=2`。

11. **st7789 `write()` 位图字体格式（2026-08-28 乱码事故）**：三个硬约束，错一个
    必乱码——① `OFFSETS` 是 **bit 偏移**不是字节偏移；② 每字符位流 =
    `width × HEIGHT` **连续位，行间无字节对齐**（固件 `get_color` 逐位
    `bs_bit++`，任何行补位都会让非 8 倍宽字形花掉）；③ 所有字形共享同一
    HEIGHT 画布与基线（墨迹底统一对齐画布底，顶对齐会把 "." 顶到字格顶）。
    生成器 `tools/make_digits_font.py` 的自检 probe 就是按固件读取方式模拟的，
    改格式先看它；宿主可视校验：按固件路径读位打 `#/.` 网格（见该脚本注释）。

12. **MicroPython 没有 str.isascii/isalnum**（2026-08-29 真机
    AttributeError）：宿主 CPython 跑冒烟测不出——smoke 的 stub 只是
    垫模块不含 CPython 行为差异。字符集判断一律逐字符 `ord()` 范围判。
    凡在宿主写「字符串方法」新代码，先想一下 MicroPython 有没有。

13. **`micropython.schedule` 队列仅 8 深，满队抛 RuntimeError**（2026-08-29
    真机 `lib/utils.py _irq_handler`）。主循环阻塞在 C 调用（PNG 解码 /
    socket 收包）期间 scheduled 回调不排空；按键 IRQ 每个边沿（含抖动沿）
    都入队，消抖又在队列之后才生效——一次按键的抖动即可灌满队列。
    原则：**ISR 里裸 `schedule()` 必须 try/except RuntimeError**（丢弃该
    边沿 + 状态机侧去重兜底）。`ClickDetector` 修法：抖动沿入队失败仅
    丢弃（首沿已排队，消抖保证同一次按键只结算一次）；软定时器回调里
    `schedule` 失败则**就地直调**回调兜底。`ActivityMonitor._irq` 同款
    try/except 早已有。

---

## 8. 约定（新代码遵守）

- 所有用户可见文本走服务端 SSR 贴图，端侧字段只作 ASCII fallback。
- 数值一律原始值，端侧不做单位换算/汇率换算（标签文案由服务端 `label_render` 决定）。
- 端侧不做 schema 校验：运行时异常 → 整响应视为无效，走不可达/重试路径。
- 增量渲染：布局签名不变走 diff，只有变化元素重画；签名变才全量重画。
- `usage_cfg.py` / `wlan_cfg.py` 模板在根目录 `*.template`；`gendist.sh` 自动去后缀复制。
- 主循环帧节奏 = 计时器结算式动态 sleep：每帧记起点，补睡
  `max(target_frame_ms−已用时, 1)`（rlvplayer 固件同款）；
  `target_frame_ms` 在 `usage_cfg` 可配置。
- 改完跑冒烟 + `gendist.sh`，再真机 `mpremote cp -r dist/* :`。