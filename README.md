# ESP32C3TFT

电池供电的 ESP32-C3 + 240x240 ST7789 徽章设备。本项目在其上跑
**AI 用量显示**应用（`usageapp/`），从自建的 `llm-usage-server`
拉取 GLM Coding Plan 等订阅的用量并常亮显示。

架构分两层：**boot.py**（固件入口 + 全部设备初始化：无线电/ADXL345/
共享 SPI/显示屏）→ **board.py**（板级包装：Board 句柄 + 共享 SPI
借还区/互斥锁）→ **main.py**（应用入口）→ `usageapp/`（应用逻辑）。
硬件与应用互不感知，仅通过 `board.Board` 句柄衔接。

> **AI 项目声明**：本分支为**纯 AI 项目**——分支主要由 AI（LLM）生成，人类仅负责需求描述与验收，对代码的可靠性不做保证。

## 目录布局

| 位置 | 内容 | 分发形态 | 理由 |
|---|---|---|---|
| 根目录 | `boot.py` `main.py` `board.py` | 源码 | 入口链与硬件框架；系统按文件名找 `boot.py`，`.mpy` 不被认 |
| 根目录 | `wlan_cfg.py` `usage_cfg.py` | 源码模板 | **用户自定义点**，复制后填写 |
| `lib/` | 共享 helper（ADXL345/uftpd/字体/wlanman/utils） | 编译 `.mpy` | 用户不改的代码：体积小、导入省 RAM |
| `usageapp/` | 用量显示应用 | 编译 `.mpy` | 同上；行为参数全部外置到 `usage_cfg.py` |
| `tools/` | 设备上运行的小工具（free/mpy） | 按需手动传 | 不进 dist |
| `usage-server/` | 端侧配套服务器（跑在电脑/VPS） | 不烧录 | 见其 README |

编译/源码的分界原则：**helper 编译、自定义点留源码**。给用户暴露的
可改项只有两个配置文件，其余一律编译分发。

## 用户可自定义项

`wlan_cfg.py`——WiFi 网络（多 AP 按序回退）：

```python
networks = {'ssid': 'password', 'backup': 'password2'}
```

`usage_cfg.py`——用量应用全部行为（服务器地址与口令/轮询周期/
调暗策略/昼夜配色，字段见模板注释；界面调试把服务端 config 设
demo: true 即可，端侧无需假数据开关）。

## 构建与烧录

```bash
./gendist.sh                        # 编译 lib/ usageapp/ 并组装 dist/
cp wlan_cfg.py usage_cfg.py dist/   # 可选：本地真实配置覆盖后再烧
mpremote cp -r dist/* :             # 全量烧录
```

## 契约与版本

契约以 `llm-usage-server/SCHEMA.md` 为唯一事实源（现行版本）；已发布版本
全文冻结在 `schema/vN.md`，git tag `schema-vN` 为锚点（`schema-v1` = v1 定稿）。
版本演进规则：**演进即 +1**、**只归档不删不改**、**服务端必须向后兼容所有
已发布版本**（升级服务端不得使本端侧失效）。

服务端仓库：`git.team.silveridge.cn:3443/groundsquare/llm-usage-server`，
本仓库与它以 `schema-vN` tag 互为版本锚点。

## 运行形态

开机即进用量应用——连 WiFi、轮询 usage-server、数据页 + 时间条
常亮循环；无操作调暗背光，拿起来（ADXL345 activity）或按键唤醒。
（出厂的图片浏览器已移除；如需找回见 git 历史 main.py/launcher.py。）

### TODO
selfcheck