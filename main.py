"""应用入口：AI 用量显示（usageapp，常亮循环，不返回）。

启动链：boot.py（入口）→ board.init()（硬件框架）→ main.run（应用）。
应用配置：usage_cfg.py（行为/配色/夜间）+ wlan_cfg.py（AP 列表），
模板见 *.template，复制后填写。
"""


def run(board):
    import wlan_cfg
    import usage_cfg
    from usageapp.app import run
    run(board, wlan_cfg.networks, usage_cfg)
