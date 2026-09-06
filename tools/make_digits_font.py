#!/usr/bin/env python3
"""生成 lib/digits.py —— 端侧位图大数字字体模块（余额大字用）。

用 PIL 光栅化 NotoSans-Regular 指定字号，对每个字符产出 1bpp 位图
（行对齐字节），转成 st7789 固件 write() 可消费的 font_module 形态
（MAP/WIDTHS/OFFSETS/BITMAPS，单字符 bitmap 整块推送，无双倍宽度
缓冲问题）。只收余额文本实际用到的字符，控制模块体积。

宿主运行：python3 tools/make_digits_font.py
输出：lib/digits.py（.mpy 编译后 flash，零 RAM 常驻、零运行时解码）
"""
import os
import sys

from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
OUT = os.path.join(ROOT, 'lib', 'digits.py')

FONT = '/Users/bcw/Documents/Personal/Projects/ESP32C3TFT/st7789_mpy/fonts/truetype/NotoSans-Regular.ttf'
SIZE = 32                       # TrueType 请求字号（下采样逼近墨迹高）
TARGET_INK_H = 24               # 余额大字目标墨迹高 px（用户定稿：24px）
# 二值化阈值：>0 会把抗锯齿灰边全部变实心（视觉加粗 1px）。
# 调高→更细（过高笔画断裂），调低→更粗；128=常规粗细。
THRESHOLD = 180
CHARS = '¥$0123456789.'         # 货币符号 + 金额数字 + 小数点（符号与大字
                                # 同字体：整串金额一次 write 画出）


def render_at(font, ch):
    """在固定大画布上渲染单字，返回 (画布, 墨迹 bbox)。无墨迹 bbox=None。"""
    img = Image.new('L', (SIZE * 3, SIZE + 8), 0)
    ImageDraw.Draw(img).text((2, 2), ch, font=font, fill=255)
    return img, img.getbbox()


def main():
    # MAP 需按 UTF-8 序列化进 .py 源码（文件头有 coding 声明）；
    # 原先的 ASCII 过滤（32<=ord<127）会把 ¥ 滤掉，改为保留全部声明字符
    chars = list(CHARS)

    # 统一字号：取「全体字符墨迹高都 ≤ TARGET_INK_H」的最大字号。
    # 所有字形必须同字号（共享基线）——逐字降字号会让 ¥ 与数字基线错位。
    size = SIZE
    font = None
    boxes = None
    while size > 8:
        font = ImageFont.truetype(FONT, size)
        boxes = [render_at(font, ch)[1] for ch in chars]
        if all(b is not None and b[3] - b[1] <= TARGET_INK_H
               for b in boxes):
            break
        size -= 1
    for ch, b in zip(chars, boxes):
        if b is None:
            sys.exit('glyph missing: %r' % ch)

    # HEIGHT = 最大墨迹高；各字形墨迹底（基线）统一对齐画布底
    glyphs = [(ch, render_at(font, ch)) for ch in chars]
    height = max(b[3] - b[1] for _, (_, b) in glyphs)
    ink_bottom = max(b[3] for _, (_, b) in glyphs)

    map_s = ''.join(chars)
    widths = []
    bitmaps = []
    offsets = []                   # bit 偏移（固件 OFFSETS 语义，非字节！）
    bit_pos = 0
    for ch, (canvas, b) in glyphs:
        w = b[2] - b[0]
        h = b[3] - b[1]
        total_bits = w * height
        data = bytearray((total_bits + 7) // 8)
        px = canvas.load()
        # 墨迹底=基线（本字符集 ¥/$/数字/. 全部止于基线，无降部）
        y_off = height - (ink_bottom - b[1])
        bit = 0
        # 固件 get_color 按位连续读（bs_bit++），行间**无字节对齐**——
        # 任何行补位都会让非 8 倍宽字形漂移花掉（首次真机乱码根因之一）
        for y in range(height):
            sy = y - y_off
            for x in range(w):
                if 0 <= sy < h and px[b[0] + x, b[1] + sy] >= THRESHOLD:
                    data[bit >> 3] |= 0x80 >> (bit & 7)
                bit += 1
        widths.append(w)
        bitmaps.append(bytes(data))
        offsets.append(bit_pos)
        bit_pos += total_bits

    max_w = max(widths)

    def bstr(bs):
        return "b'" + "".join('\\x%02x' % b for b in bs) + "'"

    lines = []
    a = lines.append
    a('# -*- coding: utf-8 -*-')
    a('# 端侧位图大字字体（余额大字用，含货币符号 ¥/$）。')
    a('# 生成脚本 tools/make_digits_font.py；单字符 1bpp 位图，')
    a('# st7789.write() 直接推 SPI：零 PNG、零 RAM 常驻。')
    a('# 字体 NotoSans-Regular %dpx（墨迹高 %dpx）；字符集 %r'
      % (size, height, CHARS))
    a('')
    a('MAP = %r' % map_s)
    a('BPP = 1')
    a('HEIGHT = %d' % height)
    a('MAX_WIDTH = %d' % max_w)
    a('OFFSET_WIDTH = 3')
    a('_WIDTHS = %s' % bstr(bytes(widths)))
    offs = bytearray()
    for o in offsets:
        offs.extend(o.to_bytes(3, 'big'))
    a('_OFFSETS = %s' % bstr(offs))
    a('_BITMAPS = %s' % bstr(b''.join(bitmaps)))
    a('WIDTHS = memoryview(_WIDTHS)')
    a('OFFSETS = memoryview(_OFFSETS)')
    a('BITMAPS = memoryview(_BITMAPS)')

    with open(OUT, 'w', encoding='utf-8') as f:
        f.write('\n'.join(lines) + '\n')

    # 自检：完整模拟固件 write() 读取方式——按 OFFSETS 的 bit 偏移、
    # 每字符 width×HEIGHT 连续位（行间无对齐）逐位读，全部字符可读出
    allbits = b''.join(bitmaps)

    def probe(ch_idx):
        w = widths[ch_idx]
        bs = offsets[ch_idx]
        on = 0
        for _yy in range(height):
            for _xx in range(w):
                on += (allbits[bs >> 3] >> (7 - (bs & 7))) & 1
                bs += 1
        return on

    assert all(probe(i) > 0 for i in range(len(chars))), 'bit stream broken'

    print('wrote', OUT, 'font=%dpx height=%d max_w=%d bytes=%d'
          % (size, height, max_w, len(allbits)))


if __name__ == '__main__':
    main()
