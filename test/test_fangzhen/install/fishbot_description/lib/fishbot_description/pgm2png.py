#!/usr/bin/env python3
"""PGM(P5/P2) → PNG 转换，方便在 VS Code 里直接预览地图。

用法：
    python3 scripts/pgm2png.py rm_field.pgm            # 生成 rm_field.png
    python3 scripts/pgm2png.py rm_field.pgm out.png
只依赖 numpy，不需要 Pillow / ImageMagick。
"""
import struct
import sys
import zlib

import numpy as np


def read_pgm(path):
    """读取 PGM，返回 (h, w) 的 uint8 数组"""
    with open(path, 'rb') as f:
        data = f.read()

    # ---- 解析头：magic W H maxval（跳过注释与空白）----
    tokens = []
    i = 0
    while len(tokens) < 4 and i < len(data):
        ch = data[i:i + 1]
        if ch.isspace():
            i += 1
            continue
        if ch == b'#':                      # 注释行
            while i < len(data) and data[i:i + 1] != b'\n':
                i += 1
            continue
        j = i
        while j < len(data) and not data[j:j + 1].isspace():
            j += 1
        tokens.append(data[i:j])
        i = j

    if len(tokens) < 4:
        raise ValueError('PGM 头解析失败')
    magic = tokens[0]
    width, height, maxval = int(tokens[1]), int(tokens[2]), int(tokens[3])
    i += 1                                   # 跳过 maxval 后的那个空白字符

    if magic == b'P5':
        arr = np.frombuffer(data[i:i + width * height], dtype=np.uint8)
    elif magic == b'P2':
        arr = np.array(data[i:].split(), dtype=np.uint8)
    else:
        raise ValueError(f'不支持的 PGM 类型: {magic}')
    return arr[:width * height].reshape(height, width)


def write_png(path, arr):
    """把灰度数组写成 8bit 灰度 PNG"""
    h, w = arr.shape
    raw = b''.join(b'\x00' + arr[y].tobytes() for y in range(h))

    def chunk(tag, payload):
        return (struct.pack('>I', len(payload)) + tag + payload +
                struct.pack('>I', zlib.crc32(tag + payload) & 0xffffffff))

    png = b'\x89PNG\r\n\x1a\n'
    png += chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 0, 0, 0, 0))
    png += chunk(b'IDAT', zlib.compress(raw, 6))
    png += chunk(b'IEND', b'')
    with open(path, 'wb') as f:
        f.write(png)


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 1
    src = sys.argv[1]
    dst = sys.argv[2] if len(sys.argv) > 2 else src.rsplit('.', 1)[0] + '.png'
    arr = read_pgm(src)
    write_png(dst, arr)
    dark = int((arr < 100).sum())
    print(f'{src} -> {dst}')
    print(f'  尺寸 {arr.shape[1]} x {arr.shape[0]} 像素')
    print(f'  深色像素(障碍)占比 {100.0 * dark / arr.size:.1f}%')
    return 0


if __name__ == '__main__':
    sys.exit(main())
