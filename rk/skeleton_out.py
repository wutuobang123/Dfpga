"""skeleton_out.py - 火柴人覆盖层：渲染 1bit 掩码 + 打包 UDP 回传 FPGA

协议 v2（分片版，与 FPGA 侧确认）：单包 ≤1420 字节，绝不触发 IP 分片。
  为什么必须分片：以太网 MTU 1500。v1 的 9616 字节整包会被内核切成 7 个 IP 分片，
  FPGA 没有 IP 重组逻辑 → 收到全是垃圾。回环测试（MTU 65536）发现不了这个坑。

  v2 包头 16 字节（大端）：
    off  len  字段
    0    4    magic = "SK01"
    4    1    version = 2
    5    1    flags   bit0=1bit, bit1=反色
    6    2    frame_id
    8    2    width
    10   2    height
    12   2    payload_len   整幅位图总字节数（9600）
    14   1    pkt_idx       0 .. pkt_total-1
    15   1    pkt_total
    16   N    data          本包数据，N ≤ 1400
    16+N 2    CRC16-CCITT(0xFFFF)，覆盖 0..16+N-1

  320x240 位图 = 9600 字节 → 7 包（6×1400 + 1×1200），15fps ≈ 105 包/秒 ≈ 1.2 Mbit/s

用法：
    python3 skeleton_out.py --test              # 自测：渲染+分片+回环重组
    python3 skeleton_out.py --size 320x240 --out sk.png
"""
import argparse
import struct

import cv2
import numpy as np

PAIRS = [(5, 6), (5, 7), (7, 9), (6, 8), (8, 10), (5, 11), (6, 12),
         (11, 12), (11, 13), (13, 15), (12, 14), (14, 16)]
KPT_CONF_MIN = 0.3
MAGIC = b"SK01"
VERSION = 2
HDR_LEN = 16          # magic4 + ver1 + flags1 + fid2 + w2 + h2 + plen2 + idx1 + total1
CHUNK = 1400          # 单包数据上限：16+1400+2 = 1418 < 1500 ✓
MAX_PKT = HDR_LEN + CHUNK + 2


def crc16_ccitt(data, init=0xFFFF):
    """CRC16-CCITT（poly 0x1021, init 0xFFFF）

    用 binascii.crc_hqx（C 速度）：9616 字节上纯 Python 要 5ms+，这里 0.05ms。
    """
    try:
        import binascii
        return binascii.crc_hqx(bytes(data), init) & 0xFFFF
    except Exception:
        crc = init
        for byte in data:
            crc ^= byte << 8
            for _ in range(8):
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
        return crc


# ---------------- 渲染 ----------------

def render_mask(kps, size=(320, 240), src_size=(640, 480),
                thickness=2, dot_radius=2, head_radius=8, pad=2):
    """kps: (17,3) 原图坐标 -> size 大小的 uint8 掩码（0/255）；kps 为 None 时只画空白"""
    w, h = size
    sw, sh = src_size
    mask = np.zeros((h, w), dtype=np.uint8)
    sx, sy = float(w) / sw, float(h) / sh

    pts = []
    if kps is not None:
        for x, y, c in kps:
            if c > KPT_CONF_MIN:
                pts.append((int(round(x * sx)), int(round(y * sy))))
            else:
                pts.append(None)
    if not pts:
        pts = [None] * 17

    for i, j in PAIRS:
        if pts[i] and pts[j]:
            cv2.line(mask, pts[i], pts[j], 255, thickness, cv2.LINE_8)

    if pts[0]:
        cv2.circle(mask, pts[0], head_radius, 255, thickness, cv2.LINE_8)

    for p in pts:
        if p:
            cv2.circle(mask, p, dot_radius, 255, -1, cv2.LINE_8)

    if pad > 0:
        mask[:pad, :] = 0
        mask[-pad:, :] = 0
        mask[:, :pad] = 0
        mask[:, -pad:] = 0
    return mask


def render_overlay(kps, lines=None, size=(320, 240), src_size=(640, 480),
                   thickness=2, dot_radius=2, head_radius=8, pad=2,
                   font_scale=0.45, line_h=14, origin=(6, 3)):
    """火柴人 + 文字一起画进同一张 1bit 掩码（FPGA 不用画线、不用字库）"""
    mask = render_mask(kps, size=size, src_size=src_size, thickness=thickness,
                       dot_radius=dot_radius, head_radius=head_radius, pad=pad)
    if lines:
        x0, y0 = origin
        y = y0 + line_h
        for item in lines:
            text, invert = item if isinstance(item, tuple) else (item, False)
            cv2.putText(mask, str(text), (x0, y), cv2.FONT_HERSHEY_SIMPLEX,
                        font_scale, 0 if invert else 255, 1, cv2.LINE_8)
            y += line_h
    return mask


# ---------------- 打包 / 解包 ----------------

def pack_1bit(mask):
    """(H,W) uint8 0/255 -> 1bit 字节流：每行字节对齐，字节内高位（左）在前"""
    bits = (mask > 127).astype(np.uint8)
    return np.packbits(bits, axis=1, bitorder='big').tobytes()


def build_packets(mask, frame_id, flags=1, chunk=CHUNK):
    """把位图打成多个 ≤1420 字节的 UDP 包（v2 分片协议）"""
    data = pack_1bit(mask)
    h, w = mask.shape
    total = (len(data) + chunk - 1) // chunk
    pkts = []
    for idx in range(total):
        part = data[idx * chunk:(idx + 1) * chunk]
        hdr = (MAGIC + bytes([VERSION, flags])
               + struct.pack(">HHHH", frame_id & 0xFFFF, w, h, len(data))
               + bytes([idx, total]))
        body = hdr + part
        pkts.append(body + struct.pack(">H", crc16_ccitt(body)))
    return pkts


def parse_packets(buf):
    """解析单个 v2 包，返回 dict（供自测 / FPGA 参考）"""
    if len(buf) < HDR_LEN + 2 or buf[0:4] != MAGIC:
        return None
    version, flags = buf[4], buf[5]
    frame_id, w, h, plen = struct.unpack(">HHHH", buf[6:14])
    idx, total = buf[14], buf[15]
    data = buf[HDR_LEN:len(buf) - 2]
    crc = struct.unpack(">H", buf[len(buf) - 2:])[0]
    return {
        'version': version, 'flags': flags, 'frame_id': frame_id,
        'w': w, 'h': h, 'payload_len': plen, 'pkt_idx': idx, 'pkt_total': total,
        'data': data, 'crc_ok': crc == crc16_ccitt(buf[:len(buf) - 2]),
    }


def reassemble(pkts):
    """把同一帧的多个包拼回位图（测试用）"""
    parts = {}
    meta = None
    for b in pkts:
        p = parse_packets(b)
        if p is None or not p['crc_ok']:
            continue
        meta = p
        parts[p['pkt_idx']] = p['data']
    if meta is None or len(parts) != meta['pkt_total']:
        return None
    blob = b''.join(parts[i] for i in range(meta['pkt_total']))
    row_bytes = (meta['w'] + 7) // 8
    bits = np.unpackbits(np.frombuffer(blob[:row_bytes * meta['h']], np.uint8),
                         bitorder='big').reshape(meta['h'], row_bytes * 8)[:, :meta['w']]
    return (bits * 255).astype(np.uint8)


# ---------------- 自测 ----------------

def demo_kps():
    base = {
        0: (320, 120), 1: (312, 115), 2: (328, 115), 3: (305, 118), 4: (335, 118),
        5: (280, 175), 6: (360, 175), 7: (255, 235), 8: (385, 235),
        9: (240, 290), 10: (400, 290),
        11: (295, 285), 12: (345, 285), 13: (290, 370), 14: (350, 370),
        15: (285, 450), 16: (355, 450),
    }
    k = np.zeros((17, 3), np.float32)
    for i, (x, y) in base.items():
        k[i] = (x, y, 0.9)
    return k


def _selftest():
    size = (320, 240)
    mask = render_overlay(demo_kps(),
                          lines=[("SCORE 1.20/1.6", False), ("JUMP OK", False), ("BAL 0.9s", False)],
                          size=size)
    pkts = build_packets(mask, frame_id=7)
    sizes = [len(p) for p in pkts]
    back = reassemble(pkts)
    print("=== SK01 v2 自测 ===")
    print("  位图 %dx%d  非零像素 %d" % (size[0], size[1], int((mask > 0).sum())))
    print("  分片数 %d  各包长 %s  最大 %d 字节（MTU 1500 限制 %d）"
          % (len(pkts), sizes, max(sizes), MAX_PKT))
    print("  30fps 带宽: %.2f Mbit/s" % (sum(sizes) * 8 * 30 / 1e6))
    print("  回环重组: %s" % ("OK 与原始位图一致" if back is not None and np.array_equal(back, mask) else "FAIL"))
    return 0 if (back is not None and np.array_equal(back, mask) and max(sizes) <= 1420) else 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--test', action='store_true')
    ap.add_argument('--size', default='320x240')
    ap.add_argument('--out', default='sk_mask.png')
    args = ap.parse_args()
    if args.test or True:
        rc = _selftest()
        if args.test:
            return rc
    w, h = [int(v) for v in args.size.split('x')]
    mask = render_overlay(demo_kps(), lines=[("SCORE 1.20/1.6", False)], size=(w, h))
    cv2.imwrite(args.out, mask)
    print("已保存 %s" % args.out)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
