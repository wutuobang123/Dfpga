"""net_recv.py - FPGA UDP RGB565 行流接收（recvmmsg 批量 + 向量化组帧 + cv2 SIMD 转色）

协议（《RK接口对接文档 v1.0》§3）:
    payload[0:2] = header = {frame_id[3:0] << 12 | row[11:0]}  大端
    payload[2:]  = 640 个 RGB565 像素，大端，整包 1282 字节
    RGB565 位布局: R=高5位 G=中6位 B=低5位（与 cv2.COLOR_BGR5652BGR 一致）

实测（RK3568，30fps / 14400 包每秒，假 FPGA 压测）：
    组帧  ~1.4 ms/帧（>u2 视图 + fancy index）
    转色  ~0.5 ms/帧（cv2.cvtColor SIMD，比 numpy 移位快 14 倍）
    合计  ~1.9 ms/帧，单核占用个位数%，内核丢包 0，整帧率 99.7%
"""
import ctypes
import ctypes.util
import os
import re
import socket
import time

import cv2
import numpy as np

WIDTH = 640
HEIGHT = 480
PKT_LEN = 2 + WIDTH * 2          # 1282
BATCH = 64


class _iovec(ctypes.Structure):
    _fields_ = [("iov_base", ctypes.c_void_p),
                ("iov_len", ctypes.c_size_t)]


class _msghdr(ctypes.Structure):
    _fields_ = [("msg_name", ctypes.c_void_p),
                ("msg_namelen", ctypes.c_uint32),
                ("_pad0", ctypes.c_uint32),
                ("msg_iov", ctypes.c_void_p),
                ("msg_iovlen", ctypes.c_size_t),
                ("msg_control", ctypes.c_void_p),
                ("msg_controllen", ctypes.c_size_t),
                ("msg_flags", ctypes.c_int32),
                ("_pad1", ctypes.c_uint32)]


class _mmsghdr(ctypes.Structure):
    _fields_ = [("msg_hdr", _msghdr),
                ("msg_len", ctypes.c_uint32),
                ("_pad", ctypes.c_uint32)]


_libc = ctypes.CDLL(ctypes.util.find_library("c") or "libc.so.6", use_errno=True)
_libc.recvmmsg.restype = ctypes.c_int
_libc.recvmmsg.argtypes = [ctypes.c_int,
                           ctypes.POINTER(_mmsghdr),
                           ctypes.c_uint,
                           ctypes.c_int,
                           ctypes.c_void_p]


class VideoReceiver:
    """API: open() / read() / pump() / get_fps() / close()

    额外属性：
        last_frame_id    最近完成的帧的 frame_id（4bit）
        last_frame_rows  该帧实际收到的行数（0~480）
    """

    def __init__(self, source: str, batch: int = BATCH, pin_cpu: int = None):
        self.source = source
        self.sock = None
        self.batch = batch
        self.pin_cpu = pin_cpu
        self.last_frame_id = 0
        self.last_frame_rows = 0
        self.ctrl_reset = 0        # 收到 FPGA 按键控制包的累计次数

        self._frames = [np.zeros((HEIGHT, WIDTH), dtype=np.uint16),
                        np.zeros((HEIGHT, WIDTH), dtype=np.uint16)]
        self._wi = 0
        self._cur_fid = None
        self._row_seen = np.zeros(HEIGHT, dtype=bool)

        # 分阶段计时（默认关，profile=True 时累加）
        self.profile = False
        self.t_recv = self.t_asm = self.t_conv = 0.0

        self._bufs = (ctypes.c_char * (batch * PKT_LEN))()
        self._buf_np = np.ctypeslib.as_array(self._bufs).view(np.uint8).reshape(batch, PKT_LEN)
        # 整批按大端 uint16 看：每包 641 个 word，word0 就是头字（省掉手工移位）
        self._buf_words = self._buf_np.view('>u2')

        self._iovs = (_iovec * batch)()
        self._msgs = (_mmsghdr * batch)()
        for i in range(batch):
            self._iovs[i].iov_base = ctypes.cast(
                ctypes.addressof(self._bufs) + i * PKT_LEN, ctypes.c_void_p)
            self._iovs[i].iov_len = PKT_LEN
            self._msgs[i].msg_hdr.msg_iov = ctypes.cast(
                ctypes.byref(self._iovs, i * ctypes.sizeof(_iovec)), ctypes.c_void_p)
            self._msgs[i].msg_hdr.msg_iovlen = 1
            self._msgs[i].msg_hdr.msg_name = None
            self._msgs[i].msg_hdr.msg_namelen = 0
            self._msgs[i].msg_hdr.msg_control = None
            self._msgs[i].msg_hdr.msg_controllen = 0

        self._host = "0.0.0.0"
        self._port = 8080
        self._parse_source(source)

    def _parse_source(self, source: str):
        m = re.match(r"^udp://([^:/]+):(\d+)$", source or "")
        if m:
            self._host, self._port = m.group(1), int(m.group(2))
            return
        m = re.match(r"^udp://(\d+)$", source or "")
        if m:
            self._port = int(m.group(1))
            return
        if source and source.isdigit():
            self._port = int(source)

    # ------------------------------------------------------------------
    def open(self) -> bool:
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 16 * 1024 * 1024)
        except OSError:
            pass
        try:
            self.sock.bind((self._host, self._port))
        except OSError:
            return False
        self.sock.setblocking(True)

        actual = self.sock.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF)
        tag = "OK" if actual >= 4 * 1024 * 1024 else "LOW - need root: net.core.rmem_max"
        print("[RX] SO_RCVBUF actual = %d bytes (%s)" % (actual, tag), flush=True)

        if self.pin_cpu is not None:
            try:
                os.sched_setaffinity(0, {self.pin_cpu})
                print("[RX] pinned to CPU %d" % self.pin_cpu, flush=True)
            except OSError as e:
                print("[RX] pin_cpu failed: %s" % e, flush=True)
        try:
            os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(80))
            print("[RX] SCHED_FIFO enabled", flush=True)
        except Exception as e:
            print("[RX] SCHED_FIFO unavailable (%s), continue" % e, flush=True)
        return True

    def _recv_batch(self):
        n = _libc.recvmmsg(self.sock.fileno(), self._msgs, self.batch, 0, None)
        if n < 0:
            e = ctypes.get_errno()
            if e == 4:                       # EINTR
                return 0
            raise OSError(e, os.strerror(e))
        return n

    # ------------------------------------------------------------------
    def pump(self):
        """收一批并组帧；凑满一帧时返回 (fid, frame_bgr)，否则 None。"""
        _p0 = time.process_time() if self.profile else 0.0
        try:
            n = self._recv_batch()
        except OSError:
            return None
        if n <= 0:
            return None

        lens = np.empty(n, dtype=np.uint32)
        msgs = self._msgs
        for i in range(n):
            lens[i] = msgs[i].msg_len

        if self.profile:
            _p1 = time.process_time(); self.t_recv += _p1 - _p0
        words = self._buf_words[:n]              # (n, 641) 大端 uint16
        hdr = words[:, 0].astype(np.uint16)      # 整个头字
        rows_all = hdr & 0x0FFF
        fid_all = (hdr >> 12) & 0x0F

        # 控制包：row==0x7FF（FPGA 按键），正常行号只到 480，不影响图像
        _ctrl = (rows_all == 0x7FE) | (rows_all == 0x7FF)
        if _ctrl.any():
            self.ctrl_reset += int(_ctrl.sum())

        valid = (lens >= PKT_LEN) & (rows_all < HEIGHT)
        if not valid.all():
            # 罕见路径（短包/越界行）：先剔除，再继续
            keep = np.flatnonzero(valid)
            if keep.size == 0:
                return None
            words = words[keep]
            rows_all = rows_all[keep]
            fid_all = fid_all[keep]
            n = keep.size

        # 批内按 frame_id 变化切段；到达顺序即时间段顺序
        cuts = np.flatnonzero(np.diff(fid_all) != 0) + 1
        bounds = np.concatenate(([0], cuts, [n]))

        ready = None
        ready_rows = 0
        write_frame = self._frames[self._wi]

        for s in range(len(bounds) - 1):
            a, b = int(bounds[s]), int(bounds[s + 1])
            f = int(fid_all[a])

            if self._cur_fid is None:
                self._cur_fid = f
                self._row_seen[:] = False
            elif f != self._cur_fid:
                ready = (self._cur_fid, write_frame)
                ready_rows = int(self._row_seen.sum())    # 必须在重置前统计
                self._wi ^= 1
                write_frame = self._frames[self._wi]
                write_frame[:] = 0
                self._cur_fid = f
                self._row_seen[:] = False

            rows = rows_all[a:b]
            # 直接切片（view）再 astype：不要先做 fancy index 复制，
            # 非本机字节序数组的 take 会走慢路径（实测慢 6 倍）
            write_frame[rows] = words[a:b, 1:].astype(np.uint16)     # 大端 -> 本机
            self._row_seen[rows] = True

        if ready is None:
            return None

        if self.profile:
            _p2 = time.process_time(); self.t_asm += _p2 - _p1
        fid, frame565 = ready
        self.last_frame_id = fid
        self.last_frame_rows = ready_rows

        # row0 前 8 像素是 FPGA 调试信息（0xDEAD + gain），按 §3.5 用邻域覆盖
        frame565[0, :8] = frame565[0, 8]

        # RGB565 -> BGR888：交给 cv2 的 SIMD（约 0.5ms，比 numpy 移位快 14 倍）
        u8 = frame565.view(np.uint8).reshape(HEIGHT, WIDTH, 2)
        bgr = cv2.cvtColor(u8, cv2.COLOR_BGR5652BGR)
        if self.profile:
            self.t_conv += time.process_time() - _p2
        return fid, bgr

    # ------------------------------------------------------------------
    def read(self):
        if self.sock is None:
            return False, None
        while True:
            out = self.pump()
            if out is not None:
                return True, out[1]
            time.sleep(0.0005)

    def get_fps(self) -> float:
        return 30.0

    def close(self):
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None


