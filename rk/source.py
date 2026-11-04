"""source.py - 统一视频源：FPGA UDP 流 / 本地视频文件 / 摄像头

API 与 net_recv.VideoReceiver 一致：
    open() -> bool
    read() -> (ok, frame_bgr)
    get_fps() -> float
    close()
    last_frame_id    最近一帧的帧号（UDP 为 FPGA 的 4bit fid；文件/摄像头为自增计数）
    last_frame_rows  最近一帧收到的行数（文件/摄像头恒为 HEIGHT）

用法:
    from source import open_source
    rx = open_source(args.source)     # 自动判断 udp:// / 文件 / camera:0
"""
import re
import time

import numpy as np

from net_recv import HEIGHT, VideoReceiver

__all__ = ['open_source', 'FileReceiver', 'CameraReceiver']


class _BaseFileSource:
    """cv2.VideoCapture 封装：文件或摄像头，均模拟 VideoReceiver 的接口。"""

    def __init__(self, target, loop=False, pace=True):
        self.target = target
        self.loop = loop
        self.pace = pace
        self.cap = None
        self.last_frame_id = 0
        self.last_frame_rows = HEIGHT
        self._fps = 30.0
        self._next_t = 0.0

    def open(self):
        import cv2
        self.cap = cv2.VideoCapture(self.target)
        if not self.cap.isOpened():
            return False
        fps = self.cap.get(cv2.CAP_PROP_FPS)
        if fps and 1.0 < fps < 240.0:
            self._fps = float(fps)
        self._next_t = time.time()
        return True

    def read(self):
        ok, frame = self.cap.read()
        if (not ok or frame is None) and self.loop:
            import cv2
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ok, frame = self.cap.read()
        if not ok or frame is None:
            return False, None

        self.last_frame_id = (self.last_frame_id + 1) & 0x0F
        self.last_frame_rows = frame.shape[0]

        if self.pace:
            self._next_t += 1.0 / self._fps
            dt = self._next_t - time.time()
            if dt > 0:
                time.sleep(dt)
            else:
                self._next_t = time.time()   # 处理跟不上时不要累积补帧
        return True, frame

    def get_fps(self):
        return self._fps

    def close(self):
        if self.cap is not None:
            self.cap.release()
            self.cap = None


class FileReceiver(_BaseFileSource):
    """本地视频文件源，用于没有 FPGA 时联调。"""

    def __init__(self, path, loop=True, pace=True):
        _BaseFileSource.__init__(self, path, loop=loop, pace=pace)


class CameraReceiver(_BaseFileSource):
    """摄像头源：camera:0 或 /dev/video0

    RK3568 的 MIPI 摄像头是 multiplanar V4L2，OpenCV 的 V4L2 后端抓不动
    （能 open 但 read 全 False）。所以优先用 GStreamer 管道取 BGR 原始帧。
    """

    def __init__(self, target, pace=False, width=800, height=600, fps=30):
        t = target
        if isinstance(t, str) and t.lower().startswith('camera:'):
            t = int(re.sub(r'[^0-9]', '', t) or 0)
        elif isinstance(t, str) and t.startswith('/dev/video'):
            t = int(re.sub(r'[^0-9]', '', t) or 0)
        self.device = '/dev/video%d' % t
        self.width, self.height, self.fps = int(width), int(height), int(fps)
        self.proc = None
        _BaseFileSource.__init__(self, t, loop=False, pace=pace)

    def open(self):
        import subprocess
        pipe = ("v4l2src device=%s io-mode=4 ! video/x-raw,width=%d,height=%d,format=NV12,"
                "framerate=%d/1 ! videoconvert ! video/x-raw,format=BGR ! fdsink fd=1"
                % (self.device, self.width, self.height, self.fps))
        args = ['gst-launch-1.0', '-q'] + pipe.split()
        try:
            self.proc = subprocess.Popen(args, stdout=subprocess.PIPE, bufsize=0)
        except OSError:
            return False
        self._fps = float(self.fps)
        self._next_t = time.time()
        return True

    def read(self):
        if self.proc is None or self.proc.stdout is None:
            return False, None
        n = self.width * self.height * 3
        chunks = []
        got = 0
        while got < n:
            chunk = self.proc.stdout.read(n - got)
            if not chunk:
                return False, None
            chunks.append(chunk)
            got += len(chunk)
        frame = np.frombuffer(b''.join(chunks), dtype=np.uint8).reshape(
            self.height, self.width, 3).copy()
        self.last_frame_id = (self.last_frame_id + 1) & 0x0F
        self.last_frame_rows = self.height
        return True, frame

    def get_fps(self):
        return float(self.fps)

    def close(self):
        if self.proc is not None:
            try:
                self.proc.terminate()
                self.proc.wait(timeout=2)
            except Exception:
                try:
                    self.proc.kill()
                except Exception:
                    pass
            self.proc = None


def open_source(source, **kwargs):
    """按 source 形态选择接收器；参数与 VideoReceiver/FileReceiver 兼容。"""
    s = (source or '').strip()
    if re.match(r'^udp://', s) or s.isdigit():
        return VideoReceiver(s, **kwargs)
    if s.lower().startswith('camera:') or s.startswith('/dev/video'):
        return CameraReceiver(s)
    return FileReceiver(s, **kwargs)
