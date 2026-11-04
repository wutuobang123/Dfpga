"""action_segment.py - 动作分段（《评分系统方案》§3）

3.1 用膝角做状态机（比髋高更稳）：
    IDLE(站立,膝角>160) → 膝角<120 → DESCENDING → 局部最小 → BOTTOM
    → 膝角>120 → ASCENDING → 膝角>150 → DONE（输出一次动作区间）

有效性校验（不满足就丢弃，避免误计数）：
    总时长 0.8~3.0 s ｜ 最低膝角 <130° ｜ 关键点置信度全程 >0.3 ｜ 两次间隔 >0.5 s

3.2 降级方案：分段调不稳时切定长窗口（不计数，只对最近 N 秒评分），
    绝不因为分段错误导致分数乱跳。
"""
import time
from collections import deque

import numpy as np

from preprocess_kps import mean_knee_angle, lean_angle, knee_ankle_dx, KPT_MIN_CONF

# 状态
IDLE, DESCENDING, BOTTOM, ASCENDING = "IDLE", "DESCENDING", "BOTTOM", "ASCENDING"

CFG = dict(
    stand_deg=160.0,      # IDLE 判定：膝角 > 160
    down_deg=120.0,       # 进入下蹲：膝角 < 120
    up_deg=150.0,         # 完成起身：膝角 > 150
    min_bottom_deg=130.0, # 最低膝角必须 < 130 才算一次
    dur_min=0.8,          # 动作总时长下限
    dur_max=3.0,          # 动作总时长上限
    gap_min=0.5,          # 两次动作最小间隔
    conf_min=KPT_MIN_CONF,
)


class RepEvent:
    """一次完整动作的结果"""
    __slots__ = ("idx", "t0", "t1", "knee_min", "lean", "align_dx", "conf_ok", "valid")

    def __init__(self, idx, t0, t1, knee_min, lean, align_dx, conf_ok, valid):
        self.idx, self.t0, self.t1 = idx, t0, t1
        self.knee_min, self.lean, self.align_dx = knee_min, lean, align_dx
        self.conf_ok, self.valid = conf_ok, valid

    @property
    def duration(self):
        return self.t1 - self.t0

    def as_dict(self):
        return dict(idx=self.idx, duration=round(self.duration, 3),
                    knee_min=round(self.knee_min, 1),
                    lean=round(self.lean, 1),
                    align_dx=(None if self.align_dx is None else round(self.align_dx, 4)),
                    valid=self.valid, conf_ok=self.conf_ok)


class SquatSegmenter:
    """深蹲状态机：喂入 (kps, t) 逐帧，吐出完整的 RepEvent"""

    def __init__(self, cfg=None):
        self.cfg = dict(CFG)
        if cfg:
            self.cfg.update(cfg)
        self.state = IDLE
        self.t_start = None
        self.t_last_rep = -1e9
        self.knee_min = 999.0
        self.lean_sum, self.lean_n = 0.0, 0
        self.dx_sum, self.dx_n = 0.0, 0
        self.conf_all_ok = True
        self.idx = 0
        self.reps = []
        self.dropped = 0
        self.aborted = 0

    def reset(self):
        self.__init__(self.cfg)

    def feed(self, kps, t=None):
        """返回 本次触发的 RepEvent（或 None）"""
        t = time.time() if t is None else float(t)
        c = self.cfg
        knee = mean_knee_angle(kps)
        if knee is None:
            self.conf_all_ok = False
            return None
        if kps[13][2] <= c["conf_min"] or kps[14][2] <= c["conf_min"]:
            self.conf_all_ok = False

        if self.state == IDLE:
            if knee > c["stand_deg"]:
                self.knee_min, self.conf_all_ok = 999.0, True
                self.lean_sum, self.lean_n, self.dx_sum, self.dx_n = 0.0, 0, 0.0, 0
            # 关键：从"开始屈膝"（< stand_deg）就计时，而不是等到 < down_deg
            # 否则 0.8s 的时长下限会把正常深蹲全判成"太快"（实测 1.4s 的动作只剩 0.7s 窗口）
            if knee < c["stand_deg"] and t - self.t_last_rep > c["gap_min"]:
                self.state = DESCENDING
                self.t_start = t
                self.knee_min = knee
                self._acc(kps)
            return None

        # 下蹲途中 / 底部 / 起身途中
        self.knee_min = min(self.knee_min, knee)
        self._acc(kps)

        if self.state == DESCENDING:
            if knee < c["down_deg"]:
                self.state = BOTTOM
            elif knee >= c["stand_deg"]:
                # 蹲了一半又站回去（半蹲）→ 这次尝试作废，立刻回到 IDLE
                # 不终止的话会把这半次和下一次深蹲合并，导致下一次因"时长超限"被丢弃
                self.state = IDLE
                self.aborted += 1
            elif t - self.t_start > c["dur_max"]:      # 蹲着不动，超时放弃
                self.state = IDLE
                self.aborted += 1
            return None

        if self.state == BOTTOM:
            if knee > c["down_deg"]:
                self.state = ASCENDING
            return None

        if self.state == ASCENDING:
            if knee > c["up_deg"]:
                ev = self._finish(t)
                self.state = IDLE
                return ev
            if knee < c["down_deg"]:            # 又蹲下去了（连续深蹲不站直）
                self.state = BOTTOM
            return None
        return None

    def _acc(self, kps):
        try:
            ln = lean_angle(kps)
            self.lean_sum += ln
            self.lean_n += 1
        except Exception:
            pass
        try:
            dx = knee_ankle_dx(kps)
            if dx is not None:
                self.dx_sum += dx
                self.dx_n += 1
        except Exception:
            pass

    def _finish(self, t):
        c = self.cfg
        dur = t - self.t_start
        lean = self.lean_sum / self.lean_n if self.lean_n else 0.0
        dx = self.dx_sum / self.dx_n if self.dx_n else None
        valid = (c["dur_min"] <= dur <= c["dur_max"]
                 and self.knee_min < c["min_bottom_deg"]
                 and self.conf_all_ok)
        self.idx += 1
        ev = RepEvent(self.idx, self.t_start, t, self.knee_min, lean, dx,
                      self.conf_all_ok, valid)
        if valid:
            self.t_last_rep = t
            self.reps.append(ev)
        else:
            self.dropped += 1
        return ev if valid else None

    def stats(self):
        return dict(count=len(self.reps), dropped=self.dropped, state=self.state,
                    knee_min_last=(round(self.reps[-1].knee_min, 1) if self.reps else None))


class FixedWindowFallback:
    """§3.2 降级方案：不做计数，只维护"最近 N 秒"窗口用于评分"""

    def __init__(self, seconds=3.0):
        self.seconds = float(seconds)
        self.buf = deque()

    def push(self, kps, t=None):
        """入队时就把该帧的三项指标算好（增量），metrics() 只做聚合

        坑：之前 metrics() 每秒对窗口内 90 帧现算 lean/dx 纯 Python 循环，
        实测把推理从 28fps 拖到 15fps。现在每帧只算一次。
        """
        t = time.time() if t is None else float(t)
        try:
            knee = mean_knee_angle(kps)
        except Exception:
            knee = None
        try:
            lean = lean_angle(kps)
        except Exception:
            lean = None
        try:
            dx = knee_ankle_dx(kps)
        except Exception:
            dx = None
        self.buf.append((t, kps, knee, lean, dx))
        while self.buf and t - self.buf[0][0] > self.seconds:
            self.buf.popleft()
        return len(self.buf)

    def frames(self):
        return [item[1] for item in self.buf]

    def knee_min(self):
        ks = [item[2] for item in self.buf if item[2] is not None]
        return min(ks) if ks else None

    def metrics(self):
        """窗口内三项指标（走已算好的增量值，聚合是纯 float 运算）"""
        n = len(self.buf)
        if n == 0:
            return dict(n=0, knee_min=None, lean=None, align_dx=None)
        ks = [it[2] for it in self.buf if it[2] is not None]
        leans = [it[3] for it in self.buf if it[3] is not None]
        dxs = [it[4] for it in self.buf if it[4] is not None]
        return dict(
            n=n,
            knee_min=(min(ks) if ks else None),
            lean=(sum(leans) / len(leans) if leans else None),
            align_dx=(sum(dxs) / len(dxs) if dxs else None),
        )


def selftest():
    """合成一个深蹲序列，验证计数/丢弃/间隔"""
    def pose(knee_deg, conf=0.9):
        """按膝角生成姿势：从脚往上推（小腿竖直、髋由膝角决定）

        K=180° 完全伸直（髋在膝正上方）；K 变小则髋向前下方移动。
        """
        thigh = shank = 85.0
        ankle = np.array([320.0, 470.0])
        knee = ankle + np.array([0.0, -shank])            # 小腿竖直
        th = np.radians(180.0 - knee_deg)
        hip = knee + thigh * np.array([np.sin(th), -np.cos(th)])
        # 躯干：肩在髋上方 95px，随膝角略前倾
        lean = th * 0.25
        sho = hip + 95.0 * np.array([np.sin(lean), -np.cos(lean)])
        k = np.tile(np.array([[sho[0], sho[1] - 40, conf]]), (17, 1)).astype(np.float32)
        k[5] = (sho[0] - 25, sho[1], conf); k[6] = (sho[0] + 25, sho[1], conf)
        k[7] = (sho[0] - 32, sho[1] + 45, conf); k[8] = (sho[0] + 32, sho[1] + 45, conf)
        k[9] = (sho[0] - 35, sho[1] + 85, conf); k[10] = (sho[0] + 35, sho[1] + 85, conf)
        k[11] = (hip[0] - 18, hip[1], conf); k[12] = (hip[0] + 18, hip[1], conf)
        k[13] = (knee[0] - 16, knee[1], conf); k[14] = (knee[0] + 16, knee[1], conf)
        k[15] = (ankle[0] - 16, ankle[1], conf); k[16] = (ankle[0] + 16, ankle[1], conf)
        return k

    FPS = 30.0
    seg = SquatSegmenter()
    t = 0.0
    events = []
    # 3 次标准深蹲（每次 1.6s：站 0.3 → 蹲到 95° → 起）
    for _ in range(3):
        for _ in range(int(0.3 * FPS)):
            t += 1 / FPS; seg.feed(pose(175), t)
        for a in np.linspace(175, 95, int(0.7 * FPS)):
            t += 1 / FPS; seg.feed(pose(a), t)
        for a in np.linspace(95, 175, int(0.7 * FPS)):
            t += 1 / FPS
            ev = seg.feed(pose(a), t)
            if ev:
                events.append(ev.as_dict())
        for _ in range(int(0.6 * FPS)):
            t += 1 / FPS; seg.feed(pose(175), t)
    print("标准 3 次深蹲 ->", seg.stats(), "事件:", events)

    # 半蹲（只到 140°，应被丢弃）
    seg2 = SquatSegmenter()
    t = 0.0
    for _ in range(int(0.3 * FPS)):
        t += 1 / FPS; seg2.feed(pose(175), t)
    for a in np.linspace(175, 140, int(0.4 * FPS)):
        t += 1 / FPS; seg2.feed(pose(a), t)
    for a in np.linspace(140, 175, int(0.4 * FPS)):
        t += 1 / FPS; seg2.feed(pose(a), t)
    for _ in range(int(0.3 * FPS)):
        t += 1 / FPS; seg2.feed(pose(175), t)
    print("半蹲(最低140°) ->", seg2.stats(), "（应 count=0：没到 120°，状态机不进入）")

    # 太快（0.3s，应丢弃）
    seg3 = SquatSegmenter()
    t = 0.0
    for a in np.linspace(175, 95, 5):
        t += 0.03; seg3.feed(pose(a), t)
    for a in np.linspace(95, 175, 5):
        t += 0.03; ev = seg3.feed(pose(a), t)
    print("太快(0.3s) ->", seg3.stats(), "（应 count=0 dropped=1）")

    fb = FixedWindowFallback(seconds=1.0)
    t = 0.0
    for i in range(60):
        t += 1 / FPS
        fb.push(pose(175 - i), t)
    print("降级窗口 ->", len(fb.frames()), "帧, 最低膝角", round(fb.knee_min() or 0, 1))
    return 0


if __name__ == "__main__":
    raise SystemExit(selftest())






