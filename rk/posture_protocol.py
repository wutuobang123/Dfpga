# -*- coding: utf-8 -*-
"""posture_protocol.py - 上半身体态筛查流程（6 步引导，每步保持 2 秒）

流程：
    ① T 字侧平举  → 高低肩 / 手臂水平度 / 伸展度
    ② Y 字上举    → 肩活动度对称 / 举高程度 / 躯干直立
    ③ 上举贴耳    → 肩前屈活动度 / 肘伸直 / 左右对称
    ④ 自然站立    → 头颈侧倾 / 肩高基线
    → 汇总：四项均分 + 最弱项 + 等级

设计原则同 posture.py：不做动作识别，只做"保持姿势 → 测量偏差"。
"""
import time
import json
import os
import numpy as np

from preprocess_kps import normalize, lean_angle, KPT_MIN_CONF

NOSE, L_EAR, R_EAR = 0, 3, 4
L_SHO, R_SHO = 5, 6
L_ELB, R_ELB = 7, 8
L_WRI, R_WRI = 9, 10
L_HIP, R_HIP = 11, 12

CFG = dict(
    hold_sec=2.0,
    lose_sec=0.40,
    cooldown=1.2,       # 完成一步后的间隔（给"下一项"提示）
    finish_hold=6.0,    # 报告停留多久后重新开始
    min_conf=KPT_MIN_CONF,
)

STAR = [(85.0, 3, "优秀"), (70.0, 2, "良好"), (55.0, 1, "合格")]


def clamp(v, lo=0.0, hi=100.0):
    return float(max(lo, min(hi, v)))


def _ang(a, b, c):
    v1 = np.asarray(a[:2], float) - np.asarray(b[:2], float)
    v2 = np.asarray(c[:2], float) - np.asarray(b[:2], float)
    n = np.linalg.norm(v1) * np.linalg.norm(v2)
    if n < 1e-6:
        return None
    return float(np.degrees(np.arccos(np.clip(float(np.dot(v1, v2) / n), -1.0, 1.0))))


def _dist(a, b):
    return float(np.linalg.norm(np.asarray(a[:2], float) - np.asarray(b[:2], float)))


class Metrics:
    """一帧的体态量（全部用躯干长归一化）"""

    @staticmethod
    def compute(kps):
        k = np.asarray(kps, dtype=np.float32)
        if k.ndim != 2 or k.shape[0] < 17:
            return None
        norm, torso, ok = normalize(k)
        if not ok:
            return None
        d = {}
        d["conf_ok"] = (k[L_WRI][2] > KPT_MIN_CONF and k[R_WRI][2] > KPT_MIN_CONF
                        and k[L_SHO][2] > KPT_MIN_CONF and k[R_SHO][2] > KPT_MIN_CONF)
        d["torso"] = torso
        # 手腕相对肩的高度（负 y 向上 -> 取负）
        d["raise_l"] = -(float(norm[L_WRI][1]) - float(norm[L_SHO][1]))
        d["raise_r"] = -(float(norm[R_WRI][1]) - float(norm[R_SHO][1]))
        d["raise_min"] = min(d["raise_l"], d["raise_r"])
        d["raise_avg"] = (d["raise_l"] + d["raise_r"]) / 2.0
        d["shoulder_diff"] = abs(d["raise_l"] - d["raise_r"])
        d["level"] = (abs(d["raise_l"]) + abs(d["raise_r"])) / 2.0
        # 展臂跨度 / 肩宽
        sho_w = abs(float(norm[L_SHO][0] - norm[R_SHO][0]))
        d["span"] = abs(float(norm[L_WRI][0] - norm[R_WRI][0])) / sho_w if sho_w > 1e-3 else 0.0
        # 腕到同侧耳的距离（贴耳动作）
        d["ear_dist_l"] = _dist(norm[L_WRI], norm[L_EAR])
        d["ear_dist_r"] = _dist(norm[R_WRI], norm[R_EAR])
        d["ear_dist"] = (d["ear_dist_l"] + d["ear_dist_r"]) / 2.0
        d["ear_dist_diff"] = abs(d["ear_dist_l"] - d["ear_dist_r"])
        # 腕到同侧髋的距离（叉腰动作）
        d["hip_dist_l"] = _dist(norm[L_WRI], norm[L_HIP])
        d["hip_dist_r"] = _dist(norm[R_WRI], norm[R_HIP])
        d["hip_dist"] = (d["hip_dist_l"] + d["hip_dist_r"]) / 2.0
        d["hip_dist_diff"] = abs(d["hip_dist_l"] - d["hip_dist_r"])
        # 肘角
        d["elbow_l"] = _ang(k[L_SHO], k[L_ELB], k[L_WRI]) if k[L_ELB][2] > KPT_MIN_CONF else None
        d["elbow_r"] = _ang(k[R_SHO], k[R_ELB], k[R_WRI]) if k[R_ELB][2] > KPT_MIN_CONF else None
        if d["elbow_l"] is not None and d["elbow_r"] is not None:
            d["elbow_min"] = min(d["elbow_l"], d["elbow_r"])
            d["elbow_diff"] = abs(d["elbow_l"] - d["elbow_r"])
        else:
            d["elbow_min"] = None
            d["elbow_diff"] = None
        # 头颈侧倾：耳到肩的垂直距离差
        h_l = float(norm[L_EAR][1]) - float(norm[L_SHO][1])
        h_r = float(norm[R_EAR][1]) - float(norm[R_SHO][1])
        d["head_tilt"] = abs(h_l - h_r) if (k[L_EAR][2] > KPT_MIN_CONF and k[R_EAR][2] > KPT_MIN_CONF) else None
        try:
            d["lean"] = lean_angle(k)
        except Exception:
            d["lean"] = 0.0
        return d


def pose_front(kps):
    """双臂前平举：腕与肩同高、两腕靠近、肘伸直"""
    m = Metrics.compute(kps)
    if m is None or not m["conf_ok"]:
        return None, None
    ok = (-0.45 < m["raise_l"] < 0.60 and -0.45 < m["raise_r"] < 0.60
          and m["span"] < 1.35)
    if not ok:
        return None, None
    s_lev = clamp(100.0 - max(0.0, abs(m["raise_avg"]) - 0.12) / 0.02 * 4.0)
    s_sym = clamp(100.0 - max(0.0, m["shoulder_diff"] - 0.08) / 0.30 * 100.0)
    s_fwd = clamp((1.20 - m["span"]) / (1.20 - 0.45) * 100.0)
    s_elb = 100.0
    if m["elbow_min"] is not None:
        s_elb = clamp((m["elbow_min"] - 140.0) / 35.0 * 100.0)
    parts = {"level": s_lev, "sym": s_sym, "forward": s_fwd, "elbow": s_elb}
    score = 0.35 * s_lev + 0.25 * s_sym + 0.25 * s_fwd + 0.15 * s_elb
    return score, dict(parts=parts, worst=min(parts, key=parts.get), metrics=m)


def pose_hip(kps):
    """双手叉腰：双手贴髋、肘部弯曲外展、左右对称"""
    m = Metrics.compute(kps)
    if m is None or not m["conf_ok"]:
        return None, None
    ok = (m["raise_l"] < -0.30 and m["raise_r"] < -0.30 and m["hip_dist"] < 0.60)
    if not ok:
        return None, None
    s_hip = clamp((0.60 - m["hip_dist"]) / (0.60 - 0.25) * 100.0)
    s_elb = 100.0
    if m["elbow_min"] is not None:
        s_elb = clamp(100.0 - abs(m["elbow_min"] - 85.0) / 45.0 * 100.0)
    s_sym = clamp(100.0 - max(0.0, m["hip_dist_diff"] - 0.08) / 0.35 * 100.0)
    parts = {"hip": s_hip, "elbow": s_elb, "sym": s_sym}
    score = 0.45 * s_hip + 0.30 * s_elb + 0.25 * s_sym
    return score, dict(parts=parts, worst=min(parts, key=parts.get), metrics=m)


# ---------------- 六个姿势 ----------------

def pose_t(kps):
    """T 字侧平举"""
    m = Metrics.compute(kps)
    if m is None or not m["conf_ok"]:
        return None, None
    ok = (-0.30 < m["raise_l"] < 1.25 and -0.30 < m["raise_r"] < 1.25 and m["span"] > 2.0)
    if not ok:
        return None, None
    s_sho = clamp(100.0 - max(0.0, m["shoulder_diff"] - 0.05) / 0.01 * 5.0)
    s_lev = clamp(100.0 - max(0.0, m["level"] - 0.10) / 0.02 * 5.0)
    s_spn = clamp(100.0 - max(0.0, 2.40 - m["span"]) / 0.20 * 5.0)
    parts = {"shoulder": s_sho, "level": s_lev, "span": s_spn}
    score = 0.45 * s_sho + 0.30 * s_lev + 0.25 * s_spn
    return score, dict(parts=parts, worst=min(parts, key=parts.get), metrics=m)


def pose_y(kps):
    """Y 字上举"""
    m = Metrics.compute(kps)
    if m is None or not m["conf_ok"]:
        return None, None
    ok = (0.90 < m["raise_l"] < 2.40 and 0.90 < m["raise_r"] < 2.40 and m["span"] > 0.50)
    if not ok:
        return None, None
    s_h = clamp((m["raise_min"] - 0.80) / (1.60 - 0.80) * 100.0)
    s_sym = clamp(100.0 - max(0.0, m["shoulder_diff"] - 0.10) / 0.30 * 100.0)
    s_lean = clamp(100.0 - max(0.0, m["lean"] - 15.0) * 2.0)
    parts = {"height": s_h, "sym": s_sym, "lean": s_lean}
    score = 0.45 * s_h + 0.30 * s_sym + 0.25 * s_lean
    return score, dict(parts=parts, worst=min(parts, key=parts.get), metrics=m)


def pose_ear(kps):
    """上举贴耳"""
    m = Metrics.compute(kps)
    if m is None or not m["conf_ok"]:
        return None, None
    ok = (m["ear_dist"] < 0.65 and m["raise_min"] > 0.35)
    if not ok:
        return None, None
    s_d = clamp((0.70 - m["ear_dist"]) / (0.70 - 0.35) * 100.0)
    s_sym = clamp(100.0 - max(0.0, m["ear_dist_diff"] - 0.10) / 0.40 * 100.0)
    s_elb = 100.0
    if m["elbow_min"] is not None:
        s_elb = clamp((m["elbow_min"] - 130.0) / 30.0 * 100.0)
    parts = {"reach": s_d, "sym": s_sym, "elbow": s_elb}
    score = 0.45 * s_d + 0.30 * s_sym + 0.25 * s_elb
    return score, dict(parts=parts, worst=min(parts, key=parts.get), metrics=m)


def pose_stand(kps):
    """自然站立（测头颈侧倾）"""
    m = Metrics.compute(kps)
    if m is None or not m["conf_ok"]:
        return None, None
    ok = (m["raise_l"] < 0.15 and m["raise_r"] < 0.15)     # 双臂自然下垂
    if not ok or m["head_tilt"] is None:
        return None, None
    s_head = clamp(100.0 - max(0.0, m["head_tilt"] - 0.06) / 0.10 * 100.0)
    s_lean = clamp(100.0 - max(0.0, m["lean"] - 12.0) * 2.5)
    parts = {"head": s_head, "lean": s_lean}
    score = 0.65 * s_head + 0.35 * s_lean
    return score, dict(parts=parts, worst=min(parts, key=parts.get), metrics=m)


POSES = [
    dict(key="t",    name="T字侧平举", short="T字",  fn=pose_t,
         tip="双臂左右平举，与肩同高"),
    dict(key="y",    name="Y字上举",   short="Y字",  fn=pose_y,
         tip="双臂向上举高，手掌朝天"),
    dict(key="ear",  name="上举贴耳",  short="贴耳",  fn=pose_ear,
         tip="双臂举过头贴近耳朵"),
    dict(key="stand",name="自然站立",  short="站立",  fn=pose_stand,
         tip="放下手臂，自然站直"),
    dict(key="front",name="双臂前平举", short="前举", fn=pose_front,
         tip="双臂向前平举，与肩同高"),
    dict(key="hip",  name="双手叉腰",   short="叉腰", fn=pose_hip,
         tip="双手叉在腰上，手肘向外"),
]

HINT = {
    "shoulder": "两肩放平", "level": "手臂抬平", "span": "手臂再展开",
    "height": "举得再高一点", "sym": "两边举一样高", "lean": "身体站直",
    "reach": "手贴近耳朵", "elbow": "手肘伸直", "head": "头摆正", "forward": "手臂往前伸", "hip": "手放到腰上",
}


def level_of(score):
    for th, stars, name in STAR:
        if score >= th:
            return stars, name
    return 0, "需改进"


class PoseEvent:
    __slots__ = ("idx", "pose_key", "pose_name", "t0", "t1", "score", "parts", "worst", "metrics")

    def __init__(self, idx, pose, t0, t1, score, detail):
        self.idx = idx
        self.pose_key = pose["key"]
        self.pose_name = pose["name"]
        self.t0, self.t1 = t0, t1
        self.score = score
        self.parts = detail["parts"]
        self.worst = detail["worst"]
        self.metrics = detail["metrics"]

    @property
    def duration(self):
        return self.t1 - self.t0


class ProtocolMeasurer:
    """4 步引导：每步保持 hold_sec 后锁定，最后出报告"""

    def __init__(self, cfg=None):
        self.cfg = dict(CFG)
        if cfg:
            self.cfg.update(cfg)
        self.reporter = None
        self.step = 0
        self.t0 = None
        self.t_lost = None
        self.acc = []
        self.cooldown_until = 0.0
        self.results = []          # 每步结果
        self.finished_at = None
        self._last_push = 0.0
        self.idx = 0

    def reset(self):
        self.__init__(self.cfg)

    @property
    def finished(self):
        return self.finished_at is not None

    def summary(self):
        if not self.results:
            return None
        scores = [r["score"] for r in self.results]
        worst = min(self.results, key=lambda r: r["score"])
        total = round(sum(scores) / len(scores), 1)
        return dict(total=total, scores=scores, worst=worst.get("pose", "-"),
                    worst_score=worst["score"], items=self.results)

    def feed(self, kps, t=None):
        t = time.time() if t is None else float(t)
        c = self.cfg

        # 报告展示完毕 -> 重新开始
        if self.finished:
            if t - self.finished_at > c["finish_hold"]:
                self.reset()
            self._push_live(t, None, 0.0)
            return None

        if t < self.cooldown_until:
            self._push_live(t, None, 0.0)
            return None

        pose = POSES[self.step]
        score, detail = pose["fn"](kps)

        if score is None:
            if self.t0 is not None:
                if self.t_lost is None:
                    self.t_lost = t
                elif t - self.t_lost > c["lose_sec"]:
                    self.t0 = None
                    self.t_lost = None
                    self.acc = []
            self._push_live(t, None, 0.0)
            return None

        self.t_lost = None
        if self.t0 is None:
            self.t0 = t
            self.acc = []
        self.acc.append((score, detail))
        if len(self.acc) > 90:
            self.acc = self.acc[-90:]

        held = t - self.t0
        prog = min(1.0, held / c["hold_sec"])
        recent = self.acc[-15:]
        avg_score = float(np.mean([x[0] for x in recent]))
        live = dict(step=self.step + 1, total=len(POSES), pose=pose["short"],
                    score=round(avg_score, 1), progress=prog)
        self._push_live(t, live, prog)

        if held >= c["hold_sec"]:
            last_score, last_detail = self.acc[-1]
            final = float(np.mean([x[0] for x in self.acc[-15:]]))
            self.idx += 1
            ev = PoseEvent(self.idx, pose, self.t0, t, round(final, 1), last_detail)
            self.results.append(dict(pose=pose["name"], short=pose["short"],
                                     score=round(final, 1), parts=last_detail["parts"],
                                     worst=last_detail["worst"]))
            self.step += 1
            self.t0 = None
            self.acc = []
            self.cooldown_until = t + c["cooldown"]
            if self.step >= len(POSES):
                self.finished_at = t
            return ev
        return None

    def _push_live(self, t, live, prog):
        if self.reporter is None:
            return
        if t - self._last_push < 1.0 / 15.0:
            return
        self._last_push = t
        try:
            self.reporter.on_frame(live, prog,
                                   step=self.step, total=len(POSES),
                                   summary=self.summary(), finished=self.finished)
        except Exception:
            pass

    def stats(self):
        return dict(count=len(self.results), step=self.step)


class ProtocolReporter:
    """供 web_stream2 直接替换（同 WorkoutReporter 接口）"""

    def __init__(self, store=None, voice=None, child_id=None, live_path="live.json",
                 target=4, action_type="posture", keep_recent=10, mode="rep"):
        self.store = store
        self.voice = voice
        self.child_id = child_id
        self.live_path = live_path
        self.target = len(POSES)
        self.action_type = action_type
        self.keep_recent = keep_recent
        self.mode = mode
        self.session_id = None
        self.scores = []
        self.hint = "准备开始：T字侧平举"
        self.last = None
        self.pb = 0.0
        self.infer_fps = 0.0
        self.infer_ms = 0.0
        self.det = False
        self._last_live_write = 0.0
        self._frame = None
        self._step = 0
        self._total = len(POSES)
        self._summary = None
        self._finished = False
        self._last_hint = ""
        if self.store is not None:
            if self.child_id is None:
                self.child_id = self.store.ensure_child("小朋友")
            self.pb = self.store.best_score(self.child_id, self.action_type)
            self.session_id = self.store.start_session(self.child_id, self.action_type)

    def on_frame(self, live, prog, step=0, total=4, summary=None, finished=False):
        self._step, self._total = step, total
        self._summary, self._finished = summary, finished
        self._frame = live
        if summary is not None and finished:
            self.hint = "筛查完成"
            return
        if live is None:
            p = POSES[min(step, len(POSES) - 1)]
            self.hint = "第%d项：%s" % (step + 1, p["tip"])
            return
        if prog < 1.0:
            self.hint = "保持 %d%%" % int(prog * 100)
        else:
            self.hint = "测量中"

    def restart(self):
        """外部触发（FPGA 按键）：结束当前会话并重新开始计分"""
        if self.store is not None and self.session_id is not None:
            try:
                self.store.finish_session(self.session_id, self.avg_score,
                                          self.count, self.best_score)
            except Exception:
                pass
        self.scores = []
        self.last = None
        self.hint = "准备开始：%s" % POSES[0]["tip"]
        self._summary = None
        self._finished = False
        self._frame = None
        self._step = 0
        self._total = len(POSES)
        self._last_hint = ""
        if self.store is not None:
            try:
                self.session_id = self.store.start_session(self.child_id, self.action_type)
            except Exception:
                pass
        self.write_live(force=True)
        return True

    def on_rep(self, ev):
        self.scores.append(ev.score)
        _stars, _lv = level_of(ev.score)
        self.last = dict(idx=ev.idx, pose=ev.pose_name, score=ev.score,
                         parts={k: round(v, 1) for k, v in ev.parts.items()},
                         worst=ev.worst, t=round(time.time(), 3),
                         level=_lv, hint=self.hint)
        worst = ev.worst
        if ev.parts[worst] < 70.0:
            hint = HINT.get(worst, "再试一次")
        elif min(ev.parts.values()) >= 85.0:
            hint = "非常好"
        else:
            hint = "不错"
        self.hint = hint
        _stars0, _lv0 = level_of(ev.score)
        if self.voice is not None and hint != self._last_hint:
            try:
                self.voice.say(hint, t_ref=ev.t1)
            except Exception:
                pass
            self._last_hint = hint
        if (self.store is not None and self.session_id is not None):
            try:
                self.store.add_action(self.session_id, ev.idx, ev.score,
                                      knee_min=0.0, lean_deg=float(ev.metrics.get("lean", 0.0)),
                                      align_dx=0.0, issue=HINT.get(worst, ""),
                                      duration_ms=int(ev.duration * 1000))
                self.store.update_session_live(self.session_id, self.avg_score,
                                               self.count, self.pb)
            except Exception:
                pass
        self.write_live(force=True)
        return self.last

    def on_window(self, a, b, c, t_ref=None):
        return dict(score=0.0, stars=0, level="需改进", hint=self.hint)

    @property
    def count(self):
        return len(self.scores)

    @property
    def avg_score(self):
        return round(sum(self.scores) / len(self.scores), 1) if self.scores else 0.0

    @property
    def best_score(self):
        return round(max(self.scores), 1) if self.scores else 0.0

    def recent(self):
        sel = self.scores[-self.keep_recent:]
        base = max(0, len(self.scores) - len(sel))
        return [dict(x=base + i + 1, y=v) for i, v in enumerate(sel)]

    def live_state(self):
        summary = self._summary
        if self._finished and summary:
            score = summary["total"]
            stars, lv = level_of(score)
            name = "筛查完成 %d/%d" % (len(summary["scores"]), self._total)
            hint = "最弱：%s %d分" % (summary["worst"], summary["worst_score"])
        elif self._frame is not None:
            score = self._frame["score"]
            stars, lv = level_of(score)
            name = "%s %d/%d" % (POSES[min(self._step, len(POSES) - 1)]["short"], self._step + 1, self._total)
            hint = self.hint
        else:
            score, stars, lv = 0.0, 0, "准备中"
            name = "第%d项" % (min(self._step, len(POSES) - 1) + 1)
            hint = self.hint
        return dict(score=round(score, 1), score_pct=round(clamp(score), 1), stars=stars,
                    count=len(self.scores), target=self._total,
                    avg=self.avg_score, best=self.best_score, pb=round(self.pb, 1),
                    det=bool(self.det), hint=hint,
                    action=self.action_type, action_name=name, level=lv,
                    step=self._step + 1, total=self._total,
                    progress=round((self._frame or {}).get("progress", 0.0), 3),
                    summary=(summary if self._finished else None),
                    infer_fps=round(self.infer_fps, 1), infer_ms=round(self.infer_ms, 1),
                    recent=self.recent(), uptime=round(time.time(), 1))

    def write_live(self, force=False, every=1.0):
        now = time.time()
        if not force and (now - self._last_live_write) < every:
            return False
        self._last_live_write = now
        try:
            d = os.path.dirname(os.path.abspath(self.live_path))
            if d and not os.path.isdir(d):
                os.makedirs(d, exist_ok=True)
            tmp = self.live_path + ".tmp"
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump(self.live_state(), f, ensure_ascii=False)
            os.replace(tmp, self.live_path)
            return True
        except Exception:
            return False

    def finish(self):
        if self.store is not None and self.session_id is not None:
            try:
                self.store.finish_session(self.session_id, self.avg_score,
                                          self.count, self.best_score)
            except Exception:
                pass
        self.write_live(force=True)
        return dict(session_id=self.session_id, count=self.count,
                    avg=self.avg_score, best=self.best_score, pb=round(self.pb, 1))