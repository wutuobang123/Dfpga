"""web_stream2.py - 三线程实时流水线（采集 / 推理 / 编码推流解耦）

为什么这样拆：
  NPU 单帧约 200ms（yolov8n-pose @640 on RK3568），端到端串行只有 4~5fps。
  但"画面流畅"和"关键点更新率"是两件事：
    - 采集线程：始终只保留最新一帧（绝不排队，避免延迟累积）
    - 推理线程：总是取最新帧推理，跳过的帧直接丢（不补算）
    - 编码线程：按目标 fps 出图推 MJPEG，画的是最新帧 + 最新关键点
  这样 HDMI/网页的画面是 30fps 顺滑的，骨架约 5Hz 更新；评分用推理帧的真实 fps 计算。

用法:
    python3 web_stream2.py --source udp://0.0.0.0:8080
    python3 web_stream2.py --source test_motion.mp4 --http-port 8099
    python3 web_stream2.py --source camera:0 --no-draw
接口:
    GET /          MJPEG 实时画面
    GET /snapshot  单帧 JPEG
    GET /stats     运行状态 JSON
"""
import argparse
import json
import threading
import time

import cv2
import numpy as np

from config import INFERENCE, SCORING
from postprocess import FrameBuffer, evaluate_all, total_score
from proto import ScoreReport
from source import open_source

# 分开 import：任何一个辅助模块坏掉，都不该把推理一起拖死
try:
    from infer import PoseInfer, draw_skeleton
except Exception as _e:                # 允许 --no-infer 时缺依赖
    PoseInfer = None
    draw_skeleton = None
    print("[Main] infer 不可用: %s" % _e, flush=True)

try:
    from preprocess import draw_report_on_frame
except Exception as _e:                # 老模块依赖 config.VIS，缺了也不影响主流程
    draw_report_on_frame = None
    print("[Main] draw_report_on_frame 不可用（不影响主流程）: %s" % _e, flush=True)


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--source", default="udp://0.0.0.0:8080")
    p.add_argument("--model", default=INFERENCE["model"])
    p.add_argument("--imgsz", type=int, default=INFERENCE["imgsz"])
    p.add_argument("--conf", type=float, default=INFERENCE["conf"])
    p.add_argument("--http-host", default="0.0.0.0")
    p.add_argument("--http-port", type=int, default=8080)
    p.add_argument("--jpeg-quality", type=int, default=70)
    p.add_argument("--out-fps", type=float, default=30.0, help="推流/显示帧率")
    p.add_argument("--no-infer", action="store_true", help="只推原始画面，不跑 NPU")
    p.add_argument("--no-draw", action="store_true", help="不叠加骨架/OSD")
    p.add_argument("--predict", action="store_true", default=True, help="关键点做速度外推减少视觉滞后")
    p.add_argument("--no-predict", dest="predict", action="store_false")
    p.add_argument("--buffer-seconds", type=float, default=8.0)
    p.add_argument("--report-every", type=float, default=0.5)
    p.add_argument("--save-dir", default="")
    p.add_argument("--infer-impl", default="v2", choices=["v2", "g3", "split"],
                   help="v2=单输出模型；g3=三路拆分模型（拆开后 cls 才不被量化压死）")
    p.add_argument("--rules", default="v2", choices=["v1", "v2"],
                   help="v1=旧5项加分制(1.6分) v2=十动作扣分制(甲组2.2分)")
    p.add_argument("--group", default="甲", choices=["甲", "乙"])
    p.add_argument("--gender", default="男", choices=["男", "女"])
    p.add_argument("--tier-mode", default="fig3", choices=["fig3", "four"],
                   help="fig3=FIG三档(可查依据) four=四档细化(项目标定)")
    p.add_argument("--profile", default="gym", choices=["gym", "squat", "arm", "posture", "protocol"],
                   help="gym=十规定动作  squat=深蹲计数  arm=双臂上举计数")
    p.add_argument("--db", default="", help="SQLite 路径（空=不存库）")
    p.add_argument("--live", default="live.json", help="网页实时状态文件")
    p.add_argument("--voice", action="store_true", help="开启语音播报")
    p.add_argument("--voice-dir", default="voice", help="录音 wav 目录")
    p.add_argument("--target", type=int, default=15, help="目标次数（OSD 的 /N）")
    p.add_argument("--degrade", default="none", choices=["none", "fixed"],
                   help="none=膝角状态机计数  fixed=定长窗口降级（不计数，只对最近3秒打分，方案 §3.2）")
    p.add_argument("--osd-debug", action="store_true", help="OSD 底部显示帧率调试行")
    p.add_argument("--send-fpga", action="store_true",
                   help="把骨架+评分画成 1bit 位图回传 FPGA（SK01 协议，UDP）")
    p.add_argument("--fpga-host", default="192.168.1.10")
    p.add_argument("--fpga-port", type=int, default=8081)
    p.add_argument("--ovl-w", type=int, default=320, help="回传位图宽")
    p.add_argument("--ovl-h", type=int, default=240, help="回传位图高")
    p.add_argument("--ovl-fps", type=float, default=15.0, help="回传帧率")
    return p.parse_args()


class Pipeline:
    def __init__(self, args):
        self.args = args
        self.running = True

        self.frame_lock = threading.Lock()
        self.frame = None
        self.frame_seq = 0
        self.frame_t = 0.0

        self.res_lock = threading.Lock()
        self.res = None            # 最近一次推理结果 list[dict]
        self.res_prev = None       # 上一次结果（用于外推）
        self.res_t = 0.0
        self.res_t_prev = 0.0
        self.res_modules = []      # [(dx/dt, dy/dt)] 外推用

        self.report = []
        self.report_total = 0.0
        self.rules_full = SCORING["total_full"]
        if args.rules == "v2":
            import rules_v2 as _r2
            from scoring_rules import full_score
            _r2.set_tier_mode(args.tier_mode)
            self.rules_full = full_score(args.group)
            print("[Rules] 扣分制 v2  组别=%s  满分=%.1f  档位=%s（fig3=FIG三档/four=四档）"
                  % (args.group, self.rules_full, args.tier_mode), flush=True)
        self.report_t = 0.0

        self.stats = {
            "rx_frames": 0, "infer_frames": 0, "out_frames": 0,
            "rx_fps": 0.0, "infer_fps": 0.0, "out_fps": 0.0,
            "infer_ms": 0.0, "encode_ms": 0.0, "det": False, "conf": 0.0,
            "fid": 0, "skipped": 0, "uptime_s": 0.0,
        }
        self.t0 = time.time()

        self.rx = open_source(args.source)
        self._ctrl_seen = 0
        self._ctrl_t = 0.0
        self.infer = None
        if not args.no_infer:
            if PoseInfer is None:
                raise RuntimeError("infer.py 不可用")
            if args.infer_impl == 'split':
                from infer_split import PoseInferSplit as _PoseInferSplit
                self.infer = _PoseInferSplit(rknn_path=args.model, imgsz=args.imgsz, conf=args.conf)
            elif args.infer_impl == 'g3':
                from infer_g3 import PoseInferG3 as _PoseInferG3
                self.infer = _PoseInferG3(rknn_path=args.model, imgsz=args.imgsz, conf=args.conf)
            else:
                self.infer = PoseInfer(rknn_path=args.model, imgsz=args.imgsz, conf=args.conf)

        self.buffer = FrameBuffer(max_frames=2000, max_seconds=args.buffer_seconds)
        self._smooth = None
        self.img_hw = None
        self.latest_jpg = None
        self.jpg_lock = threading.Lock()

        # ---- 回传 FPGA（骨架 + 评分位图）：独立线程 + 单调时钟节拍 ----
        self.ovl_sock = None
        self.ovl_period = 1.0 / max(args.ovl_fps, 1.0)
        self.last_ovl_t = 0.0
        self.ovl_fid = 0
        self.ovl_intervals = []
        self.stats["ovl_fps"] = 0.0
        self.stats["ovl_p50"] = 0.0
        self.stats["ovl_p95"] = 0.0
        self.stats["ovl_render_ms"] = 0.0
        if args.send_fpga:
            import socket as _socket
            from skeleton_out import render_mask, build_packets
            from osd_layout import OsdLayout
            self._render_mask = render_mask
            self._build_packets = build_packets
            self._osd = OsdLayout(size=(args.ovl_w, args.ovl_h))
            self._pb = 0.0
            self.ovl_sock = _socket.socket(_socket.AF_INET, _socket.SOCK_DGRAM)
            self.ovl_sock.setsockopt(_socket.SOL_SOCKET, _socket.SO_SNDBUF, 1 << 20)
            self.ovl_addr = (args.fpga_host, args.fpga_port)
            print("[Main] 骨架回传 -> %s:%d  %dx%d 1bit  @%.0ffps（独立线程 + 单调时钟节拍）"
                  % (args.fpga_host, args.fpga_port, args.ovl_w, args.ovl_h, args.ovl_fps),
                  flush=True)

        # ---- 深蹲 profile：平滑 / 分段 / 打分播报存库 ----
        self._smoother = self._segmenter = self._reporter = None
        if args.profile in ("squat", "arm", "posture", "protocol"):
            from preprocess_kps import KpsSmoother
            if args.profile == "arm":
                from arm_detect import ArmRaiseSegmenter as SquatSegmenter
                from arm_detect import ArmWindowFallback as FixedWindowFallback
                from arm_detect import ArmReporter as WorkoutReporter
            elif args.profile == "posture":
                from posture import PostureMeasurer as SquatSegmenter
                from posture import PostureMeasurer as FixedWindowFallback
                from posture import PostureReporter as WorkoutReporter
            elif args.profile == "protocol":
                from posture_protocol import ProtocolMeasurer as SquatSegmenter
                from posture_protocol import ProtocolMeasurer as FixedWindowFallback
                from posture_protocol import ProtocolReporter as WorkoutReporter
            else:
                from action_segment import SquatSegmenter
                from action_segment import FixedWindowFallback
                from workout import WorkoutReporter
            self._smoother = KpsSmoother(fps=30.0)
            self._action_label = {"arm": "双臂上举", "posture": "双臂体态"}.get(args.profile, "深蹲")
            self._fallback = None
            if args.degrade == "fixed":
                self._fallback = FixedWindowFallback(seconds=3.0)
                self._segmenter = None
                print("[Main] 降级模式: 定长窗口 3 秒（不计数，分数每秒刷新）", flush=True)
            else:
                self._segmenter = SquatSegmenter()
            st = None
            if args.db:
                from store import Store
                st = Store(args.db)
            vp = None
            if args.voice:
                from voice import VoicePlayer
                vp = VoicePlayer(wav_dir=args.voice_dir, enabled=True)
            self._reporter = WorkoutReporter(store=st, voice=vp, live_path=args.live,
                                             target=args.target, action_type=args.profile,
                                             mode=("window" if args.degrade == "fixed" else "rep"))
            if args.profile in ("posture", "protocol") and self._segmenter is not None:
                self._segmenter.reporter = self._reporter   # 让测量器每帧推进度给 OSD
            print("[Main] profile=%s  目标 %d 次  DB=%s  语音=%s  live=%s"
                  % (args.profile, args.target, args.db or "关", args.voice, args.live), flush=True)

    def _overlay_loop(self):
        """OSD 独立线程：固定节拍渲染 + 打包 + 发送，绝不与收流/推理/推流共线程

        要点（按 FPGA 侧要求）：
          - time.monotonic() 对齐绝对时刻，不用 sleep 累加（会漂移）
          - 落后就重置节拍，不补偿式连发（避免突发抖动）
          - 关键点用"最新值覆盖"，无队列（丢帧不丢延迟）
          - 7 个包背靠背发完，中间不插其它耗时操作
        """
        period = self.ovl_period
        next_t = time.monotonic()
        last_t = None
        while self.running:
            with self.res_lock:
                res = self.res
                res_t = self.res_t
            res_draw = self._predict_kps(res, res_t) if self.args.predict else res

            t_r = time.monotonic()
            try:
                self._send_overlay(res_draw)
            except Exception as e:
                print("[OVL] 发送失败: %s" % e, flush=True)
                time.sleep(0.2)
                continue
            self.stats["ovl_render_ms"] = (time.monotonic() - t_r) * 1000.0
            # 让 live.json 在"没人"时也保持刷新（write_live 内部限流 1s）
            if self._reporter is not None:
                try:
                    self._reporter.write_live()
                except Exception:
                    pass

            now = time.monotonic()
            if last_t is not None:
                self.ovl_intervals.append(now - last_t)
                if len(self.ovl_intervals) > 240:
                    del self.ovl_intervals[:-240]
            last_t = now

            # 统计（每秒刷新一次）
            if len(self.ovl_intervals) >= 10:
                iv = sorted(self.ovl_intervals[-120:])
                mean_iv = sum(iv) / len(iv)
                self.stats["ovl_fps"] = 1.0 / mean_iv if mean_iv > 0 else 0.0
                self.stats["ovl_p50"] = iv[len(iv) // 2] * 1000.0
                self.stats["ovl_p95"] = iv[int(len(iv) * 0.95)] * 1000.0

            next_t += period
            dt = next_t - time.monotonic()
            if dt > 0:
                time.sleep(dt)
            else:
                next_t = time.monotonic()   # 落后就重置，不连发补帧

    def _build_osd_state(self, kps):
        """把流水线状态映射成 OSD 布局所需字段（布局见 osd_layout.py）"""
        dbg = ""
        if self.args.osd_debug:
            dbg = "%.0ffps npu%.0fms" % (self.stats["infer_fps"], self.stats["infer_ms"])

        # 深蹲 profile：直接用"每次动作打分"的结果（方案 §4/§6）
        if self._reporter is not None:
            ls = self._reporter.live_state()
            return dict(score=ls["score"], full=100.0, stars=ls["stars"], action=ls.get("action_name", self._action_label),
                        count=ls["count"], target=ls["target"], pb=ls["pb"],
                        hint=(ls["hint"] if self.stats.get("det") else ""),
                        det=bool(self.stats.get("det")), debug=dbg)

        full = float(self.rules_full or 100.0)
        pct = 0.0 if full <= 0 else max(0.0, min(100.0, self.report_total / full * 100.0))
        self._pb = max(self._pb, pct)

        stars = 3 if pct >= 85 else (2 if pct >= 70 else (1 if pct >= 55 else 0))

        # 已完成/总动作数；提示取"最拖后腿"的那一项
        done, total, worst = 0, len(self.report) or 0, None
        for r in self.report:
            if r.get("complete"):
                done += 1
            elif worst is None:
                worst = r

        hint = ""
        if pct < 85 and worst is not None:
            no = worst.get("no")
            hint = {
                2: "开度不够", 3: "手不能撑地", 4: "倒立要竖直",
                5: "俯卧撑再做", 6: "保持时间不足", 7: "左右都要踢", 9: "跳两次",
            }.get(no, "再试一次")
        elif self.report and pct >= 85:
            hint = "很好，保持住"
        if not kps or not self.stats.get("det"):
            hint = ""

        dbg = ""
        if self.args.osd_debug:
            dbg = "%.0ffps npu%.0fms" % (self.stats["infer_fps"], self.stats["infer_ms"])

        return dict(score=pct, full=100.0, stars=stars,
                    action="动作", count=done, target=total,
                    pb=self._pb, hint=hint,
                    det=bool(self.stats.get("det")), debug=dbg)

    def _send_overlay(self, res_draw):
        """骨架 + OSD 布局 -> 1bit 位图 -> SK01 v2 分片回传"""
        kps = res_draw[0]["kps"] if res_draw else None
        size = (self.args.ovl_w, self.args.ovl_h)
        src = (self.img_hw[1], self.img_hw[0]) if self.img_hw else (640, 480)

        sk = self._render_mask(kps, size=size, src_size=src)      # 骨架层
        osd = self._osd.render(self._build_osd_state(kps))        # 布局层
        mask = np.maximum(sk, osd)                                # 1bit 叠加

        pkts = self._build_packets(mask, self.ovl_fid)   # v2 分片：7 个 ≤1420 字节的包
        self.ovl_fid = (self.ovl_fid + 1) & 0xFFFF
        # 每个分片连发两遍（链路有丢包时提高整帧到达率），间隔 200us 避免突发
        for _rep in range(2):
            for _i, pkt in enumerate(pkts):
                self.ovl_sock.sendto(pkt, self.ovl_addr)
                if _i + 1 < len(pkts):
                    time.sleep(0.0002)
        self.stats["ovl_sent"] = self.stats.get("ovl_sent", 0) + 1
        self.last_ovl_t = time.time()

    # ---------- 线程 1：采集 ----------
    def _capture_loop(self):
        last_t = time.time()
        win = 0
        fails = 0
        while self.running:
            ok, frame = self.rx.read()
            if not ok:
                fails += 1
                if fails % 500 == 1:
                    print("[Capture] 等待视频源恢复（已失败 %d 次）" % fails, flush=True)
                time.sleep(0.02)
                continue
            fails = 0
            with self.frame_lock:
                self.frame = frame
                self.frame_seq += 1
                self.frame_t = time.time()
                self.img_hw = frame.shape[:2]
            win += 1
            now = time.time()
            if now - last_t >= 1.0:
                self.stats["rx_fps"] = win / (now - last_t)
                win = 0
                last_t = now
            self.stats["rx_frames"] += 1

            # 诊断：收到控制包时该帧行数会少 1（479 而不是 480）
            _rows = int(getattr(self.rx, "last_frame_rows", 480))
            if _rows < 479:
                _t = time.time()
                if _t - getattr(self, "_rows_t", 0.0) > 1.0:
                    self._rows_t = _t
                    print("[Ctrl] 检测到控制包：本帧行数=%d，ctrl_reset=%d"
                          % (_rows, getattr(self.rx, "ctrl_reset", 0)), flush=True)

            # FPGA 按键重启：收到控制包 -> 重新开始计分（限速 1.5 秒）
            try:
                _cs = getattr(self.rx, "ctrl_reset", 0)
                if _cs != self._ctrl_seen:
                    self._ctrl_seen = _cs
                    _now = time.time()
                    if _now - self._ctrl_t > 1.5:
                        self._ctrl_t = _now
                        self._restart_session()
            except Exception as _e:
                print("[Ctrl] err: %s" % _e, flush=True)

    def _restart_session(self):
        """FPGA 按键触发：重置测量器 + 新建会话"""
        try:
            if self._segmenter is not None:
                self._segmenter.reset()
            if self._reporter is not None:
                self._reporter.restart()
            print("[Ctrl] 收到 FPGA 按键，重新开始计分", flush=True)
        except Exception as e:
            print("[Ctrl] 重启失败: %s" % e, flush=True)

    # ---------- 线程 2：推理 ----------
    def _infer_loop(self):
        last_seq = -1
        last_t = time.time()
        win = 0
        while self.running:
            with self.frame_lock:
                frame = self.frame
                seq = self.frame_seq
            if frame is None or seq == last_seq:
                time.sleep(0.003)
                continue
            if last_seq >= 0:
                self.stats["skipped"] += max(0, seq - last_seq - 1)
            last_seq = seq

            t0 = time.time()
            try:
                res = self.infer.run(frame)
            except Exception as e:
                print("[Infer] error: %s" % e, flush=True)
                time.sleep(0.2)
                continue
            dt = time.time() - t0
            self.stats["infer_ms"] = dt * 1000.0

            with self.res_lock:
                self.res_prev, self.res_t_prev = self.res, self.res_t
                self.res, self.res_t = res, time.time()
                if res and self.res_prev:
                    self.res_modules = [(res[0]["kps"][:, :2] - self.res_prev[0]["kps"][:, :2])]
                else:
                    self.res_modules = []
            self.stats["infer_frames"] += 1
            self.stats["det"] = bool(res)
            self.stats["conf"] = float(res[0]["conf"]) if res else 0.0

            # 评分缓冲：只推有检测的帧（无检测帧不进入动作判定）
            # 关键点先做 EMA 平滑：NPU 关键点本身有 ±几像素抖动，
            # 不平滑会把抖动喂成"起坐/跳跃"的假周期。
            if res:
                cur = res[0]["kps"]
                if self._smooth is None:
                    self._smooth = cur.copy()
                else:
                    a = 0.45
                    m = (cur[:, 2] > 0.3) & (self._smooth[:, 2] > 0.3)
                    self._smooth[:, :2][m] = a * cur[:, :2][m] + (1.0 - a) * self._smooth[:, :2][m]
                    self._smooth[~m, :2] = cur[~m, :2]
                    self._smooth[:, 2] = cur[:, 2]
                self.buffer.push(self._smooth, t=time.time())

                # ---- 深蹲 profile：One Euro 平滑 → 状态机分段 → 一次动作完整处理 ----
                if self._reporter is not None:
                    try:
                        now_t = time.time()
                        k = self._smoother(self._smooth, now_t) if self._smoother is not None else self._smooth
                        if self._fallback is not None:
                            self._fallback.push(k, now_t)
                            if now_t - self.stats.get("_win_t", 0.0) >= 1.0:
                                m = self._fallback.metrics()
                                if m["n"] >= 3 and m["knee_min"] is not None:
                                    w = self._reporter.on_window(m["knee_min"], m["lean"],
                                                                 m["align_dx"], t_ref=now_t)
                                    self.stats["_win_t"] = now_t
                                    if self.stats.get("reps", 0) < 1:
                                        self.stats["reps"] = 0
                                        print("[Win] %.1f 分 (%s) 最低膝角 %.1f  %s"
                                              % (w["score"], w["level"], w["knee_min"], w["hint"]), flush=True)
                        else:
                            ev = self._segmenter.feed(k, now_t) if self._segmenter is not None else None
                            if ev is not None:
                                rec = self._reporter.on_rep(ev)
                                self.stats["reps"] = self._reporter.count
                                print("[Rep] #%d %.1f 分 (%s) %s" % (rec["idx"], rec["score"], rec["level"], rec["hint"]),
                                      flush=True)
                        self._reporter.infer_fps = self.stats["infer_fps"]
                        self._reporter.infer_ms = self.stats["infer_ms"]
                        self._reporter.det = bool(res)
                        self._reporter.write_live()
                    except Exception as e:
                        if self.stats.get("reps", 0) < 3:
                            print("[Rep] 处理异常: %s" % e, flush=True)

            now = time.time()
            win += 1
            if now - last_t >= 1.0:
                self.stats["infer_fps"] = win / (now - last_t)
                win = 0
                last_t = now

            # squat profile 用自己的分段器打分，不需要再对整段缓冲跑十动作评估
            # （那是对 8 秒×30fps 的全部帧做 7 个检测器，纯 Python，实测把推理从 26fps 拖到 16fps）
            if self._reporter is not None:
                self.report_t = now
            elif now - self.report_t >= self.args.report_every and len(self.buffer) >= 4:
                fps_eff = self.stats["infer_fps"] or (1.0 / max(dt, 1e-3))
                if self.args.rules == "v2":
                    import rules_v2 as _r2
                    self.report = _r2.evaluate_10(self.buffer.frames, fps=max(fps_eff, 1.0),
                                                  img_hw=self.img_hw, gender=self.args.gender)
                    self.report_total = _r2.total(self.report)
                else:
                    self.report = evaluate_all(self.buffer.frames, fps=max(fps_eff, 1.0), img_hw=self.img_hw)
                    self.report_total = total_score(self.report)
                self.report_t = now

    # ---------- 关键点外推 ----------
    def _predict_kps(self, res, res_t):
        """按上一帧速度外推到现在，减少"骨架比画面慢 200ms"的违和感"""
        if not self.args.predict or not res or not self.res_modules:
            return res
        prev = self.res_prev
        if not prev:
            return res
        dt = res_t - self.res_t_prev
        age = time.time() - res_t
        if dt <= 0 or age <= 0:
            return res
        k = min(age / dt, 0.6)          # 最多外推 0.6 个推理间隔
        vel = self.res_modules[0]
        out = dict(res[0])
        kps = res[0]["kps"].copy()
        moving = kps[:, 2] > 0.3
        kps[moving, 0] += vel[moving, 0] * k
        kps[moving, 1] += vel[moving, 1] * k
        out["kps"] = kps
        return [out]

    # ---------- 线程 3：出图 ----------
    def encode_loop(self):
        period = 1.0 / max(self.args.out_fps, 1.0)
        last_t = time.time()
        win = 0
        enc_params = [cv2.IMWRITE_JPEG_QUALITY, self.args.jpeg_quality]
        n = 0
        while self.running:
            t_frame = time.time()

            with self.frame_lock:
                frame = self.frame
                fid = self.frame_seq
            if frame is None:
                time.sleep(0.01)
                continue

            disp = frame.copy()
            with self.res_lock:
                res = self.res
                res_t = self.res_t
            res_draw = self._predict_kps(res, res_t)

            if not self.args.no_draw:
                if res_draw and draw_skeleton:
                    disp = draw_skeleton(disp, res_draw)

                age = (time.time() - res_t) * 1000.0 if res_t else 9999
                lines = [
                    "rx %4.1f  infer %4.1f  out %4.1f fps" % (
                        self.stats["rx_fps"], self.stats["infer_fps"], self.stats["out_fps"]),
                    "npu %5.1f ms  kpt_age %4.0f ms  det %s" % (
                        self.stats["infer_ms"], age, "Y" if self.stats["det"] else "N"),
                ]
                for i, s in enumerate(lines):
                    cv2.putText(disp, s, (10, 70 + i * 22), cv2.FONT_HERSHEY_SIMPLEX,
                                0.55, (0, 255, 255), 1)
                # 总分条
                bar = int(min(self.report_total / max(self.rules_full, 1e-3), 1.0) * 220)
                cv2.rectangle(disp, (10, 120), (10 + bar, 134), (0, 200, 255), -1)
                cv2.rectangle(disp, (10, 120), (230, 134), (255, 255, 255), 1)
                cv2.putText(disp, "total %.2f/%.1f" % (self.report_total, self.rules_full),
                            (10, 155), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1)

            t0 = time.time()
            ok, jpg = cv2.imencode(".jpg", disp, enc_params)
            if ok:
                with self.jpg_lock:
                    self.latest_jpg = jpg.tobytes()
            self.stats["encode_ms"] = (time.time() - t0) * 1000.0

            if self.args.save_dir and n % 30 == 0:
                cv2.imwrite("%s/f%06d.jpg" % (self.args.save_dir, n), disp)
            n += 1
            self.stats["out_frames"] = n
            win += 1
            now = time.time()
            if now - last_t >= 1.0:
                self.stats["out_fps"] = win / (now - last_t)
                win = 0
                last_t = now

            sleep = period - (time.time() - t_frame)
            if sleep > 0:
                time.sleep(sleep)

    # ---------- HTTP ----------
    def start_http(self):
        from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
        pipe = self

        class H(BaseHTTPRequestHandler):
            protocol_version = "HTTP/1.1"

            def log_message(self, *a):
                pass

            def do_GET(self):
                path = self.path.split("?")[0]
                if path == "/stats":
                    body = json.dumps(pipe.snapshot_stats(), ensure_ascii=False).encode()
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                    return
                if path == "/snapshot":
                    with pipe.jpg_lock:
                        jpg = pipe.latest_jpg
                    if jpg is None:
                        self.send_response(503)
                        self.end_headers()
                        return
                    self.send_response(200)
                    self.send_header("Content-Type", "image/jpeg")
                    self.send_header("Content-Length", str(len(jpg)))
                    self.end_headers()
                    self.wfile.write(jpg)
                    return
                # 默认走 MJPEG
                self.send_response(200)
                self.send_header("Age", "0")
                self.send_header("Cache-Control", "no-cache, private")
                self.send_header("Pragma", "no-cache")
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=FRAME")
                self.end_headers()
                try:
                    while pipe.running:
                        with pipe.jpg_lock:
                            jpg = pipe.latest_jpg
                        if jpg is None:
                            time.sleep(0.02)
                            continue
                        self.wfile.write(b"--FRAME\r\n")
                        self.wfile.write(b"Content-Type: image/jpeg\r\n\r\n")
                        self.wfile.write(jpg)
                        self.wfile.write(b"\r\n")
                        time.sleep(1.0 / max(pipe.args.out_fps, 1.0))
                except Exception:
                    return

        class Srv(ThreadingHTTPServer):
            daemon_threads = True

        srv = Srv((self.args.http_host, self.args.http_port), H)
        threading.Thread(target=srv.serve_forever, daemon=True).start()
        return srv

    def snapshot_stats(self):
        d = dict(self.stats)
        d["uptime_s"] = round(time.time() - self.t0, 1)
        d["buffer"] = len(self.buffer)
        d["total_score"] = round(self.report_total, 2)
        d["report"] = self.report
        d["source"] = self.args.source
        return d

    def run(self):
        if not self.rx.open():
            print("[Main] 视频源打不开: %s" % self.args.source, flush=True)
            return 1
        if self.args.save_dir:
            import os
            os.makedirs(self.args.save_dir, exist_ok=True)

        print("[Main] source=%s  http=:%d  out=%dfps  infer=%s"
              % (self.args.source, self.args.http_port, int(self.args.out_fps),
                 "off" if self.args.no_infer else "on"), flush=True)

        threading.Thread(target=self._capture_loop, daemon=True).start()
        if self.infer is not None:
            threading.Thread(target=self._infer_loop, daemon=True).start()
        if self.ovl_sock is not None:
            threading.Thread(target=self._overlay_loop, daemon=True).start()
        self.start_http()
        print("[Main] MJPEG -> http://%s:%d/   stats -> /stats   snapshot -> /snapshot"
              % (self.args.http_host, self.args.http_port), flush=True)

        try:
            self.encode_loop()
        except KeyboardInterrupt:
            pass
        finally:
            self.running = False
            time.sleep(0.1)
            self.rx.close()
            if self.infer is not None:
                self.infer.release()
            if self._reporter is not None:
                fin = self._reporter.finish()
                print("\n[Main] 训练结束: 次数 %d  平均 %.1f  最好 %.1f  PB %.1f  (session=%s)"
                      % (fin["count"], fin["avg"], fin["best"], fin["pb"], fin["session_id"]), flush=True)
            print("\n[Main] 最终评分: %.2f / %.1f" % (self.report_total, self.rules_full), flush=True)
            for r in self.report:
                print("  [%s] %-8s %-22s %.2f"
                      % ("OK" if r["complete"] else "NG", r["action"], r["quality"], r["score"]), flush=True)
        return 0


def main():
    args = parse_args()
    return Pipeline(args).run()


if __name__ == "__main__":
    raise SystemExit(main())














