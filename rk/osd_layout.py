"""osd_layout.py - OSD 布局渲染器（320×240 1bit，按《评分系统方案》第 6 节）

布局（320×240）：
    ┌────────────────────────────────┐
    │ SCORE 87 ★★★        深蹲 12/15 │  y=4   左：分数+星级   右：动作+计数
    │                                │
    │          （骨架叠加区）          │  留白，不放文字
    │                                │
    │ PB 92                          │  y=214 左下：个人最佳
    │        膝盖对准脚尖              │  y=190 中下：纠错提示（大字）
    └────────────────────────────────┘

1bit 只有"画/透明"两态：靠位置和字号区分层级，不靠颜色。
中文：有 CJK 字体就用中文，没有就自动退回英文提示（不会画出豆腐块）。

API:
    osd = OsdLayout()
    mask = osd.render(state)        # state 见 DEFAULT_STATE
    mask = osd.render(st, debug=True)   # debug=True 时底部加一行帧率
"""
import os

import cv2
import numpy as np

W, H = 320, 240

# 字体（中文优先；找不到就用英文提示语）
CJK_FONT_CANDIDATES = [
    "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
    "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc",
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf",
    "/usr/share/fonts/truetype/arphic/uming.ttc",
    "C:/Windows/Fonts/msyh.ttc",
    "C:/Windows/Fonts/simhei.ttf",
]

# 中文提示 -> 英文（没有中文字体时用）
HINT_EN = {
    "蹲得再深一点": "DEEPER",
    "挺直腰背": "BACK STRAIGHT",
    "膝盖对准脚尖": "KNEES OUT",
    "非常好，保持住": "GREAT! HOLD",
    "很好，保持住": "GOOD, HOLD",
    "再试一次": "TRY AGAIN",
    "未检测到人": "NO PERSON",
    "分数不足": "LOW SCORE",
    "开度不够": "SPLIT WIDER",
    "腿要伸直": "LEGS STRAIGHT",
    "倒立要竖直": "STAND VERTICAL",
    "保持时间不足": "HOLD LONGER",
    "左右都要踢": "BOTH LEGS",
    "手不能撑地": "NO HANDS",
    "跳两次": "JUMP TWICE",
}

DEFAULT_STATE = dict(
    score=0.0,          # 当前分数（0~100 或 0~2.2 由 full 决定）
    full=100.0,         # 满分
    stars=0,            # 0~3
    action="",          # 动作名，如 "深蹲"
    count=0,            # 已完成次数
    target=0,           # 目标次数（0=不显示 /N）
    pb=0.0,             # 个人最佳
    hint="",            # 纠错提示（中文或英文都行）
    det=False,          # 是否检测到人
    debug="",           # 底部调试行（空=不画）
)


class OsdLayout:
    def __init__(self, size=(W, H), cjk_font=None):
        self.w, self.h = size
        self.font_path = cjk_font or self._find_cjk_font()
        self._pil = None
        if self.font_path:
            try:
                from PIL import Image, ImageDraw, ImageFont
                self._pil = (Image, ImageDraw, ImageFont)
            except Exception:
                self.font_path = None
        self.cjk = bool(self.font_path)
        self._cache_key = None
        self._cache_mask = None
        print("[OSD] 布局渲染 中文=%s (%s)" % (self.cjk, self.font_path or "无，用英文提示"),
              flush=True)

    @staticmethod
    def _find_cjk_font():
        for p in CJK_FONT_CANDIDATES:
            if os.path.exists(p):
                return p
        return None

    # ---------- 基础绘制 ----------
    def _text(self, mask, s, xy, scale, thick=1, align="left", cn=False):
        """写文字。align: left/right/center。中文走 PIL，英文走 cv2"""
        if not s:
            return mask
        s = str(s)
        if cn and self._pil:
            return self._text_pil(mask, s, xy, scale, align)
        if cn and not self.cjk:
            s = HINT_EN.get(s, s)
        font = cv2.FONT_HERSHEY_SIMPLEX
        (tw, th), _ = cv2.getTextSize(s, font, scale, thick)
        x, y = xy
        if align == "right":
            x -= tw
        elif align == "center":
            x -= tw // 2
        cv2.putText(mask, s, (int(x), int(y)), font, scale, 255, thick, cv2.LINE_AA)
        return mask

    def _text_pil(self, mask, s, xy, scale, align):
        Image, ImageDraw, ImageFont = self._pil
        px = max(10, int(round(scale * 26)))
        try:
            font = ImageFont.truetype(self.font_path, px)
        except Exception:
            return mask
        img = Image.fromarray((mask > 127).astype(np.uint8) * 255)
        d = ImageDraw.Draw(img)
        try:
            l, t, r, b = d.textbbox((0, 0), s, font=font)
            tw = r - l
        except Exception:
            tw = len(s) * px
        x, y = xy
        if align == "right":
            x -= tw
        elif align == "center":
            x -= tw // 2
        d.text((int(x), int(y) - px), s, font=font, fill=255)
        out = (np.asarray(img) > 127).astype(np.uint8) * 255
        mask[:, :] = np.maximum(mask, out)      # 原地写回：PIL 返回新数组，不写回就"画了个寂寞"
        return mask

    def _star(self, mask, cx, cy, r=6):
        """五角星（1bit 只有轮廓，画实心更醒目）"""
        pts = []
        for k in range(10):
            ang = -np.pi / 2 + k * np.pi / 5
            rad = r if k % 2 == 0 else r * 0.42
            pts.append([int(cx + rad * np.cos(ang)), int(cy + rad * np.sin(ang))])
        cv2.fillPoly(mask, [np.array(pts, np.int32)], 255)
        return mask

    def _hline(self, mask, y, x0=4, x1=None):
        x1 = self.w - 4 if x1 is None else x1
        cv2.line(mask, (x0, y), (x1, y), 255, 1, cv2.LINE_8)
        return mask

    # ---------- 主渲染 ----------
    def render(self, st=None, debug=False):
        state = dict(DEFAULT_STATE)
        if st:
            state.update(st)

        # 文字层缓存：分数/计数/提示不变时直接复用（板子上文字渲染占了大头，
        # 缓存后重复帧从 ~12ms 降到 <1ms；骨架层是另一张图，不受影响）
        key = (round(float(state.get("score") or 0), 2), int(state.get("stars") or 0),
               state.get("action") or "", int(state.get("count") or 0),
               int(state.get("target") or 0), round(float(state.get("pb") or 0), 2),
               state.get("hint") or "", bool(state.get("det")),
               (state.get("debug") or "") if debug else "")
        if self._cache_key == key and self._cache_mask is not None:
            return self._cache_mask.copy()

        mask = np.zeros((self.h, self.w), np.uint8)

        # ---- 顶部：分数 + 星级（左）----
        full = float(state.get("full") or 100.0)
        score = float(state.get("score") or 0.0)
        if full > 10:                      # 百分制
            score_s = "%d" % int(round(score))
        else:                              # 小分值制（如 2.2）
            score_s = "%.2f" % score
        self._text(mask, "SCORE " + score_s, (6, 22), 0.62, 2)
        sx = 6 + cv2.getTextSize("SCORE " + score_s, cv2.FONT_HERSHEY_SIMPLEX, 0.62, 2)[0][0] + 8
        stars = int(state.get("stars") or 0)
        for i in range(3):
            self._star(mask, sx + i * 15 + 7, 15, 6) if i < stars else \
                cv2.circle(mask, (int(sx + i * 15 + 7), 15), 5, 255, 1, cv2.LINE_8)

        # ---- 顶部：动作名 + 计数（右）----
        act = state.get("action") or ""
        cnt = int(state.get("count") or 0)
        tgt = int(state.get("target") or 0)
        right = act if not tgt else "%s %d/%d" % (act, cnt, tgt)
        self._text(mask, right, (self.w - 6, 22), 0.45, 1, align="right", cn=True)

        self._hline(mask, 28)

        # ---- 中下：纠错提示（大字，居中）+ 描边框（1bit 下靠边框才不至于和骨架糊在一起）----
        hint = state.get("hint") or ""
        if hint:
            txt = HINT_EN.get(hint, hint) if (not self.cjk) else hint
            fs = 0.62 if not self.cjk else 0.55
            if self.cjk:
                tw, th = 22 * len(hint), 22
            else:
                (tw, th), _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, fs, 2)
            cx, cy = self.w // 2, self.h - 40
            x0, x1 = cx - tw // 2 - 5, cx + tw // 2 + 5
            y0, y1 = cy - th - 3, cy + 5
            cv2.rectangle(mask, (max(1, x0), max(1, y0)), (min(self.w - 2, x1), min(self.h - 2, y1)),
                          255, 1, cv2.LINE_8)
            self._text(mask, hint, (cx, cy), fs, 2, align="center", cn=True)

        # ---- 左下：个人最佳 ----
        pb = float(state.get("pb") or 0.0)
        if pb > 0:
            pb_s = "PB %d" % int(round(pb)) if full > 10 else "PB %.2f" % pb
            self._text(mask, pb_s, (6, self.h - 13), 0.38, 1)
        else:
            self._text(mask, "PB --", (6, self.h - 13), 0.38, 1)

        # ---- 居中：无人提示 ----
        if not state.get("det", False):
            self._text(mask, "no det", (self.w // 2, self.h // 2 + 8), 0.8, 2, align="center")

        # ---- 底部调试行（默认关）----
        dbg = state.get("debug") or ""
        if (debug or dbg) and dbg:
            self._text(mask, dbg, (self.w - 6, self.h - 10), 0.32, 1, align="right")

        # 二值化：cv2 的 LINE_AA 会写出 1~254 的灰度，统一成 0/255，避免下游误用
        mask = ((mask > 127).astype(np.uint8)) * 255

        # 边界留白：外圈 1px 清空，避免 FPGA 端边缘残影
        mask[0, :] = 0
        mask[-1, :] = 0
        mask[:, 0] = 0
        mask[:, -1] = 0
        self._cache_key = key
        self._cache_mask = mask
        return mask


def demo_states():
    """演示用状态：正常/低分/无人"""
    return [
        dict(score=87, full=100, stars=3, action="深蹲", count=12, target=15, pb=92,
             hint="膝盖对准脚尖", det=True, debug="30.0fps npu24ms"),
        dict(score=58, full=100, stars=1, action="深蹲", count=6, target=15, pb=92,
             hint="蹲得再深一点", det=True, debug="30.0fps npu24ms"),
        dict(score=0, full=100, stars=0, action="深蹲", count=0, target=15, pb=92,
             hint="", det=False, debug="30.0fps npu24ms"),
    ]


if __name__ == "__main__":
    import sys
    out_dir = sys.argv[1] if len(sys.argv) > 1 else "out"
    os.makedirs(out_dir, exist_ok=True)
    osd = OsdLayout()
    print("中文支持:", osd.cjk)
    for i, st in enumerate(demo_states()):
        m = osd.render(st)
        p = os.path.join(out_dir, "osd_%d.png" % i)
        cv2.imwrite(p, m)
        # 放大 2 倍便于查看（模拟 FPGA 的 2x 显示）
        cv2.imwrite(p.replace(".png", "_2x.png"), cv2.resize(m, (osd.w * 2, osd.h * 2), interpolation=cv2.INTER_NEAREST))
        print("  ->", p, "非零像素", int((m > 0).sum()))





