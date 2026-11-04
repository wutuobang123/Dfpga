"""gen_voice.py - 生成 14 条语音提示 wav（AI TTS，离线可跑）

在板子或 VM 上执行一次即可：
    python3 gen_voice.py            # 自动选发音引擎
    python3 gen_voice.py --dry-run  # 只打印将执行的命令

优先用 espeak-ng（离线、无需联网、装一个 apt 包就行）：
    sudo apt install espeak-ng
中文用 cmn（普通话）；没有中文发音包时退回英文短语（不会生成乱码音频）。

如果装了 edge-tts（微软神经语音，音质最好，但需要联网）：
    pip3 install edge-tts
    python3 gen_voice.py --engine edge
"""
import argparse
import os
import shutil
import subprocess
import sys

# 与 voice.py 的 HINT_FILES 一一对应
ITEMS = [
    ("great.wav", "非常好，保持住", "Great, hold it"),
    ("good.wav", "很好，保持住", "Good, keep going"),
    ("deeper.wav", "蹲得再深一点", "Go deeper"),
    ("back.wav", "挺直腰背", "Keep your back straight"),
    ("knees.wav", "膝盖对准脚尖", "Knees over toes"),
    ("retry.wav", "再试一次", "Try again"),
    ("split_wider.wav", "开度不够", "Open wider"),
    ("legs_straight.wav", "腿要伸直", "Straighten your legs"),
    ("vertical.wav", "倒立要竖直", "Stay vertical"),
    ("hold_longer.wav", "保持时间不足", "Hold longer"),
    ("both_legs.wav", "左右都要踢", "Kick both legs"),
    ("no_hands.wav", "手不能撑地", "No hands"),
    ("jump_twice.wav", "跳两次", "Jump twice"),
    ("no_person.wav", "未检测到人", "No person detected"),
]

EDGE_VOICE = "zh-CN-XiaoxiaoNeural"


def find_engine(prefer=None):
    if prefer == "edge" or prefer is None:
        if shutil.which("edge-tts"):
            return "edge"
    for name in ("espeak-ng", "espeak"):
        if shutil.which(name):
            return name
    if prefer is None and shutil.which("edge-tts"):
        return "edge"
    return None


def espeak_langs(exe):
    """返回该 espeak 支持的语音列表（小写）"""
    try:
        out = subprocess.run([exe, "--voices"], capture_output=True, text=True, timeout=10).stdout
        return out.lower()
    except Exception:
        return ""


def pick_lang(engine):
    """挑出该 espeak 实际支持的中文语言码：新版 espeak-ng 用 cmn，老版 espeak 用 zh"""
    langs = espeak_langs(engine)
    for code in ("cmn", "zh-yue", "zh"):
        if (" %s " % code) in langs or ("\n%s " % code) in langs or ("  %s  " % code) in langs:
            return code
    return None


def build_cmd(engine, zh, en, out_path, lang=None):
    if engine == "edge":
        return ["edge-tts", "--voice", EDGE_VOICE, "--text", zh, "--write-media", out_path]
    # espeak 系列：普通话（cmn 或 zh，由 pick_lang 决定）
    return [engine, "-v", (lang or "cmn"), "-s", "175", "-w", out_path, zh]


def to_wav_16k_mono(src):
    """把 mp3 等转成 aplay 能直接播的 16bit/16k/单声道 wav（需要 ffmpeg）"""
    if not shutil.which("ffmpeg"):
        return False
    tmp = src + ".16k.wav"
    try:
        subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-i", src,
                        "-ar", "16000", "-ac", "1", "-sample_fmt", "s16", tmp],
                       timeout=60, check=True)
        os.replace(tmp, src)
        return True
    except Exception:
        return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="voice")
    ap.add_argument("--engine", default=None, help="espeak-ng / espeak / edge")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    engine = args.engine or find_engine()
    if not engine:
        print("!! 没有可用的 TTS 引擎。请在板子上装一个：")
        print("   sudo apt install espeak-ng        # 离线，推荐")
        print("   pip3 install edge-tts             # 神经语音，需联网，音质最好")
        return 2
    print("TTS 引擎:", engine)

    lang_ok = True
    lang_code = None
    if engine != "edge":
        lang_code = pick_lang(engine)
        lang_ok = lang_code is not None
        if not lang_ok:
            print("!! 该 espeak 没有中文（cmn）发音包，将用英文短语代替，避免生成怪音")
            print("   Debian/Ubuntu: sudo apt install espeak-ng-data  或检查 /usr/lib/*/espeak-ng-data")

    os.makedirs(args.out, exist_ok=True)
    made = 0
    for fn, zh, en in ITEMS:
        out_path = os.path.join(args.out, fn)
        text = zh if (engine == "edge" or lang_ok) else en
        cmd = build_cmd(engine, text, en, out_path, lang_code)
        if args.dry_run:
            print("  [dry] " + " ".join(cmd))
            continue
        try:
            r = subprocess.run(cmd, capture_output=True, timeout=60)
            if r.returncode != 0:
                print("  %-18s 失败: %s" % (fn, r.stderr.decode("utf-8", "ignore")[:80]))
                continue
            if engine == "edge":
                to_wav_16k_mono(out_path)
            if os.path.exists(out_path):
                sz = os.path.getsize(out_path)
                print("  %-18s %6d B  「%s」" % (fn, sz, text))
                made += 1
        except Exception as e:
            print("  %-18s 异常: %s" % (fn, e))

    if not args.dry_run:
        print("\n生成 %d/%d 个 wav -> %s" % (made, len(ITEMS), os.path.abspath(args.out)))
        print("播放测试： aplay %s/good.wav" % args.out)
    return 0 if (args.dry_run or made == len(ITEMS)) else 1


if __name__ == "__main__":
    sys.exit(main())


