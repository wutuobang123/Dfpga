# 阈值与分值配置
# 来源标注：official = 规程原文 / coach = 教练标定

ACTIONS = {
    "split": {
        "name": "劈叉",
        "open_angle_min": 45,          # 劈叉双腿开度（重庆规程原文 >45°）
        "score": 0.3,
        "source": "official",
    },
    "situp": {
        "name": "仰卧起坐",
        "count_required": 2,           # 连续2次
        "torso_thigh_threshold": 90,   # 起身角度（教练标定）
        "score": 0.4,
        "source": "coach",
    },
    "kick": {
        "name": "踢腿",
        "leg_lift_min_px": 30,         # 膝高于髋的最小像素（需归一化）
        "score": 0.3,
        "source": "coach",
    },
    "jump": {
        "name": "连续跳跃",
        "count_required": 2,
        "min_gap_frames": 10,          # 两次跳跃最小间隔(帧)
        "score": 0.3,
        "source": "official",
    },
    "balance": {
        "name": "单腿平衡",
        "hold_frames_min": 30,         # 单腿站≥1秒(30fps)
        "wobble_max_px": 20,           # 髋晃动容差
        "score": 0.3,
        "source": "coach",
    },
}

# 推理配置
INFERENCE = {
    "model": "yolov8n-pose.rknn",
    "imgsz": 640,
    "conf": 0.3,
    "fps": 30,
}

# 评分规则
SCORING = {
    "total_full": 1.6,  # 5个动作满分合计
}
# 画面叠加（OSD）参数 —— 老 preprocess.py 依赖这些键，保留兼容
VIS = {
    "osd_font_scale": 0.6,
    "osd_thickness": 1,
    "box_color": (0, 255, 0),
    "text_color": (0, 255, 255),
    "skel_color": (0, 255, 0),
    "kpt_radius": 3,
    "line_thickness": 2,
}
