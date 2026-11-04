import numpy as np
import cv2
from rknnlite.api import RKNNLite

class PoseInfer:
    def __init__(self, rknn_path='yolov8n-pose.rknn', imgsz=640, conf=0.3):
        self.imgsz = imgsz
        self.conf = conf
        self.rknn = RKNNLite()
        
        ret = self.rknn.load_rknn(rknn_path)
        if ret != 0: raise RuntimeError(f"Load RKNN failed: {ret}")
        
        # 指定 NPU 核心，RK3568 建议用 core_mask=RKNN.NPU_CORE_0
        ret = self.rknn.init_runtime(core_mask=RKNNLite.NPU_CORE_0)
        if ret != 0: raise RuntimeError(f"Init runtime failed: {ret}")
        print(f"[PoseInfer] RKNN loaded: {rknn_path}")

    def preprocess(self, frame):
        """Letterbox resize + BGR2RGB + CHW + Normalize"""
        h, w = frame.shape[:2]
        scale = min(self.imgsz / w, self.imgsz / h)
        nw, nh = int(w * scale), int(h * scale)
        
        resized = cv2.resize(frame, (nw, nh), interpolation=cv2.INTER_LINEAR)
        canvas = np.zeros((self.imgsz, self.imgsz, 3), dtype=np.uint8)
        canvas[(self.imgsz-nh)//2:(self.imgsz+nh)//2, 
               (self.imgsz-nw)//2:(self.imgsz+nw)//2] = resized
        
        rgb = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)
        blob = rgb.astype(np.float32) / 255.0
        return blob.transpose(2, 0, 1)[np.newaxis, ...], scale, (self.imgsz-nw)//2, (self.imgsz-nh)//2

    def run(self, frame):
        blob, scale, pad_w, pad_h = self.preprocess(frame)
        outputs = self.rknn.inference(inputs=[blob])
        
        # YOLOv8-pose 默认输出: [1, 56, 8400]
        out = outputs[0][0].T  # (8400, 56)
        
        scores = out[:, 4]
        mask = scores > self.conf
        if not np.any(mask): return []
        
        idx = np.argmax(scores[mask])
        best = out[mask][idx]
        
        # 解码 bbox
        cx, cy, w, h = best[:4]
        x1, y1 = cx - w/2, cy - h/2
        x2, y2 = cx + w/2, cy + h/2
        
        # 还原坐标 (减去 padding 再除以 scale)
        x1 = max(0, (x1 - pad_w) / scale)
        y1 = max(0, (y1 - pad_h) / scale)
        x2 = min(frame.shape[1], (x2 - pad_w) / scale)
        y2 = min(frame.shape[0], (y2 - pad_h) / scale)
        
        # 解码 17 个关键点 (每个点 x,y,conf)
        kps_raw = best[5:].reshape(17, 3)
        kps_raw[:, 0] = (kps_raw[:, 0] - pad_w) / scale
        kps_raw[:, 1] = (kps_raw[:, 1] - pad_h) / scale
        
        # 返回检测结果列表（与 main/web_stream 的 results[0] 契约一致）
        return [{
            "bbox": [x1, y1, x2, y2],
            "kps": kps_raw.astype(np.float32),
            "height": float(y2 - y1),
            "conf": float(best[4]),
        }]

def draw_skeleton(frame, results, color=(0, 255, 0)):
    """results: list[dict] 或单个 dict；每个 dict 含 kps/conf。"""
    if not results: return frame
    if isinstance(results, dict):
        results = [results]
    pairs = [(5,6),(5,7),(7,9),(6,8),(8,10),(5,11),(6,12),(11,12),(11,13),(13,15),(12,14),(14,16)]
    for result in results:
        kps = result["kps"]
        for i, j in pairs:
            if kps[i][2] > 0.3 and kps[j][2] > 0.3:
                cv2.line(frame, (int(kps[i][0]), int(kps[i][1])), (int(kps[j][0]), int(kps[j][1])), color, 2)
        for x, y, c in kps:
            if c > 0.3: cv2.circle(frame, (int(x), int(y)), 3, (0, 0, 255), -1)
        cv2.putText(frame, f"NPU conf={result['conf']:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
    return frame