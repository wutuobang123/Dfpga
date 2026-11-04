import cv2
import numpy as np


def decode_frame(encoded_data: bytes) -> np.ndarray:
    """
    将编码后的图像数据解码为 OpenCV BGR 帧
    如果已经是原始 BGR bytes，直接 reshape
    """
    # 尝试 JPEG 解码
    arr = np.frombuffer(encoded_data, dtype=np.uint8)
    frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if frame is not None:
        return frame
    return None


def encode_frame(frame: np.ndarray, quality=95) -> bytes:
    """将 OpenCV BGR 帧编码为 JPEG bytes"""
    _, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
    return buf.tobytes()