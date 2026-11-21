"""P 검출기(YOLO): ONNX(YOLOv8n, 입력 정사각형, 출력 (1,5,N))로 철판 상자를 찾는다.

- 입력 영상(RGB 640x480)을 비율을 지켜 모델 크기(예: 320x320)로 줄이고 위아래를 회색(114)으로 채운다.
- 출력은 후보마다 [cx, cy, w, h, 점수]. 철판이 하나뿐이라 점수가 가장 높은 후보 하나만 쓴다.
- 위치(u,v)는 상자 중심, 깊이는 상자 중심부(가로·세로 40%) 안의 깊이 중앙값이다.
  (상자 전체를 쓰면 철판 옆 책상 깊이가 섞여 높이가 약 1cm 흔들린다)
- 이미지 가장자리에 닿은 상자는 색 검출기와 같이 버린다(일부만 보여서 중심이 치우친다).
- onnxruntime 은 생성할 때만 불러온다. 없으면 예외가 나고, 노드가 잡아서 P 없이(V로) 계속 돈다.
"""
import os

import numpy as np

try:
    from ai_inference.detector import Detection
except ImportError:                      # ROS 패키지 밖(단독 테스트)에서는 같은 모양의 클래스를 쓴다
    from dataclasses import dataclass

    @dataclass
    class Detection:
        found: bool
        u: float = 0.0
        v: float = 0.0
        depth_m: float = 0.0
        pixel_count: int = 0
        bbox: tuple = (0, 0, 0, 0)
        det_conf: float = 0.0
        depth_valid_ratio: float = 0.0
        ref_depth_m: float = 0.0
        reason: str = ''


def letterbox(rgb, size):
    """RGB(H,W,3 uint8) -> (1,3,size,size float32 0~1, 배율, 왼쪽 패딩, 위쪽 패딩)"""
    import cv2
    h, w = rgb.shape[:2]
    s = min(size / h, size / w)
    nw, nh = int(round(w * s)), int(round(h * s))
    img = cv2.resize(rgb, (nw, nh), interpolation=cv2.INTER_LINEAR)
    left = int(round((size - nw) / 2 - 0.1))
    top = int(round((size - nh) / 2 - 0.1))
    canvas = np.full((size, size, 3), 114, np.uint8)
    canvas[top:top + nh, left:left + nw] = img
    x = np.ascontiguousarray(canvas.transpose(2, 0, 1)[None]).astype(np.float32) / 255.0
    return x, s, left, top


def decode(out, scale, left, top, conf_min):
    """모델 출력 -> (cx, cy, w, h, 점수) 원본 영상 픽셀 좌표. 기준 점수 미만이면 (None, 최고점수)"""
    p = np.asarray(out)
    p = p[0] if p.ndim == 3 else p
    if p.shape[0] != 5 and p.shape[1] == 5:
        p = p.T
    scores = p[4]
    i = int(np.argmax(scores))
    s = float(scores[i])
    if s < conf_min:
        return None, s
    cx, cy, w, h = [float(v) for v in p[0:4, i]]
    return ((cx - left) / scale, (cy - top) / scale, w / scale, h / scale, s), s


class YoloPlate:
    def __init__(self, model_path, conf=0.5, threads=2):
        import onnxruntime as ort            # 없으면 ImportError -> 노드가 잡는다
        path = os.path.expanduser(str(model_path))
        if not os.path.isfile(path):
            raise FileNotFoundError(f'YOLO 모델 파일이 없습니다: {path}')
        so = ort.SessionOptions()
        so.intra_op_num_threads = max(1, int(threads))   # B(실시간 제어)와 CPU를 나눠 쓰므로 제한
        so.inter_op_num_threads = 1
        self.sess = ort.InferenceSession(path, so, providers=['CPUExecutionProvider'])
        inp = self.sess.get_inputs()[0]
        self.name = inp.name
        self.size = int(inp.shape[2]) if isinstance(inp.shape[2], int) else 320
        self.conf = float(conf)
        self.path = path
        self.sess.run(None, {self.name: np.zeros((1, 3, self.size, self.size), np.float32)})   # 예열

    def infer(self, rgb):
        x, s, left, top = letterbox(rgb, self.size)
        out = self.sess.run(None, {self.name: x})[0]
        return decode(out, s, left, top, self.conf)


def detect_plate_yolo(model, rgb, depth, border_px=2):
    r, best = model.infer(rgb)
    if r is None:
        return Detection(False, reason=f'yolo_no_match(max={best:.2f})')
    cx, cy, w, h, score = r
    H, W = depth.shape
    x0, y0, x1, y1 = cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2
    if x0 <= border_px or y0 <= border_px or x1 >= W - 1 - border_px or y1 >= H - 1 - border_px:
        return Detection(False, bbox=(int(x0), int(y0), int(x1), int(y1)), reason='yolo_border')
    hw, hh = 0.2 * w, 0.2 * h
    u0, u1 = max(0, int(cx - hw)), min(W - 1, int(cx + hw))
    v0, v1 = max(0, int(cy - hh)), min(H - 1, int(cy + hh))
    d = depth[v0:v1 + 1, u0:u1 + 1]
    ok = np.isfinite(d) & (d > 0.0)
    n_ok = int(ok.sum())
    if n_ok == 0:
        return Detection(False, bbox=(int(x0), int(y0), int(x1), int(y1)), reason='no_valid_depth')
    return Detection(
        True, u=float(cx), v=float(cy), depth_m=float(np.median(d[ok])), pixel_count=int(w * h),
        bbox=(int(x0), int(y0), int(x1), int(y1)), det_conf=float(min(1.0, score)),
        depth_valid_ratio=float(n_ok) / float(d.size), ref_depth_m=0.0)
