"""P 검출기(색 + 형상): RGB에서 철판 후보를 색으로 뽑고, 형상 검증을 통과한 가장 큰 덩어리를 철판으로 본다.

- 1단계 색 필터: 밝기(채널 최댓값) >= v_min 이고 채널 간 차이(최댓값-최솟값) <= chroma_max
- 2단계 형상 검증: 이미지 가장자리에 닿지 않고, 가로세로 비율(회전 무관)과 직사각형도가 철판과 맞아야 함
  (실측: 철판 (206,206,213) / 작업대 (178,151,117) / 바닥 띠 (193,193,193). 바닥 띠는 색 필터를 통과하지만
   이미지 끝에 닿고 비율이 11배라서 형상 검증에서 걸러진다)
- 위치(u,v)는 색 덩어리의 무게중심, 깊이는 그 덩어리 안의 깊이 중앙값 (깊이 센서 자체는 V와 공유)
"""
import math

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


def _components(mask):
    """행 단위 run을 묶는 4-연결 성분 라벨링(순수 numpy+파이썬, run 수만큼만 반복해서 빠르다).
    반환: {root: [area, sx, sy, sxx, syy, sxy, x0, y0, x1, y1]}"""
    parent = []

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    runs = []                       # (y, x0, x1)
    prev = []                       # 직전 행의 (x0, x1, id)
    for y in range(mask.shape[0]):
        idx = np.flatnonzero(mask[y])
        if idx.size == 0:
            prev = []
            continue
        brk = np.flatnonzero(np.diff(idx) > 1)
        starts = np.concatenate(([idx[0]], idx[brk + 1])).tolist()
        ends = np.concatenate((idx[brk], [idx[-1]])).tolist()
        cur, k = [], 0
        for s, e in zip(starts, ends):
            rid = len(runs)
            runs.append((y, s, e))
            parent.append(rid)
            while k < len(prev) and prev[k][1] < s:
                k += 1
            j = k
            while j < len(prev) and prev[j][0] <= e:
                a, b = find(rid), find(prev[j][2])
                if a != b:
                    parent[a] = b
                j += 1
            cur.append((s, e, rid))
        prev = cur
    comps = {}
    for rid, (y, x0, x1) in enumerate(runs):
        r = find(rid)
        n = x1 - x0 + 1
        sx = (x0 + x1) * n / 2.0
        sxx = (x1 * (x1 + 1) * (2 * x1 + 1) - (x0 - 1) * x0 * (2 * x0 - 1)) / 6.0
        c = comps.get(r)
        if c is None:
            comps[r] = [n, sx, y * n, sxx, y * y * n, y * sx, x0, y, x1, y]
        else:
            c[0] += n
            c[1] += sx
            c[2] += y * n
            c[3] += sxx
            c[4] += y * y * n
            c[5] += y * sx
            c[6] = min(c[6], x0)
            c[8] = max(c[8], x1)
            c[9] = y
    return comps


def _shape(c):
    """(비율, 직사각형도). 2차 모멘트로 구해서 회전에 무관하다. 직사각형이면 직사각형도 ~ 1."""
    area, sx, sy, sxx, syy, sxy = c[:6]
    mx, my = sx / area, sy / area
    cxx = sxx / area - mx * mx + 1.0 / 12.0
    cyy = syy / area - my * my + 1.0 / 12.0
    cxy = sxy / area - mx * my
    tr, det = cxx + cyy, cxx * cyy - cxy * cxy
    disc = math.sqrt(max(tr * tr / 4.0 - det, 0.0))
    l1, l2 = tr / 2.0 + disc, tr / 2.0 - disc
    if l2 <= 1e-9:
        return float('inf'), 0.0
    return math.sqrt(l1 / l2), area / (12.0 * math.sqrt(l1 * l2)), mx, my


def detect_plate_color(rgb, depth, v_min=200, chroma_max=15, min_pixels=200,
                       aspect_range=(1.2, 1.9), rect_range=(0.85, 1.15)):
    h, w, _ = rgb.shape
    ch0, ch1, ch2 = rgb[:, :, 0], rgb[:, :, 1], rgb[:, :, 2]
    vmax = np.maximum(np.maximum(ch0, ch1), ch2)
    vmin = np.minimum(np.minimum(ch0, ch1), ch2)
    mask = (vmax >= v_min) & ((vmax - vmin) <= chroma_max)      # uint8끼리 연산, vmax>=vmin 이라 넘침 없음
    n_mask = int(mask.sum())
    if n_mask < min_pixels:
        return Detection(False, pixel_count=n_mask, reason='no_color_match')
    best, why = None, []
    for c in _components(mask).values():
        area, x0, y0, x1, y1 = c[0], c[6], c[7], c[8], c[9]
        if area < min_pixels:
            continue
        if x0 <= 0 or y0 <= 0 or x1 >= w - 1 or y1 >= h - 1:
            why.append('border')
            continue
        aspect, rect, mx, my = _shape(c)
        if not (aspect_range[0] <= aspect <= aspect_range[1]):
            why.append(f'aspect={aspect:.2f}')
            continue
        if not (rect_range[0] <= rect <= rect_range[1]):
            why.append(f'rect={rect:.2f}')
            continue
        if best is None or area > best[0][0]:
            best = (c, rect, mx, my)
    if best is None:
        return Detection(False, pixel_count=n_mask, reason='shape_rejected:' + ','.join(why[:4]))
    c, rect, mx, my = best
    area, x0, y0, x1, y1 = c[0], c[6], c[7], c[8], c[9]
    sub = mask[y0:y1 + 1, x0:x1 + 1]
    d = depth[y0:y1 + 1, x0:x1 + 1]
    ok = sub & np.isfinite(d) & (d > 0.0)
    n_ok = int(ok.sum())
    if n_ok == 0:
        return Detection(False, pixel_count=int(area), bbox=(x0, y0, x1, y1), reason='no_valid_depth')
    return Detection(
        True, u=float(mx), v=float(my), depth_m=float(np.median(d[ok])), pixel_count=int(area),
        bbox=(int(x0), int(y0), int(x1), int(y1)), det_conf=float(max(0.0, 1.0 - abs(1.0 - rect))),
        depth_valid_ratio=float(n_ok) / float(area), ref_depth_m=0.0)
