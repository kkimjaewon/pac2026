from dataclasses import dataclass

import numpy as np


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


def _largest_component(mask):
    """마스크에서 가장 큰 연결 덩어리만 남긴다(로봇 팔 등 다른 물체가 섞이는 것을 막는다).
    cv2, scipy가 모두 없으면 마스크를 그대로 돌려준다."""
    try:
        import cv2
        n, lab, st, _ = cv2.connectedComponentsWithStats(mask.astype(np.uint8), connectivity=8)
        if n <= 2:
            return mask
        return lab == (1 + int(np.argmax(st[1:, cv2.CC_STAT_AREA])))
    except ImportError:
        pass
    try:
        from scipy import ndimage as nd
        lab, n = nd.label(mask, structure=np.ones((3, 3), dtype=int))
        if n <= 1:
            return mask
        return lab == (1 + int(np.argmax(np.bincount(lab.ravel())[1:])))
    except ImportError:
        return mask


def _edge_center(occ, u0, u1, v0, v1, exp_u, exp_v, cu, cv, tol_px=3.0):
    """덩어리 bbox 중심. 한쪽 끝이 가려져 철판 크기보다 짧으면, 보이는 쪽 가장자리 + 철판 크기의 절반으로 구한다.
    occ: 철판보다 높이 있는 물체(로봇) 마스크. 가려진 쪽은 bbox 바깥 띠에 occ가 더 많은 쪽."""
    def axis(lo, hi, exp, n_lo, n_hi, c):
        ext = hi - lo + 1
        if abs(ext - exp) <= tol_px:
            return (lo + hi) / 2.0, 'both'
        if ext < exp - tol_px:
            if n_lo > n_hi:
                return hi - (exp - 1.0) / 2.0, 'lo_occ'
            if n_hi > n_lo:
                return lo + (exp - 1.0) / 2.0, 'hi_occ'
        return c, 'centroid'
    k = 4
    u_lo = int(occ[v0:v1 + 1, max(u0 - k, 0):u0].sum())
    u_hi = int(occ[v0:v1 + 1, u1 + 1:u1 + 1 + k].sum())
    v_lo = int(occ[max(v0 - k, 0):v0, u0:u1 + 1].sum())
    v_hi = int(occ[v1 + 1:v1 + 1 + k, u0:u1 + 1].sum())
    nu, mu = axis(u0, u1, exp_u, u_lo, u_hi, cu)
    nv, mv = axis(v0, v1, exp_v, v_lo, v_hi, cv)
    return nu, nv, f'u:{mu} v:{mv}'


def detect_plate(depth, ref_depth_m=0.0, min_height_m=0.004, max_height_m=0.15, min_pixels=200,
                 largest_only=True, fx=0.0, size_uv_m=None):
    """작업대 면보다 카메라에 가까운 픽셀 덩어리를 철판으로 본다.

    ref_depth_m이 0이면 프레임 전체의 깊이 중앙값을 작업대 면으로 쓴다(작업대가 화면 대부분일 때).
    max_height_m는 철판 윗면 높이 근처로 좁게 둔다(철판 위의 로봇을 제외).
    fx > 0 이고 size_uv_m=(이미지 u방향, v방향 철판 크기 m)를 주면, 로봇이 철판 일부를 가렸을 때도
    보이는 가장자리와 철판 크기로 중심을 구한다.
    """
    finite = np.isfinite(depth) & (depth > 0.0)
    if not finite.any():
        return Detection(False, reason='no_valid_depth')
    ref = float(ref_depth_m) if ref_depth_m > 0.0 else float(np.median(depth[finite]))
    mask = finite & (depth < ref - min_height_m) & (depth > ref - max_height_m)
    if largest_only:
        mask = _largest_component(mask)
    n = int(mask.sum())
    if n < min_pixels:
        return Detection(False, pixel_count=n, ref_depth_m=ref, reason='no_plate')
    vs, us = np.nonzero(mask)
    u0, u1, v0, v1 = int(us.min()), int(us.max()), int(vs.min()), int(vs.max())
    box = depth[v0:v1 + 1, u0:u1 + 1]
    fill = n / float(box.size)
    valid_ratio = float(np.count_nonzero(np.isfinite(box) & (box > 0.0))) / box.size
    d_med = float(np.median(depth[mask]))
    u_c, v_c, why = float(us.mean()), float(vs.mean()), ''
    if fx > 0.0 and size_uv_m is not None:
        ppm = fx / d_med
        occ = finite & (depth < ref - max_height_m)
        u_c, v_c, mode = _edge_center(occ, u0, u1, v0, v1, size_uv_m[0] * ppm, size_uv_m[1] * ppm, u_c, v_c)
        why = 'fit:' + mode
    return Detection(
        True, u=u_c, v=v_c, depth_m=d_med,
        pixel_count=n, bbox=(u0, v0, u1, v1), det_conf=float(min(1.0, fill)),
        depth_valid_ratio=valid_ratio, ref_depth_m=ref, reason=why)
