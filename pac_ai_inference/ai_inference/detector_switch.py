"""P/V 검출기 상호 검증과 전환(히스테리시스) 로직. ROS 의존성 없음 -> 단독 단위 테스트 가능.

P: 주 검출기 (지금은 색상·형상+깊이, 나중에 YOLO로 교체)
V: 검증 검출기 (깊이 평면 기반)
"""
from __future__ import annotations
import math
from dataclasses import dataclass, field
from typing import Optional, Tuple

XYZ = Tuple[float, float, float]

# ai.csv에 기존 칼럼 뒤에 붙일 칼럼
CSV_EXTRA_COLUMNS = ["active_detector", "disagree_m", "switch_event", "det_fault_mode"]  # 기존 fault_mode 칼럼은 output 층용으로 유지

STATUS_OK = 0
STATUS_NO_TARGET = 1

CONF_FALLBACK = 0.6          # B 임계값(0.5)보다 여유를 둔 고정값
CONF_NORMAL_RANGE = (0.85, 1.0)


@dataclass
class DetectorConfig:
    mode: str = "auto"          # auto | primary | verifier
    agree_tol_m: float = 0.02
    slow_ms: float = 50.0
    switch_n: int = 3
    recover_m: int = 15
    min_hold_s: float = 1.0
    pos_from_v: bool = True     # auto 모드에서 P·V가 일치할 때 위치는 V에서(P는 교차 검증용)

    @classmethod
    def from_dict(cls, d: dict) -> "DetectorConfig":
        known = {k: v for k, v in d.items() if k in cls.__dataclass_fields__}
        return cls(**known)


@dataclass
class FaultConfig:
    mode: str = "none"          # none | p_delay | p_bad_value | p_miss
    p_delay_ms: float = 80.0
    p_bad_offset_m: float = 0.10

    @classmethod
    def from_dict(cls, d: dict) -> "FaultConfig":
        known = {k: v for k, v in (d or {}).items() if k in cls.__dataclass_fields__}
        return cls(**known)


@dataclass
class FrameInput:
    t_s: float                          # 단조 시계(초)
    p_xyz: Optional[XYZ] = None         # P 검출 결과 (map 좌표). 미검출이면 None
    p_conf: float = 0.0
    p_infer_ms: float = 0.0
    v_xyz: Optional[XYZ] = None         # V 검출 결과
    v_valid_ratio: float = 0.0          # 깊이 유효 픽셀 비율
    v_conf: Optional[float] = None      # V 단독 신뢰도(고정 verifier 모드에서 사용). None이면 v_valid_ratio


@dataclass
class Decision:
    active_detector: str                # "P" | "V"
    xyz: Optional[XYZ]
    confidence: float
    status: int
    disagree_m: Optional[float]
    switch_event: str                   # none | to_V:mismatch | to_V:slow | to_V:miss | to_P:recovered
    frame_state: str                    # ok | mismatch | slow | miss (이번 프레임 판정)
    source: str = ""                    # 이번에 실제로 내보낸 값의 출처 "P" | "V" (CSV active_detector 칼럼용)


def _dist(a: XYZ, b: XYZ) -> float:
    return math.dist(a, b)


def apply_detector_fault(frame: FrameInput, fault: FaultConfig) -> FrameInput:
    """detector 층 장애 주입: P 출력에만 적용한다 (V는 건드리지 않음)."""
    if fault.mode == "none":
        return frame
    f = FrameInput(**frame.__dict__)
    if fault.mode == "p_delay":
        # 주입 지연은 추론시간에 포함해야 slow 판정이 작동한다
        f.p_infer_ms += fault.p_delay_ms
    elif fault.mode == "p_bad_value" and f.p_xyz is not None:
        x, y, z = f.p_xyz
        f.p_xyz = (x + fault.p_bad_offset_m, y, z)
    elif fault.mode == "p_miss":
        f.p_xyz = None
    return f


class DetectorSwitch:
    def __init__(self, cfg: DetectorConfig):
        self.cfg = cfg
        self.active = "V" if cfg.mode == "verifier" else "P"
        self._bad_run = 0
        self._ok_run = 0
        self._switched_at = -1e9
        self._last_bad_reason = "miss"

    # --- 프레임 판정 ---
    def _classify(self, f: FrameInput) -> Tuple[str, Optional[float]]:
        if f.p_xyz is None:
            return "miss", None
        d = _dist(f.p_xyz, f.v_xyz) if f.v_xyz is not None else None
        if f.p_infer_ms > self.cfg.slow_ms:
            return "slow", d
        if d is not None and d > self.cfg.agree_tol_m:
            return "mismatch", d
        return "ok", d

    def step(self, f: FrameInput) -> Decision:
        state, d = self._classify(f)
        event = "none"

        # 고정 모드(디버깅용): 전환 없이 한쪽만 사용
        if self.cfg.mode in ("primary", "verifier"):
            return self._output(f, state, d, event)

        if self.active == "P":
            if state == "ok":
                self._bad_run = 0
            else:
                self._bad_run += 1
                self._last_bad_reason = state
                # V가 있을 때만 전환 (V도 없으면 전환해도 의미가 없다)
                if self._bad_run >= self.cfg.switch_n and f.v_xyz is not None:
                    self.active = "V"
                    self._switched_at = f.t_s
                    self._bad_run = 0
                    self._ok_run = 0
                    event = f"to_V:{self._last_bad_reason}"
        else:  # active == "V"
            if state == "ok" and f.v_xyz is not None:
                self._ok_run += 1
            else:
                self._ok_run = 0
            held = (f.t_s - self._switched_at) >= self.cfg.min_hold_s
            if self._ok_run >= self.cfg.recover_m and held:
                self.active = "P"
                self._ok_run = 0
                self._bad_run = 0
                event = "to_P:recovered"

        return self._output(f, state, d, event)

    # --- 출력 값/신뢰도 선택 ---
    def _output(self, f: FrameInput, state: str, d: Optional[float], event: str) -> Decision:
        use_v = (self.active == "V") or (self.cfg.mode == "auto" and state != "ok")
        if self.cfg.mode == "primary":
            use_v = False

        xyz: Optional[XYZ]
        conf: float
        if not use_v:
            if f.p_xyz is None:
                xyz, conf = None, 0.0
            elif f.v_xyz is None and self.cfg.mode == "auto":
                xyz, conf = f.p_xyz, CONF_FALLBACK      # P 단독(검증 불가)
            else:
                use_v_pos = self.cfg.pos_from_v and self.cfg.mode == "auto" and f.v_xyz is not None
                xyz = f.v_xyz if use_v_pos else f.p_xyz
                conf = min(min(f.p_conf, f.v_valid_ratio or 1.0), CONF_NORMAL_RANGE[1])
        else:
            if f.v_xyz is not None:
                xyz = f.v_xyz
                if self.cfg.mode == "verifier":     # 디버깅용 고정 모드: V 자체 신뢰도 사용
                    conf = f.v_conf if f.v_conf is not None else f.v_valid_ratio
                else:
                    conf = CONF_FALLBACK
            elif f.p_xyz is not None and state == "ok":
                xyz, conf = f.p_xyz, CONF_FALLBACK
            else:
                xyz, conf = None, 0.0

        if xyz is None:
            return Decision(self.active, None, 0.0, STATUS_NO_TARGET, d, event, state, "")
        return Decision(self.active, xyz, conf, STATUS_OK, d, event, state, "V" if use_v else "P")
