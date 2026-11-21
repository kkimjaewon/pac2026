import threading
import time
from collections import deque

import numpy as np

import rclpy
from intelion_interfaces.msg import ControlCommand
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

from ai_inference.command_publisher import CommandPublisher
from ai_inference.detector import detect_plate
from ai_inference.detector_color import detect_plate_color
from ai_inference.detector_yolo import YoloPlate, detect_plate_yolo
from ai_inference.detector_switch import (
    DetectorConfig, DetectorSwitch, FaultConfig, FrameInput, apply_detector_fault)
from ai_inference.fault_injection import FaultyPublisher
from ai_inference.sensor_adapter import SensorAdapter
from ai_inference.target_mapper import camera_link_to_map, pixel_to_camera_link

ZERO_TARGET = [0.0, 0.0, 0.0, 0.0]


class AiInferenceNode(Node):
    def __init__(self):
        super().__init__('ai_inference_node')
        self.declare_parameter('fixed_target', [0.5, 0.0, 0.2])
        self.declare_parameter('table_depth_m', 0.0)
        self.declare_parameter('min_height_m', 0.004)
        self.declare_parameter('max_height_m', 0.15)
        self.declare_parameter('min_pixels', 200)
        self.declare_parameter('edge_fit', True)            # 로봇이 철판을 가려도 철판 크기로 중심 보정
        self.declare_parameter('plate_size_u_m', 0.20)      # 이미지 u방향 = map y (철판 짧은 변)
        self.declare_parameter('plate_size_v_m', 0.30)      # 이미지 v방향 = map x (철판 긴 변)
        self.declare_parameter('frame_timeout_s', 0.2)
        self.declare_parameter('confidence_min', 0.5)
        self.declare_parameter('command_max_age_ns', 200000000)
        self.declare_parameter('command_type', 'position')
        self.declare_parameter('smooth_n', 3)
        self.declare_parameter('jump_limit_m', 0.05)
        self.declare_parameter('jump_accept_after', 5)
        self.declare_parameter('fault_mode', 'none')
        self.declare_parameter('fault_delay_ms', 200)
        self.declare_parameter('log_dir', '~/pac_logs')
        self.declare_parameter('run_id', '')   # 비우면 ai_<시각>.csv, 주면 <log_dir>/<run_id>/ai.csv
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('camera_frame', 'depth_camera_link')
        # --- P·V 검출기 (detector_mode=verifier 이면 기존 동작 그대로) ---
        self.declare_parameter('detector_mode', 'verifier')   # verifier | auto | primary
        self.declare_parameter('primary_impl', 'none')        # none | mock_depth (실제 P는 이후 추가)
        self.declare_parameter('agree_tol_m', 0.02)
        self.declare_parameter('slow_ms', 50.0)
        self.declare_parameter('switch_n', 3)
        self.declare_parameter('recover_m', 15)
        self.declare_parameter('min_hold_s', 1.0)
        # detector 층 장애 주입 (실행 중 ros2 param set 으로 변경 가능)
        self.declare_parameter('det_fault_mode', 'none')      # none | p_delay | p_bad_value | p_miss
        self.declare_parameter('det_fault_p_delay_ms', 80.0)
        self.declare_parameter('det_fault_p_bad_offset_m', 0.10)
        # --- 색 P 검출기 (primary_impl:=color) ---
        self.declare_parameter('color_v_min', 200)          # 밝기(채널 최댓값) 하한
        self.declare_parameter('color_chroma_max', 15)      # 채널 간 차이 상한(무채색 정도)
        self.declare_parameter('color_min_pixels', 200)
        self.declare_parameter('plate_aspect_min', 1.2)     # 철판 비율 0.3:0.2 = 1.5
        self.declare_parameter('plate_aspect_max', 1.9)
        # --- YOLO P 검출기 (primary_impl:=yolo) ---
        self.declare_parameter('yolo_model_path', '')   # 비워 두면 패키지 share/models 의 모델 사용
        self.declare_parameter('yolo_conf', 0.5)
        self.declare_parameter('yolo_threads', 2)
        self.declare_parameter('pos_from_v', True)          # P·V 일치 시 위치는 V에서(로봇이 P를 오염시켜도 안전)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cmd = FaultyPublisher(CommandPublisher(self,
            confidence_min=float(self.get_parameter('confidence_min').value),
            max_age_ns=int(self.get_parameter('command_max_age_ns').value)), self)
        _g = self.get_parameter
        self.sw = DetectorSwitch(DetectorConfig(
            mode=str(_g('detector_mode').value), agree_tol_m=float(_g('agree_tol_m').value),
            slow_ms=float(_g('slow_ms').value), switch_n=int(_g('switch_n').value),
            recover_m=int(_g('recover_m').value), min_hold_s=float(_g('min_hold_s').value),
            pos_from_v=bool(_g('pos_from_v').value)))
        self._yolo = None
        if str(self.get_parameter('primary_impl').value) == 'yolo':
            try:
                import os
                from ament_index_python.packages import get_package_share_directory
                _mp = str(self.get_parameter('yolo_model_path').value)
                if not _mp:
                    _mp = os.path.join(get_package_share_directory('ai_inference'), 'models', 'plate_yolov8n_320.onnx')
                self._yolo = YoloPlate(
                    _mp,
                    float(self.get_parameter('yolo_conf').value),
                    int(self.get_parameter('yolo_threads').value))
                self.get_logger().info(
                    f'YOLO P 검출기 로드: {self._yolo.path} (입력 {self._yolo.size}, conf>={self._yolo.conf})')
            except Exception as e:
                self.get_logger().error(f'YOLO 로드 실패 -> P 없이 V로 동작: {e}')
        # 첫 호출만 느린 V 검출기를 시작할 때 미리 한 번 돌려 둔다(첫 프레임 약 150~680 ms -> 수 ms).
        # 실패해도 노드는 그대로 뜬다.
        try:
            import numpy as _np
            _d = _np.full((120, 160), 1.0, _np.float32)
            _d[40:70, 50:100] = 0.985
            _t = time.perf_counter()
            detect_plate(_d, 1.0, 0.004, 0.05, 200, fx=100.0, size_uv_m=(0.3, 0.2))
            self.get_logger().info(f'V 검출기 예열 {(time.perf_counter() - _t) * 1000.0:.0f} ms')
        except Exception as e:
            self.get_logger().warning(f'V 예열 실패(무시): {e}')
        self._new = threading.Event()
        self._last_seq = -1
        self._last_capture = None
        self._last_log = 0.0
        self._stop = False
        self._hist = deque(maxlen=64)
        self._last_out = None
        self._jump_count = 0
        self.adapter = SensorAdapter(
            self, '/depth_camera/image_raw', '/depth_camera/depth/image_raw',
            '/depth_camera/camera_info', on_frame=lambda f: self._new.set())
        self._worker = threading.Thread(target=self._loop, daemon=True)
        self._worker.start()
        self.get_logger().info(f'boot_id={self.cmd.boot_id} 시작: 검출 1차(고정 target)')

    def stop(self):
        self._stop = True
        self.cmd.close()

    def _loop(self):
        while not self._stop:
            got = self._new.wait(timeout=float(self.get_parameter('frame_timeout_s').value))
            self._new.clear()
            if not got:
                capture = self._last_capture if self._last_capture is not None else time.monotonic_ns()
                self.cmd.publish(self._cmd_type(), ControlCommand.ST_SENSOR_INVALID,
                                 ZERO_TARGET, 0.0, capture)
                self._log('프레임이 안 들어옴 -> status=2 발행')
                continue
            frame = self.adapter.get_latest()
            if frame is None or frame.seq == self._last_seq:
                continue
            self._last_seq = frame.seq
            self._last_capture = frame.capture_mono_ns
            self._process(frame)

    def _det_fault(self):
        g = self.get_parameter
        return FaultConfig(mode=str(g('det_fault_mode').value),
                           p_delay_ms=float(g('det_fault_p_delay_ms').value),
                           p_bad_offset_m=float(g('det_fault_p_bad_offset_m').value))

    def _detect_primary(self, f, v_det):
        impl = str(self.get_parameter('primary_impl').value)
        if impl == 'mock_depth':
            return v_det          # P를 V와 같은 결과로 흉내냄: 장애 주입으로 전환 시험용
        if impl == 'color':
            g = self.get_parameter
            return detect_plate_color(
                f.rgb, f.depth, int(g('color_v_min').value), int(g('color_chroma_max').value),
                int(g('color_min_pixels').value),
                (float(g('plate_aspect_min').value), float(g('plate_aspect_max').value)))
        if impl == 'yolo':
            if self._yolo is None:
                return None
            return detect_plate_yolo(self._yolo, f.rgb, f.depth)
        return None               # 'none': 실제 P(색상·형상 또는 YOLO)는 이후 추가

    def _to_map(self, det, ci, tf):
        p_link = pixel_to_camera_link(det.u, det.v, det.depth_m, ci.fx, ci.fy, ci.cx, ci.cy)
        t, q = tf.transform.translation, tf.transform.rotation
        return tuple(float(v) for v in camera_link_to_map(
            p_link, [t.x, t.y, t.z], [q.x, q.y, q.z, q.w]))

    def _process(self, f):
        self.cmd.set_extra(infer_start_mono_ns=time.monotonic_ns())
        get = self.get_parameter
        if str(get('detector_mode').value) == 'verifier' or not f.valid:
            self.cmd.set_extra(active_detector='V')
            return self._process_legacy(f)
        t_v0 = time.perf_counter()
        v_det = detect_plate(
            f.depth, float(get('table_depth_m').value), float(get('min_height_m').value),
            float(get('max_height_m').value), int(get('min_pixels').value),
            fx=(float(f.camera_info.fx) if bool(get('edge_fit').value) else 0.0),
            size_uv_m=(float(get('plate_size_u_m').value), float(get('plate_size_v_m').value)))
        v_ms = (time.perf_counter() - t_v0) * 1000.0
        t0 = time.perf_counter()
        p_det = self._detect_primary(f, v_det)
        p_ms = (time.perf_counter() - t0) * 1000.0
        p_found = p_det is not None and p_det.found
        tf = None
        if v_det.found or p_found:
            try:
                tf = self.tf_buffer.lookup_transform(
                    str(get('map_frame').value), str(get('camera_frame').value), Time())
            except Exception as e:
                self.cmd.publish(self._cmd_type(), ControlCommand.ST_UNKNOWN,
                                 ZERO_TARGET, 0.0, f.capture_mono_ns)
                self._log(f'TF 조회 실패: {e}')
                return
        ci = f.camera_info
        v_xyz = self._to_map(v_det, ci, tf) if v_det.found else None
        p_xyz = self._to_map(p_det, ci, tf) if p_found else None
        v_ratio = v_det.depth_valid_ratio if v_det.found else 0.0
        fin = FrameInput(
            t_s=time.monotonic(), p_xyz=p_xyz,
            p_conf=(p_det.det_conf if p_found else 0.0), p_infer_ms=p_ms,
            v_xyz=v_xyz, v_valid_ratio=v_ratio,
            v_conf=(min(v_det.det_conf, v_ratio) if v_det.found else 0.0))
        fault = self._det_fault()
        fin = apply_detector_fault(fin, fault)
        d = self.sw.step(fin)
        self.cmd.set_extra(
            active_detector=d.source or d.active_detector,
            p_infer_ms=f'{p_ms:.2f}', v_infer_ms=f'{v_ms:.2f}',
            v_ref_depth_m=f'{v_det.ref_depth_m:.4f}', v_pixels=int(v_det.pixel_count),
            v_bbox_w=(v_det.bbox[2] - v_det.bbox[0] + 1 if v_det.found else ''),
            v_bbox_h=(v_det.bbox[3] - v_det.bbox[1] + 1 if v_det.found else ''),
            v_reason=v_det.reason,
            disagree_m=('' if d.disagree_m is None else f'{d.disagree_m:.4f}'),
            switch_event=d.switch_event, det_fault_mode=fault.mode)
        if d.switch_event != 'none':
            self.get_logger().info(f'[검출기 전환] {d.switch_event} (active={d.active_detector})')
        if d.xyz is None:
            self.cmd.publish(self._cmd_type(), ControlCommand.ST_NOT_DETECTED,
                             ZERO_TARGET, 0.0, f.capture_mono_ns)
            self._log(f'철판 못 찾음: P/V 모두 미검출 (state={d.frame_state})')
            return
        out = self._smooth(list(d.xyz))
        if out is None:
            self.cmd.publish(self._cmd_type(), ControlCommand.ST_UNKNOWN,
                             ZERO_TARGET, 0.0, f.capture_mono_ns)
            self._log('점프 거부 -> status=3')
            return
        x, y, z = [float(v) for v in out]
        msg = self.cmd.publish(self._cmd_type(), ControlCommand.ST_OK,
                               [x, y, z, 0.0], d.confidence, f.capture_mono_ns)
        if msg is None:
            self._log(f'[fault={get("fault_mode").value}] 발행 중단 중')
            return
        self._log(f'seq={msg.seq} src={d.source} state={d.active_detector} map=({x:.3f},{y:.3f},{z:.3f}) '
                  f'conf={d.confidence:.2f} disagree={d.disagree_m} age={msg.age_ns / 1e6:.1f}ms')

    def _process_legacy(self, f):
        get = self.get_parameter
        if not f.valid:
            self.cmd.publish(self._cmd_type(), ControlCommand.ST_SENSOR_INVALID,
                             ZERO_TARGET, 0.0, f.capture_mono_ns)
            self._log(f'센서 무효: {f.error_reason}')
            return
        det = detect_plate(
            f.depth, float(get('table_depth_m').value), float(get('min_height_m').value),
            float(get('max_height_m').value), int(get('min_pixels').value))
        if not det.found:
            self.cmd.publish(self._cmd_type(), ControlCommand.ST_NOT_DETECTED,
                             ZERO_TARGET, 0.0, f.capture_mono_ns)
            self._log(f'철판 못 찾음: {det.reason} (pixels={det.pixel_count}, ref={det.ref_depth_m:.3f}m)')
            return
        try:
            tf = self.tf_buffer.lookup_transform(
                str(get('map_frame').value), str(get('camera_frame').value), Time())
        except Exception as e:
            self.cmd.publish(self._cmd_type(), ControlCommand.ST_UNKNOWN,
                             ZERO_TARGET, 0.0, f.capture_mono_ns)
            self._log(f'TF 조회 실패: {e}')
            return
        ci = f.camera_info
        p_link = pixel_to_camera_link(det.u, det.v, det.depth_m, ci.fx, ci.fy, ci.cx, ci.cy)
        t, q = tf.transform.translation, tf.transform.rotation
        x, y, z = [float(v) for v in camera_link_to_map(
            p_link, [t.x, t.y, t.z], [q.x, q.y, q.z, q.w])]
        conf = min(det.det_conf, det.depth_valid_ratio)
        out = self._smooth([x, y, z])
        if out is None:
            self.cmd.publish(self._cmd_type(), ControlCommand.ST_UNKNOWN,
                             ZERO_TARGET, 0.0, f.capture_mono_ns)
            self._log(f'점프 거부: ({x:.3f},{y:.3f},{z:.3f}) -> status=3')
            return
        x, y, z = [float(v) for v in out]
        msg = self.cmd.publish(self._cmd_type(), ControlCommand.ST_OK,
                               [x, y, z, 0.0], conf, f.capture_mono_ns)
        if msg is None:
            self._log(f'[fault={self.get_parameter("fault_mode").value}] 발행 중단 중')
            return
        self._log(f'seq={msg.seq} 철판 map=({x:.3f},{y:.3f},{z:.3f}) u={det.u:.0f} v={det.v:.0f} depth={det.depth_m:.3f}m '
                  f'pixels={det.pixel_count} conf={conf:.2f} age={msg.age_ns / 1e6:.1f}ms')

    def _cmd_type(self):
        name = str(self.get_parameter('command_type').value)
        return getattr(ControlCommand, 'CMD_PATH_ADJUST', 2) if name == 'path_adjust' else ControlCommand.CMD_POSITION

    def _smooth(self, xyz):
        get = self.get_parameter
        limit = float(get('jump_limit_m').value)
        n = max(1, int(get('smooth_n').value))
        xyz = np.asarray(xyz, dtype=float)
        if self._last_out is not None and np.linalg.norm(xyz - self._last_out) > limit:
            self._jump_count += 1
            if self._jump_count < int(get('jump_accept_after').value):
                return None
            self._hist.clear()
        self._jump_count = 0
        self._hist.append(xyz)
        out = np.median(np.array(list(self._hist)[-n:]), axis=0)
        self._last_out = out
        return out

    def _log(self, text):
        now = time.monotonic()
        if now - self._last_log >= 1.0:
            self._last_log = now
            self.get_logger().info(text)


def main():
    rclpy.init()
    node = AiInferenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
