import csv
import os
import time

FAULT_MODES = ('none', 'delay', 'stop', 'bad_value')
BAD_TARGET = [9.9, 9.9, 9.9, 0.0]
EXTRA_COLUMNS = ['active_detector', 'disagree_m', 'switch_event', 'det_fault_mode',
                 'capture_mono_ns', 'infer_start_mono_ns', 'infer_end_mono_ns', 'publish_mono_ns',
                 'p_infer_ms', 'v_infer_ms',
                 'v_ref_depth_m', 'v_pixels', 'v_bbox_w', 'v_bbox_h', 'v_reason']
COLUMNS = ['wall_time_ns', 'mono_ns', 'seq', 'boot_id', 'command_type', 'status',
           'x', 'y', 'z', 'confidence', 'age_ns', 'fault_mode', 'published'] + EXTRA_COLUMNS


class FaultyPublisher:
    """CommandPublisher를 감싸서 결함을 주입하고, 발행마다 CSV 한 줄을 남긴다."""

    def __init__(self, inner, node):
        self._inner = inner
        self._node = node
        log_root = os.path.expanduser(str(node.get_parameter('log_dir').value))
        run_id = str(node.get_parameter('run_id').value).strip()
        if run_id:
            # 정한 run_id: A·B·C가 같은 폴더를 쓴다. B(rt.csv)와 같은 규칙으로 덮어쓰지 않고 멈춘다.
            if '/' in run_id or run_id in ('.', '..'):
                raise ValueError(f"run_id에 '/'를 넣거나 '.', '..'를 쓸 수 없음: {run_id}")
            log_dir = os.path.join(log_root, run_id)
            os.makedirs(log_dir, exist_ok=True)
            self.path = os.path.join(log_dir, 'ai.csv')
            if os.path.exists(self.path):
                raise FileExistsError(f'이미 ai.csv가 있음 (덮어쓰지 않음): {self.path}')
        else:
            os.makedirs(log_root, exist_ok=True)
            stamp = time.strftime('%Y%m%d_%H%M%S')
            self.path = os.path.join(log_root, f'ai_{stamp}.csv')
        self._file = open(self.path, 'w', newline='')
        self._writer = csv.writer(self._file)
        self._writer.writerow(COLUMNS)
        self._rows = 0
        self._extra = {}
        node.get_logger().info(f'로그 파일: {self.path}')

    def __getattr__(self, name):
        return getattr(self._inner, name)

    def set_extra(self, **kw):
        """다음 CSV 한 줄에만 붙는 추가 칼럼 값을 설정한다."""
        self._extra.update(kw)

    def _mode(self):
        mode = str(self._node.get_parameter('fault_mode').value)
        return mode if mode in FAULT_MODES else 'none'

    def publish(self, command_type, status, target, confidence, capture_mono_ns):
        get = self._node.get_parameter
        mode = self._mode()
        self._extra.update(capture_mono_ns=int(capture_mono_ns), infer_end_mono_ns=time.monotonic_ns())
        if mode == 'stop':
            self._log(None, command_type, status, target, confidence, mode, published=0)
            return None
        if mode == 'delay':
            time.sleep(max(0.0, float(get('fault_delay_ms').value)) / 1000.0)
        if mode == 'bad_value':
            status, target, confidence = 0, list(BAD_TARGET), 1.0
        msg = self._inner.publish(command_type, status, target, confidence, capture_mono_ns)
        self._extra['publish_mono_ns'] = int(capture_mono_ns) + int(msg.age_ns)
        self._log(msg, command_type, status, target, confidence, mode, published=1)
        return msg

    def _log(self, msg, command_type, status, target, confidence, mode, published):
        if msg is not None:
            seq, boot, age = msg.seq, msg.writer_boot_id, msg.age_ns
            command_type, status, confidence = msg.command_type, msg.status, msg.confidence
            target = list(msg.target)
        else:
            seq, boot, age = '', self._inner.boot_id, ''
        self._writer.writerow([
            time.time_ns(), time.monotonic_ns(), seq, boot, command_type, status,
            f'{target[0]:.5f}', f'{target[1]:.5f}', f'{target[2]:.5f}', f'{confidence:.3f}',
            age, mode, published] + [self._extra.get(c, '') for c in EXTRA_COLUMNS])
        self._extra = {}
        self._rows += 1
        if self._rows % 15 == 0:
            self._file.flush()

    def close(self):
        try:
            self._file.flush()
            self._file.close()
        except Exception:
            pass
