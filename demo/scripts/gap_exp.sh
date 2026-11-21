#!/usr/bin/env bash
# 사용: gap_exp.sh <control|cli|stress> [측정초=180]
#  control: 아무것도 안 함 / cli: 10초마다 ros2 param get / stress: 30초마다 CPU 10초 점유
#  먼저 다른 터미널에서 시뮬(pac_bringup sim.launch.py)을 켜 두세요. 기록은 ~/pac_logs/<run_id>/ 에 남아요.
#  워크스페이스 위치를 바꾸려면 PAC_WS=/경로/PAC2026 gap_exp.sh ... (기본: 이 스크립트가 있는 저장소)
set -m
HERE="$(cd "$(dirname "$0")" && pwd)"
WS="${PAC_WS:-$(cd "$HERE/../.." && pwd)}"
MODE=${1:?control|cli|stress}; DUR=${2:-180}
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
ros2 topic list 2>/dev/null | grep -q /depth_camera/image_raw || { echo "시뮬이 안 떠 있어요 (터미널 1에서 sim.launch.py 먼저)"; exit 1; }
RUN=$(date +%Y%m%d_%H%M)_test_gap-$MODE
D=~/pac_logs/$RUN; mkdir -p "$D"
ros2 launch ai_inference ai_inference.launch.py run_id:=$RUN > "$D/a.log" 2>&1 &
APID=$!
vmstat -t 1 > "$D/vm.log" 2>&1 &
VPID=$!
sleep 15                     # 연결 대기 + 첫 추론 워밍업(약 1.2초)
echo "$(date +%s.%N) start" > "$D/events.log"
END=$(( $(date +%s) + DUR ))
case $MODE in
  cli)
    while [ "$(date +%s)" -lt "$END" ]; do
      echo "$(date +%s.%N) cli" >> "$D/events.log"
      ros2 param get /ai_inference_node fault_mode > /dev/null 2>&1
      sleep 10
    done ;;
  stress)
    while [ "$(date +%s)" -lt "$END" ]; do
      echo "$(date +%s.%N) stress" >> "$D/events.log"
      PIDS=""
      for i in $(seq "$(nproc)"); do timeout 10 sh -c 'while :; do :; done' & PIDS="$PIDS $!"; done
      wait $PIDS 2>/dev/null
      sleep 20
    done ;;
  control) sleep "$DUR" ;;
esac
kill -INT $APID 2>/dev/null; pkill -INT -f ai_inference_node; sleep 2; kill $VPID 2>/dev/null
echo "================ $RUN"
python3 -I "$HERE/gap_check.py" "$D" | grep -E "행 |발행 간격\(|150ms 넘는|150ms 초과|추론 시간"
python3 -I - "$D" <<'PY'
import csv, sys
d = sys.argv[1]
rows = list(csv.DictReader(open(d + '/ai.csv', encoding='utf-8')))
ev = [float(l.split()[0]) for l in open(d + '/events.log') if l.strip()]
t0 = ev[0]; marks = ev[1:]
prev = None; n = 0
print("\n시작 후 150ms 넘는 끊김 (정상 행 사이):")
for r in rows:
    if r['status'] != '0':
        prev = None; continue
    if prev is not None:
        g = (int(r['publish_mono_ns']) - int(prev['publish_mono_ns'])) / 1e6
        w = int(r['wall_time_ns']) / 1e9
        if g > 150 and w > t0:
            near = min(abs(w - e) for e in marks) if marks else None
            s = f", 가장 가까운 호출/부하 시작과 {near:.1f}초 차이" if near is not None else ""
            print(f"  wall {w:.3f}  간격 {g:.0f}ms{s}")
            n += 1
    prev = r
print(f"  → {n}건")
PY
echo "기록 폴더: $D (vm.log = vmstat)"
