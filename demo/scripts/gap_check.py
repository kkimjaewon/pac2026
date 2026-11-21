#!/usr/bin/env python3
"""A ai.csv 발행 끊김 분석 + B.log HOLD 대조.
사용: python3 -I ~/gap_check.py <run폴더>   (폴더 안에 ai.csv, B.log, demo_start, demo_end)
"""
import csv, re, sys, os, statistics as st

def pct(v, p):
    v = sorted(v)
    return v[min(len(v) - 1, int(len(v) * p))] if v else float('nan')

def f(x, d=0):
    try:
        return float(x)
    except Exception:
        return d

d = sys.argv[1] if len(sys.argv) > 1 else '.'
thr = float(sys.argv[2]) if len(sys.argv) > 2 else 150.0
rows = list(csv.DictReader(open(os.path.join(d, 'ai.csv'), encoding='utf-8')))
print(f"행 {len(rows)}개")
from collections import Counter
print("status별:", dict(Counter(r['status'] for r in rows)), "| published:", dict(Counter(r['published'] for r in rows)))

def wall(r): return int(r['wall_time_ns']) / 1e9
def mono(r): return int(r['mono_ns']) / 1e6          # ms
def pub(r): return f(r['publish_mono_ns']) / 1e6 if f(r['publish_mono_ns']) else mono(r)

t0 = wall(rows[0]); t1 = wall(rows[-1])
print(f"기록 길이 {t1 - t0:.1f} s (wall {t0:.3f} ~ {t1:.3f})")
try:
    ds = float(open(os.path.join(d, 'demo_start')).read()); de = float(open(os.path.join(d, 'demo_end')).read())
    print(f"장면 스크립트 구간 wall {ds:.3f} ~ {de:.3f} ({de - ds:.1f} s)")
except Exception:
    ds = de = None

# 1) 정상 행(status 0) 사이 발행 간격
ok = [r for r in rows if r['status'] == '0']
print(f"\n[1] status 0(정상) 행 {len(ok)}개, 그 외 {len(rows) - len(ok)}개")
gaps = []
for a, b in zip(rows, rows[1:]):
    g = pub(b) - pub(a)
    gaps.append((g, a, b))
vals = [g for g, a, b in gaps]
print(f"발행 간격(전체 행 기준) 중앙 {st.median(vals):.1f} ms, p99 {pct(vals, .99):.1f} ms, 최대 {max(vals):.1f} ms")
bad = [(g, a, b) for g, a, b in gaps if g > thr and not (a['status'] != '0' and b['status'] != '0')]  # 대기행(status!=0)끼리는 원래 200ms 간격
print(f"{thr:.0f}ms 넘는 간격 {len(bad)}건")

print("\n[2] 긴 간격 상세 (원인 구분)")
print("  가=촬영 간격(프레임이 늦게 옴) / 나=촬영→추론 시작 대기 / 다=추론 시간 / 라=추론 끝→발행")
print("  wall(s)      seq(앞→뒤)    간격   앞행status 뒤행status | 촬영간격  대기  추론  발행지연")
for g, a, b in bad[:60]:
    ca, cb = f(a['capture_mono_ns']) / 1e6, f(b['capture_mono_ns']) / 1e6
    cap = (cb - ca) if ca and cb else float('nan')
    wait = (f(b['infer_start_mono_ns']) - f(b['capture_mono_ns'])) / 1e6 if cb else float('nan')
    inf = (f(b['infer_end_mono_ns']) - f(b['infer_start_mono_ns'])) / 1e6 if cb else float('nan')
    lag = (f(b['publish_mono_ns']) - f(b['infer_end_mono_ns'])) / 1e6 if cb else float('nan')
    print(f"  {wall(a):.3f}  {a['seq']:>5}→{b['seq']:<5} {g:7.1f}   {a['status']:>3} {b['status']:>3}      | {cap:7.1f} {wait:6.1f} {inf:6.1f} {lag:6.1f}")
if len(bad) > 60:
    print(f"  ... {len(bad) - 60}건 더 있음")

# 3) 프레임 없는 구간(status 2 연속)
print("\n[3] status!=0 연속 구간 (A가 프레임을 못 받았거나 거절한 구간)")
seg = None; segs = []
for r in rows:
    s = r['status'] != '0'
    if s and seg is None: seg = [r, r]
    elif s: seg[1] = r
    elif seg: segs.append(seg); seg = None
if seg: segs.append(seg)
for a, b in segs:
    dur = wall(b) - wall(a)
    if dur >= 0.1:
        print(f"  wall {wall(a):.3f} ~ {wall(b):.3f} ({dur:.2f} s) seq {a['seq']}~{b['seq']} status {a['status']}")
print(f"  (0.1초 이상 구간만 표시, 전체 {len(segs)}개)")

# 4) 정상 행 사이 촬영 간격 분포
cs = [f(r['capture_mono_ns']) / 1e6 for r in rows if r['status'] == '0' and f(r['capture_mono_ns'])]
cg = [b - a for a, b in zip(cs, cs[1:])]
if cg:
    print(f"\n[4] 정상 프레임 촬영 간격: 중앙 {st.median(cg):.1f} ms, p99 {pct(cg, .99):.1f} ms, 최대 {max(cg):.1f} ms, {thr:.0f}ms 초과 {sum(1 for x in cg if x > thr)}건")
inf = [(f(r['infer_end_mono_ns']) - f(r['infer_start_mono_ns'])) / 1e6 for r in rows if r['status'] == '0' and f(r['infer_end_mono_ns'])]
if inf:
    print(f"    추론 시간: 중앙 {st.median(inf):.1f} ms, p99 {pct(inf, .99):.1f} ms, 최대 {max(inf):.1f} ms")

# 5) B.log HOLD 이벤트와 대조
bl = os.path.join(d, 'B.log')
if os.path.exists(bl):
    ev = []
    for ln in open(bl, encoding='utf-8', errors='replace'):
        m = re.search(r'\[(\d{10}\.\d+)\].*\[모드\] (\w+) → (\w+)(?: \(이유: ([^)]*)\))?', ln)
        if m: ev.append((float(m.group(1)), m.group(2), m.group(3), m.group(4) or ''))
    print(f"\n[5] B 모드 전환 {len(ev)}건 (RUN→HOLD 와 복귀)")
    i = 0
    for t, a, b, why in ev:
        if a == 'RUN' and b == 'HOLD':
            back = next(((t2, b2) for t2, a2, b2, _ in ev if t2 > t and a2 == 'HOLD'), None)   # HOLD를 벗어나는 첫 전환(RUN이 아니어도)
            near = min(bad, key=lambda x: abs(wall(x[1]) - t), default=None)
            nstr = ''
            if near and abs(wall(near[1]) - t) < 1.0:
                nstr = f" | 가까운 A 간격: seq {near[1]['seq']}→{near[2]['seq']} {near[0]:.0f}ms"
            ins = [s for s in segs if wall(s[0]) - 0.5 <= t <= wall(s[1]) + 0.5]
            sstr = f" | A status!=0 구간 {wall(ins[0][0]):.2f}~{wall(ins[0][1]):.2f}s" if ins else ''
            hold = (f"{back[0] - t:.2f}s" + ('' if back[1] == 'RUN' else f"(→{back[1]})")) if back else '끝까지'
            rb = next((t2 for t2, a2, b2, _ in ev if t2 > t and b2 == 'RUN'), None)
            tot = f' | RUN 복귀까지 총 {rb - t:.2f}s' if rb else ' | RUN 복귀 없음'
            print(f"  wall {t:.3f} RUN→HOLD({why}) 지속 {hold}{tot}{nstr}{sstr}")
            i += 1
    if i == 0: print("  RUN→HOLD 없음")
