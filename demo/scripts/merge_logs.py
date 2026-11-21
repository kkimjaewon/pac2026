#!/usr/bin/env python3
"""A·B·C 시험 기록을 합쳐 'AI 결과 -> 제어 반영' 구간별 지연을 계산한다. (요청 사항 AB-07)

사용법
  python3 merge_logs.py <run_id>                      # ~/pac_logs/<run_id>/ 의 ai.csv, ai_rx.csv, rt.csv, hal.csv
  python3 merge_logs.py --ai A.csv --rx ai_rx.csv --rt rt.csv [--hal hal.csv]   # 파일을 직접 지정
옵션
  --log-root ~/pac_logs   run_id 폴더가 있는 곳
  --a-clock same|other    A가 B·C와 같은 PC면 same(기본), 다른 PC면 other (A 시각을 B·C 시각에서 빼지 않음)
  --out-csv 파일          AI 결과 한 줄당 합친 표를 저장

연결 키
  A ai.csv (boot_id, seq) == B ai_rx.csv (writer_boot_id, seq) == B rt.csv (used_ai_boot_id, used_ai_seq)
  B rt.csv command_seq == C hal.csv command_seq   (hal.csv의 source_result_id는 rt.csv로 연결이 안 될 때만 사용)
"""
import argparse
import csv
import math
import os
import sys
from collections import Counter, defaultdict

NOTE_APPLIED = ("'로봇 반영(목표 채택)'은 Gazebo C 수신단 제어기가 관절 목표를 채택한 시각이며, "
                "실제 TCP 목표 도착이나 물리 실행 완료 시각은 아니다.")


def to_int(v):
    if v is None:
        return None
    v = str(v).strip()
    if v == "":
        return None
    try:
        return int(v)
    except ValueError:
        try:
            return int(float(v))
        except ValueError:
            return None


def pos(v):
    """0 또는 빈 값은 '기록 없음'으로 본다(mono ns 칼럼용)."""
    x = to_int(v)
    return x if (x is not None and x > 0) else None


def read_csv(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def pct(sorted_vals, p):
    k = (len(sorted_vals) - 1) * p / 100.0
    lo, hi = int(math.floor(k)), int(math.ceil(k))
    return sorted_vals[lo] + (sorted_vals[hi] - sorted_vals[lo]) * (k - lo)


def stats_ms(ns_list):
    v = sorted(x / 1e6 for x in ns_list)
    if not v:
        return None
    return dict(n=len(v), mean=sum(v) / len(v), p50=pct(v, 50), p95=pct(v, 95), p99=pct(v, 99), max=v[-1])


def fmt(s):
    if s is None:
        return "| 0 | - | - | - | - | - |"
    return "| %d | %.2f | %.2f | %.2f | %.2f | %.2f |" % (s["n"], s["mean"], s["p50"], s["p95"], s["p99"], s["max"])


def diff(a, b):
    """b - a (ns). 둘 중 하나라도 없으면 None."""
    return None if (a is None or b is None) else b - a


def main():
    ap = argparse.ArgumentParser(description="A·B·C 로그 병합과 지연 계산")
    ap.add_argument("run_id", nargs="?", help="~/pac_logs/<run_id>/ 폴더 이름")
    ap.add_argument("--log-root", default="~/pac_logs")
    ap.add_argument("--ai")
    ap.add_argument("--rx")
    ap.add_argument("--rt")
    ap.add_argument("--hal")
    ap.add_argument("--a-clock", choices=["same", "other"], default="same")
    ap.add_argument("--out-csv")
    args = ap.parse_args()

    if args.run_id:
        d = os.path.join(os.path.expanduser(args.log_root), args.run_id)
        args.ai = args.ai or os.path.join(d, "ai.csv")
        args.rx = args.rx or os.path.join(d, "ai_rx.csv")
        args.rt = args.rt or os.path.join(d, "rt.csv")
        hal = os.path.join(d, "hal.csv")
        args.hal = args.hal or (hal if os.path.exists(hal) else None)
    if not (args.ai and args.rx and args.rt):
        ap.error("run_id 또는 --ai --rx --rt 가 필요합니다")
    for p in (args.ai, args.rx, args.rt):
        if not os.path.exists(p):
            sys.exit("파일이 없습니다: %s" % p)
    same = args.a_clock == "same"

    # ---------------- A ai.csv ----------------
    a_rows = [r for r in read_csv(args.ai) if to_int(r.get("published", "1")) != 0]
    boots = {str(r.get("boot_id", "")) for r in a_rows}
    default_boot = next(iter(boots)) if len(boots) == 1 else None
    A = {}
    for r in a_rows:
        seq = to_int(r.get("seq"))
        if seq is None:
            continue
        A[(str(r.get("boot_id", "")), seq)] = dict(
            capture=pos(r.get("capture_mono_ns")), infer_start=pos(r.get("infer_start_mono_ns")),
            infer_end=pos(r.get("infer_end_mono_ns")), publish=pos(r.get("publish_mono_ns")),
            status=to_int(r.get("status")), conf=r.get("confidence"), det=r.get("active_detector", ""))

    # ---------------- B ai_rx.csv ----------------
    rx_rows = read_csv(args.rx)
    RX, rx_dropped = {}, 0
    for r in rx_rows:
        if to_int(r.get("dropped")) == 1:
            rx_dropped += 1
            continue
        key = (str(r.get("writer_boot_id", "")), to_int(r.get("seq")))
        RX.setdefault(key, pos(r.get("recv_mono_ns")))
    missing = 0
    by_boot = defaultdict(list)
    for (b, s) in RX:
        by_boot[b].append(s)
    for b, ss in by_boot.items():
        missing += (max(ss) - min(ss) + 1) - len(set(ss))

    # ---------------- B rt.csv ----------------
    rt_rows = read_csv(args.rt)
    has_boot_col = bool(rt_rows) and "used_ai_boot_id" in rt_rows[0]
    APPLY, cmd2ai = {}, {}
    for r in rt_rows:
        if to_int(r.get("has_used")) != 1:
            continue
        seq = to_int(r.get("used_ai_seq"))
        boot = str(r.get("used_ai_boot_id")) if has_boot_col else default_boot
        if seq is None or boot is None:
            continue
        key = (boot, seq)
        ap_ns, rv_ns = pos(r.get("ai_apply_mono_ns")), pos(r.get("ai_recv_mono_ns"))
        if ap_ns is not None and (key not in APPLY or ap_ns < APPLY[key][0]):
            APPLY[key] = (ap_ns, rv_ns)
        cs = to_int(r.get("command_seq"))
        if cs:
            cmd2ai.setdefault(cs, key)

    # ---------------- C hal.csv ----------------
    HAL, hal_result, hal_err = {}, Counter(), Counter()
    if args.hal:
        grouped = defaultdict(list)
        for r in read_csv(args.hal):
            grouped[(r.get("session_id"), to_int(r.get("command_seq")))].append(r)
        for (sess, cs), rows in grouped.items():
            m = {}
            for r in rows:                      # 같은 명령의 sent·applied 행은 한 명령으로 합친다
                for k, v in r.items():
                    if v not in (None, ""):
                        m[k] = v
            results = {(r.get("result") or "") for r in rows}
            res = ("applied" if "applied" in results else "rejected" if "rejected" in results
                   else "superseded" if "superseded" in results else "sent")
            if pos(m.get("receiver_applied_mono_ns")) and res == "sent":
                res = "applied"
            hal_result[res] += 1
            if res == "rejected":
                hal_err[m.get("error_code", "?")] += 1
            HAL[(sess, cs)] = (res, m)

    # ---------------- 구간 계산 ----------------
    seg = defaultdict(list)
    merged = {}

    for key, a in A.items():
        row = merged.setdefault(key, {"boot_id": key[0], "seq": key[1]})
        if a["capture"] and a["publish"]:
            seg["A 내부(촬영 -> 발행)"].append(a["publish"] - a["capture"])
            row["a_age_ns"] = a["publish"] - a["capture"]
        if a["infer_start"] and a["infer_end"]:
            seg["A 추론(시작 -> 끝)"].append(a["infer_end"] - a["infer_start"])
        rv = RX.get(key)
        if rv is not None:
            row["rx_recv_ns"] = rv
            if same and a["publish"]:
                seg["A 발행 -> B 수신 (같은 PC)"].append(rv - a["publish"])
                row["a_to_b_ns"] = rv - a["publish"]
        if key in APPLY:
            ap_ns, rv_ns = APPLY[key]
            if rv_ns:
                seg["B 수신 -> B 반영"].append(ap_ns - rv_ns)
            if same and a["capture"]:
                seg["카메라 촬영 -> B 반영 (같은 PC)"].append(ap_ns - a["capture"])
            row["b_apply_ns"] = ap_ns

    first_applied = {}                          # AI 결과마다 처음 목표 채택된 명령
    for (sess, cs), (res, m) in HAL.items():
        b_c, rcv, val = pos(m.get("b_created_mono_ns")), pos(m.get("received_mono_ns")), pos(m.get("validated_mono_ns"))
        ik, ts, te = pos(m.get("ik_end_mono_ns")), pos(m.get("transport_start_mono_ns")), pos(m.get("transport_end_mono_ns"))
        app, ack = pos(m.get("receiver_applied_mono_ns")), pos(m.get("ack_received_mono_ns"))
        if res in ("rejected", "superseded"):
            continue                            # 정상 지연 통계에서 제외 (개수만 센다)
        for name, a_, b_ in (("B 명령 생성 -> C 수신", b_c, rcv), ("C 사전 검사", rcv, val), ("C IK", val, ik),
                             ("C 전송 API 호출 (transport 시작 -> 끝)", ts, te),
                             ("C 전송 시작 -> 목표 채택 (위 구간과 겹침)", ts, app), ("목표 채택 -> ACK 수신", app, ack),
                             ("B 명령 생성 -> 목표 채택", b_c, app)):
            v = diff(a_, b_)
            if v is not None and v >= 0:
                seg[name].append(v)
        key = cmd2ai.get(cs)
        if key is None and to_int(m.get("source_result_id")) and default_boot is not None:
            key = (default_boot, to_int(m.get("source_result_id")))
        if key is not None and app is not None and (key not in first_applied or app < first_applied[key]):
            first_applied[key] = app

    for key, app in first_applied.items():
        a = A.get(key)
        row = merged.setdefault(key, {"boot_id": key[0], "seq": key[1]})
        row["c_applied_ns"] = app
        rv_ns = APPLY[key][1] if key in APPLY else None
        if a and a["capture"] and a["publish"] and rv_ns:
            # 시계가 다른 구간(A)과 B·C 구간을 합산: A 내부 age + (B 수신 -> C 목표 채택). A->B 네트워크는 제외.
            seg["AI 결과 -> 목표 채택 (A age + B 수신~C 채택, 네트워크 제외)"].append(
                (a["publish"] - a["capture"]) + (app - rv_ns))
        if same and a and a["capture"]:
            seg["카메라 촬영 -> 목표 채택 (같은 PC)"].append(app - a["capture"])

    # ---------------- 출력 ----------------
    print("# merge_logs 결과\n")
    print("- A ai.csv: %s (발행 %d개, boot_id %d종)" % (args.ai, len(A), len(boots)))
    print("- B ai_rx.csv: %s (수신 %d개, 중복·버림 %d개, seq 빠짐 %d개)" % (args.rx, len(RX), rx_dropped, missing))
    print("- B rt.csv: %s (AI 반영 %d개, boot 열 %s)" % (args.rt, len(APPLY), "있음" if has_boot_col else "없음(A boot_id로 대체)"))
    if args.hal:
        n = sum(hal_result.values())
        applied = hal_result["applied"]
        ack = sum(1 for _, (r, m) in HAL.items() if r == "applied" and pos(m.get("ack_received_mono_ns")))
        print("- C hal.csv: %s (명령 %d개: %s)" % (args.hal, n, ", ".join("%s %d" % kv for kv in sorted(hal_result.items()))))
        if hal_err:
            print("  - 거부 코드별: %s" % ", ".join("code %s: %d" % kv for kv in sorted(hal_err.items())))
        print("  - ACK 관측률: %d / %d (ACK가 없으면 0이나 실패로 채우지 않고 제외)" % (ack, applied))
    else:
        print("- C hal.csv: 없음 (A -> B 구간까지만 계산)")
    print("- A 시계: %s\n" % ("B·C와 같은 PC" if same else "다른 PC (A 시각을 B·C 시각에서 직접 빼지 않음)"))

    print("| 구간 | 개수 | 평균 ms | p50 ms | p95 ms | p99 ms | 최대 ms |")
    print("|---|---|---|---|---|---|---|")
    order = ["A 내부(촬영 -> 발행)", "A 추론(시작 -> 끝)", "A 발행 -> B 수신 (같은 PC)", "B 수신 -> B 반영",
             "카메라 촬영 -> B 반영 (같은 PC)", "B 명령 생성 -> C 수신", "C 사전 검사", "C IK",
             "C 전송 API 호출 (transport 시작 -> 끝)", "C 전송 시작 -> 목표 채택 (위 구간과 겹침)",
             "목표 채택 -> ACK 수신", "B 명령 생성 -> 목표 채택",
             "AI 결과 -> 목표 채택 (A age + B 수신~C 채택, 네트워크 제외)", "카메라 촬영 -> 목표 채택 (같은 PC)"]
    for name in order:
        if name in seg:
            print("| %s %s" % (name, fmt(stats_ms(seg[name]))))
    print("\n주: 'C 전송 API 호출'과 'C 전송 시작 -> 목표 채택'은 겹치는 구간이라 합산하지 않는다.")
    print("주: " + NOTE_APPLIED)

    if args.out_csv:
        cols = ["boot_id", "seq", "a_age_ns", "a_to_b_ns", "rx_recv_ns", "b_apply_ns", "c_applied_ns"]
        with open(os.path.expanduser(args.out_csv), "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=cols)
            w.writeheader()
            for key in sorted(merged, key=lambda k: (k[0], k[1])):
                w.writerow({c: merged[key].get(c, "") for c in cols})
        print("\n합친 표 저장: %s (AI 결과 %d행)" % (args.out_csv, len(merged)))


if __name__ == "__main__":
    main()
