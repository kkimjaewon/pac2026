#!/usr/bin/env python3
"""rt.csv 그래프·요약 도구

사용법: python3 tools/plot_rt.py ~/pac_logs/<run_id>

같은 폴더에 PNG 5개를 저장하고 터미널에 요약을 출력한다.
  1_jitter_hist.png    지터 히스토그램 (세로축 로그), 평균·99%·최대, 100us 기준선
  2_jitter_time.png    시간에 따른 지터
  3_goal_output.png    goal_y와 y_y, 배경에 모드 색
  4_weight_reject.png  반영 가중치 w와 gate 차단 이유
  5_compute_hist.png   compute_ns 히스토그램
"""
import os
import sys

try:
    import matplotlib
    matplotlib.use('Agg')  # 화면 없이 PNG만 저장
    import matplotlib.pyplot as plt
    import numpy as np
    import pandas as pd
except ImportError as e:
    print(f'필요한 파이썬 패키지가 없습니다: {e.name}')
    print('설치 명령 (Ubuntu 24.04):')
    print('  sudo apt install python3-pandas python3-matplotlib')
    sys.exit(1)


def use_korean_font():
    """설치된 한글 글꼴이 있으면 사용 (없으면 기본 글꼴, 한글이 네모로 보일 수 있음)"""
    from matplotlib import font_manager
    names = {f.name for f in font_manager.fontManager.ttflist}
    for name in ('Noto Sans CJK KR', 'NanumGothic', 'Noto Sans KR', 'UnDotum'):
        if name in names:
            plt.rcParams['font.family'] = name
            break
    plt.rcParams['axes.unicode_minus'] = False  # 한글 글꼴에서 마이너스 기호가 깨지지 않게


use_korean_font()

JITTER_LIMIT_US = 100.0
MODE_COLORS = {'RUN': None, 'HOLD': '#f6d55c', 'DECEL_STOP': '#ed8a3f', 'FAULT': '#e04040', 'INIT': '#bbbbbb'}
GATE_REASONS = ['NO_DATA', 'VERSION', 'TYPE', 'STATUS', 'EXPIRED', 'NAN', 'LOW_CONF', 'RANGE']


def mode_segments(t, mode):
    """연속된 같은 모드 구간 [(시작, 끝, 모드)]"""
    change = np.flatnonzero(mode.values[1:] != mode.values[:-1]) + 1
    starts = np.concatenate([[0], change])
    ends = np.concatenate([change, [len(mode)]])
    return [(t.iloc[s], t.iloc[e - 1], mode.iloc[s]) for s, e in zip(starts, ends)]


def shade_modes(ax, segments):
    used = set()
    for t0, t1, m in segments:
        color = MODE_COLORS.get(m)
        if color is None:
            continue
        ax.axvspan(t0, t1, color=color, alpha=0.35, lw=0, label=m if m not in used else None)
        used.add(m)


def true_intervals(t, flag):
    """flag가 True인 구간 [(시작, 길이)] (broken_barh용)"""
    f = flag.values.astype(np.int8)
    d = np.diff(np.concatenate([[0], f, [0]]))
    starts, ends = np.flatnonzero(d == 1), np.flatnonzero(d == -1) - 1
    tv = t.values
    return [(tv[s], max(tv[e] - tv[s], 0.001)) for s, e in zip(starts, ends)]


def main():
    if len(sys.argv) != 2:
        print(__doc__)
        sys.exit(2)
    run_dir = os.path.expanduser(sys.argv[1])
    path = os.path.join(run_dir, 'rt.csv')
    df = pd.read_csv(path, dtype={'mode_reason': str}, keep_default_na=False)
    if df.empty:
        print(f'{path}: 기록이 없습니다')
        sys.exit(1)

    t = (df['wake_mono_ns'] - df['wake_mono_ns'].iloc[0]) / 1e9
    jit = df['jitter_ns'] / 1e3
    comp = df['compute_ns'] / 1e3
    reject = df['reject_reason'].apply(lambda s: int(s, 16)).to_numpy(dtype=np.int64)  # Series는 >> 연산이 없음
    period_s = 1e-3

    # ---------------- 요약 ----------------
    q = jit.quantile([0.99, 0.999])
    print(f'실행 폴더: {run_dir}')
    print(f'전체 주기 수: {len(df)}  (기록 시간 {t.iloc[-1]:.3f} s)')
    print(f'overrun 합: {int(df["overrun"].sum())}')
    print(f'지터 [us]: 평균 {jit.mean():.2f} | 99% {q[0.99]:.2f} | 99.9% {q[0.999]:.2f} | 최대 {jit.max():.2f}'
          f' | {JITTER_LIMIT_US:.0f}us 초과 {int((jit > JITTER_LIMIT_US).sum())}')
    print(f'compute [us]: 평균 {comp.mean():.2f} | 최대 {comp.max():.2f}')
    print('모드별 시간:')
    for m, n in df['mode'].value_counts().items():
        print(f'  {m:<11} {n * period_s:8.3f} s ({100.0 * n / len(df):.1f}%)')
    print(f'반영 횟수: {int((df["weight"] > 0).sum())}')

    segments = mode_segments(t, df['mode'])
    saved = []

    def save(fig, name):
        out = os.path.join(run_dir, name)
        fig.tight_layout()
        fig.savefig(out, dpi=120)
        plt.close(fig)
        saved.append(out)

    # (1) 지터 히스토그램
    fig, ax = plt.subplots(figsize=(9, 5))
    ax.hist(jit, bins=200, color='#4c72b0')
    ax.set_yscale('log')
    ax.axvline(JITTER_LIMIT_US, color='red', ls='--', label=f'{JITTER_LIMIT_US:.0f} us 기준')
    ax.axvline(jit.mean(), color='green', ls=':', label=f'평균 {jit.mean():.1f} us')
    ax.axvline(q[0.99], color='orange', ls=':', label=f'99% {q[0.99]:.1f} us')
    ax.axvline(jit.max(), color='black', ls=':', label=f'최대 {jit.max():.1f} us')
    ax.set_xlabel('jitter [us] (깨어난 시각 - 예정 시각)')
    ax.set_ylabel('주기 수 (로그)')
    ax.set_title('1ms 루프 지터 분포')
    ax.legend()
    save(fig, '1_jitter_hist.png')

    # (2) 시간에 따른 지터
    fig, ax = plt.subplots(figsize=(11, 4))
    ax.plot(t, jit, lw=0.4, color='#4c72b0')
    ax.axhline(JITTER_LIMIT_US, color='red', ls='--', lw=1, label=f'{JITTER_LIMIT_US:.0f} us 기준')
    over = df['overrun'] > 0
    if over.any():
        ax.scatter(t[over], jit[over], color='red', s=12, zorder=3, label='overrun')
    ax.set_xlabel('시간 [s]')
    ax.set_ylabel('jitter [us]')
    ax.set_title('시간에 따른 지터')
    ax.legend(loc='upper right')
    save(fig, '2_jitter_time.png')

    # (3) goal_y와 y_y, 배경에 모드
    fig, ax = plt.subplots(figsize=(11, 5))
    shade_modes(ax, segments)
    ax.plot(t, df['goal_y'], lw=1.2, color='#1f77b4', label='goal_y (blend 목표)')
    ax.plot(t, df['y_y'], lw=1.2, color='#d62728', ls='--', label='y_y (limiter 출력)')
    ax.set_xlabel('시간 [s]')
    ax.set_ylabel('y [m] (map)')
    ax.set_title('목표와 출력 (배경: 모드)')
    ax.legend(loc='upper left')
    save(fig, '3_goal_output.png')

    # (4) weight와 reject_reason
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(11, 6), sharex=True, gridspec_kw={'height_ratios': [2, 1]})
    shade_modes(a1, segments)
    applied = df['weight'] > 0
    a1.vlines(t[applied], 0, df['weight'][applied], color='#2ca02c', lw=1)
    a1.plot(t[applied], df['weight'][applied], 'o', ms=2.5, color='#2ca02c', label='반영 w')
    a1.set_ylim(0, 1.05)
    a1.set_ylabel('weight')
    a1.set_title('반영 가중치와 gate 차단 이유')
    a1.legend(loc='upper left')
    rows = []
    for bit, name in enumerate(GATE_REASONS):
        flag = pd.Series(((reject >> bit) & 1) == 1)
        if flag.any():
            rows.append(name)
            a2.broken_barh(true_intervals(t, flag), (len(rows) - 1 - 0.35, 0.7), color='#c44e52')
    a2.set_yticks(range(len(rows)))
    a2.set_yticklabels(rows if rows else [])
    a2.set_ylim(-0.6, max(len(rows), 1) - 0.4)
    a2.set_xlabel('시간 [s]')
    a2.set_ylabel('차단 이유')
    save(fig, '4_weight_reject.png')

    # (5) compute_ns 히스토그램
    fig, ax = plt.subplots(figsize=(9, 5))
    ax.hist(comp, bins=200, color='#8172b2')
    ax.set_yscale('log')
    ax.axvline(comp.max(), color='black', ls=':', label=f'최대 {comp.max():.1f} us')
    ax.set_xlabel('compute [us] (깨어나서 계산이 끝날 때까지)')
    ax.set_ylabel('주기 수 (로그)')
    ax.set_title('1ms 루프 계산 시간 분포')
    ax.legend()
    save(fig, '5_compute_hist.png')

    print('저장한 그래프:')
    for p in saved:
        print(f'  {p}')


if __name__ == '__main__':
    main()
