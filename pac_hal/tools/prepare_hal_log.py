#!/usr/bin/env python3
"""Prepare C's one-command CSV for merge_logs.py. Run after C has exited/flushed.
Use PYTHONPATH=<pac_hal source> python3 tools/prepare_hal_log.py hal.csv --output-dir <new-folder>.
"""
import argparse
import csv
import json
from pathlib import Path
from pac_hal.log_contract import normalize, read_hal, INTERVALS, attach_ai_identity


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('hal_csv', type=Path)
    p.add_argument('--output-dir', type=Path, required=True)
    p.add_argument('--rt-csv', type=Path)
    p.add_argument('--ai-csv', type=Path)
    p.add_argument('--session-id', type=int)
    args=p.parse_args()
    joining = any(v is not None for v in (args.rt_csv, args.ai_csv, args.session_id))
    if joining and any(v is None for v in (args.rt_csv, args.ai_csv, args.session_id)):
        p.error('--rt-csv, --ai-csv and --session-id must be supplied together for one run')
    rows=read_hal(args.hal_csv)
    normalized,summary=normalize(rows)
    summary['source_file']=str(args.hal_csv.resolve())
    if joining:
        def read_csv(path):
            with path.open(newline='', encoding='utf-8') as stream:
                return list(csv.DictReader(stream))
        normalized, summary['ai_join_counts'] = attach_ai_identity(
            normalized, read_csv(args.rt_csv), read_csv(args.ai_csv), args.session_id)
        summary['rt_source_file'] = str(args.rt_csv.resolve())
        summary['ai_source_file'] = str(args.ai_csv.resolve())
    # Original logs remain intact; never mix results from different analyses.
    args.output_dir.mkdir(parents=True, exist_ok=False)
    with (args.output_dir/'hal_commands.csv').open('x',newline='',encoding='utf-8') as stream:
        fields=list(rows[0]) if rows else list(csv_header(args.hal_csv))
        writer=csv.DictWriter(stream,fieldnames=[*fields,'ack_observed',*INTERVALS,*(['writer_boot_id','ai_join'] if joining else [])])
        writer.writeheader();writer.writerows(normalized)
    (args.output_dir/'hal_summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(summary,indent=2))


def csv_header(path):
    with path.open(newline='',encoding='utf-8') as stream:
        return next(csv.reader(stream))


if __name__=='__main__':
    main()
