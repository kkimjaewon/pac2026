#!/usr/bin/env python3
"""BC-11: distinguish wall and thread-CPU overruns in an existing HAL trace.

Applied ACK and superseded rows are excluded from IK job counts. A code 5
can also mean nonconvergence/residual failure; consult HalError.detail before
calling every code 5 a deadline failure.
"""
import argparse
import collections
import csv
import json
import math


def percentile(values, fraction):
    values = sorted(values)
    index = (len(values) - 1) * fraction
    lo, hi = math.floor(index), math.ceil(index)
    return values[lo] + (values[hi] - values[lo]) * (index - lo)


def analyze(path, budget_ns):
    with open(path, newline='', encoding='utf-8') as stream:
        reader = csv.DictReader(stream)
        required = {'result', 'error_code', 'ik_wall_ns', 'ik_cpu_ns'}
        if not required.issubset(reader.fieldnames or []):
            raise ValueError('CSV requires result, error_code, ik_wall_ns, ik_cpu_ns')
        events = list(reader)
        if any(row['result'] not in ('sent', 'rejected', 'applied', 'superseded') for row in events):
            raise ValueError('unknown HAL result')
        # ACK duplicates and latest-target replacement are not processed IK jobs.
        rows = [row for row in events if row['result'] in ('sent', 'rejected')]
    output = {'command_rows': len(rows), 'result_counts': dict(collections.Counter(row['result'] for row in events)), 'budget_ns': budget_ns,
              'error_counts': dict(collections.Counter(row['error_code'] for row in rows))}
    output['groups'] = {}
    for code in ('0', '5', '3'):
        group = [row for row in rows if row['error_code'] == code]
        if not group:
            continue
        summary = {'count': len(group)}
        for key in ('ik_wall_ns', 'ik_cpu_ns'):
            values = [int(row[key]) for row in group if row[key]]
            if values:
                summary[key] = {'p50': percentile(values, .5),
                                'p99': percentile(values, .99), 'max': max(values)}
        measured = [row for row in group if row['ik_wall_ns'] and row['ik_cpu_ns']]
        summary['wall_over_cpu_within_budget'] = sum(
            int(row['ik_wall_ns']) > budget_ns >= int(row['ik_cpu_ns']) for row in measured)
        summary['cpu_over_budget'] = sum(int(row['ik_cpu_ns']) > budget_ns for row in measured)
        output['groups'][code] = summary
    return output


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv_path')
    parser.add_argument('--budget-ns', type=int, default=500000)
    arguments = parser.parse_args()
    if arguments.budget_ns <= 0:
        parser.error('--budget-ns must be positive')
    print(json.dumps(analyze(arguments.csv_path, arguments.budget_ns), indent=2))
