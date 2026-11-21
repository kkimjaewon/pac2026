"""Offline HAL CSV contract: one command row, ACK coverage and explicit interval boundaries.

Never infer TCP arrival, missing ACKs, A boot IDs or cross-PC clock offsets.
This module is a reference for A's merge_logs.py, not the A/B/C merger itself.
"""
from collections import Counter
import csv

KEY = ('session_id', 'command_seq')
TIMES = ('b_created_mono_ns', 'received_mono_ns', 'validated_mono_ns',
         'ik_end_mono_ns', 't3_mono_ns', 'transport_start_mono_ns',
         'transport_end_mono_ns', 'receiver_applied_mono_ns', 'ack_received_mono_ns')
# Publication may return after adoption: use start->adoption, not end->adoption.
INTERVALS = {
    'b_to_c_ns': ('b_created_mono_ns', 'received_mono_ns'),
    'c_validation_ns': ('received_mono_ns', 'validated_mono_ns'),
    'c_ik_stage_ns': ('validated_mono_ns', 'ik_end_mono_ns'),
    'c_post_ik_checks_ns': ('ik_end_mono_ns', 't3_mono_ns'),
    'c_before_transport_ns': ('t3_mono_ns', 'transport_start_mono_ns'),
    'transport_call_ns': ('transport_start_mono_ns', 'transport_end_mono_ns'),
    'transport_to_adoption_ns': ('transport_start_mono_ns', 'receiver_applied_mono_ns'),
    'adoption_to_ack_ns': ('receiver_applied_mono_ns', 'ack_received_mono_ns'),
    'b_to_adoption_ns': ('b_created_mono_ns', 'receiver_applied_mono_ns'),
}
REQUIRED = {*KEY, *TIMES, 'result', 'error_code', 'source_result_id'}


def read_hal(path):
    with open(path, newline='', encoding='utf-8') as stream:
        reader = csv.DictReader(stream)
        missing = REQUIRED - set(reader.fieldnames or ())
        if missing:
            raise ValueError('missing HAL columns: ' + ', '.join(sorted(missing)))
        return list(reader)


def _time(row, name):
    value = row.get(name, '')
    if value in ('', None):
        return None
    parsed = int(value)  # nanoseconds must never pass through float
    if parsed < 0:
        raise ValueError(f'negative timestamp: {name}')
    return parsed or None


def normalize(rows):
    """Count rejected/superseded events separately, fold sent+applied by session/seq.

    Identical duplicated success rows are idempotent. A rejected replay sharing
    a previously successful command key remains a separate rejection event.
    """
    events, errors, selected, stages = Counter(), Counter(), {}, {}
    duplicates = 0
    for row in rows:
        status = row['result']
        if status not in ('sent', 'applied', 'rejected', 'superseded'):
            raise ValueError('unknown result: ' + status)
        events[status] += 1
        if status == 'rejected':
            errors[str(int(row['error_code']))] += 1
            continue
        if status == 'superseded':
            continue
        if int(row['error_code']) != 0:
            raise ValueError('successful row has a nonzero error code')
        key = tuple(int(row[k]) for k in KEY)
        if key in selected:
            previous = selected[key]
            # Applied records copy the original command's trace fields exactly.
            identity = ('source_result_id', *TIMES[:7])
            if any(previous.get(k, '') != row.get(k, '') for k in identity):
                raise ValueError(f'conflicting command/ACK identity: {key}')
            if status in stages[key]:
                if status == 'applied' and any(previous.get(k, '') != row.get(k, '') for k in TIMES[7:]):
                    raise ValueError(f'conflicting ACK timestamps: {key}')
                duplicates += 1
        stages.setdefault(key, set()).add(status)
        if key not in selected or status == 'applied':
            selected[key] = dict(row)
    output, invalid, with_ack, ack_without_sent = [], Counter(), 0, 0
    for key, row in sorted(selected.items()):
        adopted = row['result'] == 'applied'
        if adopted:
            if not all(_time(row, k) is not None for k in TIMES[7:]):
                raise ValueError(f'applied row missing ACK timestamps: {key}')
            with_ack += 1
            ack_without_sent += 'sent' not in stages[key]
        row['ack_observed'] = int(adopted)
        for name, (start, end) in INTERVALS.items():
            a, b = _time(row, start), _time(row, end)
            row[name] = ''
            if a is not None and b is not None:
                if b >= a:
                    row[name] = b-a
                else:
                    invalid[name] += 1  # never abs(), clamp or substitute zero
        output.append(row)
    summary = {
        'event_counts': dict(events), 'rejection_error_counts': dict(errors),
        'successful_unique_commands': len(output), 'ack_observed_commands': with_ack,
        'ack_not_observed_commands': len(output)-with_ack,
        'ack_coverage': with_ack/len(output) if output else None,
        'identical_duplicate_success_rows': duplicates,
        'applied_without_sent_rows': ack_without_sent,
        'invalid_interval_counts': dict(invalid),
        'clock': 'PC-RT CLOCK_MONOTONIC; A on another PC is not directly subtractable',
        'ack_meaning': 'receiver controller target adoption; not measured TCP arrival',
    }
    return output, summary


def attach_ai_identity(commands, rt_rows, ai_rows, session_id):
    """Join one run using B's actual used boot ID, never its latest received ID.

    Different PC clocks are not subtracted. Older B logs without used_ai_boot_id
    must use the documented exact ai_rx/apply join, not this direct join.
    """
    by_command = {}
    for row in rt_rows:
        if 'used_ai_boot_id' not in row or 'has_used' not in row:
            raise ValueError('B requires used_ai_boot_id and has_used for direct AI join')
        seq = int(row['command_seq'])
        if not seq:
            continue
        identity = (int(row['used_ai_boot_id']), int(row['used_ai_seq'])) if int(row['has_used']) else None
        if seq in by_command and by_command[seq] != identity:
            raise ValueError('conflicting B command lineage')
        by_command[seq] = identity
    by_ai = {}
    for row in ai_rows:
        if str(row['published']) != '1':
            continue  # stop-mode rows have no published message identity
        boot = row.get('writer_boot_id', row.get('boot_id'))
        if boot in ('', None):
            raise ValueError('A requires writer_boot_id (or legacy boot_id)')
        key = (int(boot), int(row['seq']))
        if key in by_ai:
            raise ValueError('duplicate A message identity')
        by_ai[key] = row
    output, counts = [], Counter()
    for command in commands:
        if int(command['session_id']) != int(session_id):
            raise ValueError('HAL session differs from supplied B run session')
        row = dict(command)
        row.update(writer_boot_id='', ai_join='missing_b_command')
        seq = int(row['command_seq'])
        if seq in by_command:
            identity = by_command[seq]
            if identity is None:
                if int(row['source_result_id']) != 0:
                    raise ValueError('HAL source exists but B has no used AI result')
                row['ai_join'] = 'no_used_ai'
            else:
                boot, source = identity
                if source != int(row['source_result_id']):
                    raise ValueError('HAL source_result_id differs from B used_ai_seq')
                row['writer_boot_id'] = boot
                row['ai_join'] = 'matched' if identity in by_ai else 'missing_a_message'
        counts[row['ai_join']] += 1
        output.append(row)
    return output, dict(counts)
