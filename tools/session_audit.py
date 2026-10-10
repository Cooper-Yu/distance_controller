"""Conservative migration of pre-checkpoint JSONL: stopped records only, no guessed travel budget."""

import json
from pathlib import Path


def import_audit(path):  # noqa: PLR0912 - explicit replay branch per recorded event type
    """Replay recorded boundaries; operator must confirm the unrecorded odom epoch.

    Older audits omit odom frame/stamp and cumulative interrupted travel. Their
    partial actions permit guarded return only, never forward resume.
    """
    data = None
    busy = False
    for line in Path(path).read_text().splitlines():
        row = json.loads(line)
        event, item = row['event'], row['data']
        if event == 'origin':
            data = dict(
                version=1,
                usable=True,
                origin=item['pose'],
                steps=item['actions'],
                cursor=0,
                revision=0,
                active=[],
                partial=None,
                last_actual=item['pose'],
                frame='operator_confirmed_legacy',
                stamp=0,
                reference=None,
                wall_speed=0.06,
                legacy_audit=True,
            )
            busy = False
        elif data is None:
            continue
        elif event.startswith(
            ('turn_adjustment_', 'turn_escape_', 'turn_rewind_', 'front_adjustment_')
        ):
            raise ValueError('Turn adjustment audit requires its checkpoint file')
        elif event == 'checkpoint_restored':
            raise ValueError('This audit was itself restored; use its checkpoint file')
        elif event == 'initial_reference':
            data['reference'] = item['body_clearance_m']
            data['wall_speed'] = item['speed_cap_m_s']
        elif event in ('edit', 'policy_edit'):
            data['steps'][item['index']] = item['step']
            data['revision'] = item['revision']
        elif event == 'reload':
            data['steps'], data['revision'] = item['actions'], item['revision']
        elif event in ('started', 'return_requested'):
            busy = True
        elif event in ('completed', 'wall_result'):
            data['active'] = [r for r in data['active'] if r['index'] < item['index']] + [item]
            data['cursor'], data['partial'] = item['index'] + 1, None
            data['last_actual'] = item['end']
            data['reference'] = item.get('reference_after', data['reference'])
            busy = False
        elif event == 'incomplete':
            data['partial'] = {**item, 'resume_unavailable': True}
            data['last_actual'] = item['last_stop']
            busy = False
        elif event == 'returned':
            data['active'] = [r for r in data['active'] if r['index'] < item['index']]
            data['cursor'], data['partial'] = item['index'], None
            data['last_actual'] = item['actual']
            busy = False
        elif event == 'reference_restored':
            data['reference'] = item['body_clearance_m']
    if data is None or busy or data['last_actual'] is None:
        raise ValueError('Audit lacks a confirmed stopped boundary; migration refused')
    return data
