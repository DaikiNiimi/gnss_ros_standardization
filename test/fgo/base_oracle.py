"""Read the adopted base from raw admission, including IMU-only epochs.

A preprocessed IMU-only epoch can use the rover time as a placeholder base
field. That field is not evidence of which base was actually admitted.
"""
import json


def _milliseconds(stamp):
    return int(stamp['seconds']) * 1000 + round(stamp['fraction'] * 1000)


def actual_base_epochs(lines):
    result = {}
    for line in lines:
        # The portable raw replay exporter writes compact JSON. Also accept
        # spaced JSON; only the first short field is needed to skip IMU rows.
        if '"pair"' not in line[:40]:
            continue
        item = json.loads(line)
        if item['event'] != 'pair':
            continue
        rover = _milliseconds(item['rover']['time'])
        if rover in result:
            raise ValueError('duplicate raw rover epoch')
        base = item.get('base')
        result[rover] = -1 if base is None else _milliseconds(base['time'])
    return result
