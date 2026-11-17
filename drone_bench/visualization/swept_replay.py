"""Standalone offline time replay of recorded swept-volume planner updates."""
import json
from pathlib import Path


def write_replay(path, config, updates, initial_tube, summary):
    payload = dict(config=config, updates=updates, initial_tube=initial_tube, summary=summary)
    data = json.dumps(payload).replace('<', '\\u003c')
    template = Path(__file__).with_name('swept_replay.html').read_text()
    Path(path).write_text(template.replace('__REPLAY_DATA__', data))
