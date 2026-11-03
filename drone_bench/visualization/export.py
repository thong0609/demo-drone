"""Offline interactive viewer export."""
import json
from pathlib import Path
from .mesh import mesh


def write_viewer(payload, tube, destination):
    view = dict(payload)
    view['rings'] = mesh(tube)
    template = Path(__file__).with_name('viewer.html').read_text()
    encoded = json.dumps(view).replace('</', '<\\/')
    Path(destination).write_text(template.replace('__BENCHMARK_DATA__', encoded))
