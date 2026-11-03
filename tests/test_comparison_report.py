import csv
import tempfile
import unittest
from pathlib import Path
from drone_bench.visualization.comparison import export_comparison


class ComparisonReportTests(unittest.TestCase):
    def test_metrics_csv_does_not_replace_summary_table(self):
        with tempfile.TemporaryDirectory() as directory:
            out=Path(directory);branch=out/'dmpc';branch.mkdir()
            row=dict(time=1,min_pair_distance=12,min_boundary_clearance=7,max_pair_distance=40,max_speed=9,max_command_speed=10,max_input=20)
            with (branch/'metrics.csv').open('w',newline='') as f:
                writer=csv.DictWriter(f,fieldnames=list(row));writer.writeheader();writer.writerow(row)
            export_comparison(out,{'dmpc':dict(completed=0,error='No valid plan',success=False)},dict(benchmark='abc',initial_state='def'))
            text=(out/'report.html').read_text()
            self.assertIn('<td>UAV hoàn thành</td><td>0</td>',text)
            self.assertIn('No valid plan',text)
            self.assertNotIn("{'time':",text)
            self.assertTrue((out/'figures/max_speed.svg').exists())
