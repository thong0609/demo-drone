import copy
import json
import re
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
from drone_bench.cli.evaluate_horizon import horizon_configs,validate_study,evaluate


def inputs():
    base=json.loads(Path('configs/experiments/swept_volume_replay.json').read_text())
    study=json.loads(Path('configs/experiments/swept_horizon_study.json').read_text())
    return base,study


class HorizonStudyTests(unittest.TestCase):
    def test_only_horizon_changes_in_matched_variants(self):
        base,study=inputs();original=copy.deepcopy(base)
        study.update(horizons=[.5,3],seeds=[17],case_names=['direction_y_negative'])
        variants=horizon_configs(base,study);self.assertEqual(len(variants),2)
        a=copy.deepcopy(variants[0][3]);b=copy.deepcopy(variants[1][3])
        self.assertEqual(a.pop('horizon'),.5);self.assertEqual(b.pop('horizon'),3)
        self.assertEqual(a,b);self.assertEqual(base,original)
        self.assertEqual(len(horizon_configs(base,inputs()[1])),270)

    def test_invalid_grid_rejected(self):
        base,study=inputs()
        for horizons in ([],[0],[-1],[float('nan')],[1,1]):
            with self.assertRaises(ValueError):validate_study(dict(study,horizons=horizons))
        with self.assertRaises(ValueError):horizon_configs(base,dict(study,case_names=['unknown']))

    def test_export_failures_and_separate_evaluation_horizons(self):
        base,study=inputs();study.update(horizons=[.5,3],seeds=[17],case_names=['blocked'])
        with tempfile.TemporaryDirectory() as directory:
            summary=evaluate(base,study,directory)
            self.assertEqual([r['updates'] for r in summary],[2,2])
            self.assertTrue(all(r['accepted']==0 for r in summary))
            out=Path(directory);rows=json.loads((out/'records.json').read_text())
            self.assertEqual(len(rows),4)
            self.assertEqual({r['horizon'] for r in rows},{.5,3})
            self.assertEqual({r['status'] for r in rows},{'start_blocked'})
            self.assertTrue(all(r['horizon_covers_update_interval']==(r['horizon']>=1) for r in rows))
            self.assertTrue(all((out/r['replay']).exists() for r in rows))
            raw=json.loads((out/'runs/blocked/seed_17/H_0p5/evaluation_000.json').read_text())
            self.assertEqual(raw['checks']['reference']['horizon'],3)
            self.assertEqual(raw['checks']['next_update']['horizon'],1)
            report=(out/'report.html').read_text()
            data=json.loads(re.search(r'<script id="horizon-data" type="application/json">(.*?)</script>',report,re.S)[1])
            self.assertEqual(len(data['runs']),2)
            self.assertIn('horizonReportState',report)


if __name__=='__main__':unittest.main()
