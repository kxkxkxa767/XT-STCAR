"""Scheduler acceptance, evidence preservation and restart semantics."""
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

spec = importlib.util.spec_from_file_location('sweep', Path(__file__).parents[1] / 'local-sim-sweep.py')
sweep = importlib.util.module_from_spec(spec)
spec.loader.exec_module(sweep)


class SchedulerTests(unittest.TestCase):
    def test_resume_retains_failed_evidence_and_requires_zero_exit(self):
        with tempfile.TemporaryDirectory() as temporary:
            out = Path(temporary)
            binary = out / 'lidar_sweep'
            binary.write_text('#!/usr/bin/env python3\nimport json,sys,pathlib\np=pathlib.Path(sys.argv[3]); p.mkdir(exist_ok=True)\n(p/"events.jsonl").write_text("original evidence\\n")\nd=json.loads(pathlib.Path(sys.argv[2]).read_text())\nprint(json.dumps({"status":"finished","accepted":True,"summary":{"fault":None}}))\nsys.exit(d["exitcode"])\n')
            binary.chmod(0o755)
            (out / 'identity.json').write_text(json.dumps({'binary_sha256': sweep.digest(binary)}))
            db = sweep.connect(out)
            for index, code in enumerate([1,0]):
                db.execute('INSERT INTO jobs VALUES (?,?,?,?,?,NULL,NULL,NULL)', (str(index),'test','sync',json.dumps({'exitcode':code}),'pending'))
            db.commit()
            sweep.run(out,1)
            self.assertEqual(dict(db.execute('SELECT status,COUNT(*) FROM jobs GROUP BY status')), {'failed':1,'pending':1})
            self.assertTrue((out/'cases/0/events.jsonl.gz').exists())
            before = (out/'cases/0/result.json.gz').read_bytes()
            sweep.run(out,None)
            self.assertEqual(dict(db.execute('SELECT status,COUNT(*) FROM jobs GROUP BY status')), {'accepted':1,'failed':1})
            self.assertEqual((out/'cases/0/result.json.gz').read_bytes(),before)
            binary.write_text('changed')
            with self.assertRaises(SystemExit):
                sweep.run(out,None)
            db.close()


if __name__ == '__main__':
    unittest.main()
