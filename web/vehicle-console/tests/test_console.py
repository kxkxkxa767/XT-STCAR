"""Integration checks use simulation and PTYs only; never physical devices."""
import importlib.util
import json
import os
from pathlib import Path
import pty
import queue
import select
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time
import unittest
import urllib.request
import urllib.error
import zipfile

ROOT = Path(__file__).resolve().parents[3]
BRIDGE = ROOT / 'target/debug/vehicle-bridge'

class ConsoleTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory()
        cls.folder = Path(cls.temp.name)
        with socket.socket() as s:
            s.bind(('127.0.0.1', 0)); cls.port = s.getsockname()[1]
        cls.proc = subprocess.Popen([sys.executable, str(ROOT/'web/vehicle-console/server.py'), '--demo', '--bridge', str(BRIDGE), '--port', str(cls.port), '--output', str(cls.folder/'files'), '--access-file', str(cls.folder/'access.json')], stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
        end = time.monotonic()+10
        while time.monotonic()<end:
            try:
                cls.token=json.loads((cls.folder/'access.json').read_text())['token']
                if cls.get('/api/state')['healthy']: return
            except Exception: time.sleep(.05)
        cls.proc.terminate()
        _, errors=cls.proc.communicate(timeout=5)
        raise RuntimeError('demo startup failed: '+errors.decode())
    @classmethod
    def tearDownClass(cls):
        cls.proc.terminate();cls.proc.wait(timeout=5);cls.proc.stderr.close();cls.temp.cleanup()
    @classmethod
    def request(cls,path,data=None,token=True):
        h={'X-Control-Token':cls.token} if token else {}
        if data is not None: h['Content-Type']='application/json'
        req=urllib.request.Request(f'http://127.0.0.1:{cls.port}'+path,headers=h,data=json.dumps(data).encode() if data is not None else None)
        with urllib.request.build_opener(urllib.request.ProxyHandler({})).open(req,timeout=3) as r:return r.read()
    @classmethod
    def get(cls,path):return json.loads(cls.request(path))
    def post(self,path,data):return json.loads(self.request(path,data))
    def setUp(self):
        self.post('/api/control',{'op':'stop'});time.sleep(.08)
        self.client='test-client-0123456789';self.seq=0
    def drive(self,op,keys=None,client=None):
        self.seq+=1;s=self.get('/api/state');return self.post('/api/control',{'op':op,'boot':s['boot'],'client':client or self.client,'seq':self.seq,'tick':s['status']['control']['tick'],'keys':keys or []})
    def test_auth_and_bounds(self):
        with self.assertRaises(urllib.error.HTTPError) as e:self.request('/api/state',token=False)
        self.assertEqual(e.exception.code,403)
        with self.assertRaises(urllib.error.HTTPError):self.post('/api/settings',{'forward':2500,'reverse':1480,'left':1550,'right':1450})
        self.assertEqual(self.get('/api/state')['settings']['forward'],1550)
    def test_held_over_two_seconds_release_and_timeout(self):
        self.drive('arm');time.sleep(.04)
        start=time.monotonic()
        while time.monotonic()-start<2.4:
            self.drive('drive',['up']);time.sleep(.06)
        s=self.get('/api/state')['status']['control'];self.assertTrue(s['armed']);self.assertEqual(s['motor'],1550)
        self.drive('drive',[]);time.sleep(.06)
        self.assertEqual(self.get('/api/state')['status']['control']['motor'],1500)
        self.drive('drive',['up']);time.sleep(.5)
        s=self.get('/api/state')['status']['control'];self.assertFalse(s['armed']);self.assertEqual(s['motor'],1500)
    def test_timeout_releases_owner_and_allows_new_page(self):
        self.drive('arm')
        time.sleep(.55)
        state=self.get('/api/state')
        self.assertIsNone(state['owner'])
        self.assertFalse(state['status']['control']['armed'])
        self.drive('arm',client='new-browser-client-123456')
        time.sleep(.05)
        self.assertTrue(self.get('/api/state')['status']['control']['armed'])

    def test_first_stop_cause_survives_late_requests_and_resets_on_arm(self):
        self.drive('arm');time.sleep(.04)
        with self.assertRaises(urllib.error.HTTPError):self.drive('drive',['invalid'])
        first=self.get('/api/state')['last_stop']
        self.assertEqual(first['reason'],'invalid_keys')
        with self.assertRaises(urllib.error.HTTPError):self.drive('drive',['up'])
        self.post('/api/control',{'op':'stop'})
        self.assertEqual(self.get('/api/state')['last_stop'],first)
        time.sleep(.08);self.drive('arm');time.sleep(.04)
        self.post('/api/control',{'op':'stop'});time.sleep(.06)
        state=self.get('/api/state')
        self.assertEqual(state['last_stop']['reason'],'operator_stop')
        self.assertIsNone(state['owner']);self.assertEqual(state['status']['control']['motor'],1500)

    def test_calibrated_settings_limits(self):
        valid={'forward':1550,'reverse':1350,'left':1650,'right':1350}
        self.post('/api/settings',valid)
        for key,value in [('reverse',1349),('left',1651),('right',1349)]:
            with self.assertRaises(urllib.error.HTTPError):self.post('/api/settings',{**valid,key:value})
        self.post('/api/settings',{'forward':1550,'reverse':1450,'left':1650,'right':1350})

    def test_single_owner_and_reverse_disabled(self):
        self.drive('arm');time.sleep(.04)
        with self.assertRaises(urllib.error.HTTPError):self.drive('drive',['up'],client='another-client-0123456')
        with self.assertRaises(urllib.error.HTTPError):self.drive('drive',['down'])
        time.sleep(.06);self.assertFalse(self.get('/api/state')['status']['control']['armed'])
    def test_stop_fences_old_arm(self):
        old_tick=self.get('/api/state')['status']['control']['tick'];time.sleep(.05)
        self.post('/api/control',{'op':'stop'});time.sleep(.05)
        with self.assertRaises(urllib.error.HTTPError):self.post('/api/control',{'op':'arm','boot':self.get('/api/state')['boot'],'client':self.client,'seq':99,'tick':old_tick})
    def test_snapshot_and_recording_contents(self):
        snap=self.post('/api/snapshot',{})['file']
        with zipfile.ZipFile(self.folder/'files'/snap) as z:self.assertIn('lidar.svg',z.namelist());self.assertIn('camera.svg',z.namelist())
        self.post('/api/record/start',{});time.sleep(.6)
        name=self.post('/api/record/stop',{})['file']
        with zipfile.ZipFile(self.folder/'files'/name) as z:
            self.assertIn('lidar.jsonl',z.namelist());self.assertIn('frames.jsonl',z.namelist());self.assertGreater(json.loads(z.read('metadata.json'))['frames'],0)
        self.assertGreater(len(self.request('/download/'+name)),100)
        with self.assertRaises(urllib.error.HTTPError):self.request('/download/../access.json')

    def test_single_camera_photo_is_not_zip(self):
        name=self.post('/api/photo',{'kind':'camera'})['file']
        self.assertTrue(name.endswith('.svg'))
        image=self.request('/download/'+name)
        self.assertTrue(image.startswith(b'<svg'))

    def auto_start(self, duration=400):
        state=self.get('/api/state')
        return self.post('/api/autonomy',{'op':'probe_start','boot':state['boot'],
                         'epoch':state['autonomy']['epoch'],'tick':state['status']['control']['tick'],
                         'pwm':1580,'duration_ms':duration})

    def auto_request(self, run, op, seq=1):
        return self.post('/api/autonomy',{'op':op,'boot':self.get('/api/state')['boot'],
                         'epoch':run['epoch'],'run_id':run['run_id'],'seq':seq})

    def test_auto_probe_deadline_returns_to_neutral(self):
        run=self.auto_start()
        sequence=0
        deadline=time.monotonic()+1
        while time.monotonic()<deadline:
            s=self.get('/api/state')
            if s['autonomy']['active'] is None: break
            sequence+=1
            try:self.auto_request(run,'heartbeat',sequence)
            except urllib.error.HTTPError:break
            time.sleep(.035)
        time.sleep(.06)
        s=self.get('/api/state')
        self.assertFalse(s['status']['control']['armed'])
        self.assertEqual(s['status']['control']['motor'],1500)
        self.assertTrue(s['autonomy']['last_result']['completed'])
        self.assertEqual(s['autonomy']['last_result']['reason'],'probe_complete')
        self.assertFalse(s['autonomy']['competition_supported'])

    def test_auto_loss_of_client_stops_before_deadline(self):
        self.auto_start(500)
        time.sleep(.28)
        s=self.get('/api/state')
        self.assertFalse(s['status']['control']['armed'])
        self.assertEqual(s['status']['control']['motor'],1500)
        self.assertEqual(s['autonomy']['last_result']['reason'],'autonomy_heartbeat_timeout')

    def test_straight_cli_one_session_deadline_and_stop_fence(self):
        cli = [sys.executable, str(ROOT/'web/vehicle-console/autonomy-control.py'), 'straight',
               '--pwm', '1560', '--max-seconds', '1', '--execute', '--access-file', str(self.folder/'access.json')]
        initial_epoch = self.get('/api/state')['autonomy']['epoch']
        run = subprocess.run(cli, capture_output=True, text=True, timeout=8)
        self.assertNotEqual(run.returncode, 0, run.stdout+run.stderr)
        state = self.get('/api/state')
        report = state['autonomy']['last_result']
        self.assertEqual(state['autonomy']['epoch'], initial_epoch+1)  # one arm/stop only
        self.assertTrue(report['centering'])
        self.assertEqual(report['reason'], 'coast_standstill_unconfirmed')
        self.assertFalse(report['completed'])  # Demo's circular returns cannot observe rigid standstill.
        self.assertGreater(report['coast_ticks'], 100)
        self.assertGreater(report['motion_ticks'], 20)
        self.assertFalse(state['status']['control']['armed'])
        self.assertEqual(state['status']['control']['motor'], 1500)
        self.post('/api/control', {'op': 'stop'})
        refused = subprocess.run(cli+['--expected-run-id='+report['run_id']], capture_output=True, text=True, timeout=4)
        self.assertNotEqual(refused.returncode, 0)
        self.assertIn('refusing continuation', refused.stdout+refused.stderr)
        self.assertIsNone(self.get('/api/state')['autonomy']['active'])

    def test_straight_cli_emergency_stop_does_not_rearm(self):
        cli = [sys.executable, str(ROOT/'web/vehicle-console/autonomy-control.py'), 'straight',
               '--max-seconds', '3', '--execute', '--access-file', str(self.folder/'access.json')]
        proc = subprocess.Popen(cli, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        try:
            end = time.monotonic()+1
            while time.monotonic()<end and self.get('/api/state')['autonomy']['active'] is None:
                time.sleep(.02)
            self.assertIsNotNone(self.get('/api/state')['autonomy']['active'])
            self.post('/api/control', {'op': 'stop'})
            output, errors = proc.communicate(timeout=2)
            self.assertNotEqual(proc.returncode, 0, output+errors)
            time.sleep(.1)
            state = self.get('/api/state')
            self.assertIsNone(state['autonomy']['active'])
            self.assertFalse(state['status']['control']['armed'])
            self.assertEqual(state['autonomy']['last_result']['reason'], 'operator_stop')
        finally:
            if proc.poll() is None:
                proc.terminate(); proc.communicate(timeout=2)

    def test_stop_fences_auto_start_and_late_cancel_does_not_stop_manual(self):
        old=self.get('/api/state')
        self.post('/api/control',{'op':'stop'});time.sleep(.06)
        with self.assertRaises(urllib.error.HTTPError):
            self.post('/api/autonomy',{'op':'probe_start','boot':old['boot'],
                      'epoch':old['autonomy']['epoch'],'tick':old['status']['control']['tick'],
                      'pwm':1580,'duration_ms':400})
        run=self.auto_start();time.sleep(.06)
        s=self.get('/api/state')
        self.post('/api/control',{'op':'takeover','boot':s['boot'],'epoch':s['autonomy']['epoch']});time.sleep(.06)
        self.drive('arm');time.sleep(.04)
        with self.assertRaises(urllib.error.HTTPError):self.auto_request(run,'cancel')
        with self.assertRaises(urllib.error.HTTPError):self.auto_request(run,'heartbeat')
        state=self.get('/api/state')
        self.assertTrue(state['status']['control']['armed'])
        self.assertEqual(state['autonomy']['mode'],'manual')

    def test_auto_cli_preflight_is_read_only(self):
        state=self.get('/api/state')
        result=subprocess.run([sys.executable,str(ROOT/'web/vehicle-console/autonomy-control.py'),
                              'probe','--access-file',str(self.folder/'access.json')],
                             capture_output=True,text=True,timeout=3)
        self.assertEqual(result.returncode,0,result.stdout+result.stderr)
        self.assertFalse(json.loads(result.stdout)['motion_requested'])
        after=self.get('/api/state')
        self.assertFalse(after['status']['control']['armed'])
        self.assertEqual(state['autonomy']['epoch'],after['autonomy']['epoch'])

    def test_z_restart_reuses_port_and_stays_locked(self):
        cls=type(self)
        cls.proc.terminate();cls.proc.wait(timeout=5);cls.proc.stderr.close()
        cls.proc=subprocess.Popen([sys.executable,str(ROOT/'web/vehicle-console/server.py'),'--demo','--bridge',str(BRIDGE),'--port',str(cls.port),'--output',str(cls.folder/'files'),'--access-file',str(cls.folder/'access.json')],stdout=subprocess.DEVNULL,stderr=subprocess.PIPE)
        deadline=time.monotonic()+5
        while time.monotonic()<deadline:
            try:
                cls.token=json.loads((cls.folder/'access.json').read_text())['token']
                state=cls.get('/api/state')
                if state['healthy']:
                    self.assertFalse(state['status']['control']['armed']);return
            except Exception:pass
            time.sleep(.05)
        self.fail('restart did not become healthy')

class PtyTests(unittest.TestCase):
    def test_real_wire_output_and_eof_stop(self):
        master,slave=pty.openpty(); path=os.ttyname(slave)
        proc=subprocess.Popen([str(BRIDGE),'control','--device',path,'--execute'],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE)
        rows=queue.Queue()
        def read():
            for line in proc.stdout:rows.put(json.loads(line))
        threading.Thread(target=read,daemon=True).start()
        try:
            s=rows.get(timeout=3)['control'];time.sleep(.05)
            while not rows.empty():s=rows.get()['control']
            def send(op,seq,motor):
                while not rows.empty():s2=rows.get()['control'];self.latest=s2
                tick=getattr(self,'latest',s)['tick']
                proc.stdin.write((json.dumps({'op':op,'seq':seq,'tick':tick,'motor':motor,'servo':1500})+'\n').encode());proc.stdin.flush()
            send('arm',1,1500);time.sleep(.05);send('drive',2,1600);time.sleep(.06)
            data=b''
            while select.select([master],[],[],.02)[0]:data+=os.read(master,4096)
            frames=[data[i:i+7] for i in range(0,len(data)-6,7)]
            self.assertTrue(any(int.from_bytes(f[1:3],'little')==1600 for f in frames))
            proc.stdin.close();proc.wait(timeout=3)
            stopped=b''
            while select.select([master],[],[],.02)[0]:stopped+=os.read(master,4096)
            self.assertGreaterEqual(len(stopped),140)
            self.assertEqual(int.from_bytes(stopped[-6:-4],'little'),1500)
        finally:
            if proc.poll() is None:proc.kill();proc.wait()
            proc.stdout.close();proc.stderr.close();os.close(master);os.close(slave)

if __name__=='__main__':unittest.main()
