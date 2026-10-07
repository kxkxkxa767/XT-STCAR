#!/usr/bin/env python3
"""Shared sensor console and bounded probe; Rust owns the only chassis tty."""
import argparse
import base64
import collections
import fcntl
import hashlib
import hmac
import ipaddress
import io
import json
import math
import os
from pathlib import Path
import secrets
import select
import shutil
import signal
import socket
import socketserver
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlsplit
import zipfile
from autonomy_live import (HEARTBEAT_S, QUALITY_CONFIRM_S, QUALITY_RECOVERY_STABLE_S, COAST_MAX_S, COAST_TIME_MARGIN_S, FRONT_BOUNDARY_LOSS_S, CONTROL_AGE_LIMIT_S, AUTO_CONTROL_HEALTH_S, QualityLatch, QualityRecovery, CorridorSteering, JunctionStop, RearLaunch, ApproachRamp,
                           corridor_walls, probe_clearance, probe_parameters)
from coast_motion import CoastMotionWorker
from stop_goal import FinalStopGoal, StopGoalConsumer, StopGoalContract, StopGoalError, read_local_json
from turn_motion import TurnMotion, parse_trial_goal, validate_initial_presteer_pwm, SERVO_MIN, SERVO_MAX, TRIAL_MOTOR, MAX_DRIVE_S, MAX_PRESTEER_S, STEERING_ALLOWANCE_S
from maneuver_sequence import ManeuverSequence

ROOT = Path(__file__).resolve().parent
COMPACT_TARGET_TRIAL_SCOPE = 'first_lidar_compact_target_orbit_entry'


class Console:
    def __init__(self, args):
        self.args = args
        self.lock = threading.RLock()
        self.send_lock = threading.Lock()
        self.stop = threading.Event()
        self.token = secrets.token_urlsafe(32)
        access = Path(args.access_file)
        if access.exists():
            existing = json.loads(access.read_text()).get('token')
            if not isinstance(existing, str) or not 8 <= len(existing) <= 128 or any(c not in 'ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789_-' for c in existing):
                raise ValueError('invalid stored access token')
            if access.stat().st_mode & 0o077:
                raise ValueError('access file must be private (chmod 600)')
            self.token = existing
        self.boot = secrets.token_urlsafe(16)
        self.owner = None
        self.control_mode = 'locked'
        self.control_epoch = 0
        self.auto_session = None
        self.auto_result = None
        self.coast_worker = None
        self.stop_goal_source = getattr(args, 'stop_goal_source', None)
        self.stop_goal_contract = None
        self.stop_goal_registry = None
        self.stop_goal_processed = set()
        self.stop_goal_rejection = 'stop_goal_source_not_configured'
        self.trial_goal_registry = None
        contract_file = getattr(args, 'stop_goal_contract', None)
        if contract_file is not None:
            try:
                profile_file = getattr(args, 'stop_goal_vehicle_profile', None)
                profile = read_local_json(profile_file) if profile_file is not None else None
                self.stop_goal_contract = StopGoalContract(read_local_json(contract_file), real=not args.demo,
                                                         vehicle_profile=profile)
                self.stop_goal_rejection = 'stop_goal_not_received'
            except StopGoalError as error:
                self.stop_goal_rejection = str(error)
        self.perception_quality = QualityLatch()
        self.owner_at = 0
        self.arm_sequence = 0
        self.last_stop = None
        self.stop_latched = True
        self.stopped_tick = -1
        self.sequence = 0
        self.browser_sequence = -1
        self.status = {}
        self.control_at = 0
        self.scan = None
        self.scan_at = 0
        self.jpeg = None
        self.camera_at = 0
        self.camera_seq = 0
        self.vision = None
        if getattr(args, 'vision_shadow', None):
            from vision_shadow import VisionShadow
            self.vision = VisionShadow(args.vision_shadow)
        self.errors = {}
        self.events = collections.deque(maxlen=2000)
        self.record = None
        self.record_error = None
        self.saving = False
        self.storage_lock = threading.Lock()
        self.processes = []
        self.output = Path(args.output).resolve()
        self.output.mkdir(parents=True, exist_ok=True, mode=0o700)
        os.chmod(self.output, 0o700)
        self.settings = {'forward': 1550, 'reverse': 1450, 'left': 1650, 'right': 1350}

    def spawn(self, mode, device):
        cmd = [self.args.bridge, mode]
        if not self.args.demo:
            if os.path.realpath(device) != {'control': '/dev/ttyUSB1', 'lidar': '/dev/ttyACM0'}[mode]:
                raise RuntimeError('device identity changed: ' + device)
            check = subprocess.run(['fuser', device], capture_output=True, timeout=3)
            if check.returncode != 1:
                raise RuntimeError('device busy or owner check failed: ' + device)
            cmd += ['--device', device, '--execute']
        if mode == 'control' and self.args.allow_reverse:
            cmd += ['--allow-reverse']
        proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, bufsize=0)
        os.set_blocking(proc.stdin.fileno(), False)
        self.processes.append(proc)
        return proc

    def start(self):
        # Start the idle fit worker before control loops, not during a coast tick.
        if self.coast_worker is None:
            self.coast_worker = CoastMotionWorker()
        if self.vision:
            self.vision.start()
        self.control = self.spawn('control', '/dev/car')
        self.lidar = self.spawn('lidar', '/dev/laser')
        threading.Thread(target=self.read_bridge, args=(self.control, 'control'), daemon=True).start()
        threading.Thread(target=self.read_bridge, args=(self.lidar, 'lidar'), daemon=True).start()
        threading.Thread(target=self.camera, daemon=True).start()
        threading.Thread(target=self.recorder, daemon=True).start()
        threading.Thread(target=self.health_watch, daemon=True).start()
        threading.Thread(target=self.autonomy_watch, daemon=True).start()

    def read_bridge(self, proc, kind):
        try:
            # stdin stays unbuffered/nonblocking. Buffer only stdout: FileIO.readline
            # reads one byte per syscall and can delay entire scans under CPU load.
            reader = io.BufferedReader(proc.stdout, buffer_size=65536)
            while not self.stop.is_set():
                line = reader.readline(65537)
                if not line or len(line) > 65536:
                    raise RuntimeError(kind + ' bridge ended')
                value = json.loads(line)
                with self.lock:
                    if kind == 'control':
                        self.status = value
                        self.control_at = time.monotonic()
                        control = value.get('control', {})
                        if self.owner and not control.get('armed') and control.get('seq', -1) >= self.arm_sequence:
                            self.halt('bridge_' + control.get('reason', 'locked'))
                    else:
                        self.accept_lidar(value, time.monotonic())
        except Exception as error:
            with self.lock:
                self.errors[kind] = str(error)
            self.halt('bridge_failure')

    def accept_lidar(self, value, now):
        """Receiving a repeated JSON line cannot refresh lidar observation age."""
        if not isinstance(value, dict) or type(value.get('seq')) is not int or value['seq'] < 0:
            raise ValueError('invalid_lidar_sequence')
        published = value.get('at_ms')
        if not self.args.demo and (type(published) is not int or published < 0):
            raise ValueError('invalid_lidar_publication_clock')
        if self.scan is not None:
            old_seq, old_publication = self.scan.get('seq'), self.scan.get('at_ms')
            if type(old_seq) is int and value['seq'] <= old_seq:
                return False
            if type(old_publication) is int and (type(published) is not int or published <= old_publication):
                return False
        self.scan, self.scan_at = value, now
        return True

    def healthy(self):
        now = time.monotonic()
        return (now - self.camera_at < 1 and now - self.scan_at < 1
                and now - self.control_at < .2 and not self.errors)

    def autonomy_healthy(self):
        # Perception quality waits neutrally inside the existing bounded session.
        return time.monotonic() - self.control_at < AUTO_CONTROL_HEALTH_S and not self.errors

    def turn_healthy(self):
        return (time.monotonic()-self.control_at < AUTO_CONTROL_HEALTH_S
                and not any(kind != 'camera' for kind in self.errors))

    def health_watch(self):
        while not self.stop.wait(.05):
            with self.lock:
                healthy = (self.turn_healthy() if self.auto_session and self.auto_session.get('turn_trial') is True
                           else self.autonomy_healthy() if self.auto_session else self.healthy())
                unsafe = not healthy or (self.owner is not None and time.monotonic() - self.owner_at > .35)
                armed = self.status.get('control', {}).get('armed', False)
                if unsafe and (armed or self.owner is not None):
                    self.halt('sensor_stale' if not healthy else 'browser_timeout')
                if self.auto_session is None:
                    # A normal 1-second boundary must not reset a continuous fault.
                    now = time.monotonic()
                    try:
                        clearance = probe_clearance(self.scan, self.sensor_ages(now), self.args.demo)
                        self.perception_quality.update(clearance['quality_issues'], now)
                    except ValueError:
                        pass  # A hard fault is not evidence of perception recovery.

    def emit(self, op, motor=1500, servo=1500, tick=0):
        # Nonblocking writes: a blocked/killed bridge must never block the web watchdog.
        with self.send_lock:
            self.sequence += 1
            row = {'op': op, 'seq': self.sequence, 'tick': tick, 'motor': motor, 'servo': servo}
            data = (json.dumps(row) + '\n').encode()
            try:
                if os.write(self.control.stdin.fileno(), data) != len(data):
                    raise RuntimeError('short bridge write')
            except Exception as error:
                self.errors['control_write'] = str(error)
                raise RuntimeError('control bridge unavailable') from error
            self.events.append({'unix_s': time.time(), 'request': row})

    def halt(self, reason):
        with self.lock:
            # Keep the first cause; late requests must not mask a watchdog stop.
            if not self.stop_latched or self.last_stop is None:
                self.last_stop = {'reason': reason, 'unix_s': time.time()}
            self.stop_latched = True
            self.control_epoch += 1
            self.control_mode = 'locked'
            if self.auto_session is not None:
                report = self.auto_session['report']
                formal_complete = (reason == 'formal_goal_complete' and report.get('formal_completion_evidence', False)
                                   and report.get('coast_neutral_ack', False) and report.get('observed_armed', False))
                self.auto_result = {**self.auto_session['report'], 'reason': reason,
                                    'completed': formal_complete or (not report.get('formal_stop_goal') and not report.get('turn_trial')
                                    and reason in ('probe_complete', 'left_junction_reached')
                                    and self.auto_session['report'].get('observed_armed', False)
                                    and (not self.auto_session['report'].get('centering')
                                         or self.auto_session['report'].get('standstill_confirmed', False)))}
                if formal_complete:
                    self.stop_goal_processed.add(self.auto_session['goal_consumer'].goal.identity)
            self.auto_session = None
            if self.coast_worker is not None:
                self.coast_worker.reset()
            self.owner = None
            self.stopped_tick = self.status.get('control', {}).get('tick', -1)
            self.events.append({'unix_s': time.time(), 'stop_reason': reason})
            try:
                self.emit('stop')
            except Exception:
                pass

    def command(self, data):
        with self.lock:
            op = data.get('op')
            if op == 'stop':
                self.halt('operator_stop')
                return {'ok': True}
            if op == 'takeover':
                if data.get('boot') != self.boot or type(data.get('epoch')) is not int or data['epoch'] != self.control_epoch:
                    raise ValueError('stale_autonomy_generation')
                if self.auto_session is None:
                    raise ValueError('autonomy_session_ended')
                self.halt('manual_takeover')
                return {'ok': True}
            if self.stop.is_set():
                raise ValueError('server is shutting down')
            if data.get('boot') != self.boot:
                raise ValueError('server restarted; refresh state and unlock again')
            client = data.get('client')
            seq = data.get('seq')
            tick = data.get('tick')
            if not isinstance(client, str) or not 16 <= len(client) <= 80 or type(seq) is not int or type(tick) is not int:
                raise ValueError('invalid client/sequence/tick')
            if self.owner is not None and client != self.owner:
                raise ValueError('another tab owns control')
            if op == 'arm':
                if self.owner is not None or self.status.get('control', {}).get('armed'):
                    raise ValueError('already armed; stop first')
                if not self.healthy():
                    raise ValueError('camera/lidar/control not fresh')
                current_tick = self.status.get('control', {}).get('tick', -1)
                if not 0 <= current_tick - tick < 180 or tick <= self.stopped_tick:
                    raise ValueError('stale arm request')
                self.stop_latched = False
                self.owner = client
                self.control_mode = 'manual'
                self.browser_sequence = seq
                self.owner_at = time.monotonic()
                self.emit('arm', tick=tick)
                self.arm_sequence = self.sequence
            elif op == 'drive':
                if self.owner != client or seq <= self.browser_sequence:
                    self.halt('stale_or_unowned_request')
                    raise ValueError('stale/unowned request; unlock again')
                if not self.healthy():
                    self.halt('sensor_stale')
                    raise ValueError('sensors stale')
                keys = data.get('keys')
                if not isinstance(keys, list) or len(keys) > 4 or any(k not in ['up', 'down', 'left', 'right'] for k in keys):
                    self.halt('invalid_keys')
                    raise ValueError('invalid keys')
                if 'down' in keys and not self.args.allow_reverse:
                    self.halt('reverse_not_calibrated')
                    raise ValueError('reverse is disabled until calibrated')
                motor = 1500
                servo = 1500
                if ('up' in keys) != ('down' in keys):
                    motor = self.settings['forward' if 'up' in keys else 'reverse']
                if ('left' in keys) != ('right' in keys):
                    servo = self.settings['left' if 'left' in keys else 'right']
                self.browser_sequence = seq
                self.owner_at = time.monotonic()
                self.emit('drive', motor, servo, tick)
            else:
                raise ValueError('unknown control operation')
            return {'ok': True, 'bridge_seq': self.sequence}

    def sensor_ages(self, now):
        return {'camera': now - self.camera_at, 'lidar': now - self.scan_at,
                'control': now - self.control_at}

    def autonomy_status(self, initial_presteer_pwm=None, trial_mode='turn-left'):
        if trial_mode not in ('turn-left', 'turn-cone'):
            raise ValueError('invalid_maneuver_trial_mode')
        compact_target_trial = trial_mode == 'turn-cone'
        if compact_target_trial and initial_presteer_pwm is None:
            initial_presteer_pwm = 1720
        now = time.monotonic()
        try:
            clearance = probe_clearance(self.scan, self.sensor_ages(now), self.args.demo)
            ready, rejection = self.autonomy_healthy(), None
            if self.owner is not None or self.status.get('control', {}).get('armed'):
                ready, rejection = False, 'control_owned_or_unlocked'
        except ValueError as error:
            clearance, ready, rejection = getattr(error, 'clearance', None), False, str(error)
        formal_ready, formal_rejection, goal_summary = False, self.stop_goal_rejection, None
        if self.stop_goal_registry is not None:
            goal = self.stop_goal_registry.goal
            goal_summary = goal.summary()
            try:
                goal.check_fresh(now)
                if goal.identity in self.stop_goal_processed:
                    raise StopGoalError('stop_goal_already_completed')
                if self.owner is not None or self.status.get('control', {}).get('armed'):
                    raise StopGoalError('control_owned_or_unlocked')
                self.formal_preview(goal, 1580, now)
                formal_ready, formal_rejection = True, None
            except (StopGoalError, ValueError) as error:
                formal_rejection = str(error)
        turn_ready, turn_rejection, turn_preview, turn_clearance = False, None, None, None
        try:
            if self.owner is not None or self.auto_session is not None:
                raise ValueError('control_owned_or_unlocked')
            _, turn_preview, turn_clearance = self.turn_preview(now,
                initial_presteer_pwm=initial_presteer_pwm, compact_target_trial=compact_target_trial)
            turn_ready = True
        except ValueError as error:
            turn_clearance = getattr(error, 'clearance', None)
            turn_rejection = str(error)
        trial_goal = dict(self.trial_goal_registry) if self.trial_goal_registry else None
        if trial_goal is not None:
            if now >= trial_goal['expires_monotonic_s']:
                trial_goal.update(execution_ready=False, execution_rejection='trial_goal_expired')
            elif trial_goal['goal_type'] == 'point_stop' or 'target_point_left_m' in trial_goal:
                trial_goal.update(execution_ready=False, execution_rejection='real_pose_missing')
            else:
                trial_goal.update(execution_ready=turn_ready, execution_rejection=None if turn_ready else turn_rejection)
        return {'mode': self.control_mode, 'epoch': self.control_epoch,
                'supported': ['straight_probe', 'straight_segment', 'to_left_junction', 'formal_straight_stop',
                              'bounded_left_turn_trial', 'bounded_first_compact_target_orbit_entry_trial'],
                'competition_supported': False,
                'quality_confirm_s': QUALITY_CONFIRM_S,
                'quality_self_recovery': True, 'quality_recovery_stable_s': QUALITY_RECOVERY_STABLE_S,
                'coast_max_s': COAST_MAX_S, 'front_boundary_loss_s': FRONT_BOUNDARY_LOSS_S,
                'control_age_limit_s': CONTROL_AGE_LIMIT_S, 'auto_control_health_s': AUTO_CONTROL_HEALTH_S,
                'probe_ready': ready, 'rejection': rejection, 'clearance': clearance,
                'formal_ready': formal_ready, 'formal_rejection': formal_rejection, 'final_stop_goal': goal_summary,
                'formal_max_drive_ms': self.stop_goal_contract.budget['max_drive_ms'] if self.stop_goal_contract else None,
                'turn_ready': turn_ready, 'turn_rejection': turn_rejection, 'turn_preview': turn_preview,
                'turn_clearance': turn_clearance,
                'trial_mode': trial_mode,
                'trial_scope': COMPACT_TARGET_TRIAL_SCOPE if compact_target_trial else 'bounded_left_turn_trial',
                'turn_drive_max_s': MAX_DRIVE_S, 'turn_presteer_max_s': MAX_PRESTEER_S,
                'turn_steering_allowance_s': STEERING_ALLOWANCE_S,
                'turn_requires_operator_placement_confirmation': True,
                'steering_candidate_bounds': {'min': SERVO_MIN, 'max': SERVO_MAX, 'right_physically_validated': False},
                'trial_goal': trial_goal,
                'active': dict(self.auto_session['report']) if self.auto_session else None,
                'last_result': self.auto_result}

    def turn_preview(self, now, max_drive_s=MAX_DRIVE_S, initial_presteer_pwm=None, compact_target_trial=False):
        """Pure admission from this fresh scan; not an entrance/navigation certificate."""
        ages = self.sensor_ages(now)
        clearance = probe_clearance(self.scan, ages, self.args.demo, centering=True, rear_launch=False,
                                    camera_required=False, clearance_profile='maneuver')
        try:
            if not self.turn_healthy() or not clearance['motion_ready']:
                raise ValueError('turn_sensor_unavailable')
            control = self.status.get('control', {})
            if (self.control_mode != 'locked' or control.get('armed') is not False
                    or control.get('motor') != 1500 or control.get('servo') != 1500):
                raise ValueError('turn_requires_fresh_neutral_lock')
            controller = ManeuverSequence if compact_target_trial else TurnMotion
            if compact_target_trial and initial_presteer_pwm is None:
                initial_presteer_pwm = 1720
            motion = controller(now, max_drive_s=max_drive_s, initial_presteer_pwm=initial_presteer_pwm)
            scan = {**self.scan, 'received_at': self.scan_at} if compact_target_trial else self.scan
            decision = motion.update(scan, ages['lidar'], now, control)
            if compact_target_trial:
                decision = {**decision, 'trial_scope': COMPACT_TARGET_TRIAL_SCOPE, 'semantic_class': 'unknown',
                            'competition_supported': False, 'completed': False}
            if not decision['start_ready'] or decision['lock_requested']:
                raise ValueError(decision['reason'])
        except ValueError as error:
            error.clearance = clearance
            raise
        return motion, decision, clearance

    def start_turn_trial(self, data, compact_target_trial=False):
        allowed = {'op', 'boot', 'epoch', 'tick', 'placement_confirmed', 'max_drive_s',
                   'initial_presteer_pwm'}
        if not compact_target_trial:
            allowed.add('goal_id')
        if set(data)-allowed or data.get('placement_confirmed') is not True:
            raise ValueError('turn_trial_requires_operator_placement_confirmation')
        if self.stop.is_set() or self.owner is not None or self.auto_session is not None:
            raise ValueError('already_armed_or_shutting_down')
        initial_presteer_pwm = data.get('initial_presteer_pwm', 1720 if compact_target_trial else None)
        if 'initial_presteer_pwm' in data and initial_presteer_pwm is None:
            raise ValueError('invalid_initial_presteer_pwm')
        validate_initial_presteer_pwm(initial_presteer_pwm)
        now = time.monotonic()
        limit = data.get('max_drive_s', MAX_DRIVE_S)
        trial_goal = None
        if 'goal_id' in data:
            trial_goal = self.trial_goal_registry
            if trial_goal is None or data['goal_id'] != trial_goal['goal_id']:
                raise ValueError('trial_goal_not_registered')
            if now >= trial_goal['expires_monotonic_s']:
                raise ValueError('trial_goal_expired')
            if trial_goal['goal_type'] == 'point_stop' or 'target_point_left_m' in trial_goal:
                raise ValueError('real_pose_missing')
            if type(limit) not in (int, float) or not math.isfinite(limit):
                raise ValueError('invalid_bounded_left_turn_trial')
            limit = min(limit, trial_goal['max_seconds'])
        motion, decision, clearance = self.turn_preview(now, limit, initial_presteer_pwm, compact_target_trial)
        tick = data.get('tick')
        current_tick = self.status['control'].get('tick', -1)
        if type(tick) is not int or not 0 <= current_tick-tick < 100 or tick <= self.stopped_tick:
            raise ValueError('stale_arm_request')
        run_id = secrets.token_urlsafe(18)
        report = {'run_id': run_id, 'epoch': self.control_epoch, 'pwm': TRIAL_MOTOR, 'servo': 1500,
            'duration_ms': round(1000*(MAX_PRESTEER_S+motion.max_drive_s)), 'phase': 'presteer',
            'turn_stage': decision['turn_stage'],
            'drive_ticks': 0, 'motion_ticks': 0, 'presteer_ticks': 0, 'coast_ticks': 0, 'recovery_ticks': 0,
            'observed_pwm': False, 'observed_armed': False, 'steering_changes': 0, 'centering': True,
            'turn_trial': True, 'formal_stop_goal': False, 'endpoint': 'left_turn_trial',
            'operator_placement_confirmed': True, 'start_scope': 'operator_placed_mid_segment',
            'sensor_inputs': ['lidar', 'control'], 'camera_required': False,
            'trial_goal': dict(trial_goal) if trial_goal else None,
            'competition_navigation': False, 'wheel_motion_measured': False, 'current_pwm': 1500,
            'physical_steering_confirmed': False, 'turn_path_certified': False, 'entry_confirmed': False,
            'observed_alignment': False, 'turn': decision, 'walls': None, 'coast_motion': None,
            'standstill_confirmed': False, 'quality_issues': [], 'quality_counts': {},
            'clearance_profile': 'maneuver', 'clearance_profile_source': 'bounded_left_turn_trial',
            'clearance_start': clearance, 'clearance_current': clearance,
            'rear_launch_active': False, 'rear_launch_max_s': 0}
        if compact_target_trial:
            report.update(endpoint='first_compact_target_orbit_entry_trial',
                trial_scope=COMPACT_TARGET_TRIAL_SCOPE, semantic_class='unknown', competition_supported=False,
                completed=False, compact_target=decision.get('compact_target'),
                handover_observed=bool(decision.get('handover_observed')),
                orbit_entry_elapsed_s=decision.get('orbit_entry_elapsed_s', 0.),
                clearance_profile_source='bounded_first_compact_target_orbit_entry_trial')
        self.auto_session = {'report': report, 'turn_trial': True, 'turn_motion': motion, 'phase': 'presteer',
            'deadline': now+MAX_PRESTEER_S+motion.max_drive_s, 'last_loop': now, 'heartbeat_seq': 0,
            'quality': self.perception_quality, 'recovery': QualityRecovery(), 'rear_launch': None,
            'motion_scan_seq': None, 'turn_last_servo': 1500, 'turn_servo_sequence': None}
        if compact_target_trial:
            self.auto_session['maneuver_sequence'] = True
        self.auto_result = None
        self.owner, self.owner_at = 'auto-turn-'+run_id, now
        self.control_mode, self.stop_latched = ('auto_turn_cone_trial' if compact_target_trial else 'auto_turn_trial'), False
        try:
            self.emit('arm', tick=tick)
        except (RuntimeError, OSError):
            self.halt('turn_arm_failed')
            raise
        self.arm_sequence = self.sequence
        return {'ok': True, 'run_id': run_id, 'epoch': self.control_epoch}

    def formal_preview(self, goal, pwm, now):
        """Same admission for status and start, using an isolated preview consumer."""
        probe_clearance(self.scan, self.sensor_ages(now), self.args.demo, centering=True, rear_launch=True)
        if not self.autonomy_healthy():
            raise StopGoalError('probe_sensor_stale')
        consumer = StopGoalConsumer(goal, pwm)
        consumer.tick(now, 'drive', corridor_walls(self.scan))
        return consumer

    def accept_stop_goal(self, record, now=None):
        """Protected upstream adapter; a goal never arms, renews heartbeat or emits PWM."""
        now = time.monotonic() if now is None else now
        try:
            goal = FinalStopGoal(record, self.stop_goal_contract, now, real=not self.args.demo)
            with self.lock:
                active = self.auto_session is not None and self.auto_session['report'].get('formal_stop_goal')
                if self.stop_goal_registry is None or (not active and self.stop_goal_registry.goal.identity != goal.identity):
                    self.stop_goal_registry = StopGoalConsumer(goal, 1580)
                    updated = True
                else:
                    updated = self.stop_goal_registry.update(goal)
                self.stop_goal_rejection = None
                return {'updated': updated, 'reason': None if updated else 'repeat_source_not_refreshed',
                        'final_stop_goal': self.stop_goal_registry.goal.summary(), 'motion_requested': False}
        except (StopGoalError, KeyError, TypeError) as error:
            with self.lock:
                self.stop_goal_rejection = str(error)
                if self.auto_session is not None and self.auto_session['report'].get('formal_stop_goal'):
                    self.halt('formal_' + str(error))
                self.stop_goal_registry = None
            raise StopGoalError(str(error)) from error

    def refresh_stop_goal(self, data):
        # No arbitrary source paths or large inline goals can bypass the HTTP limit.
        if set(data) != {'op', 'boot', 'epoch'}:
            raise ValueError('stop_goal_refresh_accepts_no_paths_or_inline_goal')
        with self.lock:
            if data.get('boot') != self.boot or type(data.get('epoch')) is not int or data['epoch'] != self.control_epoch:
                raise ValueError('stale_autonomy_generation')
        if self.stop_goal_source is None:
            raise ValueError('stop_goal_source_not_configured')
        try:
            record = read_local_json(self.stop_goal_source)  # File IO is outside the control lock.
        except StopGoalError as error:
            with self.lock:
                self.stop_goal_rejection = str(error)
                if self.auto_session is not None and self.auto_session['report'].get('formal_stop_goal'):
                    self.halt(str(error))
            raise
        with self.lock:
            if data.get('boot') != self.boot or type(data.get('epoch')) is not int or data['epoch'] != self.control_epoch:
                raise ValueError('stale_autonomy_generation')
            return self.accept_stop_goal(record)

    def autonomy_command(self, data):
        if data.get('op') == 'stop_goal_refresh':
            return self.refresh_stop_goal(data)
        with self.lock:
            if data.get('boot') != self.boot or type(data.get('epoch')) is not int or data['epoch'] != self.control_epoch:
                raise ValueError('stale_autonomy_generation')
            op = data.get('op')
            if op == 'trial_goal_register':
                if set(data) != {'op', 'boot', 'epoch', 'goal'}:
                    raise ValueError('invalid_target_only_trial_registration')
                target = parse_trial_goal(data['goal'])
                now = time.monotonic()
                scan = self.scan or {}
                self.trial_goal_registry = {**target, 'reference_boot': self.boot,
                    'reference_scan_seq': scan.get('seq'), 'reference_publication_at_ms': scan.get('at_ms'),
                    'reference_frame': scan.get('frame_id'), 'received_monotonic_s': now,
                    'expires_monotonic_s': now+MAX_PRESTEER_S+target['max_seconds']+COAST_MAX_S}
                return {'registered': True, 'target_only': True, 'motion_requested': False,
                        'trial_goal': dict(self.trial_goal_registry)}
            if op in ('heartbeat', 'cancel'):
                session = self.auto_session
                if session is None or data.get('run_id') != session['report']['run_id']:
                    raise ValueError('autonomy_session_ended')
                if op == 'cancel':
                    self.halt('operator_stop')
                else:
                    # Sequence prevents a delayed heartbeat renewing a newer lease.
                    seq = data.get('seq')
                    if type(seq) is not int or seq <= session['heartbeat_seq']:
                        raise ValueError('stale_autonomy_heartbeat')
                    if time.monotonic() - self.owner_at >= HEARTBEAT_S:
                        self.halt('autonomy_heartbeat_timeout')
                        raise ValueError('autonomy_session_ended')
                    session['heartbeat_seq'] = seq
                    self.owner_at = time.monotonic()
                return {'ok': True}
            if op == 'turn_left_start':
                return self.start_turn_trial(data)
            if op == 'turn_cone_start':
                return self.start_turn_trial(data, compact_target_trial=True)
            formal = op == 'stop_goal_start'
            if op not in ('probe_start', 'straight_start', 'to_left_junction_start', 'stop_goal_start'):
                raise ValueError('only_bounded_straight_control_is_implemented')
            if self.stop.is_set() or self.owner is not None or self.status.get('control', {}).get('armed'):
                raise ValueError('already_armed_or_shutting_down')
            centering = op != 'probe_start'
            if formal:
                if set(data) != {'op', 'boot', 'epoch', 'tick', 'pwm'}:
                    raise ValueError('stop_goal_start_requires_registered_source_only')
                if self.stop_goal_registry is None:
                    raise ValueError(self.stop_goal_rejection or 'stop_goal_not_received')
                goal = self.stop_goal_registry.goal
                goal.check_fresh(time.monotonic())
                if goal.identity in self.stop_goal_processed:
                    raise ValueError('stop_goal_already_completed')
                duration = goal.contract.budget['max_drive_ms']
                pwm, _ = probe_parameters({'pwm': data.get('pwm'), 'duration_ms': 1000}, straight=True)
                consumer = self.formal_preview(goal, pwm, time.monotonic())
            else:
                pwm, duration = probe_parameters(data, straight=centering)
            if op == 'to_left_junction_start' and self.scan is not None and not all(k in self.scan for k in ['left_junction', 'front_boundary_m']):
                raise ValueError('native_junction_detector_not_installed')
            now = time.monotonic()
            clearance = probe_clearance(self.scan, self.sensor_ages(now), self.args.demo, centering,
                                        rear_launch=centering)
            if not self.autonomy_healthy():
                raise ValueError('probe_sensor_stale')
            tick = data.get('tick')
            current_tick = self.status.get('control', {}).get('tick', -1)
            if type(tick) is not int or not 0 <= current_tick - tick < 100 or tick <= self.stopped_tick:
                raise ValueError('stale_arm_request')
            run_id = secrets.token_urlsafe(18)
            report = {'run_id': run_id, 'pwm': pwm, 'servo': 1500,
                      'duration_ms': duration, 'drive_ticks': 0, 'observed_pwm': False, 'observed_armed': False,
                      'centering': centering, 'steering_changes': 0, 'walls': None,
                      'endpoint': 'final_stop_goal' if formal else 'left_junction' if op == 'to_left_junction_start' else None,
                      'formal_stop_goal': formal, 'formal_completion_evidence': False,
                      'junction_confirmations': 0, 'junction_geometry': None,
                      'rear_launch_active': centering, 'rear_launch_max_s': 1,
                      'current_pwm': pwm, 'approach_front_m': None, 'pwm_ramp_changes': 0,
                      'lane_correcting': False, 'lane_speed_limited': False,
                      'front_time_margin_s': COAST_TIME_MARGIN_S if centering else None,
                      'front_boundary_loss_s': FRONT_BOUNDARY_LOSS_S if centering else None,
                      'boundary_closing_mps': None, 'boundary_time_to_clearance_s': None, 'cruise_limited': False,
                      'motion_ticks': 0, 'recovery_ticks': 0,
                      'quality_issues': clearance['quality_issues'], 'quality_elapsed_ms': 0, 'quality_counts': {},
                      'quality_confirmed': False, 'perception_recovering': not clearance['motion_ready'], 'recovery_stable_ms': 0,
                      'phase': 'drive', 'coast_ticks': 0, 'coast_motion': None, 'standstill_confirmed': False,
                      'wheel_motion_measured': False, 'competition_navigation': False,
                      'clearance_profile': 'straight', 'clearance_profile_source': 'default_straight',
                      'clearance_start': clearance, 'clearance_current': clearance, 'epoch': self.control_epoch}
            self.auto_session = {'report': report, 'deadline': now + duration / 1000,
                                 'last_loop': now, 'heartbeat_seq': 0, 'quality': self.perception_quality, 'recovery': QualityRecovery(),
                                 'phase': 'drive', 'motion_scan_seq': None,
                                 'steering': CorridorSteering(), 'wall_scan_seq': None, 'junction': JunctionStop(),
                                 'rear_launch': RearLaunch() if centering else None,
                                 'ramp': ApproachRamp(pwm) if centering else None}
            if formal:
                self.stop_goal_registry = consumer
                self.auto_session['goal_consumer'] = consumer
                report['formal_goal'] = goal.summary()
            self.auto_session['quality'].update(clearance['quality_issues'], now)
            self.owner = 'auto-' + run_id
            self.owner_at = now
            self.control_mode = 'auto_goal' if formal else 'auto_probe'
            self.stop_latched = False
            try:
                self.emit('arm', tick=tick)
            except (RuntimeError, OSError):
                self.halt('autonomy_arm_failed')
                raise
            self.arm_sequence = self.sequence
            return {'ok': True, 'run_id': run_id, 'epoch': self.control_epoch}

    def autonomy_watch(self):
        while not self.stop.wait(.02):
            with self.lock:
                session = self.auto_session
                if session is None:
                    continue
                now = time.monotonic()
                self.autonomy_tick(now)

    def begin_coast(self, reason, now):
        """Normal planned neutral phase; retains only the existing owner's steering lease."""
        session = self.auto_session
        if session.get('phase') != 'coast':
            session['phase'] = 'coast'
            session['coast_since'] = now
            session['coast_deadline'] = now+COAST_MAX_S
            session['coast_reason'] = reason
            if session.get('turn_motion') is not None:
                session['turn_motion'].begin_coast(reason, now)
            if self.coast_worker is None:
                self.coast_worker = CoastMotionWorker()
            session['motion_generation'] = self.coast_worker.reset()
            session['motion_ready_for_coast'] = False
            session['motion_scan_seq'] = None
            session['coast_start_seq'] = self.scan['seq']
            session['coast_neutral_sequence'] = None
            session['coast_endpoint_verified'] = False
            session['coast_junction_scan_seq'] = None
            if session.get('goal_consumer') is not None:
                session['goal_consumer'].neutral_latched = True
                session['formal_coast_pose_at'] = session['goal_consumer'].goal.raw['pose_source_at']
                session['formal_neutral_after_ms'] = None
            if reason == 'left_junction_reached':
                session['junction'] = JunctionStop()
            if session.get('rear_launch'):
                session['rear_launch'].active = False
            session['report'].update(phase='coast', planned_stop_reason=reason,
                                      current_pwm=1500, coast_ticks=0, standstill_confirmed=False)

    def coast_tick(self, session, now, motion_ready, servo):
        report = session['report']
        if now >= session['coast_deadline']:
            self.halt('coast_standstill_unconfirmed')
            return
        if session.get('turn_motion') is not None:
            control = self.status['control']
            pending_servo = session['turn_servo_sequence']
            turn_control = {**control, 'command_acked': pending_servo is None or control.get('seq', -1) >= pending_servo}
            scan = {**self.scan, 'received_at': self.scan_at} if session.get('maneuver_sequence') else self.scan
            decision = session['turn_motion'].update(scan, report['sensor_ages']['lidar'], now,
                turn_control, safe=motion_ready)
            report['turn'] = decision
            report['physical_steering_confirmed'] = False
            report['observed_alignment'] = decision['observed_alignment']
            report['alignment_evidence'] = decision['alignment_evidence']
            report['entry_confirmed'] = decision['entry_confirmed']
            report['turn_stage'] = decision['turn_stage']
            self.record_compact_target_decision(session, decision)
            if decision['lock_requested']:
                self.halt(decision['reason'])
                return
            if decision['motor'] != 1500 or not SERVO_MIN <= decision['servo'] <= SERVO_MAX:
                raise ValueError('invalid_turn_coast_output')
            servo = decision['servo']
        if (session['coast_reason'] == 'left_junction_reached'
                and (session['coast_junction_scan_seq'] != self.scan['seq'] or not motion_ready)):
            session['coast_endpoint_verified'] = session['junction'].update(
                self.scan, report['sensor_ages']['lidar'], now, motion_ready)
            session['coast_junction_scan_seq'] = self.scan['seq']
        control = self.status['control']
        neutral_ack = (session['coast_neutral_sequence'] is not None
                       and control.get('seq', -1) >= session['coast_neutral_sequence']
                       and control.get('motor') == 1500 and control.get('armed'))
        report['coast_neutral_ack'] = bool(neutral_ack)
        if session.get('goal_consumer') is not None:
            consumer = session['goal_consumer']
            if neutral_ack and session['formal_neutral_after_ms'] is None:
                session['formal_neutral_after_ms'] = consumer.goal.contract.now_interval(self.control_at)[1]
            cutoff = session['formal_neutral_after_ms']
            fresh_pose_after_neutral = (cutoff is not None
                and consumer.goal.raw['pose_source_at'] > max(cutoff, session['formal_coast_pose_at']))
            report['formal_plan'] = consumer.tick(now, 'coast' if neutral_ack and fresh_pose_after_neutral
                                                 else 'neutral_pending', report.get('walls'))
        if not motion_ready or not neutral_ack or self.scan['seq'] <= session['coast_start_seq']:
            if session['motion_ready_for_coast']:
                session['motion_generation'] = self.coast_worker.reset()
            session['motion_ready_for_coast'] = False
            session['motion_scan_seq'] = None
            report['coast_motion'] = self.coast_worker.unknown(
                'perception_unavailable' if not motion_ready else 'awaiting_neutral_and_new_scan')
        else:
            session['motion_ready_for_coast'] = True
            age, received = report['sensor_ages']['lidar'], self.scan_at
            generation = session['motion_generation']
            error = self.coast_worker.submit(generation, self.scan, age, received, now)
            report['coast_motion'] = (self.coast_worker.unknown(error) if error else
                                     self.coast_worker.result(generation, self.scan, age, received, now))
            if error:
                session['motion_generation'] = self.coast_worker.reset()
                session['motion_ready_for_coast'] = False
            session['motion_scan_seq'] = report['coast_motion'].get('source_scan_seq')
        motion = report.get('coast_motion') or {}
        if session.get('goal_consumer') is not None:
            report['standstill_confirmed'] = bool(motion.get('observable') and motion.get('stationary'))
            if (neutral_ack and report['formal_plan']['completed']
                    and session['coast_reason'] == 'formal_goal_neutral'):
                report['formal_completion_evidence'] = True
                self.halt('formal_goal_complete')
                return
        elif motion.get('observable') and motion.get('stationary'):
            report['standstill_confirmed'] = True
            reason = session['coast_reason']
            if reason == 'left_junction_reached' and not session['coast_endpoint_verified']:
                reason = 'left_junction_coast_position_unverified'
            self.halt(reason)
            return
        # No branch in coast may restore positive motor PWM.
        if servo != report.get('servo', 1500):
            report['steering_changes'] += 1
        report['servo'] = servo
        report['current_pwm'] = 1500
        self.emit('drive', 1500, servo, self.status['control']['tick'])
        if session.get('turn_motion') is not None and servo != session['turn_last_servo']:
            session['turn_last_servo'], session['turn_servo_sequence'] = servo, self.sequence
        if session['coast_neutral_sequence'] is None:
            session['coast_neutral_sequence'] = self.sequence
        report['coast_ticks'] += 1

    def autonomy_tick(self, now):
        """Caller holds the control lock. Clock injection permits fault-path tests."""
        session = self.auto_session
        if session is None:
            return
        if now - self.owner_at >= HEARTBEAT_S:
            self.halt('autonomy_heartbeat_timeout')
            return
        if now - session['last_loop'] > .08:
            self.halt('autonomy_control_gap')
            return
        session['last_loop'] = now
        if session.get('phase') == 'coast' and now >= session['coast_deadline']:
            self.halt('coast_standstill_unconfirmed')
            return
        try:
            report = session['report']
            report['sensor_ages'] = self.sensor_ages(now)
            report['sensor_errors'] = dict(self.errors)
            control = self.status.get('control', {})
            rear = session.get('rear_launch')
            rear_active = rear.update(self.scan, now, bool(control.get('armed') and control.get('motor', 1500) > 1500
                                                         and control.get('seq', -1) >= self.arm_sequence)) if rear else False
            report['rear_launch_active'] = rear_active
            # Only this server's turn-start path sets the session mode. Camera
            # omission and request fields cannot select the maneuver envelope.
            turn_trial = session.get('turn_trial') is True
            clearance = probe_clearance(self.scan, report['sensor_ages'], self.args.demo, report.get('centering', False),
                                        rear_launch=rear_active, camera_required=not turn_trial,
                                        clearance_profile='maneuver' if turn_trial else 'straight')
            report['clearance_current'] = clearance
            if not (self.turn_healthy() if turn_trial else self.autonomy_healthy()):
                raise ValueError('probe_sensor_unavailable')
            report['quality_issues'] = clearance['quality_issues']
            counts = report.setdefault('quality_counts', {})
            for issue in clearance['quality_issues']:
                counts[issue] = counts.get(issue, 0)+1
            confirmed = session['quality'].update(clearance['quality_issues'], now)
            report['quality_elapsed_ms'] = session['quality'].elapsed_ms(now)
            report['quality_confirmed'] = confirmed
            recovery = session.get('recovery')
            if recovery is None:
                recovery = session['recovery'] = QualityRecovery()
            motion_ready = recovery.update(clearance['motion_ready'], clearance['quality_issues'], self.scan['seq'], now)
            report['perception_recovering'] = recovery.waiting
            report['recovery_stable_ms'] = recovery.stable_ms(now)
            if session.get('turn_motion') is not None:
                self.turn_tick(session, now, motion_ready)
                return
            if session.get('phase') != 'coast' and now >= session['deadline']:
                if session.get('goal_consumer') is not None:
                    self.halt('formal_goal_deadline')
                    return
                reason = ('perception_recovery_deadline' if not motion_ready or not report.get('motion_ticks', 0) else
                          'left_junction_timeout' if report.get('endpoint') == 'left_junction' else 'probe_complete')
                if report.get('centering') and reason != 'perception_recovery_deadline':
                    self.begin_coast(reason, now)
                    rear_active = False
                else:
                    self.halt(reason)
                    return
            control = self.status.get('control', {})
            if not control.get('armed') or control.get('seq', -1) < self.arm_sequence:
                return
            report['observed_armed'] = True
            report['observed_pwm'] |= control.get('motor') == report['pwm']
            servo = 1500
            if report.get('centering'):
                if session['wall_scan_seq'] != self.scan['seq'] or not motion_ready:
                    report['walls'] = corridor_walls(self.scan) if motion_ready else None
                    session['wall_scan_seq'] = self.scan['seq']
                servo = session['steering'].update(report['walls'], self.scan['seq'], now,
                                                  motion_ready and not rear_active, self.scan.get('at_ms'))
            report['lane_correcting'] = bool(report.get('centering') and session['steering'].correcting)
            if session.get('phase') == 'coast':
                self.coast_tick(session, now, motion_ready, servo)
                return
            if session.get('goal_consumer') is not None:
                report['formal_plan'] = session['goal_consumer'].tick(now, 'drive', report.get('walls'))
                if report['formal_plan']['request_coast']:
                    self.begin_coast('formal_goal_neutral', now)
                    self.coast_tick(session, now, motion_ready, servo)
                    return
            ramp = session.get('ramp')
            if ramp:
                report['current_pwm'] = ramp.update(self.scan, report['sensor_ages']['lidar'], now,
                                                   report['lane_correcting'], motion_ready)
                report['approach_front_m'] = ramp.closest_front
                report['pwm_ramp_changes'] = ramp.changes
                report['lane_speed_limited'] = ramp.lane_limited
                report['boundary_closing_mps'] = ramp.closing_speed
                report['boundary_time_to_clearance_s'] = ramp.time_to_clearance
                report['cruise_limited'] = ramp.cruise_limited
            if report.get('endpoint') == 'left_junction':
                reached = session['junction'].update(self.scan, report['sensor_ages']['lidar'], now, motion_ready)
                report['junction_confirmations'] = session['junction'].count
                report['junction_geometry'] = session['junction'].previous
                if reached:
                    self.begin_coast('left_junction_reached', now)
                    self.coast_tick(session, now, motion_ready, servo)
                    return
            if ramp and ramp.stop_requested:
                self.begin_coast('front_boundary_stop', now)
                self.coast_tick(session, now, motion_ready, servo)
                return
            motor = report.get('current_pwm', report['pwm']) if motion_ready else 1500
            if session.get('goal_consumer') is not None:
                motor = min(motor, report['formal_plan']['motor_cap'])
            if servo != report.get('servo', 1500):
                report['steering_changes'] += 1
            report['servo'] = servo
            self.emit('drive', motor, servo, control['tick'])
            report['drive_ticks'] += 1
            report['motion_ticks' if motion_ready else 'recovery_ticks'] += 1
        except (ValueError, RuntimeError, OSError, KeyError) as error:
            obstacle_clearance = getattr(error, 'clearance', None)
            if obstacle_clearance is not None:
                session['report']['clearance_current'] = obstacle_clearance
            self.halt(str(error))

    def turn_tick(self, session, now, motion_ready):
        """One trial owner; no rear exemption, segment restart or navigation completion."""
        report = session['report']
        control = self.status.get('control', {})
        motion = session['turn_motion']
        neutral_wait = (not motion_ready and session.get('phase') == motion.phase == 'presteer'
            and control.get('armed') is True and control.get('seq', -1) >= self.arm_sequence
            and type(control.get('motor')) is int and control['motor'] == 1500
            and report.get('current_pwm') == 1500
            and set(report.get('quality_issues', [])) <= {'front_sparse'}
            and session['recovery'].waiting)
        if not motion_ready and not neutral_wait:
            self.halt('turn_perception_unavailable')
            return
        if session.get('phase') == 'coast':
            self.coast_tick(session, now, motion_ready, report['servo'])
            return
        if now >= session['deadline']:
            self.halt('turn_total_deadline')
            return
        if not control.get('armed') or control.get('seq', -1) < self.arm_sequence:
            return
        report['observed_armed'] = True
        report['observed_pwm'] |= control.get('motor') == TRIAL_MOTOR
        pending_servo = session['turn_servo_sequence']
        turn_control = {**control, 'command_acked': pending_servo is None or control.get('seq', -1) >= pending_servo}
        scan = {**self.scan, 'received_at': self.scan_at} if session.get('maneuver_sequence') else self.scan
        decision = motion.update(scan, report['sensor_ages']['lidar'], now, turn_control,
                                 safe=True, presteer_wait=neutral_wait)
        report['turn'] = decision
        report['physical_steering_confirmed'] = False
        report['observed_alignment'] = decision['observed_alignment']
        report['alignment_evidence'] = decision['alignment_evidence']
        report['entry_confirmed'] = decision['entry_confirmed']
        report['turn_stage'] = decision['turn_stage']
        self.record_compact_target_decision(session, decision)
        if decision['lock_requested']:
            self.halt(decision['reason'])
            return
        if neutral_wait and (decision['phase'] != 'presteer' or decision['motor'] != 1500
                             or decision['servo'] != report['servo']):
            raise ValueError('invalid_turn_presteer_wait_output')
        if decision['stop_requested']:
            self.begin_coast(decision['reason'], now)
            self.coast_tick(session, now, True, decision['servo'])
            return
        session['phase'] = report['phase'] = decision['phase']
        motor, servo = decision['motor'], decision['servo']
        if motor not in (1500, TRIAL_MOTOR) or not 1500 <= servo <= SERVO_MAX:
            raise ValueError('invalid_turn_trial_output')
        changed = servo != session['turn_last_servo']
        if changed:
            report['steering_changes'] += 1
        report['servo'], report['current_pwm'] = servo, motor
        self.emit('drive', motor, servo, control['tick'])
        if changed:
            session['turn_last_servo'], session['turn_servo_sequence'] = servo, self.sequence
        if neutral_wait:
            report['recovery_ticks'] += 1
        if decision['phase'] == 'presteer':
            report['presteer_ticks'] += 1
        elif motor > 1500:
            report['motion_ticks'] += 1
        if motor > 1500:
            report['drive_ticks'] += 1

    @staticmethod
    def record_compact_target_decision(session, decision):
        if session.get('maneuver_sequence'):
            session['report'].update(trial_scope=COMPACT_TARGET_TRIAL_SCOPE, semantic_class='unknown',
                competition_supported=False, completed=False, compact_target=decision.get('compact_target'),
                handover_observed=bool(decision.get('handover_observed')),
                orbit_entry_elapsed_s=decision.get('orbit_entry_elapsed_s', 0.))

    def configure(self, data):
        with self.lock:
            if self.owner or self.status.get('control', {}).get('armed'):
                raise ValueError('stop and lock before editing PWM')
            ranges = {'forward': (1500, 1620), 'reverse': (1350, 1500), 'left': (1500, SERVO_MAX), 'right': (SERVO_MIN, 1500)}
            if set(data) != set(ranges):
                raise ValueError('four PWM settings required')
            for k, (lo, hi) in ranges.items():
                if type(data[k]) is not int or not lo <= data[k] <= hi:
                    raise ValueError(k + ' is outside server limits')
            self.settings = dict(data)
            return self.settings

    def camera(self):
        cap = None
        try:
            if not self.args.demo:
                check = subprocess.run(['fuser', self.args.camera], capture_output=True, timeout=3)
                if check.returncode != 1:
                    raise RuntimeError('camera busy or owner check failed')
                import cv2
                cap = cv2.VideoCapture(self.args.camera, cv2.CAP_V4L2)
                if not cap.isOpened():
                    raise RuntimeError('camera open failed')
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            encoded = 0
            while not self.stop.is_set():
                if cap is not None:
                    ok, frame = cap.read()
                    if not ok:
                        raise RuntimeError('camera read failed')
                    captured_at = time.monotonic()
                    now = captured_at
                    if now - encoded < .1:
                        continue
                    ok, image = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
                    if not ok:
                        raise RuntimeError('JPEG failed')
                    image = image.tobytes()
                else:
                    image = b'<svg xmlns="http://www.w3.org/2000/svg" width="640" height="480"><rect width="640" height="480" fill="#162d35"/><text x="130" y="250" font-size="32" fill="#76e7c0">DEMO / NO VEHICLE</text></svg>'
                    time.sleep(.1)
                    captured_at = time.monotonic()
                with self.lock:
                    self.jpeg = image
                    self.camera_at = captured_at
                    self.camera_seq += 1
                    sequence = self.camera_seq
                if self.vision and cap is not None:
                    self.vision.submit(image, captured_at, sequence)
                encoded = time.monotonic()
        except Exception as error:
            self.camera_failure(error)
        finally:
            if cap is not None:
                cap.release()

    def camera_failure(self, error):
        with self.lock:
            self.errors['camera'] = str(error)
            if self.auto_session is None or self.auto_session.get('turn_trial') is not True:
                self.halt('camera_failed')

    def state(self, initial_presteer_pwm=None, trial_mode='turn-left'):
        with self.lock:
            now = time.monotonic()
            return {'demo': self.args.demo, 'boot': self.boot, 'status': self.status, 'settings': self.settings,
                    'reverse_enabled': self.args.allow_reverse, 'healthy': self.healthy(),
                    'ages': {'camera': now - self.camera_at, 'lidar': now - self.scan_at, 'control': now - self.control_at},
                    'errors': dict(self.errors), 'recording': self.record is not None, 'saving': self.saving,
                    'record_error': self.record_error, 'owner': self.owner, 'last_stop': self.last_stop,
                    'autonomy': self.autonomy_status(initial_presteer_pwm=initial_presteer_pwm, trial_mode=trial_mode),
                    'files': sorted(x.name for x in self.output.iterdir() if x.suffix in ('.zip', '.jpg', '.png', '.svg'))[-30:]}

    def storage_available(self):
        total = sum(x.stat().st_size for x in self.output.rglob('*') if x.is_file())
        return total < 2 * 1024**3 and shutil.disk_usage(self.output).free > 512 * 1024**2

    def start_record(self):
        with self.lock:
            if self.record or self.saving:
                raise ValueError('recording or saving already active')
            if not self.healthy():
                raise ValueError('sensors not ready')
            if not self.storage_available():
                raise ValueError('recording storage full; download and clear old files on vehicle')
            name = time.strftime('%Y%m%d-%H%M%S') + '-' + secrets.token_hex(3)
            folder = self.output / name
            folder.mkdir(mode=0o700)
            self.record = {'folder': folder, 'started': time.monotonic(), 'frames': 0, 'bytes': 0, 'last_camera': -1, 'last_scan': -1}
            self.record_error = None
            return {'name': name}

    def finish_record(self):
        with self.lock:
            rec = self.record
            if not rec:
                return {'ok': True}
            self.record = None
            self.saving = True
            saved_events = list(self.events)
            saved_settings = dict(self.settings)
        try:
            with self.storage_lock:
                folder = rec['folder']
                (folder / 'control-events.json').write_text(json.dumps(saved_events, ensure_ascii=False))
                (folder / 'metadata.json').write_text(json.dumps({'created_unix_s': time.time(), 'duration_s': time.monotonic() - rec['started'], 'frames': rec['frames'], 'camera_format': 'SVG demo' if self.args.demo else 'JPEG sequence at up to 10 fps', 'lidar': 'raw visualization bins, NOT navigation-certified', 'clock': 'host receive monotonic; not hardware synchronized', 'settings': saved_settings, 'record_error': self.record_error}, ensure_ascii=False, indent=2))
                temporary = self.output / (folder.name + '.zip.part')
                with zipfile.ZipFile(temporary, 'w', compression=zipfile.ZIP_STORED) as z:
                    for file in folder.iterdir():
                        z.write(file, file.name)
                final = temporary.with_suffix('')
                temporary.rename(final)
                shutil.rmtree(folder)
            return {'file': final.name}
        except Exception as error:
            self.record_error = str(error)
            raise
        finally:
            self.saving = False

    def recorder(self):
        while not self.stop.wait(.1):
            finish = False
            try:
                with self.storage_lock:
                    with self.lock:
                        rec = self.record
                        if rec is None:
                            continue
                        image, seq, scan = self.jpeg, self.camera_seq, self.scan
                        camera_at, scan_at = self.camera_at, self.scan_at
                    now = time.monotonic()
                    if image and seq != rec['last_camera'] and now - camera_at < 1:
                        ext = 'svg' if self.args.demo else 'jpg'
                        (rec['folder'] / f'camera-{rec["frames"]:06d}.{ext}').write_bytes(image)
                        with (rec['folder'] / 'frames.jsonl').open('a') as f:
                            f.write(json.dumps({'frame': rec['frames'], 'seq': seq, 'received_monotonic_s': camera_at}) + '\n')
                        rec['frames'] += 1
                        rec['bytes'] += len(image)
                        rec['last_camera'] = seq
                    if scan and scan['seq'] != rec['last_scan'] and now - scan_at < 1:
                        row = json.dumps({'received_monotonic_s': scan_at, **scan}) + '\n'
                        with (rec['folder'] / 'lidar.jsonl').open('a') as f:
                            f.write(row)
                        rec['bytes'] += len(row)
                        rec['last_scan'] = scan['seq']
                    finish = now - rec['started'] >= 60 or rec['bytes'] >= 250 * 1024**2
                    if rec['frames'] % 20 == 0 and not self.storage_available():
                        self.record_error = 'storage quota / free space limit'
                        finish = True
            except Exception as error:
                self.record_error = str(error)
                finish = True
            if finish:
                try:
                    self.finish_record()
                except Exception:
                    pass

    def snapshot(self):
        with self.lock:
            if not self.healthy():
                raise ValueError('no fresh sensor data')
            image, scan, status = self.jpeg, dict(self.scan), dict(self.status)
        name = time.strftime('snapshot-%Y%m%d-%H%M%S-') + secrets.token_hex(3) + '.zip'
        with self.storage_lock:
            if not self.storage_available():
                raise ValueError('storage full')
            temporary = self.output / (name + '.part')
            with zipfile.ZipFile(temporary, 'w') as z:
                z.writestr('camera.svg' if self.args.demo else 'camera.jpg', image)
                z.writestr('lidar.json', json.dumps(scan))
                z.writestr('status.json', json.dumps(status))
                dots = []
                for angle, distance in enumerate(scan['ranges']):
                    if distance is not None:
                        # Display forward up, left left; N10 factory y=-r*sin(angle).
                        x = 320 + math.sin(math.radians(angle)) * distance * 24
                        y = 320 - math.cos(math.radians(angle)) * distance * 24
                        dots.append(f'<circle cx="{x:.2f}" cy="{y:.2f}" r="2" fill="#68e6be"/>')
                z.writestr('lidar.svg', '<svg xmlns="http://www.w3.org/2000/svg" width="640" height="640"><rect width="640" height="640" fill="#10252d"/>' + ''.join(dots) + '</svg>')
            temporary.rename(self.output / name)
        return {'file': name}

    def photo(self, kind):
        if kind == 'vision':
            state = self.vision.snapshot() if self.vision else None
            result = state.get('result') if state else None
            if not result or state.get('error') or result['age_ms'] >= 1500:
                raise ValueError('vision result unavailable or stale')
            payload = base64.b64decode(result['jpeg_base64'], validate=True)
            return self.write_photo('vision-' + str(result['sequence']), 'jpg', payload)
        if kind not in ('camera', 'lidar', 'combined'):
            raise ValueError('unknown photo type')
        with self.lock:
            now = time.monotonic()
            if kind != 'lidar' and (self.jpeg is None or now - self.camera_at >= 1):
                raise ValueError('camera frame stale')
            if kind != 'camera' and (self.scan is None or now - self.scan_at >= 1):
                raise ValueError('lidar frame stale')
            image, scan = self.jpeg, self.scan
        ext = 'jpg' if kind == 'camera' else 'png'
        if self.args.demo:
            if kind != 'camera':
                raise ValueError('PNG rendering requires vehicle OpenCV; demo camera SVG is available')
            ext = 'svg'
            payload = image
        elif kind == 'camera':
            payload = image
        else:
            import cv2
            import numpy as np
            radar = np.full((480, 640, 3), (36, 28, 13), dtype=np.uint8)
            for radius in range(2, 13, 2):
                cv2.circle(radar, (320, 255), radius * 18, (77, 65, 41), 1)
                cv2.putText(radar, str(radius) + 'm', (325, 255-radius*18+14), cv2.FONT_HERSHEY_SIMPLEX, .4, (158, 146, 120), 1)
            cv2.line(radar, (320, 20), (320, 465), (77, 65, 41), 1)
            cv2.line(radar, (30, 255), (610, 255), (77, 65, 41), 1)
            for angle, distance in enumerate(scan['ranges']):
                if distance is not None:
                    x = round(320 + math.sin(math.radians(angle))*distance*18)
                    y = round(255 - math.cos(math.radians(angle))*distance*18)
                    cv2.circle(radar, (x, y), 2, (188, 226, 114), -1)
            cv2.putText(radar, 'FRONT', (296, 18), cv2.FONT_HERSHEY_SIMPLEX, .45, (188, 226, 114), 1)
            cv2.circle(radar, (320, 255), 5, (188, 226, 114), -1)
            result = radar
            if kind == 'combined':
                camera = cv2.imdecode(np.frombuffer(image, dtype=np.uint8), cv2.IMREAD_COLOR)
                if camera is None:
                    raise ValueError('camera JPEG decode failed')
                result = np.vstack((np.full((40, 1280, 3), (36, 28, 13), dtype=np.uint8), np.hstack((cv2.resize(camera, (640,480)), radar))))
                cv2.putText(result, 'XT-STCAR | CAMERA + LIDAR | '+time.strftime('%Y-%m-%d %H:%M:%S'), (18,27), cv2.FONT_HERSHEY_SIMPLEX, .55, (230,240,240), 1)
            ok, encoded = cv2.imencode('.png', result)
            if not ok:
                raise ValueError('PNG encoding failed')
            payload = encoded.tobytes()
        return self.write_photo(kind, ext, payload)

    def write_photo(self, kind, ext, payload):
        name = kind+'-'+time.strftime('%Y%m%d-%H%M%S')+'-'+secrets.token_hex(3)+'.'+ext
        with self.storage_lock:
            if not self.storage_available():
                raise ValueError('storage full')
            temporary = self.output/(name+'.part')
            temporary.write_bytes(payload)
            temporary.rename(self.output/name)
        return {'file': name}

    def close(self):
        self.stop.set()
        self.halt('server_shutdown')
        if self.coast_worker is not None:
            self.coast_worker.close()
        if self.vision:
            self.vision.close()
        if hasattr(self, 'control'):
            self.control.stdin.close()  # Bridge independently observes EOF and sends neutral frames.
            try:
                self.control.wait(timeout=3)
            except subprocess.TimeoutExpired:
                pass  # Never kill the neutral-sending owner as an automatic fallback.
        if hasattr(self, 'lidar'):
            self.lidar.terminate()
        try:
            self.finish_record()
        except Exception:
            pass


def serve(args):
    app = Console(args)
    limiter = threading.BoundedSemaphore(12)

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass  # URLs may contain the access token; never log them.

        def setup(self):
            super().setup()
            self.connection.settimeout(3)
            self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

        def reply(self, status, body, content='application/json'):
            if not isinstance(body, bytes):
                body = json.dumps(body, ensure_ascii=False).encode()
            self.send_response(status)
            self.send_header('Content-Type', content)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.send_header('Referrer-Policy', 'no-referrer')
            self.send_header('Content-Security-Policy', "default-src 'self'; img-src 'self' blob:; style-src 'self'; script-src 'self'; frame-ancestors 'none'; base-uri 'none'")
            self.end_headers()
            self.wfile.write(body)

        def authorized(self):
            supplied = self.headers.get('X-Control-Token', '') or parse_qs(urlsplit(self.path).query).get('token', [''])[0]
            return hmac.compare_digest(supplied, app.token)

        def do_GET(self):
            if not limiter.acquire(False):
                self.reply(503, {'error': 'busy'})
                return
            try:
                path = urlsplit(self.path).path
                if path in ['/', '/app.js', '/style.css']:
                    file = ROOT / {'/': 'index.html', '/app.js': 'app.js', '/style.css': 'style.css'}[path]
                    kind = {'/': 'text/html; charset=utf-8', '/app.js': 'text/javascript', '/style.css': 'text/css'}[path]
                    self.reply(200, file.read_bytes(), kind)
                    return
                if not self.authorized():
                    self.reply(403, {'error': 'access token required'})
                    return
                if path == '/api/state':
                    query = parse_qs(urlsplit(self.path).query, keep_blank_values=True)
                    initial = query.get('initial_presteer_pwm')
                    mode = query.get('trial_mode')
                    if mode is not None and (len(mode) != 1 or mode[0] not in ('turn-left', 'turn-cone')):
                        self.reply(400, {'error': 'invalid_maneuver_trial_mode'})
                        return
                    if initial is None and mode is None:
                        self.reply(200, app.state())
                    else:
                        try:
                            initial_pwm = None
                            if initial is not None:
                                if len(initial) != 1 or not initial[0].isascii() or not initial[0].isdigit():
                                    raise ValueError('invalid_initial_presteer_pwm')
                                initial_pwm = validate_initial_presteer_pwm(int(initial[0]))
                        except ValueError:
                            self.reply(400, {'error': 'invalid_initial_presteer_pwm'})
                            return
                        self.reply(200, app.state(initial_presteer_pwm=initial_pwm,
                                                  trial_mode=mode[0] if mode is not None else 'turn-left'))
                elif path == '/api/vision':
                    self.reply(200, app.vision.snapshot() if app.vision else {'enabled': False})
                elif path == '/api/lidar':
                    with app.lock:
                        snapshot = {'scan': app.scan, 'age_s': time.monotonic() - app.scan_at}
                    self.reply(200, snapshot)
                elif path == '/camera.jpg':
                    with app.lock:
                        image, age = app.jpeg, time.monotonic() - app.camera_at
                    if image is None or age > 1:
                        self.reply(503, {'error': 'camera stale'})
                    else:
                        self.reply(200, image, 'image/svg+xml' if args.demo else 'image/jpeg')
                elif path.startswith('/download/'):
                    name = path[len('/download/'):]
                    if '/' in name or '\\' in name or Path(name).suffix not in ('.zip', '.jpg', '.png', '.svg') or name.startswith('.'):
                        self.reply(404, {'error': 'not found'})
                        return
                    file = app.output / name
                    if not file.is_file():
                        self.reply(404, {'error': 'not found'})
                        return
                    self.send_response(200)
                    self.send_header('Content-Type', {'.zip':'application/zip', '.jpg':'image/jpeg', '.png':'image/png', '.svg':'image/svg+xml'}[file.suffix])
                    self.send_header('Content-Length', str(file.stat().st_size))
                    self.send_header('Content-Disposition', 'attachment; filename="' + name + '"')
                    self.send_header('Cache-Control', 'no-store')
                    self.end_headers()
                    with file.open('rb') as f:
                        shutil.copyfileobj(f, self.wfile, 65536)
                else:
                    self.reply(404, {'error': 'not found'})
            except (BrokenPipeError, ConnectionResetError, TimeoutError):
                pass
            finally:
                limiter.release()

        def do_POST(self):
            if not self.authorized():
                self.reply(403, {'error': 'access token required'})
                return
            origin = self.headers.get('Origin')
            if origin and origin != 'http://' + self.headers.get('Host', ''):
                self.reply(403, {'error': 'cross-origin request rejected'})
                return
            try:
                size = int(self.headers.get('Content-Length', '0'))
                if not 0 < size <= 2048 or self.headers.get('Content-Type') != 'application/json':
                    raise ValueError('bounded JSON body required')
                data = json.loads(self.rfile.read(size))
                if not isinstance(data, dict):
                    raise ValueError('JSON object required')
                path = urlsplit(self.path).path
                if path == '/api/control': result = app.command(data)
                elif path == '/api/autonomy': result = app.autonomy_command(data)
                elif path == '/api/settings': result = app.configure(data)
                elif path == '/api/snapshot': result = app.snapshot()
                elif path == '/api/photo': result = app.photo(data.get('kind'))
                elif path == '/api/record/start': result = app.start_record()
                elif path == '/api/record/stop': result = app.finish_record()
                else: raise ValueError('unknown action')
                self.reply(200, result)
            except (ValueError, RuntimeError, OSError) as error:
                self.reply(400, {'error': str(error)})

    class Server(ThreadingHTTPServer):
        daemon_threads = True
        request_queue_size = 8
        def server_bind(self):
            # Avoid reverse DNS lookup delaying startup on offline vehicle networks.
            socketserver.TCPServer.server_bind(self)
            self.server_name = self.server_address[0]
            self.server_port = self.server_address[1]
        def process_request(self, request, client_address):
            # Bound thread creation, not just route work. Reserve capacity for stop.
            if not self.slots.acquire(False):
                request.close()
                return
            try:
                super().process_request(request, client_address)
            except Exception:
                self.slots.release()
                raise
        def process_request_thread(self, request, client_address):
            try:
                super().process_request_thread(request, client_address)
            finally:
                self.slots.release()

    servers = []
    try:
        for address in dict.fromkeys([args.bind] + ([args.lan_bind] if args.lan_bind else [])):
            server = Server((address, args.port), Handler)
            server.slots = threading.BoundedSemaphore(20)
            server.timeout = .2
            servers.append(server)
    except Exception:
        for server in servers:
            server.server_close()
        raise
    for sig in (signal.SIGTERM, signal.SIGINT):
        signal.signal(sig, lambda *_: app.stop.set())
    access = Path(args.access_file)
    access.parent.mkdir(parents=True, exist_ok=True)
    fd = os.open(access, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
    os.fchmod(fd, 0o600)
    with os.fdopen(fd, 'w') as f:
        json.dump({'token': app.token, 'port': args.port, 'pid': os.getpid()}, f)
    try:
        app.start()
        print(f'Console listening on {args.bind}:{args.port}; credentials in {access}', flush=True)
        while not app.stop.is_set():
            for ready in select.select(servers, [], [], .2)[0]:
                ready.handle_request()
    finally:
        app.close()
        for server in servers:
            server.server_close()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--bridge', required=True)
    p.add_argument('--bind', default='127.0.0.1')
    p.add_argument('--lan-bind', help='Additional explicit private IPv4 address for LAN browsers')
    p.add_argument('--port', type=int, default=8081)
    p.add_argument('--camera', default='/dev/video20')
    p.add_argument('--output', default='./recordings')
    p.add_argument('--access-file', default='./access.json')
    p.add_argument('--demo', action='store_true')
    vision_group = p.add_mutually_exclusive_group()
    vision_group.add_argument('--vision-config', type=Path, help='optional persistent shadow configuration JSON')
    vision_group.add_argument('--vision-shadow', nargs=5, metavar=('BIN', 'ROAD_CONFIG', 'MODEL', 'ORT_LIB', 'MODEL_SPEC'), help='optional perception-only process; reuses camera frames, never drives')
    p.add_argument('--allow-reverse', action='store_true', help='Only after supervised ESC reverse calibration')
    p.add_argument('--stop-goal-source', type=Path, help='Explicit local upstream final-goal file; HTTP cannot choose a path')
    p.add_argument('--stop-goal-contract', type=Path, help='Verified clock/origin/errors/measured stopping contract')
    p.add_argument('--stop-goal-vehicle-profile', type=Path, help='Explicit measured vehicle profile; current unverified profile blocks real goals')
    args = p.parse_args()
    if args.vision_config:
        from vision_shadow import load_config
        try:
            args.vision_shadow = load_config(args.vision_config)
        except (OSError, ValueError) as error:
            p.error(str(error))
    if args.demo and args.vision_shadow:
        p.error('demo SVG camera cannot feed JPEG perception; use recorded JPEG IPC for offline tests')
    for bind in [args.bind] + ([args.lan_bind] if args.lan_bind else []):
        address = ipaddress.ip_address(bind)
        if address.version != 4 or address.is_unspecified or address.is_multicast or not (address.is_private or address.is_loopback):
            p.error('explicit private/loopback IPv4 bind required')
    lock = open('/tmp/xt-stcar-console-demo-' + str(args.port) + '.lock' if args.demo else '/tmp/xt-stcar-console.lock', 'a')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    serve(args)


if __name__ == '__main__':
    main()
