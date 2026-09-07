#!/usr/bin/env python3
"""Opt-in real Rust robot pipeline test; no hardware or Python inference backend."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parent.parent


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', type=Path, default=ROOT / 'target/debug/xt-stcar-robot')
    parser.add_argument('--model', type=Path, default=ROOT / 'models/yolo26n.onnx')
    parser.add_argument('--runtime-lib', type=Path, required=True)
    parser.add_argument('--image', type=Path, required=True)
    parser.add_argument('--report', type=Path)
    args = parser.parse_args()
    image = args.image.resolve()
    events = [json.loads(line) for line in (ROOT / 'examples/robot-sim.jsonl').read_text().splitlines()]
    for at, sequence in ((20, 1), (40, 2)):
        events.append(dict(at=at, event=dict(type='vision_frame', path=str(image), sequence=sequence, frame_id='sim_camera')))
    events.sort(key=lambda event: event['at'])
    with tempfile.TemporaryDirectory(prefix='xt-stcar-robot-reference-') as directory:
        work = Path(directory)
        source, output = work / 'events.jsonl', work / 'output.jsonl'
        source.write_text(''.join(json.dumps(event) + '\n' for event in events))
        command = [str(args.binary.resolve()), 'replay', '--config', str(ROOT / 'config/robot-sim.json'),
                   '--events', str(source), '--model', str(args.model.resolve()),
                   '--runtime-lib', str(args.runtime_lib.resolve()), '--vision-config', str(ROOT / 'config/yolo26n.json'),
                   '--output', str(output)]
        process = subprocess.run(command, capture_output=True, text=True, timeout=90, check=False)
        assert process.returncode == 0, process.stderr
        records = [json.loads(line) for line in output.read_text().splitlines()]
        frames = [record['vision'] for record in records if record.get('vision')]
        assert len(frames) == 2
        assert all(frame['backend'] == 'rust-onnxruntime' and frame['inference_performed'] for frame in frames)
        assert frames[0]['detections'] == frames[1]['detections'], 'same input must agree across a persistent session'
        assert all(record['physical_output_enabled'] is False for record in records)
        assert records[-2]['terminal'] == 'end_of_stream'
        assert records[-2]['step']['output']['command']['type'] == 'stop'
        summary = records[-1]
        assert summary['vision_frames'] == 2 and summary['final_state'] == 'disarmed'
        frame_counts = [len(frame['detections']) for frame in frames]
        # Real native model loading succeeds, then a corrupt camera image must
        # terminate the replay with a complete emergency-stop log and nonzero exit.
        corrupt = work / 'bad.png'
        corrupt.write_bytes(b'not an image')
        bad_events = [event for event in events if event['at'] < 20]
        bad_events.append(dict(at=20, event=dict(type='vision_frame', path=str(corrupt), sequence=1, frame_id='sim_camera')))
        source.write_text(''.join(json.dumps(event) + '\n' for event in bad_events))
        process = subprocess.run(command, capture_output=True, text=True, timeout=90, check=False)
        assert process.returncode != 0 and 'stop was recorded' in process.stderr
        error_records = [json.loads(line) for line in output.read_text().splitlines()]
        terminal = error_records[-1]
        assert terminal['terminal'] == 'vision_error'
        assert terminal['step']['state'] == 'fault'
        assert terminal['step']['output']['command']['type'] == 'stop'
        assert terminal['step']['emergency_stop_latched'] is True
    report = dict(status='passed', mode='offline replay with real Rust ORT inference',
                  physical_output_enabled=False, model_sha256=hashlib.sha256(args.model.read_bytes()).hexdigest(),
                  image_sha256=hashlib.sha256(image.read_bytes()).hexdigest(),
                  repeated_frame_detection_counts=frame_counts, summary=summary,
                  corrupt_frame_result='nonzero exit with complete latched emergency-stop record',
                  timing_scope='Event timestamps are simulated; no real-time control or vehicle behavior was tested.')
    serialized = json.dumps(report, indent=2, ensure_ascii=False) + '\n'
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(serialized)
    print(serialized, end='')


if __name__ == '__main__':
    main()
