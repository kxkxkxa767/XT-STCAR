//! Real ROS2 transport for the unchanged race control thread. The hardware
//! workers are separate ROS2 processes, so their native Guard survives a
//! controller crash. DDS publishing never blocks the control thread.
use rclrs::*;
use ros_env::std_msgs::msg::{Bool, String as RosString};
use std::{
    io::Read,
    os::unix::process::CommandExt,
    path::Path,
    process::{Command, Stdio},
    sync::{Arc, mpsc},
    thread::JoinHandle,
    time::Instant,
};
use crate::race::{Event, Options, Transport, check_device, spawn_named};

pub(crate) const COMMAND: &str = "/xt_stcar/control_command";
pub(crate) const CONTROL: &str = "/xt_stcar/control_state";
pub(crate) const SCAN: &str = "/xt_stcar/scan_frame";

pub(crate) struct Controller {
    commands: Arc<ExecutorCommands>,
    executor: JoinHandle<()>,
    publisher: JoinHandle<()>,
    _node: Node,
    _inputs: Vec<Subscription<RosString>>,
    _start: Subscription<Bool>,
    _stop: Subscription<Bool>,
}

impl Controller {
    pub fn close(self) {
        let _ = self.publisher.join();
        self.commands.halt_spinning();
        let _ = self.executor.join();
    }
}

pub(crate) fn connect(o: &Options, tx: mpsc::Sender<Event>) -> Result<Transport, String> {
    let setup = || -> Result<Transport, Box<dyn std::error::Error>> {
        if !o.demo {
            check_device(&o.control_device, "/dev/ttyUSB1")?;
            check_device(&o.lidar_device, "/dev/ttyACM0")?;
        }
        let context = Context::new(std::iter::once("route-engine".to_owned()).chain(o.ros_args.iter().cloned()), InitOptions::default())?;
        let mut executor = context.create_basic_executor();
        let node = executor.create_node("xt_stcar_race")?;
        let command = node.create_publisher::<RosString>(COMMAND.keep_last(4).reliable().volatile())?;
        let mut inputs = Vec::new();
        for (topic, kind) in [(CONTROL, "control"), (SCAN, "lidar")] {
            let events = tx.clone();
            inputs.push(node.create_subscription::<RosString, _>(topic.keep_last(1).best_effort().volatile(),
                move |msg: RosString| {
                    let event = if msg.data.len() > crate::race::LINE_LIMIT {
                        Event::Ended(kind, format!("{kind} ROS2 frame exceeds existing line limit"))
                    } else { Event::Line(kind, msg.data, Instant::now()) };
                    let _ = events.send(event);
                })?);
        }
        let events = tx.clone();
        let start = node.create_subscription::<Bool, _>("/xt_stcar/start".keep_last(1).reliable().volatile(),
            move |msg: Bool| { if msg.data { let _ = events.send(Event::Enter); } })?;
        let events = tx.clone();
        let stop = node.create_subscription::<Bool, _>("/xt_stcar/stop".keep_last(1).reliable().volatile(),
            move |msg: Bool| { if msg.data { let _ = events.send(Event::Halt("ros2_operator_stop")); } })?;
        let events = tx.clone();
        ctrlc::set_handler(move || { let _ = events.send(Event::Halt("ros2_shutdown")); })?;

        // Spawn the independent hardware nodes with an EOF lifeline only.
        // All sensor/status/command data uses ROS2 topics; stdin carries none.
        let executable = std::env::current_exe()?;
        let mut children = Vec::new();
        for (kind, device) in [("control", &o.control_device), ("lidar", &o.lidar_device)] {
            let mut child = Command::new(&executable);
            child.args(["ros2-hardware", kind, "--parent-lifeline"]);
            if !o.demo { child.args(["--device", device, "--execute"]); }
            if kind == "lidar" {
                let folder = if o.bridge.is_empty() { executable.parent() } else { Path::new(&o.bridge).parent() };
                let calibration = o.lidar_calibration.clone()
                    .or_else(|| folder.map(|d| d.join("lidar-calibration.json")).filter(|p| p.is_file()));
                if let Some(path) = calibration {
                    child.arg("--lidar-calibration").arg(path);
                }
            }
            match child.stdin(Stdio::piped()).stdout(Stdio::piped()).stderr(Stdio::inherit()).process_group(0).spawn() {
                Ok(mut child) => {
                    let mut lifeline = child.stdout.take().unwrap();
                    let events = tx.clone();
                    spawn_named(&format!("watch-{kind}"), move || {
                        let mut bytes = [0; 256];
                        loop {
                            match lifeline.read(&mut bytes) {
                                Ok(0) => break,
                                Ok(_) => {},
                                Err(error) if error.kind() == std::io::ErrorKind::Interrupted => {},
                                Err(_) => break,
                            }
                        }
                        let _ = events.send(Event::Ended(kind, format!("{kind} ROS2 hardware node ended")));
                    });
                    children.push((kind, child));
                },
                Err(error) => {
                    for (_, mut child) in children { drop(child.stdin.take()); let _ = child.wait(); }
                    return Err(error.into());
                }
            }
        }

        let (send, requests) = mpsc::sync_channel::<String>(4);
        let events = tx.clone();
        let publisher = spawn_named("ros2-command", move || {
            for data in requests {
                if let Err(error) = command.publish(RosString { data }) {
                    eprintln!("ROS2 command publish: {error}");
                    let _ = events.send(Event::Halt("ros2_command_failed"));
                    break;
                }
            }
        });
        let commands = executor.commands().clone();
        let events = tx.clone();
        let runner = spawn_named("ros2-executor", move || {
            // One long-running spin: no new executor thread per control tick.
            let errors = executor.spin(SpinOptions::default());
            if !errors.is_empty() { eprintln!("ROS2 executor: {errors:?}"); }
            let _ = events.send(Event::Halt("ros2_executor_ended"));
        });
        Ok(Transport {
            writer: Some(Box::new(move |emit| send.try_send(crate::py::to_json(&emit.row())).map_err(|e| e.to_string()))),
            children,
            ros2: Some(Controller { commands, executor: runner, publisher, _node: node,
                _inputs: inputs, _start: start, _stop: stop }),
        })
    };
    setup().map_err(|e| format!("ROS2 startup: {e}"))
}
