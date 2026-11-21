//! Numbered field entry. Only the selected binary spawns the shared race unit.
use std::os::unix::process::CommandExt;
use std::path::{Path, PathBuf};
use std::process::Command;

#[derive(Debug, PartialEq)]
struct Choice<'a> {
    program: &'a str,
    seconds: &'a str,
    profile: &'a str,
    action: Option<&'a str>,
}

fn parse<'a>(name: &'a str, flags: &'a [String]) -> Result<Choice<'a>, String> {
    let entry = name.strip_prefix("start_").and_then(|v| v.split_once('_'));
    let (program, seconds) = entry.ok_or("用法为 start_程序号_最大等灯秒数，例如 start_4_7")?;
    if !matches!(program, "4" | "5") {
        return Err("程序 4 为视觉建图版，程序 5 为视觉 1560 版".into());
    }
    let decimal = seconds.split_once('.');
    let digits = |v: &str| !v.is_empty() && v.bytes().all(|b| b.is_ascii_digit());
    if !decimal.map_or_else(|| digits(seconds), |(a, b)| digits(a) && digits(b))
        || !seconds.parse::<f64>().is_ok_and(|s| s.is_finite() && s >= 0.)
    {
        return Err("最大等灯秒数须为非负数字，例如 7 或 7.5".into());
    }
    let mut choice = Choice {
        program,
        seconds,
        profile: "rust",
        action: None,
    };
    let mut flags = flags.iter();
    while let Some(flag) = flags.next() {
        match flag.as_str() {
            "--profile" => {
                choice.profile = flags
                    .next()
                    .map(String::as_str)
                    .filter(|p| *p == "rust")
                    .ok_or("视觉版本的 profile 为 rust")?;
            }
            "--show" | "--status" | "--stop" | "--check-only" if choice.action.is_none() => {
                choice.action = Some(flag);
            }
            _ => return Err(format!("不支持的组合参数 {flag}")),
        }
    }
    Ok(choice)
}

impl Choice<'_> {
    fn command(&self, dir: &Path) -> Command {
        let stem = format!("program-{}", self.program);
        let mut command = Command::new(dir.join(&stem));
        command
            .arg("start")
            .arg("--config")
            .arg(dir.join(stem).with_extension("json"))
            .args(["--profile", self.profile, "--light-max-wait-s", self.seconds]);
        if let Some(action) = self.action {
            command.arg(action);
        }
        command
    }
}

pub fn run(name: &str, flags: &[String]) -> i32 {
    let choice = match parse(name, flags) {
        Ok(choice) => choice,
        Err(error) => {
            eprintln!("{error}");
            return 2;
        }
    };
    let dir: PathBuf = match std::env::current_exe()
        .ok()
        .and_then(|p| p.parent().map(Path::to_owned))
    {
        Some(dir) => dir,
        None => {
            eprintln!("无法定位已安装的比赛程序");
            return 1;
        }
    };
    let error = choice.command(&dir).exec();
    eprintln!("无法执行程序 {}，{error}", choice.program);
    1
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn names_choose_the_binary_and_forward_seconds_without_overrides() {
        let flags = ["--profile", "rust", "--check-only"].map(String::from);
        let c = parse("start_5_7.5", &flags).unwrap();
        let command = c.command(Path::new("/race"));
        assert_eq!(command.get_program(), Path::new("/race/program-5"));
        assert_eq!(
            command.get_args().collect::<Vec<_>>(),
            [
                "start",
                "--config",
                "/race/program-5.json",
                "--profile",
                "rust",
                "--light-max-wait-s",
                "7.5",
                "--check-only"
            ]
        );
        assert_eq!(parse("start_4_0", &[]).unwrap().profile, "rust");
        assert!(parse("start_4_240", &[]).is_ok());
        for bad in ["start_3_7", "start_4_-1", "start_4_nan", "start_5_7_extra", "start_4_"] {
            assert!(parse(bad, &[]).is_err());
        }
        assert!(parse("start_4_7", &["--light-max-wait-s".into(), "999".into()]).is_err());
    }
}
