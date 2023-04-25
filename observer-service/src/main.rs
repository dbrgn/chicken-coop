use std::{
    io::{BufRead, BufReader},
    path::PathBuf,
    time::Duration,
};

use anyhow::Context;
use clap::{arg, command, Parser};
use lazy_static::lazy_static;
use regex::Regex;
use threema_gateway::{ApiBuilder, RecipientKey};

mod config;

use crate::config::{Config, RawConfig};

#[derive(Parser, Debug)]
#[command(about, author = "Danilo Bargen", version)]
struct Args {
    /// Path to config file
    #[arg(short, long, default_value = "config.toml")]
    config: PathBuf,
}

fn main() -> anyhow::Result<()> {
    // Parse command line arguments
    let args = Args::parse();

    // Parse config
    let raw_config = match RawConfig::load(&args.config) {
        Ok(val) => val,
        Err(e) => {
            println!("Error: Failed to load config: {:#}", e);
            println!();
            println!(
                "Example config:\n\n{}",
                toml::to_string(&RawConfig::example())?
            );
            return Ok(());
        }
    };
    let config: Config = raw_config.try_into()?;

    // Connect to serial device
    let raw_port = serialport::new(config.serial.port.to_str().unwrap(), config.serial.baudrate)
        .timeout(Duration::from_secs(30))
        .open()
        .context(format!(
            "Failed to open serial port at {:?}",
            config.serial.port
        ))?;

    // Buffered reading
    let mut port = BufReader::new(raw_port);

    // Main loop
    let mut line_buffer = String::new();
    loop {
        match port.read_line(&mut line_buffer) {
            Ok(_size) => process_line(line_buffer.trim()),
            Err(e) => eprintln!("Error while reading: {}", e),
        }
        line_buffer.clear();
    }

    Ok(())
}

#[derive(Debug)]
enum Event {
    Update { current_state: String },
    Transition { from: String, to: String },
}

/// Parse a line and return a parsed
fn parse_line(line: &str) -> Option<Event> {
    // Patterns
    lazy_static! {
        static ref UPDATE_RE: Regex =
            Regex::new(r#"^:: [0-9:]* Update \[State=(?P<state>[a-zA-Z]*)\]$"#).unwrap();
        static ref TRANSITION_RE: Regex =
            Regex::new("^:: State transition: (?P<from>[a-zA-Z]*) -> (?P<to>[a-zA-Z]*)$").unwrap();
    }

    // Check for matches
    if let Some(capture) = UPDATE_RE.captures(line) {
        return Some(Event::Update {
            current_state: capture["state"].to_string(),
        });
    }
    if let Some(capture) = TRANSITION_RE.captures(line) {
        return Some(Event::Transition {
            from: capture["from"].to_string(),
            to: capture["to"].to_string(),
        });
    }

    // No match
    None
}

fn process_line(line: &str) {
    println!("Line: {:?}", parse_line(line));
}
