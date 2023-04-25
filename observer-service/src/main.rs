use std::{
    io::{BufRead, BufReader, Read},
    path::PathBuf,
    time::Duration,
};

use anyhow::Context;
use clap::{arg, command, Parser};
use lazy_static::lazy_static;
use regex::Regex;
use threema_gateway::RecipientKey;

mod config;

use crate::config::{Config, RawConfig};

#[derive(Parser, Debug)]
#[command(about, author = "Danilo Bargen", version)]
struct Args {
    /// Path to config file
    #[arg(short, long, default_value = "config.toml")]
    config: PathBuf,
}

#[tokio::main]
async fn main() -> anyhow::Result<()> {
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
    let config = Config::from_raw(raw_config).await?;

    // Connect to serial device
    let mut raw_port =
        serialport::new(config.serial.port.to_str().unwrap(), config.serial.baudrate)
            .timeout(Duration::from_millis(1))
            .open()
            .context(format!(
                "Failed to open serial port at {:?}",
                config.serial.port
            ))?;

    // Clear input buffer by reading and discarding everything
    {
        let mut buf = Vec::new();
        let _ = raw_port.read_to_end(&mut buf);
        println!("Cleared {} bytes from input buffer", buf.len());
    }

    // Reconfigure timeout
    raw_port
        .set_timeout(Duration::from_secs(30))
        .context("Could not set timeout")?;

    // Main loop
    let mut line_buffer = String::new();
    let mut port = BufReader::new(raw_port);
    println!("Waiting for serial data...");
    loop {
        match port.read_line(&mut line_buffer) {
            Ok(_size) => process_line(line_buffer.trim(), &config).await,
            Err(e) => {
                eprintln!("Error while reading: {}", e);
                std::process::exit(1);
            }
        }
        line_buffer.clear();
    }
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

async fn process_line(line: &str, config: &Config) {
    match parse_line(line) {
        Some(Event::Update { current_state }) => {
            println!("State: {}", current_state);
        }
        Some(Event::Transition { from, to }) => {
            println!("Transition from {} to {}", from, to);
            if let Some(threema) = &config.threema {
                let emoji = match &*to {
                    "PreOpening" => "🐔",
                    "Open" => "☀️",
                    "PreClosing" => "🐔",
                    "Closed" => "🌙",
                    "Error" => "😱",
                    _ => "😶",
                };
                for (recipient, public_key) in &threema.recipients {
                    let msg = threema.api.encrypt_text_msg(
                        &format!("{} Chicken door status: {} → {}", emoji, from, to),
                        &RecipientKey(public_key.clone()), // TODO: https://github.com/dbrgn/threema-gateway-rs/issues/68, then move into config
                    );
                    if let Err(e) = threema.api.send(&recipient, &msg, false).await {
                        println!(
                            "Error: Failed to notify Threema user {}: {:#}",
                            recipient, e
                        );
                    }
                }
            }
        }
        None => {}
    }
}
