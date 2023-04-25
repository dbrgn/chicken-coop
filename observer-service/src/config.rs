use std::{
    collections::HashMap,
    fs::File,
    io::Read,
    path::{Path, PathBuf},
    str::FromStr,
};

use anyhow::Context;
use serde::{Deserialize, Serialize};
use threema_gateway::{ApiBuilder, E2eApi, PublicKey};

#[derive(Debug, PartialEq, Deserialize, Serialize)]
pub struct RawConfig {
    pub serial: Serial,
    pub threema: Option<RawThreema>,
}

impl RawConfig {
    /// Parse the config file at the specified path.
    pub fn load(path: &Path) -> anyhow::Result<Self> {
        let mut file =
            File::open(path).context(format!("Failed to open config file at {:?}", path))?;
        let mut contents = String::new();
        file.read_to_string(&mut contents)
            .context("Failed to read config file to string")?;
        Ok(toml::from_str(&contents)?)
    }

    pub fn example() -> Self {
        Self {
            serial: Serial {
                port: PathBuf::from_str("/dev/ttyACM0").unwrap(),
                baudrate: 9600,
            },
            threema: Some(RawThreema {
                gateway_id: "*YOUR_ID".to_string(),
                api_secret: "your-gateway-secret".to_string(),
                private_key: "00112233..CCDDEEFF".to_string(),
                recipients: vec!["AAAAAAAA".to_string(), "BBBBBBBB".to_string()],
            }),
        }
    }
}

#[derive(Debug, PartialEq, Deserialize, Serialize)]
pub struct Serial {
    /// The serial port.
    pub port: PathBuf,

    /// The baud rate.
    pub baudrate: u32,
}

#[derive(Debug, PartialEq, Deserialize, Serialize)]
pub struct RawThreema {
    /// Gateway ID (8 characters)
    pub gateway_id: String,

    /// API secret (from the Gateway website)
    pub api_secret: String,

    /// Private key (32 bytes as lowercase hex string)
    pub private_key: String,

    /// List of recipients (Threema IDs)
    pub recipients: Vec<String>,
}

#[derive(Debug)]
pub struct Config {
    pub serial: Serial,
    pub threema: Option<Threema>,
}

#[derive(Debug)]
pub struct Threema {
    /// E2E API instance
    pub api: E2eApi,

    /// List of recipients (Threema IDs)
    pub recipients: HashMap<String, PublicKey>,
}

impl Config {
    pub async fn from_raw(raw_config: RawConfig) -> anyhow::Result<Self> {
        // Validate RawThreema config
        let threema = if let Some(raw_threema) = raw_config.threema {
            // Create API instance
            let api = ApiBuilder::new(raw_threema.gateway_id, raw_threema.api_secret)
                .with_private_key_str(&raw_threema.private_key)
                .and_then(|builder| builder.into_e2e())
                .context("Failed to create Threema E2eApi instance")?;

            // Fetch public keys for recipients
            let mut recipients = HashMap::new();
            println!(
                "Fetching {} Threema public keys",
                raw_threema.recipients.len()
            );
            for recipient in raw_threema.recipients {
                let pubkey = api.lookup_pubkey(&recipient).await.context(format!(
                    "Could not fetch public key for recipient {}",
                    &recipient
                ))?;
                recipients.insert(recipient, pubkey);
            }

            Some(Threema { api, recipients })
        } else {
            None
        };

        Ok(Config {
            serial: raw_config.serial,
            threema,
        })
    }
}
