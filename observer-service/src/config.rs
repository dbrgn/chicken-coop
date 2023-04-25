use std::{
    fs::File,
    io::Read,
    path::{Path, PathBuf},
    str::FromStr,
};

use anyhow::Context;
use data_encoding::HEXLOWER_PERMISSIVE;
use serde::{Deserialize, Serialize};
use threema_gateway::SecretKey;

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
                gateway_secret: "your-gateway-secret".to_string(),
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

    /// Gateway secret (from the Gateway website)
    pub gateway_secret: String,

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
    /// Gateway ID (8 characters)
    pub gateway_id: String,

    /// Gateway secret (from the Gateway website)
    pub gateway_secret: String,

    /// Private key (32 bytes as lowercase hex string)
    pub private_key: SecretKey,

    /// List of recipients (Threema IDs)
    pub recipients: Vec<String>,
}

impl TryFrom<RawConfig> for Config {
    type Error = anyhow::Error;

    fn try_from(raw_config: RawConfig) -> Result<Self, Self::Error> {
        // Validate RawThreema config
        let threema = match raw_config.threema {
            Some(raw_threema) => {
                let private_key = SecretKey::from_slice(
                    HEXLOWER_PERMISSIVE
                        .decode(raw_threema.private_key.as_bytes())
                        .context("Could not decode Threema private key hex string")?
                        .as_ref(),
                )
                .ok_or(anyhow::anyhow!("Invalid Threema private key"))?;
                Some(Threema {
                    gateway_id: raw_threema.gateway_id,
                    gateway_secret: raw_threema.gateway_secret,
                    private_key,
                    recipients: raw_threema.recipients,
                })
            }
            None => None,
        };

        Ok(Config {
            serial: raw_config.serial,
            threema,
        })
    }
}
