//! Envelope v2 (`XSE2`): a fixed 64-byte little-endian header in front of an
//! opaque payload. It is identical on every transport (loopback, shared
//! memory, Zenoh), so audit and decode never depend on the transport.
//!
//! | off | size | field                                   |
//! |-----|------|-----------------------------------------|
//! | 0   | 4    | magic `XSE2`                            |
//! | 4   | 2    | version (2)                             |
//! | 6   | 2    | flags                                   |
//! | 8   | 4    | channel_id                              |
//! | 12  | 2    | origin_id                               |
//! | 14  | 2    | reserved (0)                            |
//! | 16  | 8    | seq (per origin+channel, from 1)        |
//! | 24  | 8    | round                                   |
//! | 32  | 8    | t_produce (Session ns)                  |
//! | 40  | 8    | t_tx (Session ns)                       |
//! | 48  | 4    | clock_bound_ns (sender, saturating)     |
//! | 52  | 4    | payload_len                             |
//! | 56  | 4    | crc32c(header with crc=0, payload)      |
//! | 60  | 4    | reserved (0)                            |

use crate::{ChannelId, OriginId};

pub const MAGIC: [u8; 4] = *b"XSE2";
pub const VERSION: u16 = 2;
pub const HEADER_LEN: usize = 64;
pub const MAX_PAYLOAD: usize = 16 * 1024 * 1024;

pub const FLAG_SIM_CLOCK: u16 = 1 << 0;
pub const FLAG_CLOCK_DEGRADED: u16 = 1 << 1;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Header {
    pub flags: u16,
    pub channel: ChannelId,
    pub origin: OriginId,
    pub seq: u64,
    pub round: u64,
    pub t_produce: i64,
    pub t_tx: i64,
    pub clock_bound_ns: u32,
    pub payload_len: u32,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum EnvelopeError {
    Short(usize),
    Magic,
    Version(u16),
    Length { declared: u32, actual: usize },
    TooLarge(usize),
    Crc { declared: u32, actual: u32 },
}

impl std::fmt::Display for EnvelopeError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Short(n) => write!(f, "frame of {n} bytes is shorter than the {HEADER_LEN}-byte header"),
            Self::Magic => write!(f, "bad magic"),
            Self::Version(v) => write!(f, "unsupported envelope version {v}"),
            Self::Length { declared, actual } => write!(f, "payload_len {declared} but {actual} bytes follow"),
            Self::TooLarge(n) => write!(f, "payload of {n} bytes exceeds {MAX_PAYLOAD}"),
            Self::Crc { declared, actual } => write!(f, "crc32c {declared:#010x} != computed {actual:#010x}"),
        }
    }
}

impl std::error::Error for EnvelopeError {}

/// Encode `header` + `payload` into `out` (cleared first). `payload_len` is
/// taken from `payload` and the checksum is computed here.
pub fn encode_into(header: &Header, payload: &[u8], out: &mut Vec<u8>) -> Result<(), EnvelopeError> {
    if payload.len() > MAX_PAYLOAD {
        return Err(EnvelopeError::TooLarge(payload.len()));
    }
    out.clear();
    out.reserve(HEADER_LEN + payload.len());
    out.extend_from_slice(&MAGIC);
    out.extend_from_slice(&VERSION.to_le_bytes());
    out.extend_from_slice(&header.flags.to_le_bytes());
    out.extend_from_slice(&header.channel.to_le_bytes());
    out.extend_from_slice(&header.origin.to_le_bytes());
    out.extend_from_slice(&0u16.to_le_bytes());
    out.extend_from_slice(&header.seq.to_le_bytes());
    out.extend_from_slice(&header.round.to_le_bytes());
    out.extend_from_slice(&header.t_produce.to_le_bytes());
    out.extend_from_slice(&header.t_tx.to_le_bytes());
    out.extend_from_slice(&header.clock_bound_ns.to_le_bytes());
    out.extend_from_slice(&(payload.len() as u32).to_le_bytes());
    out.extend_from_slice(&0u32.to_le_bytes());
    out.extend_from_slice(&0u32.to_le_bytes());
    out.extend_from_slice(payload);
    let crc = crc32c_update(crc32c_update(!0, &out[..HEADER_LEN]), payload) ^ !0;
    out[56..60].copy_from_slice(&crc.to_le_bytes());
    Ok(())
}

pub fn encode(header: &Header, payload: &[u8]) -> Result<Vec<u8>, EnvelopeError> {
    let mut out = Vec::new();
    encode_into(header, payload, &mut out)?;
    Ok(out)
}

/// Decode and verify a frame. It returns the header and a borrowed payload.
pub fn decode(frame: &[u8]) -> Result<(Header, &[u8]), EnvelopeError> {
    if frame.len() < HEADER_LEN {
        return Err(EnvelopeError::Short(frame.len()));
    }
    if frame[0..4] != MAGIC {
        return Err(EnvelopeError::Magic);
    }
    let u16_at = |o: usize| u16::from_le_bytes([frame[o], frame[o + 1]]);
    let u32_at = |o: usize| u32::from_le_bytes(frame[o..o + 4].try_into().unwrap());
    let u64_at = |o: usize| u64::from_le_bytes(frame[o..o + 8].try_into().unwrap());
    let version = u16_at(4);
    if version != VERSION {
        return Err(EnvelopeError::Version(version));
    }
    let payload_len = u32_at(52);
    let payload = &frame[HEADER_LEN..];
    if payload.len() != payload_len as usize {
        return Err(EnvelopeError::Length { declared: payload_len, actual: payload.len() });
    }
    let declared = u32_at(56);
    let mut crc = crc32c_update(!0, &frame[..56]);
    crc = crc32c_update(crc, &[0; 4]);
    crc = crc32c_update(crc, &frame[60..HEADER_LEN]);
    let actual = crc32c_update(crc, payload) ^ !0;
    if declared != actual {
        return Err(EnvelopeError::Crc { declared, actual });
    }
    Ok((
        Header {
            flags: u16_at(6),
            channel: u32_at(8),
            origin: u16_at(12),
            seq: u64_at(16),
            round: u64_at(24),
            t_produce: u64_at(32) as i64,
            t_tx: u64_at(40) as i64,
            clock_bound_ns: u32_at(48),
            payload_len,
        },
        payload,
    ))
}

const fn crc32c_table() -> [u32; 256] {
    let mut table = [0u32; 256];
    let mut i = 0;
    while i < 256 {
        let mut c = i as u32;
        let mut k = 0;
        while k < 8 {
            c = if c & 1 != 0 { 0x82F6_3B78 ^ (c >> 1) } else { c >> 1 };
            k += 1;
        }
        table[i] = c;
        i += 1;
    }
    table
}

static CRC32C: [u32; 256] = crc32c_table();

/// Raw CRC-32C (Castagnoli) register update, with no pre or post inversion.
pub fn crc32c_update(mut crc: u32, data: &[u8]) -> u32 {
    for &b in data {
        crc = CRC32C[((crc ^ b as u32) & 0xff) as usize] ^ (crc >> 8);
    }
    crc
}

/// CRC-32C of `data` (standard form, same as runtime-sync `EnvelopeCodec::crc32c`).
pub fn crc32c(data: &[u8]) -> u32 {
    crc32c_update(!0, data) ^ !0
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample() -> Header {
        Header {
            flags: FLAG_SIM_CLOCK,
            channel: 7,
            origin: 3,
            seq: 42,
            round: 1000,
            t_produce: 1_700_000_000_000_000_000,
            t_tx: 1_700_000_000_000_100_000,
            clock_bound_ns: 1500,
            payload_len: 0,
        }
    }

    #[test]
    fn crc32c_matches_the_standard_check_value() {
        assert_eq!(crc32c(b"123456789"), 0xE306_9283);
    }

    #[test]
    fn round_trips_header_and_payload() {
        let frame = encode(&sample(), b"neighbor plan").unwrap();
        assert_eq!(frame.len(), HEADER_LEN + 13);
        let (header, payload) = decode(&frame).unwrap();
        assert_eq!(header, Header { payload_len: 13, ..sample() });
        assert_eq!(payload, b"neighbor plan");
    }

    #[test]
    fn rejects_every_single_byte_corruption() {
        let frame = encode(&sample(), b"xyz").unwrap();
        for i in 0..frame.len() {
            let mut bad = frame.clone();
            bad[i] ^= 0x01;
            assert!(decode(&bad).is_err(), "flip at byte {i} was accepted");
        }
    }

    #[test]
    fn rejects_short_and_truncated_frames() {
        let frame = encode(&sample(), b"xyz").unwrap();
        assert_eq!(decode(&frame[..10]), Err(EnvelopeError::Short(10)));
        assert!(matches!(decode(&frame[..frame.len() - 1]), Err(EnvelopeError::Length { .. })));
    }
}
