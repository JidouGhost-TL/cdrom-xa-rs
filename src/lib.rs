//! Pure-Rust, zero-dependency CD-ROM and CD-ROM XA library.
//!
//! Provides Yellow Book physical sector framing, EDC/ECC generation and
//! verification, CUE sheet parsing, ISO-9660 filesystem traversal with
//! CD-ROM XA System Use attributes, XA-ADPCM audio decoding, and
//! non-destructive Mode 2/2352 disc repacking.
//!
//! # Feature Flags
//!
//! - `std` *(enabled by default)* — enables `std::io` / filesystem helpers and
//!   `std::error::Error`. Implies `alloc`.
//! - `alloc` — enables heap-backed parsers (`cue`, `iso`, `repack`,
//!   `xa::XaDecoder`, `xa::XaStream`) in `no_std` environments.
//! - With `--no-default-features`, the crate builds in bare `no_std` mode with
//!   zero allocations (`error`, `sector`, and `xa` sound-group decoding /
//!   `RIFF`/`CDXA` unwrapping).

#![cfg_attr(not(feature = "std"), no_std)]
#![forbid(unsafe_code)]
#![warn(missing_docs)]

#[cfg(feature = "alloc")]
extern crate alloc;

#[cfg(feature = "alloc")]
pub mod cue;
pub mod error;
#[cfg(feature = "alloc")]
pub mod iso;
#[cfg(feature = "alloc")]
pub mod repack;
pub mod sector;
pub mod xa;

pub use error::{Error, Result};
