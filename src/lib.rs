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

#[cfg(test)]
mod tests {
    fn assert_send_sync<T: Send + Sync>() {}

    #[test]
    fn public_types_are_send_and_sync() {
        assert_send_sync::<crate::Error>();
        assert_send_sync::<crate::sector::SectorMode>();
        assert_send_sync::<crate::sector::Subheader>();
        assert_send_sync::<crate::sector::Msf>();
        assert_send_sync::<crate::xa::XaCoding>();
        assert_send_sync::<crate::xa::XaState>();

        #[cfg(feature = "alloc")]
        {
            assert_send_sync::<crate::cue::CueSheet>();
            assert_send_sync::<crate::cue::CueFile>();
            assert_send_sync::<crate::cue::CueTrack>();
            assert_send_sync::<crate::cue::CueIndex>();
            assert_send_sync::<crate::cue::TrackMode>();
            assert_send_sync::<crate::iso::IsoImage>();
            assert_send_sync::<crate::iso::IsoRecord>();
            assert_send_sync::<crate::iso::PrimaryVolumeDescriptor>();
            assert_send_sync::<crate::iso::XaAttributes>();
            assert_send_sync::<crate::repack::IsoRepacker>();
            assert_send_sync::<crate::repack::RepackReport>();
            assert_send_sync::<crate::xa::XaDecoder>();
            assert_send_sync::<crate::xa::XaStream>();
            assert_send_sync::<crate::xa::XaStreamInfo>();
            assert_send_sync::<crate::xa::XaStreamKey>();
        }
    }
}
