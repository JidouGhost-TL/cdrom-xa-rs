//! Pure-Rust, zero-dependency CD-ROM and CD-ROM XA library.
//!
//! Provides Yellow Book physical sector framing, EDC/ECC generation and
//! verification, CUE sheet parsing, ISO-9660 filesystem traversal with
//! CD-ROM XA System Use attributes, XA-ADPCM audio decoding, and
//! non-destructive ISO-9660 / CD-ROM XA disc repacking (supporting
//! `Mode1_2352`, `Mode2_2352`, `Mode2_2336`, and `Cooked2048` layouts).
//!
//! # Quick Example
//!
//! Building, verifying, and regenerating a raw Mode 2 Form 1 sector works in
//! bare `no_std` mode with zero allocations:
//!
//! ```
//! use cdrom_xa::sector::{
//!     build_mode2_form1_sector, regenerate_mode2_form1_edc_ecc, verify_mode2_form1_edc_ecc,
//!     Subheader, COOKED_SECTOR_SIZE, SUBMODE_DATA, SUBMODE_EOF, SUBMODE_EOR,
//! };
//!
//! let user_data = [0x42u8; COOKED_SECTOR_SIZE];
//! let subheader = Subheader::new(0, 0, SUBMODE_DATA | SUBMODE_EOR | SUBMODE_EOF, 0);
//!
//! let mut raw = build_mode2_form1_sector(16, &user_data, subheader)?;
//! verify_mode2_form1_edc_ecc(&raw)?;
//!
//! // Corrupt user data, observe verification failure, then regenerate EDC/ECC:
//! raw[24] ^= 0xFF;
//! assert!(verify_mode2_form1_edc_ecc(&raw).is_err());
//! regenerate_mode2_form1_edc_ecc(&mut raw);
//! verify_mode2_form1_edc_ecc(&raw)?;
//! # Ok::<(), cdrom_xa::Error>(())
//! ```
//!
//! # Feature Flags
//!
//! - `std` *(enabled by default)* — enables `std::io` / filesystem helpers
//!   (`iso::IsoImage::open`, `iso::IsoImage::from_reader`,
//!   `iso::IsoImage::from_reader_with_mode`, `repack::IsoRepacker::open`,
//!   `repack::IsoRepacker::write_to`), `Error::Io`, and `std::error::Error`.
//!   Implies `alloc`.
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
