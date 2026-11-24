//! Error and result types for `cdrom-xa`.

use core::fmt;

/// Convenience result alias using [`Error`].
pub type Result<T> = core::result::Result<T, Error>;

/// Errors returned by CD-ROM / CD-ROM XA operations.
#[derive(Debug)]
#[non_exhaustive]
pub enum Error {
    /// Input ended before the expected number of bytes were available.
    UnexpectedEof,
    /// Sector or buffer length does not match the expected sector size.
    InvalidSectorSize(usize),
    /// Raw 2352-byte sector is missing the 12-byte Yellow Book sync pattern or
    /// has an unexpected mode byte.
    BadSync,
    /// Sector Error Detection Code (CRC-32) mismatch.
    BadEdc {
        /// Freshly computed EDC value.
        expected: u32,
        /// Stored EDC value read from the sector.
        actual: u32,
    },
    /// Sector Reed-Solomon P or Q Error Correction Code mismatch.
    BadEcc,
    /// Malformed or unsupported CUE sheet syntax.
    InvalidCue(&'static str),
    /// Malformed or unsupported ISO-9660 volume or directory structure.
    InvalidIso(&'static str),
    /// Requested file path was not found in the ISO-9660 filesystem.
    FileNotFound,
    /// Malformed or unsupported CD-ROM XA audio stream or sound group.
    InvalidXa(&'static str),
    /// Malformed `RIFF`/`CDXA` container header or payload.
    InvalidRiffCdxa(&'static str),
    /// Arithmetic or addressing overflow (e.g. LBA/MSF or byte offset).
    Overflow,
    /// Underlying I/O error (only available with the `std` feature).
    #[cfg(feature = "std")]
    Io(std::io::Error),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::UnexpectedEof => write!(f, "unexpected end of input"),
            Error::InvalidSectorSize(size) => write!(f, "invalid sector size: {size} bytes"),
            Error::BadSync => write!(f, "invalid raw CD sector sync pattern or mode byte"),
            Error::BadEdc { expected, actual } => {
                write!(
                    f,
                    "sector EDC mismatch: expected 0x{expected:08x}, found 0x{actual:08x}"
                )
            }
            Error::BadEcc => write!(f, "sector ECC P/Q parity mismatch"),
            Error::InvalidCue(msg) => write!(f, "invalid CUE sheet: {msg}"),
            Error::InvalidIso(msg) => write!(f, "invalid ISO-9660 structure: {msg}"),
            Error::FileNotFound => write!(f, "file not found in ISO-9660 volume"),
            Error::InvalidXa(msg) => write!(f, "invalid CD-ROM XA stream: {msg}"),
            Error::InvalidRiffCdxa(msg) => write!(f, "invalid RIFF/CDXA container: {msg}"),
            Error::Overflow => write!(f, "address or length arithmetic overflow"),
            #[cfg(feature = "std")]
            Error::Io(err) => write!(f, "I/O error: {err}"),
        }
    }
}

#[cfg(feature = "std")]
impl std::error::Error for Error {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Error::Io(err) => Some(err),
            _ => None,
        }
    }
}

#[cfg(feature = "std")]
impl From<std::io::Error> for Error {
    fn from(err: std::io::Error) -> Self {
        Error::Io(err)
    }
}
