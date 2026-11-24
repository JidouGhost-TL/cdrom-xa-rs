//! CUE sheet parser supporting multi-file and multi-track optical disc images.
//!
//! Parses standard `FILE`, `TRACK`, `INDEX`, `PREGAP`, and `POSTGAP`
//! directives while ignoring metadata directives (`REM`, `CATALOG`, `ISRC`,
//! `TITLE`, `PERFORMER`, `FLAGS`, etc.).
//!
//! # Example
//!
//! ```
//! use cdrom_xa::cue::{CueSheet, TrackMode};
//! use cdrom_xa::sector::SectorMode;
//!
//! let text = "\
//! FILE \"disc.bin\" BINARY
//!   TRACK 01 MODE2/2352
//!     INDEX 01 00:00:00
//!   TRACK 02 AUDIO
//!     INDEX 00 10:00:00
//!     INDEX 01 10:02:00
//! ";
//!
//! let sheet = CueSheet::parse(text)?;
//! let (file, track) = sheet.first_data_track().expect("data track present");
//! assert_eq!(file.filename, "disc.bin");
//! assert_eq!(track.mode, TrackMode::Mode2_2352);
//! assert_eq!(track.mode.sector_mode(), Some(SectorMode::Mode2_2352));
//! # Ok::<(), cdrom_xa::Error>(())
//! ```

use alloc::string::{String, ToString};
use alloc::vec::Vec;

use crate::error::{Error, Result};
use crate::sector::{Msf, SectorMode, RAW_SECTOR_SIZE};

/// Maximum number of tracks allowed in a standard Red/Yellow Book CUE sheet.
pub const MAX_CUE_TRACKS: usize = 99;

/// Track mode declared by a `TRACK <num> <mode>` directive.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TrackMode {
    /// Red Book CD-DA audio track (2352 bytes/sector).
    Audio,
    /// Cooked 2048-byte data track (`MODE1/2048` or `MODE2/2048` in CUE sheets).
    Mode1_2048,
    /// Raw Mode 1 data track (2352 bytes/sector).
    Mode1_2352,
    /// Stripped Mode 2 data track (2336 bytes/sector).
    Mode2_2336,
    /// Raw Mode 2 (CD-ROM XA) data track (2352 bytes/sector).
    Mode2_2352,
}

impl TrackMode {
    /// Returns the corresponding [`SectorMode`] for data tracks, or `None` for audio tracks.
    #[must_use]
    pub const fn sector_mode(self) -> Option<SectorMode> {
        match self {
            TrackMode::Audio => None,
            TrackMode::Mode1_2048 => Some(SectorMode::Cooked2048),
            TrackMode::Mode1_2352 => Some(SectorMode::Mode1_2352),
            TrackMode::Mode2_2336 => Some(SectorMode::Mode2_2336),
            TrackMode::Mode2_2352 => Some(SectorMode::Mode2_2352),
        }
    }

    /// Stored byte size per sector for this track mode.
    #[must_use]
    pub const fn sector_size(self) -> usize {
        match self {
            TrackMode::Audio => RAW_SECTOR_SIZE,
            TrackMode::Mode1_2048 => SectorMode::Cooked2048.stride(),
            TrackMode::Mode1_2352 => SectorMode::Mode1_2352.stride(),
            TrackMode::Mode2_2336 => SectorMode::Mode2_2336.stride(),
            TrackMode::Mode2_2352 => SectorMode::Mode2_2352.stride(),
        }
    }
}

/// One `INDEX <num> <mm:ss:ff>` marker within a [`CueTrack`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CueIndex {
    /// Index number (`00` = pregap, `01` = track start, `02..=99` = sub-indexes).
    pub number: u8,
    /// Position relative to the start of the enclosing `FILE` as an [`Msf`] timestamp.
    pub position: Msf,
}

/// One `TRACK` entry within a [`CueFile`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CueTrack {
    /// Track number (`1..=99`).
    pub number: u8,
    /// Track data or audio mode.
    pub mode: TrackMode,
    /// Ordered list of `INDEX` markers.
    pub indexes: Vec<CueIndex>,
    /// Optional explicit `PREGAP` duration.
    pub pregap: Option<Msf>,
    /// Optional explicit `POSTGAP` duration.
    pub postgap: Option<Msf>,
}

impl CueTrack {
    /// Look up the [`Msf`] position of a specific index number (e.g. `0` or `1`).
    #[must_use]
    pub fn index(&self, number: u8) -> Option<Msf> {
        self.indexes
            .iter()
            .find(|idx| idx.number == number)
            .map(|idx| idx.position)
    }
}

/// One `FILE` section in a CUE sheet and the tracks stored within it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CueFile {
    /// Filename referenced by the `FILE` directive.
    pub filename: String,
    /// Representation type (e.g. `"BINARY"`, `"WAVE"`, `"MOTOROLA"`).
    pub file_type: String,
    /// Tracks belonging to this file.
    pub tracks: Vec<CueTrack>,
}

/// A parsed CUE sheet containing one or more [`CueFile`] entries.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CueSheet {
    /// Files declared in the CUE sheet, in order.
    pub files: Vec<CueFile>,
}

impl CueSheet {
    /// Parse a CUE sheet from UTF-8 text (stripping a leading UTF-8 BOM if present).
    ///
    /// # Errors
    ///
    /// Returns [`Error::InvalidCue`] if any `FILE`, `TRACK`, `INDEX`, `PREGAP`,
    /// or `POSTGAP` directive is malformed, appears out of order, uses an
    /// unsupported track mode, contains an out-of-range track/index/MSF value,
    /// has non-increasing track numbers or timestamps, or exceeds
    /// [`MAX_CUE_TRACKS`].
    pub fn parse(text: &str) -> Result<Self> {
        let text = text.strip_prefix('\u{FEFF}').unwrap_or(text);
        let mut files: Vec<CueFile> = Vec::new();
        let mut total_tracks = 0usize;
        let mut last_track_number: Option<u8> = None;

        for raw_line in text.lines() {
            let line = raw_line.trim();
            if line.is_empty() {
                continue;
            }
            let first = match line.split_whitespace().next() {
                Some(tok) => tok,
                None => continue,
            };

            if first.eq_ignore_ascii_case("FILE") {
                let (filename, file_type) = parse_file_directive(&line[4..])?;
                files.push(CueFile {
                    filename,
                    file_type,
                    tracks: Vec::new(),
                });
                continue;
            }

            let mut tokens = line.split_whitespace();
            let directive = tokens.next().unwrap_or("");
            if directive.eq_ignore_ascii_case("TRACK") {
                let num_str = tokens
                    .next()
                    .ok_or(Error::InvalidCue("TRACK missing track number"))?;
                let number = num_str
                    .parse::<u8>()
                    .ok()
                    .filter(|&n| (1..=99).contains(&n))
                    .ok_or(Error::InvalidCue("TRACK number out of range 1..=99"))?;
                if let Some(prev) = last_track_number {
                    if number <= prev {
                        return Err(Error::InvalidCue(
                            "TRACK numbers must be strictly increasing",
                        ));
                    }
                }
                let mode_str = tokens
                    .next()
                    .ok_or(Error::InvalidCue("TRACK missing mode"))?;
                let mode = parse_track_mode(mode_str)?;
                total_tracks += 1;
                if total_tracks > MAX_CUE_TRACKS {
                    return Err(Error::InvalidCue("CUE sheet exceeds 99 tracks"));
                }
                let file = files
                    .last_mut()
                    .ok_or(Error::InvalidCue("TRACK appears before any FILE directive"))?;
                file.tracks.push(CueTrack {
                    number,
                    mode,
                    indexes: Vec::new(),
                    pregap: None,
                    postgap: None,
                });
                last_track_number = Some(number);
            } else if directive.eq_ignore_ascii_case("INDEX") {
                let track = files
                    .last_mut()
                    .and_then(|f| f.tracks.last_mut())
                    .ok_or(Error::InvalidCue("INDEX appears before TRACK"))?;
                let num_str = tokens
                    .next()
                    .ok_or(Error::InvalidCue("INDEX missing number"))?;
                let number = num_str
                    .parse::<u8>()
                    .ok()
                    .filter(|&n| n <= 99)
                    .ok_or(Error::InvalidCue("INDEX number out of range 0..=99"))?;
                let time_str = tokens
                    .next()
                    .ok_or(Error::InvalidCue("INDEX missing mm:ss:ff timestamp"))?;
                let position = parse_msf(time_str)?;
                if let Some(prev) = track.indexes.last() {
                    if prev.number >= number || prev.position.to_frames() > position.to_frames() {
                        return Err(Error::InvalidCue(
                            "INDEX numbers and timestamps must be non-decreasing",
                        ));
                    }
                }
                track.indexes.push(CueIndex { number, position });
            } else if directive.eq_ignore_ascii_case("PREGAP") {
                let track = files
                    .last_mut()
                    .and_then(|f| f.tracks.last_mut())
                    .ok_or(Error::InvalidCue("PREGAP appears before TRACK"))?;
                let time_str = tokens
                    .next()
                    .ok_or(Error::InvalidCue("PREGAP missing mm:ss:ff duration"))?;
                let duration = parse_msf(time_str)?;
                if track.pregap.replace(duration).is_some() {
                    return Err(Error::InvalidCue("duplicate PREGAP directive on track"));
                }
            } else if directive.eq_ignore_ascii_case("POSTGAP") {
                let track = files
                    .last_mut()
                    .and_then(|f| f.tracks.last_mut())
                    .ok_or(Error::InvalidCue("POSTGAP appears before TRACK"))?;
                let time_str = tokens
                    .next()
                    .ok_or(Error::InvalidCue("POSTGAP missing mm:ss:ff duration"))?;
                let duration = parse_msf(time_str)?;
                if track.postgap.replace(duration).is_some() {
                    return Err(Error::InvalidCue("duplicate POSTGAP directive on track"));
                }
            }
        }

        // Validate that track start positions within each FILE are non-decreasing.
        for file in &files {
            let mut prev_start: Option<u32> = None;
            for track in &file.tracks {
                let start = track
                    .index(0)
                    .or_else(|| track.index(1))
                    .map(Msf::to_frames);
                if let Some(s) = start {
                    if let Some(prev) = prev_start {
                        if s <= prev {
                            return Err(Error::InvalidCue(
                                "track positions within a FILE must be strictly increasing",
                            ));
                        }
                    }
                    prev_start = Some(s);
                }
            }
        }

        Ok(CueSheet { files })
    }

    /// Iterate over all `(file, track)` pairs in order.
    pub fn tracks(&self) -> impl Iterator<Item = (&CueFile, &CueTrack)> {
        self.files
            .iter()
            .flat_map(|file| file.tracks.iter().map(move |track| (file, track)))
    }

    /// Return the first data track (non-audio) and its enclosing [`CueFile`], if any.
    #[must_use]
    pub fn first_data_track(&self) -> Option<(&CueFile, &CueTrack)> {
        self.tracks()
            .find(|(_, track)| track.mode.sector_mode().is_some())
    }
}

fn parse_file_directive(rest: &str) -> Result<(String, String)> {
    let rest = rest.trim_start();
    if let Some(quoted) = rest.strip_prefix('"') {
        let end = quoted
            .find('"')
            .ok_or(Error::InvalidCue("unterminated quote in FILE directive"))?;
        let name = &quoted[..end];
        if name.is_empty() {
            return Err(Error::InvalidCue("empty filename in FILE directive"));
        }
        let file_type = quoted[end + 1..]
            .split_whitespace()
            .next()
            .ok_or(Error::InvalidCue("FILE directive missing file type"))?;
        Ok((name.to_string(), file_type.to_ascii_uppercase()))
    } else {
        let mut fields = rest.split_whitespace();
        let name = fields
            .next()
            .filter(|s| !s.is_empty())
            .ok_or(Error::InvalidCue("FILE directive missing filename"))?;
        let file_type = fields
            .next()
            .ok_or(Error::InvalidCue("FILE directive missing file type"))?;
        Ok((name.to_string(), file_type.to_ascii_uppercase()))
    }
}

fn parse_track_mode(ty: &str) -> Result<TrackMode> {
    if ty.eq_ignore_ascii_case("AUDIO") {
        Ok(TrackMode::Audio)
    } else if ty.eq_ignore_ascii_case("MODE1/2048") || ty.eq_ignore_ascii_case("MODE2/2048") {
        Ok(TrackMode::Mode1_2048)
    } else if ty.eq_ignore_ascii_case("MODE1/2352") {
        Ok(TrackMode::Mode1_2352)
    } else if ty.eq_ignore_ascii_case("MODE2/2336") {
        Ok(TrackMode::Mode2_2336)
    } else if ty.eq_ignore_ascii_case("MODE2/2352") {
        Ok(TrackMode::Mode2_2352)
    } else {
        Err(Error::InvalidCue("unsupported TRACK mode"))
    }
}

fn parse_msf(value: &str) -> Result<Msf> {
    let mut parts = value.split(':');
    let min = parts
        .next()
        .and_then(|s| s.parse::<u8>().ok())
        .filter(|&m| m <= 99)
        .ok_or(Error::InvalidCue("invalid minutes in mm:ss:ff"))?;
    let sec = parts
        .next()
        .and_then(|s| s.parse::<u8>().ok())
        .filter(|&s| s < 60)
        .ok_or(Error::InvalidCue("invalid seconds in mm:ss:ff"))?;
    let frame = parts
        .next()
        .and_then(|s| s.parse::<u8>().ok())
        .filter(|&f| f < 75)
        .ok_or(Error::InvalidCue("invalid frames in mm:ss:ff"))?;
    if parts.next().is_some() {
        return Err(Error::InvalidCue("too many components in mm:ss:ff"));
    }
    Ok(Msf::new(min, sec, frame))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_multi_file_multi_track_cue() {
        let text = "\
REM COMMENT LINE
FILE \"track 01.bin\" BINARY
  TRACK 01 MODE2/2352
    INDEX 01 00:00:00
FILE track02.bin BINARY
  TRACK 02 AUDIO
    PREGAP 00:02:00
    INDEX 01 00:00:00
    POSTGAP 00:01:10
";
        let cue = CueSheet::parse(text).unwrap();
        assert_eq!(cue.files.len(), 2);
        assert_eq!(cue.files[0].filename, "track 01.bin");
        assert_eq!(cue.files[0].file_type, "BINARY");
        assert_eq!(cue.files[0].tracks.len(), 1);
        assert_eq!(cue.files[0].tracks[0].mode, TrackMode::Mode2_2352);
        assert_eq!(cue.files[0].tracks[0].index(1), Some(Msf::new(0, 0, 0)));

        assert_eq!(cue.files[1].filename, "track02.bin");
        assert_eq!(cue.files[1].tracks[0].mode, TrackMode::Audio);
        assert_eq!(cue.files[1].tracks[0].pregap, Some(Msf::new(0, 2, 0)));
        assert_eq!(cue.files[1].tracks[0].postgap, Some(Msf::new(0, 1, 10)));

        let (file, track) = cue.first_data_track().unwrap();
        assert_eq!(file.filename, "track 01.bin");
        assert_eq!(track.number, 1);
        assert_eq!(track.mode.sector_mode(), Some(SectorMode::Mode2_2352));
    }

    #[test]
    fn parses_mixed_single_bin_with_indexes() {
        let text = "\
FILE \"disc.bin\" BINARY
  TRACK 01 MODE1/2352
    INDEX 01 00:00:00
  TRACK 02 AUDIO
    INDEX 00 10:00:00
    INDEX 01 10:02:00
";
        let cue = CueSheet::parse(text).unwrap();
        assert_eq!(cue.files.len(), 1);
        assert_eq!(cue.files[0].tracks.len(), 2);
        assert_eq!(cue.files[0].tracks[1].index(0), Some(Msf::new(10, 0, 0)));
        assert_eq!(cue.files[0].tracks[1].index(1), Some(Msf::new(10, 2, 0)));
        assert_eq!(cue.files[0].tracks[1].mode.sector_size(), 2352);
    }

    #[test]
    fn rejects_invalid_cue_constructs() {
        assert!(CueSheet::parse("TRACK 01 MODE2/2352\n").is_err());
        assert!(CueSheet::parse("FILE \"x.bin\" BINARY\n TRACK 01 UNKNOWN/2448\n").is_err());
        assert!(CueSheet::parse("FILE \"unterminated BINARY\n").is_err());
        assert!(
            CueSheet::parse("FILE x.bin BINARY\n TRACK 01 AUDIO\n INDEX 01 00:60:00\n").is_err()
        );
        assert!(
            CueSheet::parse("FILE x.bin BINARY\n TRACK 01 AUDIO\n INDEX 01 00:00:75\n").is_err()
        );
        assert!(CueSheet::parse("FILE x.bin BINARY\n TRACK 02 AUDIO\n TRACK 01 AUDIO\n").is_err());
    }

    #[test]
    fn parses_utf8_bom_and_crlf_cue_sheet() {
        let text = "\u{FEFF}FILE \"disc.bin\" BINARY\r\n  TRACK 01 MODE2/2352\r\n    INDEX 01 00:00:00\r\n  TRACK 02 AUDIO\r\n    INDEX 00 05:00:00\r\n    INDEX 01 05:02:00\r\n";
        let cue = CueSheet::parse(text).unwrap();
        assert_eq!(cue.files.len(), 1);
        assert_eq!(cue.files[0].filename, "disc.bin");
        assert_eq!(cue.files[0].tracks.len(), 2);
        assert_eq!(cue.files[0].tracks[0].mode, TrackMode::Mode2_2352);
        assert_eq!(cue.files[0].tracks[1].index(0), Some(Msf::new(5, 0, 0)));
        assert_eq!(cue.files[0].tracks[1].index(1), Some(Msf::new(5, 2, 0)));
    }
}
