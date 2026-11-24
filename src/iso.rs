//! ISO-9660 filesystem reader with CD-ROM XA System Use extensions.
//!
//! Supports reading from in-memory byte buffers (`no_std` + `alloc`) as well as
//! `Read + Seek` streams and `.cue`/`.bin`/`.iso` files on disk (`std`),
//! automatically detecting `Mode1_2352`, `Mode2_2352`, `Mode2_2336`, and
//! `Cooked2048` sector layouts.

use alloc::collections::BTreeSet;
use alloc::string::{String, ToString};
#[cfg(any(feature = "std", test))]
use alloc::vec;
use alloc::vec::Vec;

#[cfg(feature = "std")]
use alloc::boxed::Box;
#[cfg(feature = "std")]
use std::fs::File;
#[cfg(feature = "std")]
use std::io::{Read, Seek, SeekFrom};
#[cfg(feature = "std")]
use std::path::Path;
#[cfg(feature = "std")]
use std::sync::Mutex;

#[cfg(feature = "std")]
use crate::cue::CueSheet;
use crate::error::{Error, Result};
use crate::sector::{detect_sector_mode, extract_user_data, SectorMode, COOKED_SECTOR_SIZE};
#[cfg(any(feature = "std", test))]
use crate::sector::{RAW_SECTOR_SIZE, SYNC_PATTERN};

/// Logical Block Address of the ISO-9660 Primary Volume Descriptor.
pub const PVD_LBA: u32 = 16;

const FLAG_DIRECTORY: u8 = 0x02;
const PVD_ROOT_RECORD_OFFSET: usize = 156;
const DIR_RECORD_FIXED_LEN: usize = 33;
const MAX_WALK_DEPTH: usize = 64;

/// CD-ROM XA System Use attributes attached to an ISO-9660 directory record.
///
/// Defined by the CD-ROM XA System Description (14-byte record carrying the
/// `"XA"` signature at bytes `6..8`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct XaAttributes {
    /// Owner group ID (big-endian on disc).
    pub owner_group_id: u16,
    /// Owner user ID (big-endian on disc).
    pub owner_user_id: u16,
    /// XA attribute bitfield (big-endian on disc).
    pub attributes: u16,
    /// File number associated with interleaved XA sectors.
    pub file_number: u8,
}

impl XaAttributes {
    /// Parse a 14-byte CD-ROM XA System Use extension if the `"XA"` signature is present.
    pub fn parse(system_use: &[u8]) -> Option<Self> {
        if system_use.len() < 14 || &system_use[6..8] != b"XA" {
            return None;
        }
        Some(Self {
            owner_group_id: u16::from_be_bytes([system_use[0], system_use[1]]),
            owner_user_id: u16::from_be_bytes([system_use[2], system_use[3]]),
            attributes: u16::from_be_bytes([system_use[4], system_use[5]]),
            file_number: system_use[8],
        })
    }

    /// Returns `true` if the Mode 2 Form 1 flag (bit 11, `0x0800`) is set.
    pub const fn is_mode2_form1(self) -> bool {
        self.attributes & 0x0800 != 0
    }

    /// Returns `true` if the Mode 2 Form 2 flag (bit 12, `0x1000`) is set.
    pub const fn is_mode2_form2(self) -> bool {
        self.attributes & 0x1000 != 0
    }

    /// Returns `true` if the Interleaved flag (bit 13, `0x2000`) is set.
    pub const fn is_interleaved(self) -> bool {
        self.attributes & 0x2000 != 0
    }

    /// Returns `true` if the CD-DA audio flag (bit 14, `0x4000`) is set.
    pub const fn is_cdda(self) -> bool {
        self.attributes & 0x4000 != 0
    }

    /// Returns `true` if the Directory flag (bit 15, `0x8000`) is set.
    pub const fn is_directory(self) -> bool {
        self.attributes & 0x8000 != 0
    }
}

/// Parsed ISO-9660 Primary Volume Descriptor (PVD) at LBA 16.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PrimaryVolumeDescriptor {
    /// System identifier string (trimmed of trailing spaces/NULs).
    pub system_id: String,
    /// Volume identifier string (trimmed of trailing spaces/NULs).
    pub volume_id: String,
    /// Total number of logical blocks in the volume.
    pub volume_space_size: u32,
    /// Logical block size in bytes (standard ISO-9660 uses `2048`).
    pub logical_block_size: u16,
    /// Byte size of the path table.
    pub path_table_size: u32,
    /// LBA of the little-endian (L-type) path table.
    pub path_table_lba: u32,
    /// Extent LBA of the root directory record.
    pub root_lba: u32,
    /// Byte length of the root directory extent.
    pub root_size: u32,
}

impl PrimaryVolumeDescriptor {
    /// Parse a Primary Volume Descriptor from a 2048-byte cooked block.
    pub fn parse(block: &[u8]) -> Result<Self> {
        if block.len() < COOKED_SECTOR_SIZE {
            return Err(Error::UnexpectedEof);
        }
        if block[0] != 0x01 || &block[1..6] != b"CD001" {
            return Err(Error::InvalidIso("missing Primary Volume Descriptor magic"));
        }
        let system_id = trim_iso_str(&block[8..40]);
        let volume_id = trim_iso_str(&block[40..72]);
        let volume_space_size = read_u32_le(block, 80)?;
        let logical_block_size = read_u16_le(block, 128)?;
        let path_table_size = read_u32_le(block, 132)?;
        let path_table_lba = read_u32_le(block, 140)?;
        let root_raw = parse_dir_entry(&block[PVD_ROOT_RECORD_OFFSET..])?;
        Ok(Self {
            system_id,
            volume_id,
            volume_space_size,
            logical_block_size,
            path_table_size,
            path_table_lba,
            root_lba: root_raw.lba,
            root_size: root_raw.size,
        })
    }
}

/// One file or directory entry discovered in an ISO-9660 filesystem.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct IsoRecord {
    /// Full `/`-prefixed normalized path (e.g. `"/DATA/ASSET.BIN"`).
    pub path: String,
    /// Decoded entry name with any trailing `;version` suffix stripped.
    pub name: String,
    /// Starting Logical Block Address of the file or directory extent.
    pub lba: u32,
    /// Logical byte length of the file or directory extent.
    pub size: u32,
    /// `true` if this record represents a directory.
    pub is_dir: bool,
    /// Optional CD-ROM XA System Use attributes attached to this record.
    pub xa_attributes: Option<XaAttributes>,
}

#[derive(Debug, Clone)]
pub(crate) struct RawDirEntry {
    pub name: String,
    pub raw_identifier: Vec<u8>,
    pub lba: u32,
    pub size: u32,
    pub flags: u8,
    pub xa_attributes: Option<XaAttributes>,
}

impl RawDirEntry {
    pub(crate) fn is_dir(&self) -> bool {
        self.flags & FLAG_DIRECTORY != 0
    }

    pub(crate) fn is_special(&self) -> bool {
        self.raw_identifier == [0x00] || self.raw_identifier == [0x01]
    }
}

#[cfg(feature = "std")]
trait ReadSeek: Read + Seek + Send {}
#[cfg(feature = "std")]
impl<T: Read + Seek + Send> ReadSeek for T {}

enum ImageSource {
    Bytes(Vec<u8>),
    #[cfg(feature = "std")]
    Reader {
        inner: Mutex<Box<dyn ReadSeek>>,
        base_offset: u64,
        byte_len: u64,
    },
}

/// Mounted ISO-9660 disc image over an in-memory buffer or seekable reader.
pub struct IsoImage {
    source: ImageSource,
    mode: SectorMode,
    sector_count: u32,
    pvd: PrimaryVolumeDescriptor,
}

impl IsoImage {
    /// Mount an ISO-9660 image from a byte slice, auto-detecting the [`SectorMode`].
    pub fn from_bytes(data: &[u8]) -> Result<Self> {
        Self::from_vec(data.to_vec())
    }

    /// Mount an ISO-9660 image from an owned `Vec<u8>`, auto-detecting the [`SectorMode`].
    pub fn from_vec(data: Vec<u8>) -> Result<Self> {
        let mode = detect_sector_mode(&data)
            .ok_or(Error::InvalidIso("unable to detect CD-ROM sector layout"))?;
        Self::from_vec_with_mode(data, mode)
    }

    /// Mount an ISO-9660 image from a byte slice using an explicit [`SectorMode`].
    pub fn from_bytes_with_mode(data: &[u8], mode: SectorMode) -> Result<Self> {
        Self::from_vec_with_mode(data.to_vec(), mode)
    }

    /// Mount an ISO-9660 image from an owned `Vec<u8>` using an explicit [`SectorMode`].
    pub fn from_vec_with_mode(data: Vec<u8>, mode: SectorMode) -> Result<Self> {
        let stride = mode.stride();
        let sectors_usize = data.len() / stride;
        let sector_count = u32::try_from(sectors_usize).map_err(|_| Error::Overflow)?;
        let source = ImageSource::Bytes(data);
        Self::finish_mount(source, mode, sector_count)
    }

    /// Mount an ISO-9660 image from any `Read + Seek` stream, auto-detecting the [`SectorMode`].
    #[cfg(feature = "std")]
    pub fn from_reader<R: Read + Seek + Send + 'static>(mut reader: R) -> Result<Self> {
        let byte_len = reader.seek(SeekFrom::End(0))?;
        let mode = detect_sector_mode_reader(&mut reader, 0, byte_len)
            .ok_or(Error::InvalidIso("unable to detect CD-ROM sector layout"))?;
        Self::from_reader_range(reader, 0, byte_len, mode)
    }

    /// Mount an ISO-9660 image from any `Read + Seek` stream with an explicit [`SectorMode`].
    #[cfg(feature = "std")]
    pub fn from_reader_with_mode<R: Read + Seek + Send + 'static>(
        mut reader: R,
        mode: SectorMode,
    ) -> Result<Self> {
        let byte_len = reader.seek(SeekFrom::End(0))?;
        Self::from_reader_range(reader, 0, byte_len, mode)
    }

    #[cfg(feature = "std")]
    fn from_reader_range<R: Read + Seek + Send + 'static>(
        reader: R,
        base_offset: u64,
        byte_len: u64,
        mode: SectorMode,
    ) -> Result<Self> {
        let stride = mode.stride() as u64;
        let sector_count = u32::try_from(byte_len / stride).map_err(|_| Error::Overflow)?;
        let source = ImageSource::Reader {
            inner: Mutex::new(Box::new(reader)),
            base_offset,
            byte_len,
        };
        Self::finish_mount(source, mode, sector_count)
    }

    /// Open a `.cue`, `.bin`, or `.iso` image file from disk.
    ///
    /// When given a `.cue` sheet, parses the sheet and opens the first data
    /// track's backing `.bin` file with its declared [`SectorMode`] and track
    /// offset. Otherwise opens the image directly and auto-detects its layout.
    #[cfg(feature = "std")]
    pub fn open(path: impl AsRef<Path>) -> Result<Self> {
        let path = path.as_ref();
        let is_cue = path
            .extension()
            .and_then(|e| e.to_str())
            .is_some_and(|ext| ext.eq_ignore_ascii_case("cue"));
        if is_cue {
            let text = std::fs::read_to_string(path)?;
            let cue = CueSheet::parse(&text)?;
            let (cue_file, track) = cue
                .first_data_track()
                .ok_or(Error::InvalidCue("CUE sheet contains no data track"))?;
            let mode = track
                .mode
                .sector_mode()
                .ok_or(Error::InvalidCue("data track has no sector mode"))?;
            let bin_path = path
                .parent()
                .unwrap_or_else(|| Path::new("."))
                .join(&cue_file.filename);
            let mut file = File::open(&bin_path)?;
            let total_len = file.seek(SeekFrom::End(0))?;
            let start_frame = track
                .index(1)
                .or_else(|| track.index(0))
                .map(|m| m.to_frames() as u64)
                .unwrap_or(0);
            let base_offset = start_frame
                .checked_mul(mode.stride() as u64)
                .ok_or(Error::Overflow)?;
            if base_offset > total_len {
                return Err(Error::UnexpectedEof);
            }
            return Self::from_reader_range(file, base_offset, total_len - base_offset, mode);
        }

        let file = File::open(path)?;
        Self::from_reader(file)
    }

    fn finish_mount(source: ImageSource, mode: SectorMode, sector_count: u32) -> Result<Self> {
        let pvd_raw = read_raw_from_source(&source, mode, sector_count, PVD_LBA)?;
        let pvd_user = extract_user_data(&pvd_raw, mode)?;
        let pvd = PrimaryVolumeDescriptor::parse(pvd_user)?;
        Ok(Self {
            source,
            mode,
            sector_count,
            pvd,
        })
    }

    /// Detected or configured physical [`SectorMode`].
    pub const fn mode(&self) -> SectorMode {
        self.mode
    }

    /// Total number of complete sectors available in the image.
    pub const fn sector_count(&self) -> u32 {
        self.sector_count
    }

    /// Total stored byte length of the underlying image (or track slice).
    pub fn stored_len(&self) -> u64 {
        match &self.source {
            ImageSource::Bytes(buf) => buf.len() as u64,
            #[cfg(feature = "std")]
            ImageSource::Reader { byte_len, .. } => *byte_len,
        }
    }

    /// Borrow the parsed [`PrimaryVolumeDescriptor`].
    pub const fn pvd(&self) -> &PrimaryVolumeDescriptor {
        &self.pvd
    }

    /// Read one complete stored sector at `lba` in its native [`SectorMode`] stride.
    pub fn read_sector_raw(&self, lba: u32) -> Result<Vec<u8>> {
        read_raw_from_source(&self.source, self.mode, self.sector_count, lba)
    }

    /// Read the 2048-byte cooked user data of logical block `lba`.
    pub fn read_sector_cooked(&self, lba: u32) -> Result<[u8; COOKED_SECTOR_SIZE]> {
        let raw = self.read_sector_raw(lba)?;
        let user = extract_user_data(&raw, self.mode)?;
        if user.len() < COOKED_SECTOR_SIZE {
            return Err(Error::UnexpectedEof);
        }
        let mut block = [0u8; COOKED_SECTOR_SIZE];
        block.copy_from_slice(&user[..COOKED_SECTOR_SIZE]);
        Ok(block)
    }

    /// Walk the entire ISO-9660 directory tree depth-first and return every file
    /// and directory record (excluding `.` and `..`).
    pub fn records(&self) -> Result<Vec<IsoRecord>> {
        let mut out = Vec::new();
        let mut visited = BTreeSet::new();
        visited.insert(self.pvd.root_lba);
        self.walk_dir(
            self.pvd.root_lba,
            self.pvd.root_size,
            "",
            0,
            &mut visited,
            &mut out,
        )?;
        Ok(out)
    }

    /// Resolve a slash-separated path (e.g. `"/DATA/ASSET.BIN"` or `"README.TXT"`)
    /// case-insensitively to its [`IsoRecord`].
    pub fn find(&self, path: &str) -> Result<Option<IsoRecord>> {
        let components: Vec<&str> = path.split(['/', '\\']).filter(|c| !c.is_empty()).collect();
        if components.is_empty() {
            return Ok(Some(IsoRecord {
                path: "/".to_string(),
                name: "/".to_string(),
                lba: self.pvd.root_lba,
                size: self.pvd.root_size,
                is_dir: true,
                xa_attributes: None,
            }));
        }

        let mut cur_lba = self.pvd.root_lba;
        let mut cur_size = self.pvd.root_size;
        let mut cur_is_dir = true;
        let mut visited = BTreeSet::new();
        visited.insert(cur_lba);
        let mut full_path = String::new();

        for (idx, component) in components.iter().enumerate() {
            if !cur_is_dir {
                return Ok(None);
            }
            let entries = self.read_dir_entries(cur_lba, cur_size)?;
            let have_exact = entries
                .iter()
                .any(|e| e.raw_identifier.eq_ignore_ascii_case(component.as_bytes()));
            let found = entries.into_iter().find(|e| {
                if have_exact {
                    e.raw_identifier.eq_ignore_ascii_case(component.as_bytes())
                } else {
                    e.name.eq_ignore_ascii_case(component)
                }
            });
            let entry = match found {
                Some(e) => e,
                None => return Ok(None),
            };
            full_path.push('/');
            full_path.push_str(&entry.name);
            if idx + 1 == components.len() {
                let is_dir = entry.is_dir();
                return Ok(Some(IsoRecord {
                    path: full_path,
                    name: entry.name,
                    lba: entry.lba,
                    size: entry.size,
                    is_dir,
                    xa_attributes: entry.xa_attributes,
                }));
            }
            if !entry.is_dir() {
                return Ok(None);
            }
            if !visited.insert(entry.lba) {
                return Err(Error::InvalidIso("cyclic directory reference"));
            }
            cur_lba = entry.lba;
            cur_size = entry.size;
            cur_is_dir = true;
        }
        Ok(None)
    }

    /// Read a file's logical cooked contents (2048 bytes/sector, truncated to `record.size`).
    pub fn read_file(&self, record: &IsoRecord) -> Result<Vec<u8>> {
        if record.is_dir {
            return Err(Error::InvalidIso("cannot read_file on a directory record"));
        }
        let sectors = sector_span(record.size);
        self.check_extent(record.lba, sectors)?;
        let mut out = Vec::with_capacity(record.size as usize);
        let mut remaining = record.size as usize;
        for i in 0..sectors {
            let block = self.read_sector_cooked(record.lba + i)?;
            let take = remaining.min(COOKED_SECTOR_SIZE);
            out.extend_from_slice(&block[..take]);
            remaining -= take;
        }
        Ok(out)
    }

    /// Read all raw stored sectors spanned by `record` (preserving full 2352-byte
    /// or 2336-byte framing, subheaders, and Form 2 payloads).
    pub fn read_file_raw_sectors(&self, record: &IsoRecord) -> Result<Vec<u8>> {
        if record.is_dir {
            return Err(Error::InvalidIso(
                "cannot read_file_raw_sectors on a directory record",
            ));
        }
        let sectors = sector_span(record.size);
        self.check_extent(record.lba, sectors)?;
        let total_bytes = (sectors as usize)
            .checked_mul(self.mode.stride())
            .ok_or(Error::Overflow)?;
        let mut out = Vec::with_capacity(total_bytes);
        for i in 0..sectors {
            let raw = self.read_sector_raw(record.lba + i)?;
            out.extend_from_slice(&raw);
        }
        Ok(out)
    }

    fn check_extent(&self, lba: u32, count: u32) -> Result<()> {
        let end = lba.checked_add(count).ok_or(Error::Overflow)?;
        if end > self.sector_count {
            return Err(Error::UnexpectedEof);
        }
        Ok(())
    }

    pub(crate) fn read_dir_entries(&self, lba: u32, size: u32) -> Result<Vec<RawDirEntry>> {
        let sectors = sector_span(size);
        self.check_extent(lba, sectors)?;
        let mut out = Vec::new();
        for s in 0..sectors {
            let block = self.read_sector_cooked(lba + s)?;
            let mut off = 0usize;
            while off < COOKED_SECTOR_SIZE {
                let len = block[off] as usize;
                if len == 0 {
                    break;
                }
                if len < DIR_RECORD_FIXED_LEN || off + len > COOKED_SECTOR_SIZE {
                    return Err(Error::InvalidIso("malformed directory record length"));
                }
                out.push(parse_dir_entry(&block[off..off + len])?);
                off += len;
            }
        }
        Ok(out)
    }

    fn walk_dir(
        &self,
        lba: u32,
        size: u32,
        prefix: &str,
        depth: usize,
        visited: &mut BTreeSet<u32>,
        out: &mut Vec<IsoRecord>,
    ) -> Result<()> {
        if depth > MAX_WALK_DEPTH {
            return Err(Error::InvalidIso("directory nesting exceeds depth limit"));
        }
        for entry in self.read_dir_entries(lba, size)? {
            if entry.is_special() {
                continue;
            }
            let mut path = String::with_capacity(prefix.len() + 1 + entry.name.len());
            path.push_str(prefix);
            path.push('/');
            path.push_str(&entry.name);
            let is_dir = entry.is_dir();
            let child_lba = entry.lba;
            let child_size = entry.size;
            out.push(IsoRecord {
                path: path.clone(),
                name: entry.name,
                lba: child_lba,
                size: child_size,
                is_dir,
                xa_attributes: entry.xa_attributes,
            });
            if is_dir && visited.insert(child_lba) {
                self.walk_dir(child_lba, child_size, &path, depth + 1, visited, out)?;
            }
        }
        Ok(())
    }
}

pub(crate) fn sector_span(byte_len: u32) -> u32 {
    if byte_len == 0 {
        0
    } else {
        (byte_len - 1) / (COOKED_SECTOR_SIZE as u32) + 1
    }
}

pub(crate) fn parse_dir_entry(bytes: &[u8]) -> Result<RawDirEntry> {
    let len = *bytes.first().ok_or(Error::UnexpectedEof)? as usize;
    if len < DIR_RECORD_FIXED_LEN || bytes.len() < len {
        return Err(Error::InvalidIso("directory record too short"));
    }
    let rec = &bytes[..len];
    let lba = read_u32_le(rec, 2)?;
    let size = read_u32_le(rec, 10)?;
    let flags = rec[25];
    let name_len = rec[32] as usize;
    let name_end = DIR_RECORD_FIXED_LEN
        .checked_add(name_len)
        .ok_or(Error::Overflow)?;
    if name_end > len {
        return Err(Error::InvalidIso(
            "directory record name extends past record end",
        ));
    }
    let raw_id = &rec[DIR_RECORD_FIXED_LEN..name_end];
    let name = decode_iso_name(raw_id);

    // System Use area starts after the file identifier plus 1 pad byte if name_len is even.
    let sys_use_start = name_end + usize::from(name_len % 2 == 0);
    let xa_attributes = rec.get(sys_use_start..len).and_then(XaAttributes::parse);

    Ok(RawDirEntry {
        name,
        raw_identifier: raw_id.to_vec(),
        lba,
        size,
        flags,
        xa_attributes,
    })
}

fn decode_iso_name(raw: &[u8]) -> String {
    if raw == [0x00] {
        return ".".to_string();
    }
    if raw == [0x01] {
        return "..".to_string();
    }
    let stripped = strip_version_suffix(raw);
    String::from_utf8_lossy(stripped).into_owned()
}

fn strip_version_suffix(id: &[u8]) -> &[u8] {
    if let Some(pos) = id.iter().rposition(|&b| b == b';') {
        let ver = &id[pos + 1..];
        if !ver.is_empty() && ver.iter().all(u8::is_ascii_digit) {
            return &id[..pos];
        }
    }
    id
}

fn trim_iso_str(field: &[u8]) -> String {
    String::from_utf8_lossy(field)
        .trim_end_matches([' ', '\0'])
        .to_string()
}

fn read_u16_le(bytes: &[u8], off: usize) -> Result<u16> {
    let slice = bytes.get(off..off + 2).ok_or(Error::UnexpectedEof)?;
    Ok(u16::from_le_bytes([slice[0], slice[1]]))
}

fn read_u32_le(bytes: &[u8], off: usize) -> Result<u32> {
    let slice = bytes.get(off..off + 4).ok_or(Error::UnexpectedEof)?;
    Ok(u32::from_le_bytes([slice[0], slice[1], slice[2], slice[3]]))
}

fn read_raw_from_source(
    source: &ImageSource,
    mode: SectorMode,
    sector_count: u32,
    lba: u32,
) -> Result<Vec<u8>> {
    if lba >= sector_count {
        return Err(Error::UnexpectedEof);
    }
    let stride = mode.stride();
    match source {
        ImageSource::Bytes(buf) => {
            let start = (lba as usize).checked_mul(stride).ok_or(Error::Overflow)?;
            let end = start.checked_add(stride).ok_or(Error::Overflow)?;
            let slice = buf.get(start..end).ok_or(Error::UnexpectedEof)?;
            Ok(slice.to_vec())
        }
        #[cfg(feature = "std")]
        ImageSource::Reader {
            inner,
            base_offset,
            byte_len,
        } => {
            let rel = (lba as u64)
                .checked_mul(stride as u64)
                .ok_or(Error::Overflow)?;
            if rel + stride as u64 > *byte_len {
                return Err(Error::UnexpectedEof);
            }
            let abs = base_offset.checked_add(rel).ok_or(Error::Overflow)?;
            let mut guard = inner
                .lock()
                .map_err(|_| Error::InvalidIso("reader lock poisoned"))?;
            guard.seek(SeekFrom::Start(abs))?;
            let mut buf = vec![0u8; stride];
            guard.read_exact(&mut buf)?;
            Ok(buf)
        }
    }
}

#[cfg(feature = "std")]
fn detect_sector_mode_reader<R: Read + Seek>(
    reader: &mut R,
    base_offset: u64,
    byte_len: u64,
) -> Option<SectorMode> {
    const PVD_MAGIC: &[u8; 6] = b"\x01CD001";
    const CANDIDATES: [SectorMode; 4] = [
        SectorMode::Mode2_2352,
        SectorMode::Mode1_2352,
        SectorMode::Mode2_2336,
        SectorMode::Cooked2048,
    ];

    for mode in CANDIDATES {
        let stride = mode.stride() as u64;
        let sector_off = 16u64.checked_mul(stride)?;
        if sector_off + stride > byte_len {
            continue;
        }
        if reader
            .seek(SeekFrom::Start(base_offset + sector_off))
            .is_err()
        {
            continue;
        }
        let mut sec = vec![0u8; mode.stride()];
        if reader.read_exact(&mut sec).is_err() {
            continue;
        }
        let user_off = mode.user_offset();
        if sec.get(user_off..user_off + 6) == Some(PVD_MAGIC.as_slice()) {
            match mode {
                SectorMode::Mode1_2352 => {
                    if sec[..12] == SYNC_PATTERN && sec[15] == 1 {
                        return Some(mode);
                    }
                }
                SectorMode::Mode2_2352 => {
                    if sec[..12] == SYNC_PATTERN && sec[15] == 2 {
                        return Some(mode);
                    }
                }
                SectorMode::Mode2_2336 | SectorMode::Cooked2048 => return Some(mode),
            }
        }
    }

    // Fallback: inspect first raw sector sync/mode.
    if byte_len >= RAW_SECTOR_SIZE as u64
        && byte_len % (RAW_SECTOR_SIZE as u64) == 0
        && reader.seek(SeekFrom::Start(base_offset)).is_ok()
    {
        let mut hdr = [0u8; 16];
        if reader.read_exact(&mut hdr).is_ok() && hdr[..12] == SYNC_PATTERN {
            return match hdr[15] {
                1 => Some(SectorMode::Mode1_2352),
                2 => Some(SectorMode::Mode2_2352),
                _ => None,
            };
        }
    }

    None
}

#[cfg(test)]
pub(crate) mod test_helpers {
    use super::*;
    use crate::sector::{
        build_mode1_sector, build_mode2_2336_sector, build_mode2_form1_sector, Subheader,
        SUBMODE_DATA, SUBMODE_EOF, SUBMODE_EOR, SUBMODE_FORM2,
    };

    pub(crate) fn make_dir_record(
        name: &[u8],
        lba: u32,
        len: u32,
        is_dir: bool,
        xa_attrs: Option<u16>,
    ) -> Vec<u8> {
        let pad = usize::from(name.len() % 2 == 0);
        let xa_len = if xa_attrs.is_some() { 14 } else { 0 };
        let record_len = DIR_RECORD_FIXED_LEN + name.len() + pad + xa_len;
        let mut r = vec![0u8; record_len];
        r[0] = record_len as u8;
        r[2..6].copy_from_slice(&lba.to_le_bytes());
        r[6..10].copy_from_slice(&lba.to_be_bytes());
        r[10..14].copy_from_slice(&len.to_le_bytes());
        r[14..18].copy_from_slice(&len.to_be_bytes());
        r[25] = if is_dir { FLAG_DIRECTORY } else { 0 };
        r[32] = name.len() as u8;
        r[33..33 + name.len()].copy_from_slice(name);
        if let Some(attrs) = xa_attrs {
            let su = 33 + name.len() + pad;
            r[su + 4..su + 6].copy_from_slice(&attrs.to_be_bytes());
            r[su + 6..su + 8].copy_from_slice(b"XA");
            r[su + 8] = 1;
        }
        r
    }

    pub(crate) fn build_synthetic_blocks() -> Vec<[u8; COOKED_SECTOR_SIZE]> {
        const TOTAL: u32 = 24;
        let mut blocks = vec![[0u8; COOKED_SECTOR_SIZE]; TOTAL as usize];

        // LBA 16: PVD
        let pvd = &mut blocks[16];
        pvd[0] = 0x01;
        pvd[1..6].copy_from_slice(b"CD001");
        pvd[6] = 0x01;
        pvd[8..18].copy_from_slice(b"GENERIC_CD");
        for b in &mut pvd[18..40] {
            *b = b' ';
        }
        pvd[40..48].copy_from_slice(b"TEST_VOL");
        for b in &mut pvd[48..72] {
            *b = b' ';
        }
        pvd[80..84].copy_from_slice(&TOTAL.to_le_bytes());
        pvd[84..88].copy_from_slice(&TOTAL.to_be_bytes());
        pvd[128..130].copy_from_slice(&(COOKED_SECTOR_SIZE as u16).to_le_bytes());
        pvd[130..132].copy_from_slice(&(COOKED_SECTOR_SIZE as u16).to_be_bytes());
        let root_rec = make_dir_record(&[0x00], 18, COOKED_SECTOR_SIZE as u32, true, None);
        pvd[156..156 + root_rec.len()].copy_from_slice(&root_rec);

        // LBA 17: Terminator
        blocks[17][0] = 0xFF;
        blocks[17][1..6].copy_from_slice(b"CD001");
        blocks[17][6] = 0x01;

        // LBA 18: Root directory
        let mut cur = 0;
        for rec in [
            make_dir_record(&[0x00], 18, COOKED_SECTOR_SIZE as u32, true, None),
            make_dir_record(&[0x01], 18, COOKED_SECTOR_SIZE as u32, true, None),
            make_dir_record(b"DATA", 19, COOKED_SECTOR_SIZE as u32, true, Some(0x8800)),
            make_dir_record(b"README.TXT;1", 20, 32, false, Some(0x0800)),
        ] {
            blocks[18][cur..cur + rec.len()].copy_from_slice(&rec);
            cur += rec.len();
        }

        // LBA 19: /DATA directory
        let mut cur = 0;
        for rec in [
            make_dir_record(&[0x00], 19, COOKED_SECTOR_SIZE as u32, true, None),
            make_dir_record(&[0x01], 18, COOKED_SECTOR_SIZE as u32, true, None),
            make_dir_record(b"ASSET.BIN;1", 21, 512, false, Some(0x0800)),
        ] {
            blocks[19][cur..cur + rec.len()].copy_from_slice(&rec);
            cur += rec.len();
        }

        // LBA 20: /README.TXT
        let readme = b"Hello from generic ISO-9660 disc";
        blocks[20][..readme.len()].copy_from_slice(readme);

        // LBA 21: /DATA/ASSET.BIN
        for (i, b) in blocks[21][..512].iter_mut().enumerate() {
            *b = (i & 0xFF) as u8;
        }

        blocks
    }

    pub(crate) fn build_synthetic_image(mode: SectorMode) -> Vec<u8> {
        let blocks = build_synthetic_blocks();
        let mut out = Vec::with_capacity(blocks.len() * mode.stride());
        let sub = Subheader::new(0, 0, SUBMODE_DATA | SUBMODE_EOR | SUBMODE_EOF, 0);
        for (lba, block) in blocks.iter().enumerate() {
            let lba_u32 = lba as u32;
            match mode {
                SectorMode::Cooked2048 => out.extend_from_slice(block),
                SectorMode::Mode1_2352 => {
                    let raw = build_mode1_sector(lba_u32, block).unwrap();
                    out.extend_from_slice(&raw);
                }
                SectorMode::Mode2_2336 => {
                    let stripped = build_mode2_2336_sector(block, sub);
                    out.extend_from_slice(&stripped);
                }
                SectorMode::Mode2_2352 => {
                    if lba_u32 == 22 {
                        // Synthetic Form 2 sector to test verbatim preservation.
                        let mut raw = [0u8; RAW_SECTOR_SIZE];
                        raw[..12].copy_from_slice(&SYNC_PATTERN);
                        let bcd = crate::sector::lba_to_msf(22).unwrap().to_bcd().unwrap();
                        raw[12..15].copy_from_slice(&bcd);
                        raw[15] = 2;
                        let f2_sub = Subheader::new(1, 2, SUBMODE_FORM2 | 0x04 | 0x40, 0x01);
                        raw[16..24].copy_from_slice(&f2_sub.to_bytes());
                        for (i, b) in raw[24..2348].iter_mut().enumerate() {
                            *b = ((i * 7 + 3) & 0xFF) as u8;
                        }
                        out.extend_from_slice(&raw);
                    } else {
                        let raw = build_mode2_form1_sector(lba_u32, block, sub).unwrap();
                        out.extend_from_slice(&raw);
                    }
                }
            }
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::test_helpers::build_synthetic_image;
    use super::*;

    #[test]
    fn auto_detects_all_four_sector_modes_and_reads_files() {
        for mode in [
            SectorMode::Mode2_2352,
            SectorMode::Mode1_2352,
            SectorMode::Mode2_2336,
            SectorMode::Cooked2048,
        ] {
            let bytes = build_synthetic_image(mode);
            let iso = IsoImage::from_bytes(&bytes).unwrap();
            assert_eq!(iso.mode(), mode);
            assert_eq!(iso.pvd().volume_id, "TEST_VOL");
            assert_eq!(iso.pvd().system_id, "GENERIC_CD");
            assert_eq!(iso.pvd().volume_space_size, 24);

            let recs = iso.records().unwrap();
            assert_eq!(recs.len(), 3);
            assert_eq!(recs[0].path, "/DATA");
            assert!(recs[0].is_dir);
            assert!(recs[0].xa_attributes.unwrap().is_directory());

            let asset = iso.find("/data/asset.bin").unwrap().unwrap();
            assert_eq!(asset.lba, 21);
            assert_eq!(asset.size, 512);
            assert!(asset.xa_attributes.unwrap().is_mode2_form1());
            let data = iso.read_file(&asset).unwrap();
            assert_eq!(data.len(), 512);
            assert_eq!(data[0], 0);
            assert_eq!(data[255], 255);

            let raw_secs = iso.read_file_raw_sectors(&asset).unwrap();
            assert_eq!(raw_secs.len(), mode.stride());

            let readme = iso.find("README.TXT;1").unwrap().unwrap();
            assert_eq!(
                iso.read_file(&readme).unwrap(),
                b"Hello from generic ISO-9660 disc"
            );
            assert!(iso.find("/NO/SUCH/FILE").unwrap().is_none());
        }
    }

    #[cfg(feature = "std")]
    #[test]
    fn from_reader_auto_detects_and_reads() {
        let bytes = build_synthetic_image(SectorMode::Mode2_2352);
        let cursor = std::io::Cursor::new(bytes);
        let iso = IsoImage::from_reader(cursor).unwrap();
        assert_eq!(iso.mode(), SectorMode::Mode2_2352);
        let readme = iso.find("/README.TXT").unwrap().unwrap();
        assert_eq!(
            iso.read_file(&readme).unwrap(),
            b"Hello from generic ISO-9660 disc"
        );
    }
}
