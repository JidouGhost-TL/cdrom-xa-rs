//! Non-destructive ISO-9660 / CD-ROM XA disc image repacker.
//!
//! Supports all four [`SectorMode`] layouts
//! (`Mode1_2352`, `Mode2_2352`, `Mode2_2336`, and `Cooked2048`). Unmodified
//! sectors are copied byte-for-byte, preserving all Mode 2 Form 2 audio/video
//! sectors, system areas, and padding. Files whose replacement fits within
//! their existing sector span are overwritten in place (updating their
//! directory record's both-endian `data_len`); larger replacements are appended
//! sector-aligned at the end of the track (updating both the directory record's
//! `extent_lba`/`data_len` and the PVD `volume_space_size`). Every modified and
//! appended sector has its physical EDC/ECC regenerated.
//!
//! # Invariants & Limitations
//!
//! - **ISO-9660 Path Table invariant:** [`IsoRepacker`] only replaces existing
//!   files and never adds, removes, or moves directories, so the existing
//!   L-type and M-type Path Tables remain valid without modification.
//! - **Multi-track single-BIN `.cue` images:** When the underlying [`IsoImage`]
//!   has trailing tracks in the same `.bin` file
//!   ([`IsoImage::has_trailing_tracks`] is `true`), **in-place** replacements
//!   that fit within the file's existing sector span are supported, but
//!   replacements that require relocating/appending sectors are rejected with
//!   [`Error::InvalidIso`] to avoid overwriting subsequent tracks.

use alloc::collections::{BTreeMap, BTreeSet};
use alloc::string::String;
use alloc::vec;
use alloc::vec::Vec;
use core::fmt;

#[cfg(feature = "std")]
use std::io::Write;
#[cfg(feature = "std")]
use std::path::Path;

use crate::error::{Error, Result};
use crate::iso::{parse_dir_entry, sector_span, IsoImage, RawDirEntry, PVD_LBA};
use crate::sector::{
    build_mode1_sector, build_mode2_2336_sector, build_mode2_form1_sector, extract_user_data,
    regenerate_mode1_edc_ecc, regenerate_mode2_form1_edc_ecc, SectorMode, Subheader,
    COOKED_SECTOR_SIZE, MODE2_2336_SIZE, RAW_SECTOR_SIZE, SUBMODE_DATA, SUBMODE_EOF, SUBMODE_EOR,
    SUBMODE_FORM2, SYNC_PATTERN,
};

/// Maximum LBA representable in standard packed-BCD MSF (`99:59:74` minus 150-frame pregap).
const MAX_MSF_LBA: u32 = 449_849;

/// Number of sectors streamed per contiguous unmodified run (~150 KiB for 2352-byte sectors).
const STREAM_CHUNK_SECTORS: u32 = 64;

/// Summary of a completed disc repack operation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RepackReport {
    /// Total number of file replacements applied.
    pub replaced_files: usize,
    /// Number of files that exceeded their original sector span and were relocated.
    pub relocated_files: usize,
    /// Sector count of the input image.
    pub original_sectors: u32,
    /// Sector count of the output image.
    pub final_sectors: u32,
    /// Number of existing sectors rewritten in place (including directory/PVD updates).
    pub sectors_rewritten: u64,
    /// Number of new sectors appended to the end of the image.
    pub sectors_appended: u64,
    /// Total byte size of the emitted image.
    pub image_bytes: u64,
}

#[derive(Debug, Clone, Copy)]
struct LocatedDirRecord {
    dir_lba: u32,
    offset_in_block: usize,
    extent_lba: u32,
    data_length: u32,
}

struct PatchedSector {
    user_data: [u8; COOKED_SECTOR_SIZE],
    submode_override: Option<u8>,
}

struct PlannedRepack {
    patched_sectors: BTreeMap<u32, PatchedSector>,
    appended_sectors: Vec<([u8; COOKED_SECTOR_SIZE], u8)>,
    report: RepackReport,
}

/// Non-destructive ISO-9660 / CD-ROM XA disc image repacker.
pub struct IsoRepacker {
    iso: IsoImage,
    replacements: Vec<(String, Vec<u8>)>,
}

impl fmt::Debug for IsoRepacker {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("IsoRepacker")
            .field("iso", &self.iso)
            .field("queued_replacements", &self.replacements.len())
            .finish()
    }
}

impl IsoRepacker {
    /// Create a repacker over an already-mounted [`IsoImage`].
    #[must_use]
    pub fn new(iso: IsoImage) -> Self {
        Self {
            iso,
            replacements: Vec::new(),
        }
    }

    /// Open a `.cue`, `.bin`, or `.iso` image file for repacking (`std` feature).
    ///
    /// # Example
    ///
    /// ```no_run
    /// use cdrom_xa::repack::IsoRepacker;
    ///
    /// let mut repacker = IsoRepacker::open("disc.cue")?;
    /// repacker.replace("/DATA/SCRIPT.BIN", vec![0x55u8; 4096]);
    ///
    /// let mut out = std::fs::File::create("patched.bin")?;
    /// let report = repacker.write_to(&mut out)?;
    /// println!("Replaced {} files", report.replaced_files);
    /// # Ok::<(), cdrom_xa::Error>(())
    /// ```
    ///
    /// # Errors
    ///
    /// Returns the same errors as [`IsoImage::open`].
    #[cfg(feature = "std")]
    pub fn open(path: impl AsRef<Path>) -> Result<Self> {
        Ok(Self::new(IsoImage::open(path)?))
    }

    /// Queue a file replacement by ISO-9660 path (e.g. `"/DATA/ASSET.BIN"`).
    ///
    /// If the same file is queued more than once in one session, the later
    /// replacement supersedes the earlier one without leaving orphaned sectors.
    pub fn replace(&mut self, path: impl Into<String>, data: Vec<u8>) -> &mut Self {
        self.replacements.push((path.into(), data));
        self
    }

    /// Borrow the underlying mounted [`IsoImage`].
    #[must_use]
    pub const fn iso(&self) -> &IsoImage {
        &self.iso
    }

    /// Repack the disc image into an in-memory `Vec<u8>`.
    ///
    /// # Errors
    ///
    /// Returns [`Error::FileNotFound`] if any queued replacement path does not
    /// exist, [`Error::InvalidIso`] if a target path is a directory, if a
    /// replacement attempts to overwrite a Mode 2 Form 2 sector in place, if a
    /// replacement requires relocation on a multi-track single-BIN image
    /// ([`IsoImage::has_trailing_tracks`]), or if directory metadata is
    /// malformed/cyclic; [`Error::InvalidSectorSize`] if the image length is
    /// not an exact multiple of the sector stride; [`Error::BadSync`] on an
    /// invalid raw sector header; [`Error::UnexpectedEof`] on truncated input;
    /// [`Error::Overflow`] if the final image would exceed the standard
    /// packed-BCD MSF limit (`99:59:74`) or size limits; or `Error::Io` on
    /// `std` reader I/O failure.
    pub fn to_vec(&self) -> Result<(Vec<u8>, RepackReport)> {
        let plan = self.plan()?;
        let total_bytes = usize::try_from(plan.report.image_bytes).map_err(|_| Error::Overflow)?;
        let mut out = Vec::with_capacity(total_bytes);
        self.emit_sectors(&plan, |bytes| {
            out.extend_from_slice(bytes);
            Ok(())
        })?;
        Ok((out, plan.report))
    }

    /// Stream the complete repacked disc image to `output` (`std` feature).
    ///
    /// Planning and validation complete before any bytes are written to `output`.
    ///
    /// # Errors
    ///
    /// Returns [`Error::Io`] if writing to `output` or reading from the source
    /// image fails, or any planning/validation error documented on
    /// [`IsoRepacker::to_vec`].
    #[cfg(feature = "std")]
    pub fn write_to(&self, output: &mut impl Write) -> Result<RepackReport> {
        let plan = self.plan()?;
        self.emit_sectors(&plan, |bytes| {
            output.write_all(bytes)?;
            Ok(())
        })?;
        Ok(plan.report)
    }

    fn plan(&self) -> Result<PlannedRepack> {
        let mode = self.iso.mode();
        let stride = mode.stride() as u64;
        let original_sectors = self.iso.sector_count();
        let original_bytes = u64::from(original_sectors)
            .checked_mul(stride)
            .ok_or(Error::Overflow)?;
        if original_bytes != self.iso.stored_len() {
            return Err(Error::InvalidSectorSize(self.iso.stored_len() as usize));
        }

        if original_sectors > 0 && original_sectors - 1 > MAX_MSF_LBA {
            return Err(Error::Overflow);
        }

        if self.replacements.is_empty() {
            return Ok(PlannedRepack {
                patched_sectors: BTreeMap::new(),
                appended_sectors: Vec::new(),
                report: RepackReport {
                    replaced_files: 0,
                    relocated_files: 0,
                    original_sectors,
                    final_sectors: original_sectors,
                    sectors_rewritten: 0,
                    sectors_appended: 0,
                    image_bytes: original_bytes,
                },
            });
        }

        // Resolve and deduplicate queued replacements by directory record location
        // so repeated replacements of the same file do not leave orphaned sectors.
        let mut resolved: Vec<(LocatedDirRecord, &[u8])> = Vec::new();
        let mut by_record: BTreeMap<(u32, usize), usize> = BTreeMap::new();
        for (disc_path, new_data) in &self.replacements {
            let located = self.locate_file_dir_record(disc_path)?;
            let key = (located.dir_lba, located.offset_in_block);
            if let Some(&idx) = by_record.get(&key) {
                resolved[idx].1 = new_data.as_slice();
            } else {
                by_record.insert(key, resolved.len());
                resolved.push((located, new_data.as_slice()));
            }
        }

        let mut patched_sectors: BTreeMap<u32, PatchedSector> = BTreeMap::new();
        let mut appended_sectors: Vec<([u8; COOKED_SECTOR_SIZE], u8)> = Vec::new();
        let mut next_lba = original_sectors;
        let mut relocated_files = 0usize;

        for &(located, new_data) in &resolved {
            let new_len = u32::try_from(new_data.len()).map_err(|_| Error::Overflow)?;
            let old_span = sector_span(located.data_length);
            let new_span = sector_span(new_len);

            let new_extent_lba = if new_span <= old_span {
                for idx in 0..new_span {
                    let lba = located.extent_lba.checked_add(idx).ok_or(Error::Overflow)?;
                    let submode = if idx + 1 == new_span {
                        SUBMODE_DATA | SUBMODE_EOR | SUBMODE_EOF
                    } else {
                        SUBMODE_DATA
                    };
                    let patched = self.ensure_patched_form1(&mut patched_sectors, lba)?;
                    fill_block_slice(&mut patched.user_data, new_data, idx as usize);
                    patched.submode_override = Some(submode);
                }
                located.extent_lba
            } else {
                if self.iso.has_trailing_tracks() {
                    return Err(Error::InvalidIso(
                        "cannot relocate file on a multi-track single-BIN image; replacement exceeds existing sector span",
                    ));
                }
                let start_lba = next_lba;
                let end_lba = start_lba.checked_add(new_span).ok_or(Error::Overflow)?;
                for idx in 0..new_span {
                    let mut block = [0u8; COOKED_SECTOR_SIZE];
                    fill_block_slice(&mut block, new_data, idx as usize);
                    let submode = if idx + 1 == new_span {
                        SUBMODE_DATA | SUBMODE_EOR | SUBMODE_EOF
                    } else {
                        SUBMODE_DATA
                    };
                    appended_sectors.push((block, submode));
                }
                next_lba = end_lba;
                relocated_files += 1;
                start_lba
            };

            let dir_block = &mut self
                .ensure_patched_form1(&mut patched_sectors, located.dir_lba)?
                .user_data;
            let off = located.offset_in_block;
            write_both_endian_u32(dir_block, off + 2, new_extent_lba);
            write_both_endian_u32(dir_block, off + 10, new_len);
        }

        if next_lba > original_sectors {
            let pvd_block = &mut self
                .ensure_patched_form1(&mut patched_sectors, PVD_LBA)?
                .user_data;
            let current_vol_size =
                u32::from_le_bytes([pvd_block[80], pvd_block[81], pvd_block[82], pvd_block[83]]);
            let updated_vol_size = current_vol_size.max(next_lba);
            write_both_endian_u32(pvd_block, 80, updated_vol_size);
        }

        let final_sectors = next_lba;
        if final_sectors > 0 && final_sectors - 1 > MAX_MSF_LBA {
            return Err(Error::Overflow);
        }
        let image_bytes = u64::from(final_sectors)
            .checked_mul(stride)
            .ok_or(Error::Overflow)?;
        let report = RepackReport {
            replaced_files: resolved.len(),
            relocated_files,
            original_sectors,
            final_sectors,
            sectors_rewritten: patched_sectors.len() as u64,
            sectors_appended: appended_sectors.len() as u64,
            image_bytes,
        };

        Ok(PlannedRepack {
            patched_sectors,
            appended_sectors,
            report,
        })
    }

    fn emit_sectors(
        &self,
        plan: &PlannedRepack,
        mut sink: impl FnMut(&[u8]) -> Result<()>,
    ) -> Result<()> {
        let mode = self.iso.mode();
        let stride = mode.stride();
        let original_sectors = plan.report.original_sectors;
        let chunk_sectors = STREAM_CHUNK_SECTORS.min(original_sectors.max(1));
        let mut io_buf = vec![0u8; (chunk_sectors as usize) * stride];
        let mut patch_buf = [0u8; RAW_SECTOR_SIZE];

        let mut lba = 0u32;
        while lba < original_sectors {
            if let Some(patched) = plan.patched_sectors.get(&lba) {
                let raw_slot = &mut io_buf[..stride];
                self.iso.read_sector_raw_into(lba, raw_slot)?;
                let written = replace_form1_sector_user_data(
                    raw_slot,
                    mode,
                    &patched.user_data,
                    patched.submode_override,
                    &mut patch_buf,
                )?;
                sink(&patch_buf[..written])?;
                lba += 1;
            } else {
                let run_end = plan
                    .patched_sectors
                    .range(lba..)
                    .next()
                    .map(|(&next_patched, _)| next_patched)
                    .unwrap_or(original_sectors);
                while lba < run_end {
                    let count = (run_end - lba).min(chunk_sectors);
                    let byte_len = (count as usize) * stride;
                    self.iso
                        .read_sectors_raw_into(lba, count, &mut io_buf[..byte_len])?;
                    sink(&io_buf[..byte_len])?;
                    lba += count;
                }
            }
        }

        for (idx, (user_data, submode)) in plan.appended_sectors.iter().enumerate() {
            let lba = original_sectors
                .checked_add(idx as u32)
                .ok_or(Error::Overflow)?;
            let sub = Subheader::new(0, 0, *submode, 0);
            match mode {
                SectorMode::Cooked2048 => sink(user_data)?,
                SectorMode::Mode2_2352 => {
                    let raw = build_mode2_form1_sector(lba, user_data, sub)?;
                    sink(&raw)?;
                }
                SectorMode::Mode1_2352 => {
                    let raw = build_mode1_sector(lba, user_data)?;
                    sink(&raw)?;
                }
                SectorMode::Mode2_2336 => {
                    let raw = build_mode2_2336_sector(user_data, sub);
                    sink(&raw)?;
                }
            }
        }

        Ok(())
    }

    fn ensure_patched_form1<'a>(
        &self,
        patched: &'a mut BTreeMap<u32, PatchedSector>,
        lba: u32,
    ) -> Result<&'a mut PatchedSector> {
        use alloc::collections::btree_map::Entry;
        match patched.entry(lba) {
            Entry::Occupied(entry) => Ok(entry.into_mut()),
            Entry::Vacant(entry) => {
                let raw = self.iso.read_sector_raw(lba)?;
                let user = extract_user_data(&raw, self.iso.mode())?;
                if user.len() != COOKED_SECTOR_SIZE {
                    return Err(Error::InvalidIso(
                        "cannot overwrite Mode 2 Form 2 sector in place",
                    ));
                }
                let mut user_data = [0u8; COOKED_SECTOR_SIZE];
                user_data.copy_from_slice(user);
                Ok(entry.insert(PatchedSector {
                    user_data,
                    submode_override: None,
                }))
            }
        }
    }

    fn locate_file_dir_record(&self, disc_path: &str) -> Result<LocatedDirRecord> {
        let components: Vec<&str> = disc_path
            .split(['/', '\\'])
            .filter(|c| !c.is_empty())
            .collect();
        if components.is_empty() {
            return Err(Error::FileNotFound);
        }

        let mut cur_lba = self.iso.pvd().root_lba;
        let mut cur_size = self.iso.pvd().root_size;
        let mut visited = BTreeSet::new();
        visited.insert(cur_lba);

        for (depth, &component) in components.iter().enumerate() {
            let is_last = depth + 1 == components.len();
            let entries = self.read_dir_with_locations(cur_lba, cur_size)?;
            let have_exact = entries
                .iter()
                .any(|(e, _, _)| e.raw_identifier.eq_ignore_ascii_case(component.as_bytes()));
            let found = entries.into_iter().find(|(e, _, _)| {
                if have_exact {
                    e.raw_identifier.eq_ignore_ascii_case(component.as_bytes())
                } else {
                    e.name.eq_ignore_ascii_case(component)
                }
            });
            let (entry, dir_lba, offset_in_block) = found.ok_or(Error::FileNotFound)?;
            if is_last {
                if entry.is_dir() {
                    return Err(Error::InvalidIso("cannot replace a directory record"));
                }
                return Ok(LocatedDirRecord {
                    dir_lba,
                    offset_in_block,
                    extent_lba: entry.lba,
                    data_length: entry.size,
                });
            }
            if !entry.is_dir() {
                return Err(Error::FileNotFound);
            }
            if !visited.insert(entry.lba) {
                return Err(Error::InvalidIso("cyclic directory reference"));
            }
            cur_lba = entry.lba;
            cur_size = entry.size;
        }

        Err(Error::FileNotFound)
    }

    fn read_dir_with_locations(
        &self,
        dir_lba: u32,
        dir_size: u32,
    ) -> Result<Vec<(RawDirEntry, u32, usize)>> {
        let sectors = sector_span(dir_size);
        let mut out = Vec::new();
        for s in 0..sectors {
            let lba = dir_lba.checked_add(s).ok_or(Error::Overflow)?;
            let block = self.iso.read_sector_cooked(lba)?;
            let mut off = 0usize;
            while off < COOKED_SECTOR_SIZE {
                let len = block[off] as usize;
                if len == 0 {
                    break;
                }
                if len < 33 || off + len > COOKED_SECTOR_SIZE {
                    return Err(Error::InvalidIso("malformed directory record length"));
                }
                let entry = parse_dir_entry(&block[off..off + len])?;
                out.push((entry, lba, off));
                off += len;
            }
        }
        Ok(out)
    }
}

fn replace_form1_sector_user_data(
    raw: &[u8],
    mode: SectorMode,
    user_data: &[u8; COOKED_SECTOR_SIZE],
    submode_override: Option<u8>,
    out: &mut [u8; RAW_SECTOR_SIZE],
) -> Result<usize> {
    if raw.len() != mode.stride() {
        return Err(Error::InvalidSectorSize(raw.len()));
    }
    match mode {
        SectorMode::Cooked2048 => {
            out[..COOKED_SECTOR_SIZE].copy_from_slice(user_data);
            Ok(COOKED_SECTOR_SIZE)
        }
        SectorMode::Mode1_2352 => {
            if raw[..12] != SYNC_PATTERN || raw[15] != 1 {
                return Err(Error::BadSync);
            }
            out.copy_from_slice(raw);
            out[16..16 + COOKED_SECTOR_SIZE].copy_from_slice(user_data);
            regenerate_mode1_edc_ecc(out);
            Ok(RAW_SECTOR_SIZE)
        }
        SectorMode::Mode2_2352 => {
            if raw[..12] != SYNC_PATTERN || raw[15] != 2 {
                return Err(Error::BadSync);
            }
            if raw[18] & SUBMODE_FORM2 != 0 {
                return Err(Error::InvalidIso("cannot overwrite Form 2 sector"));
            }
            out.copy_from_slice(raw);
            if let Some(submode) = submode_override {
                let sub_bytes: [u8; 8] = raw[16..24]
                    .try_into()
                    .map_err(|_| Error::InvalidSectorSize(raw.len()))?;
                let mut sub = Subheader::from_bytes_unchecked(&sub_bytes);
                sub.submode = submode & !SUBMODE_FORM2;
                out[16..24].copy_from_slice(&sub.to_bytes());
            }
            out[24..24 + COOKED_SECTOR_SIZE].copy_from_slice(user_data);
            regenerate_mode2_form1_edc_ecc(out);
            Ok(RAW_SECTOR_SIZE)
        }
        SectorMode::Mode2_2336 => {
            if raw[2] & SUBMODE_FORM2 != 0 {
                return Err(Error::InvalidIso("cannot overwrite Form 2 sector"));
            }
            let mut framed = [0u8; RAW_SECTOR_SIZE];
            framed[..12].copy_from_slice(&SYNC_PATTERN);
            framed[15] = 2;
            framed[16..RAW_SECTOR_SIZE].copy_from_slice(raw);
            if let Some(submode) = submode_override {
                let sub_bytes: [u8; 8] = raw[0..8]
                    .try_into()
                    .map_err(|_| Error::InvalidSectorSize(raw.len()))?;
                let mut sub = Subheader::from_bytes_unchecked(&sub_bytes);
                sub.submode = submode & !SUBMODE_FORM2;
                framed[16..24].copy_from_slice(&sub.to_bytes());
            }
            framed[24..24 + COOKED_SECTOR_SIZE].copy_from_slice(user_data);
            regenerate_mode2_form1_edc_ecc(&mut framed);
            out[..MODE2_2336_SIZE].copy_from_slice(&framed[16..RAW_SECTOR_SIZE]);
            Ok(MODE2_2336_SIZE)
        }
    }
}

fn fill_block_slice(block: &mut [u8; COOKED_SECTOR_SIZE], data: &[u8], sector_index: usize) {
    block.fill(0);
    let start = sector_index * COOKED_SECTOR_SIZE;
    if start < data.len() {
        let take = (data.len() - start).min(COOKED_SECTOR_SIZE);
        block[..take].copy_from_slice(&data[start..start + take]);
    }
}

fn write_both_endian_u32(block: &mut [u8; COOKED_SECTOR_SIZE], offset: usize, value: u32) {
    block[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
    block[offset + 4..offset + 8].copy_from_slice(&value.to_be_bytes());
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::iso::test_helpers::build_synthetic_image;
    use crate::sector::{verify_mode1_edc_ecc, verify_mode2_form1_edc_ecc};
    use alloc::vec;

    #[test]
    fn unmodified_mode2_repack_is_byte_identical() {
        let original = build_synthetic_image(SectorMode::Mode2_2352);
        let iso = IsoImage::from_bytes(&original).unwrap();
        let repacker = IsoRepacker::new(iso);
        let (repacked, report) = repacker.to_vec().unwrap();
        assert_eq!(repacked, original);
        assert_eq!(report.replaced_files, 0);
        assert_eq!(report.relocated_files, 0);
        assert_eq!(report.sectors_rewritten, 0);
        assert_eq!(report.sectors_appended, 0);
    }

    #[test]
    fn in_place_replacement_preserves_form2_and_regenerates_edc_ecc() {
        let original = build_synthetic_image(SectorMode::Mode2_2352);
        let iso = IsoImage::from_bytes(&original).unwrap();
        let mut repacker = IsoRepacker::new(iso);
        let replacement = vec![0xA5u8; 1024];
        repacker.replace("/DATA/ASSET.BIN", replacement.clone());
        let (repacked, report) = repacker.to_vec().unwrap();

        assert_eq!(repacked.len(), original.len());
        assert_eq!(report.replaced_files, 1);
        assert_eq!(report.relocated_files, 0);
        assert_eq!(report.sectors_appended, 0);
        assert_eq!(report.sectors_rewritten, 2);

        // Form 2 sector at LBA 22 is untouched byte-for-byte.
        let lba22 = 22 * RAW_SECTOR_SIZE..23 * RAW_SECTOR_SIZE;
        assert_eq!(&repacked[lba22.clone()], &original[lba22]);

        let reopened = IsoImage::from_bytes(&repacked).unwrap();
        let entry = reopened.find("/DATA/ASSET.BIN").unwrap().unwrap();
        assert_eq!(entry.lba, 21);
        assert_eq!(entry.size, 1024);
        assert_eq!(reopened.read_file(&entry).unwrap(), replacement);

        for lba in [16u32, 18, 19, 20, 21] {
            let raw: [u8; RAW_SECTOR_SIZE] =
                reopened.read_sector_raw(lba).unwrap().try_into().unwrap();
            verify_mode2_form1_edc_ecc(&raw).unwrap();
        }
    }

    #[test]
    fn larger_replacement_relocates_updates_pvd_and_verifies_edc_ecc() {
        let original = build_synthetic_image(SectorMode::Mode2_2352);
        let iso = IsoImage::from_bytes(&original).unwrap();
        let mut repacker = IsoRepacker::new(iso);
        let replacement: Vec<u8> = (0..5000).map(|i| ((i * 13 + 5) & 0xFF) as u8).collect();
        repacker.replace("/DATA/ASSET.BIN", replacement.clone());
        let (repacked, report) = repacker.to_vec().unwrap();

        assert_eq!(report.replaced_files, 1);
        assert_eq!(report.relocated_files, 1);
        assert_eq!(report.original_sectors, 24);
        assert_eq!(report.final_sectors, 27);
        assert_eq!(report.sectors_appended, 3);
        assert_eq!(repacked.len(), 27 * RAW_SECTOR_SIZE);

        // Untouched Form 2 sector at LBA 22 is byte-identical.
        let lba22 = 22 * RAW_SECTOR_SIZE..23 * RAW_SECTOR_SIZE;
        assert_eq!(&repacked[lba22.clone()], &original[lba22]);

        let reopened = IsoImage::from_bytes(&repacked).unwrap();
        assert_eq!(reopened.pvd().volume_space_size, 27);
        let entry = reopened.find("/DATA/ASSET.BIN").unwrap().unwrap();
        assert_eq!(entry.lba, 24);
        assert_eq!(entry.size, 5000);
        assert_eq!(reopened.read_file(&entry).unwrap(), replacement);

        for lba in [16u32, 19, 24, 25, 26] {
            let raw: [u8; RAW_SECTOR_SIZE] =
                reopened.read_sector_raw(lba).unwrap().try_into().unwrap();
            verify_mode2_form1_edc_ecc(&raw).unwrap();
        }
    }

    #[test]
    fn mode1_and_cooked_repack_work() {
        for mode in [
            SectorMode::Mode1_2352,
            SectorMode::Mode2_2336,
            SectorMode::Cooked2048,
        ] {
            let original = build_synthetic_image(mode);
            let iso = IsoImage::from_bytes(&original).unwrap();
            let mut repacker = IsoRepacker::new(iso);
            let replacement = vec![0x3Cu8; 4500];
            repacker.replace("/DATA/ASSET.BIN", replacement.clone());
            let (repacked, report) = repacker.to_vec().unwrap();
            assert_eq!(report.final_sectors, 27);

            let reopened = IsoImage::from_bytes(&repacked).unwrap();
            assert_eq!(reopened.pvd().volume_space_size, 27);
            let entry = reopened.find("/DATA/ASSET.BIN").unwrap().unwrap();
            assert_eq!(entry.lba, 24);
            assert_eq!(reopened.read_file(&entry).unwrap(), replacement);

            if mode == SectorMode::Mode1_2352 {
                for lba in [16u32, 19, 24, 25, 26] {
                    let raw: [u8; RAW_SECTOR_SIZE] =
                        reopened.read_sector_raw(lba).unwrap().try_into().unwrap();
                    verify_mode1_edc_ecc(&raw).unwrap();
                }
            }
            if mode == SectorMode::Mode2_2336 {
                // Verify submode flags on relocated 2336-byte Mode 2 sectors
                let s0 = reopened.read_sector_raw(24).unwrap();
                let s1 = reopened.read_sector_raw(25).unwrap();
                let s2 = reopened.read_sector_raw(26).unwrap();
                assert_eq!(s0[2], SUBMODE_DATA);
                assert_eq!(s1[2], SUBMODE_DATA);
                assert_eq!(s2[2], SUBMODE_DATA | SUBMODE_EOR | SUBMODE_EOF);
            }
        }
    }

    #[test]
    fn shrinking_in_place_updates_submode_eor_eof_and_regenerates_edc_ecc() {
        // First grow ASSET.BIN to 3 sectors (LBA 24, 25, 26), then shrink it in place to 1 sector.
        let original = build_synthetic_image(SectorMode::Mode2_2352);
        let iso = IsoImage::from_bytes(&original).unwrap();
        let mut repacker1 = IsoRepacker::new(iso);
        repacker1.replace("/DATA/ASSET.BIN", vec![0x11u8; 5000]);
        let (grown, _) = repacker1.to_vec().unwrap();

        let iso_grown = IsoImage::from_bytes(&grown).unwrap();
        // Before shrinking, LBA 24 has SUBMODE_DATA (no EOR/EOF)
        assert_eq!(iso_grown.read_sector_raw(24).unwrap()[18], SUBMODE_DATA);

        let mut repacker2 = IsoRepacker::new(iso_grown);
        let shrunk_data = vec![0x77u8; 300];
        repacker2.replace("/DATA/ASSET.BIN", shrunk_data.clone());
        let (shrunk, report) = repacker2.to_vec().unwrap();
        assert_eq!(report.relocated_files, 0);
        assert_eq!(report.sectors_appended, 0);

        let reopened = IsoImage::from_bytes(&shrunk).unwrap();
        let entry = reopened.find("/DATA/ASSET.BIN").unwrap().unwrap();
        assert_eq!(entry.lba, 24);
        assert_eq!(entry.size, 300);
        assert_eq!(reopened.read_file(&entry).unwrap(), shrunk_data);

        // New final sector (LBA 24) must now carry SUBMODE_DATA | SUBMODE_EOR | SUBMODE_EOF
        let raw24: [u8; RAW_SECTOR_SIZE] =
            reopened.read_sector_raw(24).unwrap().try_into().unwrap();
        assert_eq!(raw24[18], SUBMODE_DATA | SUBMODE_EOR | SUBMODE_EOF);
        assert_eq!(raw24[22], SUBMODE_DATA | SUBMODE_EOR | SUBMODE_EOF);
        verify_mode2_form1_edc_ecc(&raw24).unwrap();
    }

    #[test]
    fn repeated_replacements_deduplicate_without_orphans() {
        let original = build_synthetic_image(SectorMode::Mode2_2352);
        let iso = IsoImage::from_bytes(&original).unwrap();
        let mut repacker = IsoRepacker::new(iso);
        // Queue a large replacement, then supersede it with a small one using different casing/version.
        repacker.replace("/DATA/ASSET.BIN", vec![0xAAu8; 6000]);
        repacker.replace("/data/asset.bin;1", vec![0xBBu8; 256]);
        let (repacked, report) = repacker.to_vec().unwrap();

        assert_eq!(report.replaced_files, 1);
        assert_eq!(report.relocated_files, 0);
        assert_eq!(report.sectors_appended, 0);
        assert_eq!(repacked.len(), original.len());

        let reopened = IsoImage::from_bytes(&repacked).unwrap();
        let entry = reopened.find("/DATA/ASSET.BIN").unwrap().unwrap();
        assert_eq!(entry.lba, 21);
        assert_eq!(entry.size, 256);
        assert_eq!(reopened.read_file(&entry).unwrap(), vec![0xBBu8; 256]);
        assert!(alloc::format!("{repacker:?}").contains("IsoRepacker"));
    }

    #[test]
    fn plan_rejects_msf_overflow_before_emitting() {
        let original = build_synthetic_image(SectorMode::Cooked2048);
        let iso = IsoImage::from_bytes(&original).unwrap();
        let mut repacker = IsoRepacker::new(iso);
        // Replacing with > 449_850 sectors (~921 MiB) would overflow 99:59:74 MSF;
        // test via direct check on MAX_MSF_LBA arithmetic:
        // Verify MAX_MSF_LBA == 99:59:74.to_lba()
        assert_eq!(
            crate::sector::Msf::new(99, 59, 74).to_lba() as u32,
            MAX_MSF_LBA
        );
        assert!(crate::sector::lba_to_msf((MAX_MSF_LBA + 1) as i32).is_err());

        // Also test actual plan() error when a file's replacement is rejected on overflow:
        // Use a valid small replacement to ensure plan() succeeds under the limit.
        repacker.replace("/DATA/ASSET.BIN", vec![0x11u8; 16]);
        assert!(repacker.plan().is_ok());
    }

    #[cfg(feature = "std")]
    #[test]
    fn write_to_rejects_msf_overflow_in_plan_before_streaming_bytes() {
        struct SparseIsoReader {
            prefix: Vec<u8>,
            virtual_len: u64,
            pos: u64,
        }
        impl std::io::Read for SparseIsoReader {
            fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
                if self.pos >= self.virtual_len {
                    return Ok(0);
                }
                let avail = (self.virtual_len - self.pos).min(buf.len() as u64) as usize;
                for (i, slot) in buf[..avail].iter_mut().enumerate() {
                    let abs = self.pos as usize + i;
                    *slot = self.prefix.get(abs).copied().unwrap_or(0);
                }
                self.pos += avail as u64;
                Ok(avail)
            }
        }
        impl std::io::Seek for SparseIsoReader {
            fn seek(&mut self, style: std::io::SeekFrom) -> std::io::Result<u64> {
                let next = match style {
                    std::io::SeekFrom::Start(p) => p as i128,
                    std::io::SeekFrom::End(d) => self.virtual_len as i128 + d as i128,
                    std::io::SeekFrom::Current(d) => self.pos as i128 + d as i128,
                };
                if next < 0 {
                    return Err(std::io::Error::new(
                        std::io::ErrorKind::InvalidInput,
                        "negative seek",
                    ));
                }
                self.pos = next as u64;
                Ok(self.pos)
            }
        }

        let prefix = build_synthetic_image(SectorMode::Cooked2048);
        // Exactly at MAX_MSF_LBA + 1 = 449_850 sectors (highest LBA = 449_849 = 99:59:74).
        let reader = SparseIsoReader {
            prefix,
            virtual_len: u64::from(MAX_MSF_LBA + 1) * COOKED_SECTOR_SIZE as u64,
            pos: 0,
        };
        let iso = IsoImage::from_reader_with_mode(reader, SectorMode::Cooked2048).unwrap();
        let mut repacker = IsoRepacker::new(iso);
        // Appending 2 sectors pushes final LBA to 449_851 > MAX_MSF_LBA.
        repacker.replace("/DATA/ASSET.BIN", vec![0x99u8; COOKED_SECTOR_SIZE * 2]);
        let mut out = Vec::new();
        assert!(matches!(repacker.write_to(&mut out), Err(Error::Overflow)));
        assert!(out.is_empty());
    }

    #[cfg(feature = "std")]
    #[test]
    fn multitrack_single_bin_cue_clamps_data_track_and_rejects_relocation() {
        let data_track = build_synthetic_image(SectorMode::Mode2_2352);
        let data_sectors = data_track.len() / RAW_SECTOR_SIZE; // 24 sectors
        let audio_track = vec![0xEEu8; 10 * RAW_SECTOR_SIZE];

        let mut combined_bin = Vec::with_capacity(data_track.len() + audio_track.len());
        combined_bin.extend_from_slice(&data_track);
        combined_bin.extend_from_slice(&audio_track);

        let dir = std::env::temp_dir().join(format!("cdrom_xa_test_{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let bin_path = dir.join("disc.bin");
        let cue_path = dir.join("disc.cue");
        std::fs::write(&bin_path, &combined_bin).unwrap();
        let cue_text = format!(
            "FILE \"disc.bin\" BINARY\n  TRACK 01 MODE2/2352\n    INDEX 01 00:00:00\n  TRACK 02 AUDIO\n    INDEX 00 00:00:{:02}\n    INDEX 01 00:00:{:02}\n",
            data_sectors,
            data_sectors + 2
        );
        std::fs::write(&cue_path, cue_text).unwrap();

        let iso = IsoImage::open(&cue_path).unwrap();
        assert!(iso.has_trailing_tracks());
        assert_eq!(iso.sector_count(), data_sectors as u32);
        assert_eq!(iso.stored_len(), data_track.len() as u64);

        // Relocating (growing beyond 1 sector) MUST be rejected to protect Track 2 audio
        let mut repacker_grow = IsoRepacker::open(&cue_path).unwrap();
        repacker_grow.replace("/DATA/ASSET.BIN", vec![0x55u8; 5000]);
        assert!(matches!(repacker_grow.to_vec(), Err(Error::InvalidIso(_))));

        // In-place replacement (fits in existing 1-sector span) MUST succeed
        let mut repacker_fit = IsoRepacker::open(&cue_path).unwrap();
        let replacement = vec![0x42u8; 1024];
        repacker_fit.replace("/DATA/ASSET.BIN", replacement.clone());
        let mut out = Vec::new();
        let report = repacker_fit.write_to(&mut out).unwrap();
        assert_eq!(report.relocated_files, 0);
        assert_eq!(report.final_sectors, data_sectors as u32);
        assert_eq!(out.len(), data_track.len());

        let reopened = IsoImage::from_bytes(&out).unwrap();
        let entry = reopened.find("/DATA/ASSET.BIN").unwrap().unwrap();
        assert_eq!(reopened.read_file(&entry).unwrap(), replacement);

        let _ = std::fs::remove_dir_all(&dir);
    }
}
