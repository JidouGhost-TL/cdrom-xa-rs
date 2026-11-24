//! Non-destructive ISO-9660 / CD-ROM XA disc image repacker.
//!
//! Unmodified sectors are copied byte-for-byte, preserving all Mode 2 Form 2
//! audio/video sectors, system areas, and padding. Files whose replacement fits
//! within their existing sector span are overwritten in place (updating their
//! directory record's both-endian `data_len`); larger replacements are appended
//! sector-aligned at the end of the track (updating both the directory record's
//! `extent_lba`/`data_len` and the PVD `volume_space_size`). Every modified and
//! appended sector has its physical EDC/ECC regenerated.

use alloc::collections::{BTreeMap, BTreeSet};
use alloc::string::String;
use alloc::vec::Vec;

#[cfg(feature = "std")]
use std::io::Write;
#[cfg(feature = "std")]
use std::path::Path;

use crate::error::{Error, Result};
use crate::iso::{parse_dir_entry, sector_span, IsoImage, RawDirEntry, PVD_LBA};
use crate::sector::{
    build_mode1_sector, build_mode2_2336_sector, build_mode2_form1_sector,
    regenerate_mode1_edc_ecc, regenerate_mode2_form1_edc_ecc, SectorMode, Subheader,
    COOKED_SECTOR_SIZE, MODE2_2336_SIZE, RAW_SECTOR_SIZE, SUBMODE_DATA, SUBMODE_EOF, SUBMODE_EOR,
    SUBMODE_FORM2, SYNC_PATTERN,
};

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

struct PlannedRepack {
    patched_sectors: BTreeMap<u32, [u8; COOKED_SECTOR_SIZE]>,
    appended_sectors: Vec<([u8; COOKED_SECTOR_SIZE], u8)>,
    report: RepackReport,
}

/// Non-destructive ISO-9660 / CD-ROM XA disc image repacker.
pub struct IsoRepacker {
    iso: IsoImage,
    replacements: Vec<(String, Vec<u8>)>,
}

impl IsoRepacker {
    /// Create a repacker over an already-mounted [`IsoImage`].
    pub fn new(iso: IsoImage) -> Self {
        Self {
            iso,
            replacements: Vec::new(),
        }
    }

    /// Open a `.cue`, `.bin`, or `.iso` image file for repacking (`std` feature).
    #[cfg(feature = "std")]
    pub fn open(path: impl AsRef<Path>) -> Result<Self> {
        Ok(Self::new(IsoImage::open(path)?))
    }

    /// Queue a file replacement by ISO-9660 path (e.g. `"/DATA/ASSET.BIN"`).
    ///
    /// Replacements are applied in the order queued.
    pub fn replace(&mut self, path: impl Into<String>, data: Vec<u8>) -> &mut Self {
        self.replacements.push((path.into(), data));
        self
    }

    /// Borrow the underlying mounted [`IsoImage`].
    pub const fn iso(&self) -> &IsoImage {
        &self.iso
    }

    /// Repack the disc image into an in-memory `Vec<u8>`.
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

        let mut patched_sectors: BTreeMap<u32, [u8; COOKED_SECTOR_SIZE]> = BTreeMap::new();
        let mut appended_sectors: Vec<([u8; COOKED_SECTOR_SIZE], u8)> = Vec::new();
        let mut next_lba = original_sectors;
        let mut relocated_files = 0usize;

        for (disc_path, new_data) in &self.replacements {
            let located = self.locate_file_dir_record(&patched_sectors, disc_path)?;
            let new_len = u32::try_from(new_data.len()).map_err(|_| Error::Overflow)?;
            let old_span = sector_span(located.data_length);
            let new_span = sector_span(new_len);

            let new_extent_lba = if new_span <= old_span {
                for idx in 0..new_span {
                    let lba = located.extent_lba.checked_add(idx).ok_or(Error::Overflow)?;
                    if lba >= original_sectors {
                        let app_idx = (lba - original_sectors) as usize;
                        fill_block_slice(&mut appended_sectors[app_idx].0, new_data, idx as usize);
                    } else {
                        let block = self.ensure_patched_form1(&mut patched_sectors, lba)?;
                        fill_block_slice(block, new_data, idx as usize);
                    }
                }
                located.extent_lba
            } else {
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

            let dir_block = self.ensure_patched_form1(&mut patched_sectors, located.dir_lba)?;
            let off = located.offset_in_block;
            write_both_endian_u32(dir_block, off + 2, new_extent_lba);
            write_both_endian_u32(dir_block, off + 10, new_len);
        }

        if next_lba > original_sectors {
            let pvd_block = self.ensure_patched_form1(&mut patched_sectors, PVD_LBA)?;
            let current_vol_size =
                u32::from_le_bytes([pvd_block[80], pvd_block[81], pvd_block[82], pvd_block[83]]);
            let updated_vol_size = current_vol_size.max(next_lba);
            write_both_endian_u32(pvd_block, 80, updated_vol_size);
        }

        let final_sectors = next_lba;
        let image_bytes = u64::from(final_sectors)
            .checked_mul(stride)
            .ok_or(Error::Overflow)?;
        let report = RepackReport {
            replaced_files: self.replacements.len(),
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
        let original_sectors = plan.report.original_sectors;

        for lba in 0..original_sectors {
            if let Some(user_data) = plan.patched_sectors.get(&lba) {
                let raw = self.iso.read_sector_raw(lba)?;
                let updated = replace_form1_sector_user_data(&raw, mode, user_data)?;
                sink(&updated)?;
            } else {
                let raw = self.iso.read_sector_raw(lba)?;
                sink(&raw)?;
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

    fn read_cooked_block(
        &self,
        patched: &BTreeMap<u32, [u8; COOKED_SECTOR_SIZE]>,
        lba: u32,
    ) -> Result<[u8; COOKED_SECTOR_SIZE]> {
        if let Some(block) = patched.get(&lba) {
            return Ok(*block);
        }
        self.iso.read_sector_cooked(lba)
    }

    fn ensure_patched_form1<'a>(
        &self,
        patched: &'a mut BTreeMap<u32, [u8; COOKED_SECTOR_SIZE]>,
        lba: u32,
    ) -> Result<&'a mut [u8; COOKED_SECTOR_SIZE]> {
        if let alloc::collections::btree_map::Entry::Vacant(entry) = patched.entry(lba) {
            let raw = self.iso.read_sector_raw(lba)?;
            let mode = self.iso.mode();
            if let Some(sub_off) = mode.subheader_offset() {
                if raw[sub_off + 2] & SUBMODE_FORM2 != 0 {
                    return Err(Error::InvalidIso(
                        "cannot overwrite Mode 2 Form 2 sector in place",
                    ));
                }
            }
            let cooked = self.iso.read_sector_cooked(lba)?;
            entry.insert(cooked);
        }
        Ok(patched.get_mut(&lba).expect("inserted above"))
    }

    fn locate_file_dir_record(
        &self,
        patched: &BTreeMap<u32, [u8; COOKED_SECTOR_SIZE]>,
        disc_path: &str,
    ) -> Result<LocatedDirRecord> {
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
            let entries = self.read_dir_with_locations(patched, cur_lba, cur_size)?;
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
        patched: &BTreeMap<u32, [u8; COOKED_SECTOR_SIZE]>,
        dir_lba: u32,
        dir_size: u32,
    ) -> Result<Vec<(RawDirEntry, u32, usize)>> {
        let sectors = sector_span(dir_size);
        let mut out = Vec::new();
        for s in 0..sectors {
            let lba = dir_lba.checked_add(s).ok_or(Error::Overflow)?;
            let block = self.read_cooked_block(patched, lba)?;
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
) -> Result<Vec<u8>> {
    if raw.len() != mode.stride() {
        return Err(Error::InvalidSectorSize(raw.len()));
    }
    match mode {
        SectorMode::Cooked2048 => Ok(user_data.to_vec()),
        SectorMode::Mode1_2352 => {
            if raw[..12] != SYNC_PATTERN || raw[15] != 1 {
                return Err(Error::BadSync);
            }
            let mut buf: [u8; RAW_SECTOR_SIZE] = raw
                .try_into()
                .map_err(|_| Error::InvalidSectorSize(raw.len()))?;
            buf[16..16 + COOKED_SECTOR_SIZE].copy_from_slice(user_data);
            regenerate_mode1_edc_ecc(&mut buf);
            Ok(buf.to_vec())
        }
        SectorMode::Mode2_2352 => {
            if raw[..12] != SYNC_PATTERN || raw[15] != 2 {
                return Err(Error::BadSync);
            }
            if raw[18] & SUBMODE_FORM2 != 0 {
                return Err(Error::InvalidIso("cannot overwrite Form 2 sector"));
            }
            let mut buf: [u8; RAW_SECTOR_SIZE] = raw
                .try_into()
                .map_err(|_| Error::InvalidSectorSize(raw.len()))?;
            buf[24..24 + COOKED_SECTOR_SIZE].copy_from_slice(user_data);
            regenerate_mode2_form1_edc_ecc(&mut buf);
            Ok(buf.to_vec())
        }
        SectorMode::Mode2_2336 => {
            if raw[2] & SUBMODE_FORM2 != 0 {
                return Err(Error::InvalidIso("cannot overwrite Form 2 sector"));
            }
            let mut framed = [0u8; RAW_SECTOR_SIZE];
            framed[..12].copy_from_slice(&SYNC_PATTERN);
            framed[15] = 2;
            framed[16..RAW_SECTOR_SIZE].copy_from_slice(raw);
            framed[24..24 + COOKED_SECTOR_SIZE].copy_from_slice(user_data);
            regenerate_mode2_form1_edc_ecc(&mut framed);
            Ok(framed[16..16 + MODE2_2336_SIZE].to_vec())
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
        for mode in [SectorMode::Mode1_2352, SectorMode::Cooked2048] {
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
        }
    }
}
