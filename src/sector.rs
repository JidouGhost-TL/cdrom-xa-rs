//! Physical CD-ROM / CD-ROM XA sector framing, MSF/LBA addressing, subheaders,
//! and Yellow Book EDC/ECC math.
//!
//! All functions in this module are `no_std` and allocation-free.
//!
//! # Supported Sector Layouts
//!
//! | Layout | Bytes | Structure |
//! |---|---|---|
//! | [`SectorMode::Mode1_2352`] | 2352 | 12 sync + 4 header + 2048 user + 4 EDC + 8 zero + 276 ECC |
//! | [`SectorMode::Mode2_2352`] | 2352 | 12 sync + 4 header + 8 subheader + 2048 user + 4 EDC + 276 ECC (Form 1) or 2324 user + 4 EDC (Form 2) |
//! | [`SectorMode::Mode2_2336`] | 2336 | 8 subheader + 2328 payload/EDC/ECC (sync + header stripped) |
//! | [`SectorMode::Cooked2048`] | 2048 | 2048 user data only |

use crate::error::{Error, Result};

/// Standard 12-byte Yellow Book raw sector sync pattern.
pub const SYNC_PATTERN: [u8; 12] = [
    0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00,
];

/// Full raw CD-ROM sector size in bytes (Yellow Book / Red Book frame).
pub const RAW_SECTOR_SIZE: usize = 2352;
/// Stripped Mode 2 sector size in bytes (sync + 4-byte header removed).
pub const MODE2_2336_SIZE: usize = 2336;
/// Cooked Mode 1 or Mode 2 Form 1 user-data size in bytes.
pub const COOKED_SECTOR_SIZE: usize = 2048;
/// Mode 2 Form 2 user-data size in bytes.
pub const FORM2_PAYLOAD_SIZE: usize = 2324;
/// Size of the duplicated CD-ROM XA subheader in bytes.
pub const SUBHEADER_SIZE: usize = 8;

/// Submode bit 0: End of Record (last sector of a logical record).
pub const SUBMODE_EOR: u8 = 0x01;
/// Submode bit 1: Video sector.
pub const SUBMODE_VIDEO: u8 = 0x02;
/// Submode bit 2: Audio sector.
pub const SUBMODE_AUDIO: u8 = 0x04;
/// Submode bit 3: Data sector.
pub const SUBMODE_DATA: u8 = 0x08;
/// Submode bit 4: Trigger / application interrupt.
pub const SUBMODE_TRIGGER: u8 = 0x10;
/// Submode bit 5: Form 2 (2324-byte payload) when set, Form 1 (2048-byte) when clear.
pub const SUBMODE_FORM2: u8 = 0x20;
/// Submode bit 6: Real-time (streaming) sector.
pub const SUBMODE_REALTIME: u8 = 0x40;
/// Submode bit 7: End of File (last sector of a file).
pub const SUBMODE_EOF: u8 = 0x80;

const MODE1_EDC_OFFSET: usize = 0x810;
const MODE2_FORM1_EDC_OFFSET: usize = 0x818;
const MODE2_FORM1_ECC_P_OFFSET: usize = 0x81C;
const MODE2_FORM1_ECC_Q_OFFSET: usize = 0x8C8;
const MODE2_FORM2_EDC_OFFSET: usize = 0x92C;

const ECC_P_LEN: usize = MODE2_FORM1_ECC_Q_OFFSET - MODE2_FORM1_ECC_P_OFFSET; // 172
const ECC_Q_LEN: usize = RAW_SECTOR_SIZE - MODE2_FORM1_ECC_Q_OFFSET; // 104

/// Compile-time CRC-32 lookup table for Yellow Book EDC polynomial `0xD8018001`.
const EDC_LUT: [u32; 256] = {
    let mut table = [0u32; 256];
    let mut i = 0usize;
    while i < 256 {
        let mut value = i as u32;
        let mut bit = 0;
        while bit < 8 {
            if value & 1 != 0 {
                value = (value >> 1) ^ 0xD801_8001;
            } else {
                value >>= 1;
            }
            bit += 1;
        }
        table[i] = value;
        i += 1;
    }
    table
};

/// Compile-time GF(2^8) forward multiplication-by-2 table (polynomial `0x11D`).
const ECC_F_LUT: [u8; 256] = {
    let mut forward = [0u8; 256];
    let mut i = 0usize;
    while i < 256 {
        let mut value = i << 1;
        if i & 0x80 != 0 {
            value ^= 0x11D;
        }
        forward[i] = value as u8;
        i += 1;
    }
    forward
};

/// Compile-time GF(2^8) backward lookup table for Reed-Solomon P/Q parity.
const ECC_B_LUT: [u8; 256] = {
    let mut backward = [0u8; 256];
    let mut i = 0usize;
    while i < 256 {
        let mut value = i << 1;
        if i & 0x80 != 0 {
            value ^= 0x11D;
        }
        backward[i ^ value] = i as u8;
        i += 1;
    }
    backward
};

/// Stored physical layout of sectors in a CD-ROM disc image.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SectorMode {
    /// Raw 2352-byte Mode 1 sectors.
    Mode1_2352,
    /// Raw 2352-byte Mode 2 (CD-ROM XA) sectors.
    Mode2_2352,
    /// Stripped 2336-byte Mode 2 (CD-ROM XA) sectors without sync/header.
    Mode2_2336,
    /// Cooked 2048-byte user-data sectors without physical framing.
    Cooked2048,
}

impl SectorMode {
    /// Byte stride of one stored sector in this layout.
    #[must_use]
    pub const fn stride(self) -> usize {
        match self {
            SectorMode::Mode1_2352 | SectorMode::Mode2_2352 => RAW_SECTOR_SIZE,
            SectorMode::Mode2_2336 => MODE2_2336_SIZE,
            SectorMode::Cooked2048 => COOKED_SECTOR_SIZE,
        }
    }

    /// Byte offset of the user-data field within a stored sector.
    #[must_use]
    pub const fn user_offset(self) -> usize {
        match self {
            SectorMode::Cooked2048 => 0,
            SectorMode::Mode1_2352 => 16,
            SectorMode::Mode2_2352 => 24,
            SectorMode::Mode2_2336 => 8,
        }
    }

    /// Byte offset of the 8-byte CD-ROM XA subheader, if present in this layout.
    #[must_use]
    pub const fn subheader_offset(self) -> Option<usize> {
        match self {
            SectorMode::Cooked2048 | SectorMode::Mode1_2352 => None,
            SectorMode::Mode2_2352 => Some(16),
            SectorMode::Mode2_2336 => Some(0),
        }
    }
}

/// Decoded 4-byte CD-ROM XA subheader (stored twice on disc as 8 bytes).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Subheader {
    /// File number (0..=255).
    pub file: u8,
    /// Channel number (0..=31 used for interleaving).
    pub channel: u8,
    /// Submode bitfield (see `SUBMODE_*` constants).
    pub submode: u8,
    /// Coding information byte (audio parameters for XA audio sectors; 0 for data).
    pub coding: u8,
}

impl Subheader {
    /// Construct a new subheader from its four fields.
    #[must_use]
    pub const fn new(file: u8, channel: u8, submode: u8, coding: u8) -> Self {
        Self {
            file,
            channel,
            submode,
            coding,
        }
    }

    /// Parse an 8-byte duplicated subheader, verifying that both 4-byte copies match.
    ///
    /// # Errors
    ///
    /// Returns [`Error::InvalidXa`] if the first 4 bytes do not match the
    /// duplicated second 4 bytes.
    pub fn from_bytes(bytes: &[u8; SUBHEADER_SIZE]) -> Result<Self> {
        if bytes[0..4] != bytes[4..8] {
            return Err(Error::InvalidXa("duplicated CD-ROM XA subheader mismatch"));
        }
        Ok(Self::from_bytes_unchecked(bytes))
    }

    /// Parse an 8-byte subheader from its first 4 bytes without checking the duplicate copy.
    #[must_use]
    pub const fn from_bytes_unchecked(bytes: &[u8; SUBHEADER_SIZE]) -> Self {
        Self {
            file: bytes[0],
            channel: bytes[1],
            submode: bytes[2],
            coding: bytes[3],
        }
    }

    /// Encode this subheader into its duplicated 8-byte on-disc representation.
    #[must_use]
    pub const fn to_bytes(self) -> [u8; SUBHEADER_SIZE] {
        [
            self.file,
            self.channel,
            self.submode,
            self.coding,
            self.file,
            self.channel,
            self.submode,
            self.coding,
        ]
    }

    /// Returns `true` if the Form 2 bit (`0x20`) is set (2324-byte user payload).
    #[must_use]
    pub const fn is_form2(self) -> bool {
        self.submode & SUBMODE_FORM2 != 0
    }

    /// Returns `true` if the Audio bit (`0x04`) is set.
    #[must_use]
    pub const fn is_audio(self) -> bool {
        self.submode & SUBMODE_AUDIO != 0
    }

    /// Returns `true` if the Video bit (`0x02`) is set.
    #[must_use]
    pub const fn is_video(self) -> bool {
        self.submode & SUBMODE_VIDEO != 0
    }

    /// Returns `true` if the Data bit (`0x08`) is set.
    #[must_use]
    pub const fn is_data(self) -> bool {
        self.submode & SUBMODE_DATA != 0
    }

    /// Returns `true` if the End of Record bit (`0x01`) is set.
    #[must_use]
    pub const fn is_eor(self) -> bool {
        self.submode & SUBMODE_EOR != 0
    }

    /// Returns `true` if the End of File bit (`0x80`) is set.
    #[must_use]
    pub const fn is_eof(self) -> bool {
        self.submode & SUBMODE_EOF != 0
    }

    /// Returns `true` if the Trigger bit (`0x10`) is set.
    #[must_use]
    pub const fn is_trigger(self) -> bool {
        self.submode & SUBMODE_TRIGGER != 0
    }

    /// Returns `true` if the Real-time bit (`0x40`) is set.
    #[must_use]
    pub const fn is_realtime(self) -> bool {
        self.submode & SUBMODE_REALTIME != 0
    }
}

/// Physical Minute:Second:Frame (MSF) CD address (75 frames/sec, 60 sec/min).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Default)]
pub struct Msf {
    /// Minutes (`0..=99` for standard BCD).
    pub min: u8,
    /// Seconds (`0..=59`).
    pub sec: u8,
    /// Frames (`0..=74`).
    pub frame: u8,
}

impl Msf {
    /// Create a new MSF address.
    #[must_use]
    pub const fn new(min: u8, sec: u8, frame: u8) -> Self {
        Self { min, sec, frame }
    }

    /// Convert this MSF address to a Logical Block Address (LBA), subtracting
    /// the standard 150-sector (2-second) lead-in pregap.
    #[must_use]
    pub const fn to_lba(self) -> i32 {
        msf_to_lba(self)
    }

    /// Convert a Logical Block Address (`lba >= -150`) to an MSF address.
    ///
    /// # Errors
    ///
    /// Returns [`Error::Overflow`] if `lba < -150` or if the resulting minute
    /// count exceeds `99` (`lba > 449_849`).
    pub const fn from_lba(lba: i32) -> Result<Self> {
        lba_to_msf(lba)
    }

    /// Decode a 3-byte packed-BCD MSF address (as stored at raw sector bytes `12..15`).
    ///
    /// # Errors
    ///
    /// Returns [`Error::Overflow`] if any byte contains an invalid BCD nibble
    /// (`> 9`) or if `sec >= 60` or `frame >= 75`.
    pub fn from_bcd(bytes: [u8; 3]) -> Result<Self> {
        let min = bcd_to_u8(bytes[0])?;
        let sec = bcd_to_u8(bytes[1])?;
        let frame = bcd_to_u8(bytes[2])?;
        if sec >= 60 || frame >= 75 {
            return Err(Error::Overflow);
        }
        Ok(Self { min, sec, frame })
    }

    /// Encode this MSF address as 3 packed-BCD bytes.
    ///
    /// Rejects out-of-range components (`min > 99`, `sec >= 60`, or `frame >= 75`)
    /// with [`Error::Overflow`], symmetric with [`Msf::from_bcd`].
    ///
    /// # Errors
    ///
    /// Returns [`Error::Overflow`] if `min > 99`, `sec >= 60`, or `frame >= 75`.
    pub fn to_bcd(self) -> Result<[u8; 3]> {
        if self.min > 99 || self.sec >= 60 || self.frame >= 75 {
            return Err(Error::Overflow);
        }
        Ok([
            u8_to_bcd(self.min)?,
            u8_to_bcd(self.sec)?,
            u8_to_bcd(self.frame)?,
        ])
    }

    /// Convert this MSF value into a raw frame count without subtracting the 150-frame pregap.
    #[must_use]
    pub const fn to_frames(self) -> u32 {
        ((self.min as u32) * 60 + (self.sec as u32)) * 75 + (self.frame as u32)
    }
}

/// Convert an [`Msf`] address to a Logical Block Address (LBA), subtracting the
/// standard 150-sector (2-second) pregap.
#[must_use]
pub const fn msf_to_lba(msf: Msf) -> i32 {
    ((msf.min as i32) * 60 + (msf.sec as i32)) * 75 + (msf.frame as i32) - 150
}

/// Convert a Logical Block Address (`lba >= -150`) to an [`Msf`] address.
///
/// # Errors
///
/// Returns [`Error::Overflow`] if `lba < -150` or if the resulting minute
/// count exceeds `99` (`lba > 449_849`).
pub const fn lba_to_msf(lba: i32) -> Result<Msf> {
    let total = match lba.checked_add(150) {
        Some(v) if v >= 0 => v,
        _ => return Err(Error::Overflow),
    };
    let frame = (total % 75) as u8;
    let total_secs = total / 75;
    let sec = (total_secs % 60) as u8;
    let min = total_secs / 60;
    if min > 99 {
        return Err(Error::Overflow);
    }
    Ok(Msf {
        min: min as u8,
        sec,
        frame,
    })
}

/// Decode a single packed-BCD byte (`0x00..=0x99`) to a binary `u8`.
///
/// # Errors
///
/// Returns [`Error::Overflow`] if either nibble is greater than `9`.
pub const fn bcd_to_u8(bcd: u8) -> Result<u8> {
    let hi = bcd >> 4;
    let lo = bcd & 0x0F;
    if hi > 9 || lo > 9 {
        return Err(Error::Overflow);
    }
    Ok(hi * 10 + lo)
}

/// Encode a binary `u8` (`0..=99`) to a packed-BCD byte.
///
/// # Errors
///
/// Returns [`Error::Overflow`] if `value > 99`.
pub const fn u8_to_bcd(value: u8) -> Result<u8> {
    if value > 99 {
        return Err(Error::Overflow);
    }
    Ok(((value / 10) << 4) | (value % 10))
}

/// Compute the Yellow Book / CD-ROM XA Error Detection Code (CRC-32, polynomial `0xD8018001`).
#[must_use]
pub fn compute_edc(bytes: &[u8]) -> u32 {
    let mut edc = 0u32;
    for &byte in bytes {
        edc = (edc >> 8) ^ EDC_LUT[((edc as u8) ^ byte) as usize];
    }
    edc
}

/// Core Reed-Solomon ECC computation over GF(2^8) (`0x11D`).
fn compute_ecc_block(
    source: &[u8],
    major_count: usize,
    minor_count: usize,
    major_mult: usize,
    minor_inc: usize,
    output: &mut [u8],
) {
    let size = major_count * minor_count;
    debug_assert!(source.len() >= size);
    debug_assert_eq!(output.len(), major_count * 2);
    for major in 0..major_count {
        let mut index = (major >> 1) * major_mult + (major & 1);
        let mut a = 0u8;
        let mut b = 0u8;
        for _ in 0..minor_count {
            let value = source[index];
            index += minor_inc;
            if index >= size {
                index -= size;
            }
            a ^= value;
            b ^= value;
            a = ECC_F_LUT[a as usize];
        }
        a = ECC_B_LUT[(ECC_F_LUT[a as usize] ^ b) as usize];
        output[major] = a;
        output[major + major_count] = a ^ b;
    }
}

/// Regenerate Mode 2 Form 1 EDC (bytes `2072..2076`) and Reed-Solomon P/Q ECC
/// (bytes `2076..2352`) in place on a raw 2352-byte sector.
///
/// Per the Yellow Book / CD-ROM XA specification:
/// - EDC is computed over bytes `16..2072` (subheader + user data).
/// - P-parity (172 bytes) and Q-parity (104 bytes) are computed with header
///   bytes `12..16` temporarily zeroed.
pub fn regenerate_mode2_form1_edc_ecc(raw: &mut [u8; RAW_SECTOR_SIZE]) {
    let checksum = compute_edc(&raw[0x10..MODE2_FORM1_EDC_OFFSET]);
    raw[MODE2_FORM1_EDC_OFFSET..MODE2_FORM1_EDC_OFFSET + 4]
        .copy_from_slice(&checksum.to_le_bytes());

    let saved_header = [raw[0x0C], raw[0x0D], raw[0x0E], raw[0x0F]];
    raw[0x0C..0x10].fill(0);

    {
        let (p_src, p_dst) = raw.split_at_mut(MODE2_FORM1_ECC_P_OFFSET);
        compute_ecc_block(&p_src[0x0C..], 86, 24, 2, 86, &mut p_dst[..ECC_P_LEN]);
    }
    {
        let (q_src, q_dst) = raw.split_at_mut(MODE2_FORM1_ECC_Q_OFFSET);
        compute_ecc_block(&q_src[0x0C..], 52, 43, 86, 88, &mut q_dst[..ECC_Q_LEN]);
    }

    raw[0x0C..0x10].copy_from_slice(&saved_header);
}

/// Verify Mode 2 Form 1 EDC and Reed-Solomon P/Q ECC on a raw 2352-byte sector.
///
/// # Errors
///
/// Returns [`Error::BadEdc`] if the stored 4-byte CRC-32 EDC does not match,
/// or [`Error::BadEcc`] if either P-parity or Q-parity does not match.
pub fn verify_mode2_form1_edc_ecc(raw: &[u8; RAW_SECTOR_SIZE]) -> Result<()> {
    let expected = compute_edc(&raw[0x10..MODE2_FORM1_EDC_OFFSET]);
    let actual = u32::from_le_bytes([
        raw[MODE2_FORM1_EDC_OFFSET],
        raw[MODE2_FORM1_EDC_OFFSET + 1],
        raw[MODE2_FORM1_EDC_OFFSET + 2],
        raw[MODE2_FORM1_EDC_OFFSET + 3],
    ]);
    if actual != expected {
        return Err(Error::BadEdc { expected, actual });
    }

    let mut p_source = [0u8; MODE2_FORM1_ECC_P_OFFSET - 0x0C];
    p_source.copy_from_slice(&raw[0x0C..MODE2_FORM1_ECC_P_OFFSET]);
    p_source[..4].fill(0);

    let mut parity_p = [0u8; ECC_P_LEN];
    compute_ecc_block(&p_source, 86, 24, 2, 86, &mut parity_p);
    if raw[MODE2_FORM1_ECC_P_OFFSET..MODE2_FORM1_ECC_Q_OFFSET] != parity_p {
        return Err(Error::BadEcc);
    }

    let mut q_source = [0u8; MODE2_FORM1_ECC_Q_OFFSET - 0x0C];
    q_source[..p_source.len()].copy_from_slice(&p_source);
    q_source[p_source.len()..].copy_from_slice(&parity_p);

    let mut parity_q = [0u8; ECC_Q_LEN];
    compute_ecc_block(&q_source, 52, 43, 86, 88, &mut parity_q);
    if raw[MODE2_FORM1_ECC_Q_OFFSET..RAW_SECTOR_SIZE] != parity_q {
        return Err(Error::BadEcc);
    }

    Ok(())
}

/// Regenerate Mode 1 EDC (bytes `2064..2068`), intermediate 8 zero bytes
/// (`2068..2076`), and Reed-Solomon P/Q ECC (bytes `2076..2352`) in place.
pub fn regenerate_mode1_edc_ecc(raw: &mut [u8; RAW_SECTOR_SIZE]) {
    let checksum = compute_edc(&raw[..MODE1_EDC_OFFSET]);
    raw[MODE1_EDC_OFFSET..MODE1_EDC_OFFSET + 4].copy_from_slice(&checksum.to_le_bytes());
    raw[MODE1_EDC_OFFSET + 4..MODE2_FORM1_ECC_P_OFFSET].fill(0);

    {
        let (p_src, p_dst) = raw.split_at_mut(MODE2_FORM1_ECC_P_OFFSET);
        compute_ecc_block(&p_src[0x0C..], 86, 24, 2, 86, &mut p_dst[..ECC_P_LEN]);
    }
    {
        let (q_src, q_dst) = raw.split_at_mut(MODE2_FORM1_ECC_Q_OFFSET);
        compute_ecc_block(&q_src[0x0C..], 52, 43, 86, 88, &mut q_dst[..ECC_Q_LEN]);
    }
}

/// Verify Mode 1 EDC, zeroed intermediate field, and Reed-Solomon P/Q ECC.
///
/// # Errors
///
/// Returns [`Error::BadEdc`] if the stored 4-byte CRC-32 EDC does not match,
/// or [`Error::BadEcc`] if the 8-byte intermediate zero field is non-zero or
/// either P-parity or Q-parity does not match.
pub fn verify_mode1_edc_ecc(raw: &[u8; RAW_SECTOR_SIZE]) -> Result<()> {
    let expected = compute_edc(&raw[..MODE1_EDC_OFFSET]);
    let actual = u32::from_le_bytes([
        raw[MODE1_EDC_OFFSET],
        raw[MODE1_EDC_OFFSET + 1],
        raw[MODE1_EDC_OFFSET + 2],
        raw[MODE1_EDC_OFFSET + 3],
    ]);
    if actual != expected {
        return Err(Error::BadEdc { expected, actual });
    }
    if raw[MODE1_EDC_OFFSET + 4..MODE2_FORM1_ECC_P_OFFSET]
        .iter()
        .any(|&b| b != 0)
    {
        return Err(Error::BadEcc);
    }

    let mut parity_p = [0u8; ECC_P_LEN];
    compute_ecc_block(
        &raw[0x0C..MODE2_FORM1_ECC_P_OFFSET],
        86,
        24,
        2,
        86,
        &mut parity_p,
    );
    if raw[MODE2_FORM1_ECC_P_OFFSET..MODE2_FORM1_ECC_Q_OFFSET] != parity_p {
        return Err(Error::BadEcc);
    }

    let mut parity_q = [0u8; ECC_Q_LEN];
    compute_ecc_block(
        &raw[0x0C..MODE2_FORM1_ECC_Q_OFFSET],
        52,
        43,
        86,
        88,
        &mut parity_q,
    );
    if raw[MODE2_FORM1_ECC_Q_OFFSET..RAW_SECTOR_SIZE] != parity_q {
        return Err(Error::BadEcc);
    }

    Ok(())
}

/// Regenerate Mode 2 Form 2 EDC (bytes `2348..2352`, computed over bytes `16..2348`).
pub fn regenerate_mode2_form2_edc(raw: &mut [u8; RAW_SECTOR_SIZE]) {
    let checksum = compute_edc(&raw[0x10..MODE2_FORM2_EDC_OFFSET]);
    raw[MODE2_FORM2_EDC_OFFSET..RAW_SECTOR_SIZE].copy_from_slice(&checksum.to_le_bytes());
}

/// Verify Mode 2 Form 2 EDC on a raw 2352-byte sector.
///
/// Note that the CD-ROM XA specification allows Form 2 EDC to be zeroed (omitted);
/// this function strictly checks the computed CRC-32 against the stored 4 bytes.
/// Use [`verify_mode2_form2_edc_allow_zero`] to accept zeroed Form 2 EDC fields.
///
/// # Errors
///
/// Returns [`Error::BadEdc`] if the stored 4-byte EDC does not match the
/// freshly computed CRC-32 over bytes `16..2348`.
pub fn verify_mode2_form2_edc(raw: &[u8; RAW_SECTOR_SIZE]) -> Result<()> {
    let expected = compute_edc(&raw[0x10..MODE2_FORM2_EDC_OFFSET]);
    let actual = u32::from_le_bytes([
        raw[MODE2_FORM2_EDC_OFFSET],
        raw[MODE2_FORM2_EDC_OFFSET + 1],
        raw[MODE2_FORM2_EDC_OFFSET + 2],
        raw[MODE2_FORM2_EDC_OFFSET + 3],
    ]);
    if actual != expected {
        return Err(Error::BadEdc { expected, actual });
    }
    Ok(())
}

/// Verify Mode 2 Form 2 EDC on a raw 2352-byte sector, accepting an all-zero
/// EDC field (`0x0000_0000`) as permitted by the CD-ROM XA specification.
///
/// # Example
///
/// ```
/// use cdrom_xa::sector::{
///     regenerate_mode2_form2_edc, verify_mode2_form2_edc,
///     verify_mode2_form2_edc_allow_zero, Subheader, RAW_SECTOR_SIZE,
///     SUBMODE_AUDIO, SUBMODE_FORM2, SYNC_PATTERN,
/// };
///
/// let mut raw = [0u8; RAW_SECTOR_SIZE];
/// raw[..12].copy_from_slice(&SYNC_PATTERN);
/// raw[15] = 2;
/// let sub = Subheader::new(1, 0, SUBMODE_AUDIO | SUBMODE_FORM2, 0x01);
/// raw[16..24].copy_from_slice(&sub.to_bytes());
///
/// // An all-zero Form 2 EDC is accepted by verify_mode2_form2_edc_allow_zero:
/// verify_mode2_form2_edc_allow_zero(&raw)?;
///
/// // Once populated, strict verification passes as well:
/// regenerate_mode2_form2_edc(&mut raw);
/// verify_mode2_form2_edc(&raw)?;
/// verify_mode2_form2_edc_allow_zero(&raw)?;
/// # Ok::<(), cdrom_xa::Error>(())
/// ```
///
/// # Errors
///
/// Returns [`Error::BadEdc`] if the stored 4-byte EDC is non-zero and does not
/// match the freshly computed CRC-32 over bytes `16..2348`.
pub fn verify_mode2_form2_edc_allow_zero(raw: &[u8; RAW_SECTOR_SIZE]) -> Result<()> {
    let actual = u32::from_le_bytes([
        raw[MODE2_FORM2_EDC_OFFSET],
        raw[MODE2_FORM2_EDC_OFFSET + 1],
        raw[MODE2_FORM2_EDC_OFFSET + 2],
        raw[MODE2_FORM2_EDC_OFFSET + 3],
    ]);
    if actual == 0 {
        return Ok(());
    }
    let expected = compute_edc(&raw[0x10..MODE2_FORM2_EDC_OFFSET]);
    if actual != expected {
        return Err(Error::BadEdc { expected, actual });
    }
    Ok(())
}

/// Build a complete raw 2352-byte Mode 1 sector for `lba` with freshly computed EDC/ECC.
///
/// # Errors
///
/// Returns [`Error::Overflow`] if `lba` cannot be represented as a standard
/// packed-BCD MSF address (`lba > 449_849`).
pub fn build_mode1_sector(
    lba: u32,
    user_data: &[u8; COOKED_SECTOR_SIZE],
) -> Result<[u8; RAW_SECTOR_SIZE]> {
    let lba_i32 = i32::try_from(lba).map_err(|_| Error::Overflow)?;
    let msf = lba_to_msf(lba_i32)?;
    let bcd = msf.to_bcd()?;
    let mut raw = [0u8; RAW_SECTOR_SIZE];
    raw[..12].copy_from_slice(&SYNC_PATTERN);
    raw[12..15].copy_from_slice(&bcd);
    raw[15] = 1;
    raw[16..16 + COOKED_SECTOR_SIZE].copy_from_slice(user_data);
    regenerate_mode1_edc_ecc(&mut raw);
    Ok(raw)
}

/// Build a complete raw 2352-byte Mode 2 Form 1 sector for `lba` with freshly computed EDC/ECC.
///
/// The Form 2 bit in `subheader.submode` is cleared automatically to guarantee a valid Form 1 sector.
///
/// # Errors
///
/// Returns [`Error::Overflow`] if `lba` cannot be represented as a standard
/// packed-BCD MSF address (`lba > 449_849`).
pub fn build_mode2_form1_sector(
    lba: u32,
    user_data: &[u8; COOKED_SECTOR_SIZE],
    mut subheader: Subheader,
) -> Result<[u8; RAW_SECTOR_SIZE]> {
    let lba_i32 = i32::try_from(lba).map_err(|_| Error::Overflow)?;
    let msf = lba_to_msf(lba_i32)?;
    let bcd = msf.to_bcd()?;
    subheader.submode &= !SUBMODE_FORM2;
    let mut raw = [0u8; RAW_SECTOR_SIZE];
    raw[..12].copy_from_slice(&SYNC_PATTERN);
    raw[12..15].copy_from_slice(&bcd);
    raw[15] = 2;
    raw[16..24].copy_from_slice(&subheader.to_bytes());
    raw[24..24 + COOKED_SECTOR_SIZE].copy_from_slice(user_data);
    regenerate_mode2_form1_edc_ecc(&mut raw);
    Ok(raw)
}

/// Build a stripped 2336-byte Mode 2 Form 1 sector with freshly computed EDC/ECC.
///
/// The Form 2 bit in `subheader.submode` is cleared automatically.
#[must_use]
pub fn build_mode2_2336_sector(
    user_data: &[u8; COOKED_SECTOR_SIZE],
    mut subheader: Subheader,
) -> [u8; MODE2_2336_SIZE] {
    subheader.submode &= !SUBMODE_FORM2;
    let mut raw = [0u8; RAW_SECTOR_SIZE];
    raw[..12].copy_from_slice(&SYNC_PATTERN);
    raw[15] = 2;
    raw[16..24].copy_from_slice(&subheader.to_bytes());
    raw[24..24 + COOKED_SECTOR_SIZE].copy_from_slice(user_data);
    regenerate_mode2_form1_edc_ecc(&mut raw);
    let mut out = [0u8; MODE2_2336_SIZE];
    out.copy_from_slice(&raw[16..RAW_SECTOR_SIZE]);
    out
}

/// Extract the user-data slice from a single sector in the given [`SectorMode`].
///
/// - For `Cooked2048` and `Mode1_2352`, returns 2048 bytes.
/// - For `Mode2_2352` and `Mode2_2336`, inspects the XA subheader and returns
///   2048 bytes for Form 1 or 2324 bytes for Form 2.
///
/// # Errors
///
/// Returns [`Error::InvalidSectorSize`] if `sector.len() != mode.stride()`, or
/// [`Error::BadSync`] for raw 2352-byte modes if the 12-byte sync pattern or
/// mode byte (`1` or `2`) is invalid.
pub fn extract_user_data(sector: &[u8], mode: SectorMode) -> Result<&[u8]> {
    if sector.len() != mode.stride() {
        return Err(Error::InvalidSectorSize(sector.len()));
    }
    match mode {
        SectorMode::Cooked2048 => Ok(sector),
        SectorMode::Mode1_2352 => {
            if sector[..12] != SYNC_PATTERN || sector[15] != 1 {
                return Err(Error::BadSync);
            }
            Ok(&sector[16..16 + COOKED_SECTOR_SIZE])
        }
        SectorMode::Mode2_2352 => {
            if sector[..12] != SYNC_PATTERN || sector[15] != 2 {
                return Err(Error::BadSync);
            }
            let is_form2 = sector[18] & SUBMODE_FORM2 != 0;
            let len = if is_form2 {
                FORM2_PAYLOAD_SIZE
            } else {
                COOKED_SECTOR_SIZE
            };
            Ok(&sector[24..24 + len])
        }
        SectorMode::Mode2_2336 => {
            let is_form2 = sector[2] & SUBMODE_FORM2 != 0;
            let len = if is_form2 {
                FORM2_PAYLOAD_SIZE
            } else {
                COOKED_SECTOR_SIZE
            };
            Ok(&sector[8..8 + len])
        }
    }
}

/// Auto-detect the [`SectorMode`] of a disc image slice by inspecting the
/// ISO-9660 Primary Volume Descriptor signature (`\x01CD001`) at LBA 16.
///
/// If no PVD signature is found (for example, when fewer than 17 sectors are
/// present or LBA 16 is not an ISO-9660 PVD), falls back to inspecting the
/// leading raw 2352-byte sync pattern and mode byte when `data.len()` is a
/// non-zero multiple of `2352`.
#[must_use]
pub fn detect_sector_mode(data: &[u8]) -> Option<SectorMode> {
    const PVD_MAGIC: &[u8; 6] = b"\x01CD001";
    const CANDIDATES: [SectorMode; 4] = [
        SectorMode::Mode2_2352,
        SectorMode::Mode1_2352,
        SectorMode::Mode2_2336,
        SectorMode::Cooked2048,
    ];

    for mode in CANDIDATES {
        let pvd_sector_start = 16usize.checked_mul(mode.stride())?;
        let user_start = pvd_sector_start.checked_add(mode.user_offset())?;
        if let Some(magic) = data.get(user_start..user_start + 6) {
            if magic == PVD_MAGIC {
                match mode {
                    SectorMode::Mode1_2352 => {
                        if data.get(pvd_sector_start..pvd_sector_start + 12)
                            == Some(SYNC_PATTERN.as_slice())
                            && data.get(pvd_sector_start + 15) == Some(&1)
                        {
                            return Some(mode);
                        }
                    }
                    SectorMode::Mode2_2352 => {
                        if data.get(pvd_sector_start..pvd_sector_start + 12)
                            == Some(SYNC_PATTERN.as_slice())
                            && data.get(pvd_sector_start + 15) == Some(&2)
                        {
                            return Some(mode);
                        }
                    }
                    SectorMode::Mode2_2336 | SectorMode::Cooked2048 => return Some(mode),
                }
            }
        }
    }

    // Fallback: inspect leading raw sync header if present.
    if data.len() >= RAW_SECTOR_SIZE
        && data.len() % RAW_SECTOR_SIZE == 0
        && data[..12] == SYNC_PATTERN
    {
        return match data[15] {
            1 => Some(SectorMode::Mode1_2352),
            2 => Some(SectorMode::Mode2_2352),
            _ => None,
        };
    }

    None
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn msf_lba_and_bcd_round_trip() {
        assert_eq!(lba_to_msf(0).unwrap(), Msf::new(0, 2, 0));
        assert_eq!(msf_to_lba(Msf::new(0, 2, 0)), 0);
        assert_eq!(lba_to_msf(-150).unwrap(), Msf::new(0, 0, 0));
        assert!(lba_to_msf(-151).is_err());

        for lba in [0, 16, 75, 4500, 150_000, 333_000] {
            let msf = lba_to_msf(lba).unwrap();
            assert_eq!(msf_to_lba(msf), lba);
            let bcd = msf.to_bcd().unwrap();
            assert_eq!(Msf::from_bcd(bcd).unwrap(), msf);
        }

        assert_eq!(u8_to_bcd(0).unwrap(), 0x00);
        assert_eq!(u8_to_bcd(59).unwrap(), 0x59);
        assert_eq!(u8_to_bcd(99).unwrap(), 0x99);
        assert!(u8_to_bcd(100).is_err());
        assert!(bcd_to_u8(0x0A).is_err());
        assert!(bcd_to_u8(0xF0).is_err());

        // Symmetry between Msf::to_bcd and Msf::from_bcd bounds checks
        assert!(matches!(Msf::new(100, 0, 0).to_bcd(), Err(Error::Overflow)));
        assert!(matches!(Msf::new(0, 60, 0).to_bcd(), Err(Error::Overflow)));
        assert!(matches!(Msf::new(0, 0, 75).to_bcd(), Err(Error::Overflow)));
        assert!(matches!(
            Msf::from_bcd([0x00, 0x60, 0x00]),
            Err(Error::Overflow)
        ));
        assert!(matches!(
            Msf::from_bcd([0x00, 0x00, 0x75]),
            Err(Error::Overflow)
        ));
    }

    #[test]
    fn mode2_form1_build_verify_and_corrupt_detection() {
        let mut user = [0u8; COOKED_SECTOR_SIZE];
        for (i, b) in user.iter_mut().enumerate() {
            *b = ((i * 31 + 7) & 0xFF) as u8;
        }
        let sub = Subheader::new(0, 0, SUBMODE_DATA | SUBMODE_EOR | SUBMODE_EOF, 0);
        let mut raw = build_mode2_form1_sector(16, &user, sub).unwrap();
        verify_mode2_form1_edc_ecc(&raw).unwrap();
        assert_eq!(
            extract_user_data(&raw, SectorMode::Mode2_2352).unwrap(),
            &user[..]
        );

        // Corrupt user data -> EDC mismatch
        raw[24] ^= 0x01;
        assert!(matches!(
            verify_mode2_form1_edc_ecc(&raw),
            Err(Error::BadEdc { .. })
        ));

        // Regenerate -> valid again
        regenerate_mode2_form1_edc_ecc(&mut raw);
        verify_mode2_form1_edc_ecc(&raw).unwrap();

        // Corrupt ECC P-parity -> BadEcc
        raw[MODE2_FORM1_ECC_P_OFFSET] ^= 0x80;
        assert!(matches!(
            verify_mode2_form1_edc_ecc(&raw),
            Err(Error::BadEcc)
        ));
        regenerate_mode2_form1_edc_ecc(&mut raw);

        // Corrupt ECC Q-parity -> BadEcc
        raw[MODE2_FORM1_ECC_Q_OFFSET] ^= 0x01;
        assert!(matches!(
            verify_mode2_form1_edc_ecc(&raw),
            Err(Error::BadEcc)
        ));
    }

    #[test]
    fn mode1_build_verify_and_extract() {
        let user = [0xA5u8; COOKED_SECTOR_SIZE];
        let mut raw = build_mode1_sector(20, &user).unwrap();
        verify_mode1_edc_ecc(&raw).unwrap();
        assert_eq!(
            extract_user_data(&raw, SectorMode::Mode1_2352).unwrap(),
            &user[..]
        );

        raw[16] ^= 0xFF;
        assert!(matches!(
            verify_mode1_edc_ecc(&raw),
            Err(Error::BadEdc { .. })
        ));
        regenerate_mode1_edc_ecc(&mut raw);
        verify_mode1_edc_ecc(&raw).unwrap();
    }

    #[test]
    fn mode2_2336_build_and_extract() {
        let user = [0x3Cu8; COOKED_SECTOR_SIZE];
        let sub = Subheader::new(1, 2, SUBMODE_DATA, 0);
        let stripped = build_mode2_2336_sector(&user, sub);
        assert_eq!(
            extract_user_data(&stripped, SectorMode::Mode2_2336).unwrap(),
            &user[..]
        );
        let parsed_sub = Subheader::from_bytes(&stripped[0..8].try_into().unwrap()).unwrap();
        assert_eq!(parsed_sub, sub);
        assert!(parsed_sub.is_data());
        assert!(!parsed_sub.is_form2());
    }

    #[test]
    fn subheader_predicates_and_duplicate_check() {
        let sub = Subheader::new(
            3,
            5,
            SUBMODE_FORM2 | SUBMODE_AUDIO | SUBMODE_REALTIME | SUBMODE_EOR | SUBMODE_EOF,
            0x01,
        );
        let bytes = sub.to_bytes();
        assert_eq!(Subheader::from_bytes(&bytes).unwrap(), sub);
        assert!(sub.is_form2());
        assert!(sub.is_audio());
        assert!(sub.is_realtime());
        assert!(sub.is_eor());
        assert!(sub.is_eof());
        assert!(!sub.is_video());
        assert!(!sub.is_data());

        let mut bad = bytes;
        bad[4] ^= 1;
        assert!(Subheader::from_bytes(&bad).is_err());
    }

    #[test]
    fn ecma130_all_zero_mode1_and_mode2_form1_golden_edc_ecc() {
        let zero_user = [0u8; COOKED_SECTOR_SIZE];

        // Mode 2 Form 1 with all-zero subheader and all-zero user data:
        // EDC is 0x0000_0000 and P/Q ECC parity blocks are all zero.
        let m2_zero = build_mode2_form1_sector(0, &zero_user, Subheader::default()).unwrap();
        verify_mode2_form1_edc_ecc(&m2_zero).unwrap();
        assert_eq!(
            u32::from_le_bytes(
                m2_zero[MODE2_FORM1_EDC_OFFSET..MODE2_FORM1_EDC_OFFSET + 4]
                    .try_into()
                    .unwrap()
            ),
            0x0000_0000
        );
        assert!(m2_zero[MODE2_FORM1_ECC_P_OFFSET..RAW_SECTOR_SIZE]
            .iter()
            .all(|&b| b == 0));

        // Mode 2 Form 1 with SUBMODE_DATA (0x08) and all-zero user data:
        // Golden ECMA-130 EDC over 2056 bytes is 0x9481_880B.
        let m2_data =
            build_mode2_form1_sector(0, &zero_user, Subheader::new(0, 0, SUBMODE_DATA, 0)).unwrap();
        verify_mode2_form1_edc_ecc(&m2_data).unwrap();
        assert_eq!(
            u32::from_le_bytes(
                m2_data[MODE2_FORM1_EDC_OFFSET..MODE2_FORM1_EDC_OFFSET + 4]
                    .try_into()
                    .unwrap()
            ),
            0x9481_880B
        );

        // Mode 1 at LBA 0 (header 00:02:00, mode 1) with all-zero user data:
        // Golden ECMA-130 EDC over bytes 0..2064 is 0x2B68_13C5.
        let m1_zero = build_mode1_sector(0, &zero_user).unwrap();
        verify_mode1_edc_ecc(&m1_zero).unwrap();
        assert_eq!(
            u32::from_le_bytes(
                m1_zero[MODE1_EDC_OFFSET..MODE1_EDC_OFFSET + 4]
                    .try_into()
                    .unwrap()
            ),
            0x2B68_13C5
        );
    }

    #[test]
    fn mode1_ecc_and_intermediate_zero_corruption_detection() {
        let user = [0x5Au8; COOKED_SECTOR_SIZE];
        let mut raw = build_mode1_sector(32, &user).unwrap();
        verify_mode1_edc_ecc(&raw).unwrap();

        // Corrupt 8-byte intermediate zero field -> BadEcc
        raw[MODE1_EDC_OFFSET + 4] = 0x01;
        assert!(matches!(verify_mode1_edc_ecc(&raw), Err(Error::BadEcc)));
        regenerate_mode1_edc_ecc(&mut raw);
        verify_mode1_edc_ecc(&raw).unwrap();

        // Corrupt P-parity -> BadEcc
        raw[MODE2_FORM1_ECC_P_OFFSET + 10] ^= 0x40;
        assert!(matches!(verify_mode1_edc_ecc(&raw), Err(Error::BadEcc)));
        regenerate_mode1_edc_ecc(&mut raw);

        // Corrupt Q-parity -> BadEcc
        raw[MODE2_FORM1_ECC_Q_OFFSET + 5] ^= 0x02;
        assert!(matches!(verify_mode1_edc_ecc(&raw), Err(Error::BadEcc)));
    }

    #[test]
    fn mode2_form2_edc_verify_corrupt_and_allow_zero() {
        let mut raw = [0u8; RAW_SECTOR_SIZE];
        raw[..12].copy_from_slice(&SYNC_PATTERN);
        raw[15] = 2;
        let sub = Subheader::new(1, 0, SUBMODE_FORM2 | SUBMODE_AUDIO | SUBMODE_REALTIME, 0x01);
        raw[16..24].copy_from_slice(&sub.to_bytes());
        for (i, b) in raw[24..MODE2_FORM2_EDC_OFFSET].iter_mut().enumerate() {
            *b = ((i * 17 + 11) & 0xFF) as u8;
        }

        regenerate_mode2_form2_edc(&mut raw);
        verify_mode2_form2_edc(&raw).unwrap();
        verify_mode2_form2_edc_allow_zero(&raw).unwrap();

        // Corrupt payload -> BadEdc for both strict and allow-zero
        raw[100] ^= 0x01;
        assert!(matches!(
            verify_mode2_form2_edc(&raw),
            Err(Error::BadEdc { .. })
        ));
        assert!(matches!(
            verify_mode2_form2_edc_allow_zero(&raw),
            Err(Error::BadEdc { .. })
        ));

        // Zeroed EDC -> rejected by strict, accepted by allow-zero
        raw[MODE2_FORM2_EDC_OFFSET..RAW_SECTOR_SIZE].fill(0);
        assert!(matches!(
            verify_mode2_form2_edc(&raw),
            Err(Error::BadEdc { actual: 0, .. })
        ));
        verify_mode2_form2_edc_allow_zero(&raw).unwrap();
    }
}
