//! CD-ROM XA ADPCM audio decoder, interleaved Mode 2 Form 2 demuxer, and
//! `RIFF`/`CDXA` container unwrapper.
//!
//! Core sound-group decoding ([`XaCoding`], [`XaState`],
//! [`decode_sound_group_4bit`], [`decode_sound_group_8bit`]) and
//! [`unwrap_riff_cdxa`] are `no_std` and allocation-free. Higher-level
//! [`XaDecoder`] and [`XaStream`] are available under the `alloc` feature.

#[cfg(feature = "alloc")]
use alloc::collections::BTreeMap;
#[cfg(feature = "alloc")]
use alloc::vec::Vec;

use crate::error::{Error, Result};
#[cfg(feature = "alloc")]
use crate::sector::{SectorMode, Subheader, FORM2_PAYLOAD_SIZE, MODE2_2336_SIZE};
use crate::sector::{RAW_SECTOR_SIZE, SYNC_PATTERN};

/// Size of one CD-ROM XA ADPCM sound group in bytes (16-byte header + 112-byte sample data).
pub const SOUND_GROUP_SIZE: usize = 128;
/// Number of sound groups stored in one Mode 2 Form 2 audio sector.
pub const SOUND_GROUPS_PER_SECTOR: usize = 18;
/// Active sound-data payload bytes per Form 2 audio sector (`18 * 128 = 2304`).
pub const SOUND_DATA_SIZE: usize = SOUND_GROUP_SIZE * SOUND_GROUPS_PER_SECTOR;
/// Exact byte length of a standard `RIFF`/`CDXA` wrapper header.
pub const RIFF_CDXA_HEADER_SIZE: usize = 44;

const SAMPLES_PER_UNIT: usize = 28;
const UNITS_PER_BLOCK: usize = 4;

/// Fixed-point second-order ADPCM predictor coefficients (`K0 / 64`, `K1 / 64`).
const K0: [i32; 4] = [0, 60, 115, 98];
const K1: [i32; 4] = [0, 0, -52, -55];
/// CD-ROM XA rounds predictions to nearest (`+ 32` before `>> 6`).
const XA_ROUND_BIAS: i32 = 32;

/// Decoded CD-ROM XA coding-info byte (subheader byte 3).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct XaCoding {
    /// `true` for stereo (2 channels), `false` for mono (1 channel).
    pub stereo: bool,
    /// Sample rate in Hz (`37800` or `18900`).
    pub sample_rate_hz: u32,
    /// Bits per ADPCM sample (`4` for Level B/C 4-bit, `8` for Level A 8-bit).
    pub bits_per_sample: u8,
    /// De-emphasis flag.
    pub emphasis: bool,
}

impl XaCoding {
    /// Decode a CD-ROM XA subheader coding-info byte.
    pub const fn from_byte(byte: u8) -> Self {
        Self {
            stereo: byte & 0x01 != 0,
            sample_rate_hz: if byte & 0x04 != 0 { 18_900 } else { 37_800 },
            bits_per_sample: if byte & 0x10 != 0 { 8 } else { 4 },
            emphasis: byte & 0x40 != 0,
        }
    }

    /// Encode this coding descriptor back into a subheader coding-info byte.
    pub const fn to_byte(self) -> u8 {
        let mut b = 0u8;
        if self.stereo {
            b |= 0x01;
        }
        if self.sample_rate_hz <= 18_900 {
            b |= 0x04;
        }
        if self.bits_per_sample == 8 {
            b |= 0x10;
        }
        if self.emphasis {
            b |= 0x40;
        }
        b
    }

    /// Number of audio channels (`2` for stereo, `1` for mono).
    pub const fn channels(self) -> u8 {
        if self.stereo {
            2
        } else {
            1
        }
    }
}

/// Per-channel second-order ADPCM predictor history.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
struct ChannelHistory {
    prev1: i16,
    prev2: i16,
}

impl ChannelHistory {
    #[inline]
    fn step(&mut self, delta: i32, filter: usize) -> i16 {
        let f = filter.min(K0.len() - 1);
        let predicted =
            (i32::from(self.prev1) * K0[f] + i32::from(self.prev2) * K1[f] + XA_ROUND_BIAS) >> 6;
        let sample = (delta + predicted).clamp(i32::from(i16::MIN), i32::from(i16::MAX)) as i16;
        self.prev2 = self.prev1;
        self.prev1 = sample;
        sample
    }
}

/// Persistent predictor state for a continuous CD-ROM XA ADPCM stream.
///
/// Maintain one `XaState` per `(file, channel)` stream across sectors so
/// predictor history carries seamlessly across sector boundaries.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct XaState {
    left: ChannelHistory,
    right: ChannelHistory,
}

impl XaState {
    /// Create a zero-initialized predictor state.
    pub const fn new() -> Self {
        Self {
            left: ChannelHistory { prev1: 0, prev2: 0 },
            right: ChannelHistory { prev1: 0, prev2: 0 },
        }
    }
}

/// Decode one 128-byte 4-bit CD-ROM XA sound group into 224 `i16` PCM samples.
///
/// For stereo streams, the 224 samples are interleaved `[L, R, L, R, ...]`
/// (112 frames). For mono streams, all 224 samples belong to the single mono
/// channel.
pub fn decode_sound_group_4bit(
    group: &[u8; SOUND_GROUP_SIZE],
    stereo: bool,
    state: &mut XaState,
    out: &mut [i16; 224],
) {
    let header = &group[0..16];
    let data = &group[16..128];
    let mut out_pos = 0usize;

    for slot in 0..UNITS_PER_BLOCK {
        let hdr_off = 4 + 2 * slot;
        let hdr_lo = header[hdr_off];
        let hdr_hi = header[hdr_off + 1];

        let shift_lo = (hdr_lo & 0x0F).min(12);
        let filter_lo = ((hdr_lo >> 4) & 0x0F) as usize;
        let shift_hi = (hdr_hi & 0x0F).min(12);
        let filter_hi = ((hdr_hi >> 4) & 0x0F) as usize;

        if stereo {
            let mut left_buf = [0i16; SAMPLES_PER_UNIT];
            let mut right_buf = [0i16; SAMPLES_PER_UNIT];
            for j in 0..SAMPLES_PER_UNIT {
                let byte = data[4 * j + slot];
                let nib_lo = i16::from(byte & 0x0F);
                let delta_lo = i32::from((nib_lo << 12) >> shift_lo);
                left_buf[j] = state.left.step(delta_lo, filter_lo);

                let nib_hi = i16::from((byte >> 4) & 0x0F);
                let delta_hi = i32::from((nib_hi << 12) >> shift_hi);
                right_buf[j] = state.right.step(delta_hi, filter_hi);
            }
            for j in 0..SAMPLES_PER_UNIT {
                out[out_pos] = left_buf[j];
                out[out_pos + 1] = right_buf[j];
                out_pos += 2;
            }
        } else {
            for j in 0..SAMPLES_PER_UNIT {
                let nib_lo = i16::from(data[4 * j + slot] & 0x0F);
                let delta_lo = i32::from((nib_lo << 12) >> shift_lo);
                out[out_pos] = state.left.step(delta_lo, filter_lo);
                out_pos += 1;
            }
            for j in 0..SAMPLES_PER_UNIT {
                let nib_hi = i16::from((data[4 * j + slot] >> 4) & 0x0F);
                let delta_hi = i32::from((nib_hi << 12) >> shift_hi);
                out[out_pos] = state.left.step(delta_hi, filter_hi);
                out_pos += 1;
            }
        }
    }
}

/// Decode one 128-byte 8-bit CD-ROM XA sound group into 112 `i16` PCM samples.
///
/// For stereo streams, the 112 samples are interleaved `[L, R, L, R, ...]`
/// (56 frames). For mono streams, all 112 samples belong to the single mono
/// channel.
pub fn decode_sound_group_8bit(
    group: &[u8; SOUND_GROUP_SIZE],
    stereo: bool,
    state: &mut XaState,
    out: &mut [i16; 112],
) {
    let header = &group[0..16];
    let data = &group[16..128];
    let mut out_pos = 0usize;

    if stereo {
        // 4 sound units paired as (0=L, 1=R) and (2=L, 3=R), 28 samples each.
        for pair in 0..2 {
            let slot_l = pair * 2;
            let slot_r = slot_l + 1;
            let hdr_l = header[4 + slot_l];
            let hdr_r = header[4 + slot_r];
            let shift_l = (hdr_l & 0x0F).min(8);
            let filter_l = ((hdr_l >> 4) & 0x0F) as usize;
            let shift_r = (hdr_r & 0x0F).min(8);
            let filter_r = ((hdr_r >> 4) & 0x0F) as usize;

            let mut left_buf = [0i16; SAMPLES_PER_UNIT];
            let mut right_buf = [0i16; SAMPLES_PER_UNIT];
            for j in 0..SAMPLES_PER_UNIT {
                let raw_l = i16::from(data[4 * j + slot_l] as i8);
                let delta_l = i32::from((raw_l << 8) >> shift_l);
                left_buf[j] = state.left.step(delta_l, filter_l);

                let raw_r = i16::from(data[4 * j + slot_r] as i8);
                let delta_r = i32::from((raw_r << 8) >> shift_r);
                right_buf[j] = state.right.step(delta_r, filter_r);
            }
            for j in 0..SAMPLES_PER_UNIT {
                out[out_pos] = left_buf[j];
                out[out_pos + 1] = right_buf[j];
                out_pos += 2;
            }
        }
    } else {
        for slot in 0..UNITS_PER_BLOCK {
            let hdr = header[4 + slot];
            let shift = (hdr & 0x0F).min(8);
            let filter = ((hdr >> 4) & 0x0F) as usize;
            for j in 0..SAMPLES_PER_UNIT {
                let raw = i16::from(data[4 * j + slot] as i8);
                let delta = i32::from((raw << 8) >> shift);
                out[out_pos] = state.left.step(delta, filter);
                out_pos += 1;
            }
        }
    }
}

/// Validate a 44-byte `RIFF`/`CDXA` wrapper header and return its underlying
/// raw Mode 2/2352 sector payload slice.
pub fn unwrap_riff_cdxa(bytes: &[u8]) -> Result<&[u8]> {
    if bytes.len() < RIFF_CDXA_HEADER_SIZE {
        return Err(Error::InvalidRiffCdxa("input shorter than 44-byte header"));
    }
    if &bytes[0..4] != b"RIFF" {
        return Err(Error::InvalidRiffCdxa("missing RIFF magic"));
    }
    let riff_size = u32::from_le_bytes([bytes[4], bytes[5], bytes[6], bytes[7]]) as usize;
    if riff_size < RIFF_CDXA_HEADER_SIZE - 8 {
        return Err(Error::InvalidRiffCdxa("RIFF chunk size too small"));
    }
    let riff_end = 8usize.checked_add(riff_size).ok_or(Error::Overflow)?;
    if riff_end > bytes.len() {
        return Err(Error::InvalidRiffCdxa("RIFF chunk extends past EOF"));
    }
    if &bytes[8..12] != b"CDXA" {
        return Err(Error::InvalidRiffCdxa("RIFF form type is not CDXA"));
    }
    if &bytes[12..16] != b"fmt " {
        return Err(Error::InvalidRiffCdxa("missing fmt chunk"));
    }
    let fmt_size = u32::from_le_bytes([bytes[16], bytes[17], bytes[18], bytes[19]]);
    if fmt_size != 16 {
        return Err(Error::InvalidRiffCdxa("unexpected fmt chunk size"));
    }
    if &bytes[36..40] != b"data" {
        return Err(Error::InvalidRiffCdxa("missing data chunk at offset 36"));
    }
    let data_size = u32::from_le_bytes([bytes[40], bytes[41], bytes[42], bytes[43]]) as usize;
    if data_size == 0 || data_size % RAW_SECTOR_SIZE != 0 {
        return Err(Error::InvalidRiffCdxa(
            "data chunk size is not a non-empty multiple of 2352",
        ));
    }
    let data_end = RIFF_CDXA_HEADER_SIZE
        .checked_add(data_size)
        .ok_or(Error::Overflow)?;
    if data_end > riff_end {
        return Err(Error::InvalidRiffCdxa(
            "data chunk extends past RIFF container",
        ));
    }
    let payload = &bytes[RIFF_CDXA_HEADER_SIZE..data_end];
    if payload[..12] != SYNC_PATTERN || payload[15] != 2 {
        return Err(Error::InvalidRiffCdxa(
            "payload does not begin with a raw Mode 2/2352 sector",
        ));
    }
    Ok(payload)
}

/// Stateful streaming CD-ROM XA ADPCM decoder (`alloc` feature).
#[cfg(feature = "alloc")]
#[derive(Debug, Clone)]
pub struct XaDecoder {
    coding: XaCoding,
    state: XaState,
    samples: Vec<i16>,
}

#[cfg(feature = "alloc")]
impl XaDecoder {
    /// Create a new decoder for the given [`XaCoding`].
    pub fn new(coding: XaCoding) -> Self {
        Self {
            coding,
            state: XaState::new(),
            samples: Vec::new(),
        }
    }

    /// Return the configured [`XaCoding`].
    pub const fn coding(&self) -> XaCoding {
        self.coding
    }

    /// Decode one Form 2 audio sector payload (at least `2304` bytes, typically
    /// `2324` bytes) and return its PCM samples while advancing internal
    /// predictor state.
    pub fn decode_sector(&mut self, payload: &[u8]) -> Result<Vec<i16>> {
        if payload.len() < SOUND_DATA_SIZE {
            return Err(Error::UnexpectedEof);
        }
        let samples_per_group = if self.coding.bits_per_sample == 8 {
            112
        } else {
            224
        };
        let mut out = Vec::with_capacity(SOUND_GROUPS_PER_SECTOR * samples_per_group);
        for chunk in payload[..SOUND_DATA_SIZE].chunks_exact(SOUND_GROUP_SIZE) {
            let group: &[u8; SOUND_GROUP_SIZE] = chunk
                .try_into()
                .map_err(|_| Error::InvalidXa("short sound group"))?;
            if self.coding.bits_per_sample == 8 {
                let mut buf = [0i16; 112];
                decode_sound_group_8bit(group, self.coding.stereo, &mut self.state, &mut buf);
                out.extend_from_slice(&buf);
            } else {
                let mut buf = [0i16; 224];
                decode_sound_group_4bit(group, self.coding.stereo, &mut self.state, &mut buf);
                out.extend_from_slice(&buf);
            }
        }
        Ok(out)
    }

    /// Decode one Form 2 audio sector payload and append its PCM samples to the
    /// internal buffer.
    pub fn push_sector(&mut self, payload: &[u8]) -> Result<()> {
        let mut sector_pcm = self.decode_sector(payload)?;
        self.samples.append(&mut sector_pcm);
        Ok(())
    }

    /// Consume the decoder and return all accumulated interleaved `i16` PCM samples.
    pub fn finish(self) -> Vec<i16> {
        self.samples
    }
}

/// Demultiplexing key identifying an interleaved CD-ROM XA audio stream.
#[cfg(feature = "alloc")]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct XaStreamKey {
    /// Subheader file number.
    pub file: u8,
    /// Subheader channel number.
    pub channel: u8,
}

/// Summary metadata for one demultiplexed `(file, channel)` audio stream.
#[cfg(feature = "alloc")]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct XaStreamInfo {
    /// `(file, channel)` stream key.
    pub key: XaStreamKey,
    /// Audio coding parameters read from the stream's subheader.
    pub coding: XaCoding,
    /// Number of Mode 2 Form 2 audio sectors belonging to this stream.
    pub sector_count: usize,
}

#[cfg(feature = "alloc")]
struct IndexedChannel {
    coding: XaCoding,
    sector_offsets: Vec<usize>,
}

/// Interleaved Mode 2 Form 2 CD-ROM XA audio stream demultiplexer (`alloc` feature).
///
/// Accepts raw Mode 2/2352 streams, stripped Mode 2/2336 streams, or
/// `RIFF`/`CDXA`-wrapped Mode 2/2352 containers.
#[cfg(feature = "alloc")]
pub struct XaStream {
    data: Vec<u8>,
    mode: SectorMode,
    sector_count: usize,
    channels: BTreeMap<XaStreamKey, IndexedChannel>,
}

#[cfg(feature = "alloc")]
impl XaStream {
    /// Parse and index all Mode 2 Form 2 audio streams in `bytes`.
    pub fn parse(bytes: &[u8]) -> Result<Self> {
        let payload = match unwrap_riff_cdxa(bytes) {
            Ok(unwrapped) => unwrapped,
            Err(_) => bytes,
        };
        if payload.is_empty() {
            return Err(Error::InvalidXa("empty XA stream"));
        }
        let mode = if payload.len() % RAW_SECTOR_SIZE == 0
            && payload[..12] == SYNC_PATTERN
            && payload[15] == 2
        {
            SectorMode::Mode2_2352
        } else if payload.len() % MODE2_2336_SIZE == 0 {
            SectorMode::Mode2_2336
        } else {
            return Err(Error::InvalidSectorSize(payload.len()));
        };

        let stride = mode.stride();
        let sub_off = mode
            .subheader_offset()
            .ok_or(Error::InvalidXa("Mode 2 layout requires subheader"))?;
        let user_off = mode.user_offset();
        let sector_count = payload.len() / stride;
        let mut channels: BTreeMap<XaStreamKey, IndexedChannel> = BTreeMap::new();

        for idx in 0..sector_count {
            let base = idx * stride;
            let sec = &payload[base..base + stride];
            if mode == SectorMode::Mode2_2352 && (sec[..12] != SYNC_PATTERN || sec[15] != 2) {
                return Err(Error::BadSync);
            }
            let sub_bytes: &[u8; 8] = sec[sub_off..sub_off + 8]
                .try_into()
                .map_err(|_| Error::InvalidXa("short subheader"))?;
            let sub = Subheader::from_bytes(sub_bytes)?;
            if !sub.is_form2() || !sub.is_audio() {
                continue;
            }
            let key = XaStreamKey {
                file: sub.file,
                channel: sub.channel,
            };
            let coding = XaCoding::from_byte(sub.coding);
            let entry = channels.entry(key).or_insert_with(|| IndexedChannel {
                coding,
                sector_offsets: Vec::new(),
            });
            entry.sector_offsets.push(base + user_off);
        }

        if channels.is_empty() {
            return Err(Error::InvalidXa(
                "no Mode 2 Form 2 audio sectors found in stream",
            ));
        }

        Ok(Self {
            data: payload.to_vec(),
            mode,
            sector_count,
            channels,
        })
    }

    /// Physical sector framing of the parsed stream (`Mode2_2352` or `Mode2_2336`).
    pub const fn mode(&self) -> SectorMode {
        self.mode
    }

    /// Total number of sectors in the underlying stream buffer.
    pub const fn sector_count(&self) -> usize {
        self.sector_count
    }

    /// List all indexed `(file, channel)` audio streams.
    pub fn streams(&self) -> Vec<XaStreamInfo> {
        self.channels
            .iter()
            .map(|(&key, ch)| XaStreamInfo {
                key,
                coding: ch.coding,
                sector_count: ch.sector_offsets.len(),
            })
            .collect()
    }

    /// Decode the selected `(file, channel)` stream to `(XaCoding, Vec<i16>)`.
    pub fn decode(&self, file: u8, channel: u8) -> Result<(XaCoding, Vec<i16>)> {
        let key = XaStreamKey { file, channel };
        let indexed = self
            .channels
            .get(&key)
            .ok_or(Error::InvalidXa("requested (file, channel) not found"))?;
        let mut decoder = XaDecoder::new(indexed.coding);
        for &offset in &indexed.sector_offsets {
            let payload = self
                .data
                .get(offset..offset + FORM2_PAYLOAD_SIZE)
                .ok_or(Error::UnexpectedEof)?;
            decoder.push_sector(payload)?;
        }
        Ok((indexed.coding, decoder.finish()))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sector::{SUBMODE_AUDIO, SUBMODE_FORM2, SUBMODE_REALTIME};

    #[test]
    fn coding_byte_round_trip() {
        for byte in [0x00u8, 0x01, 0x04, 0x05, 0x10, 0x11, 0x41, 0x55] {
            let coding = XaCoding::from_byte(byte);
            assert_eq!(coding.to_byte(), byte);
        }
    }

    #[test]
    fn decode_4bit_group_matches_hand_computed_recurrence() {
        let mut group = [0u8; SOUND_GROUP_SIZE];
        group[4] = 0x10; // slot 0 low nibble: filter 1 (K0=60), shift 0
        group[16] = 0x01; // row 0, slot 0: low nibble = 1
        let mut state = XaState::new();
        let mut out = [0i16; 224];
        decode_sound_group_4bit(&group, false, &mut state, &mut out);
        assert_eq!(out[0], 4096);
        assert_eq!(out[1], 3840);
    }

    #[test]
    fn decode_4bit_stereo_deinterleaves_low_and_high_nibbles() {
        let mut group = [0u8; SOUND_GROUP_SIZE];
        group[4] = 0x00; // left: filter 0, shift 0
        group[5] = 0x00; // right: filter 0, shift 0
        group[16] = 0x5A; // low nibble = 0xA (-6), high nibble = 0x5 (+5)
        let mut state = XaState::new();
        let mut out = [0i16; 224];
        decode_sound_group_4bit(&group, true, &mut state, &mut out);
        assert_eq!(out[0], (-6i16) << 12);
        assert_eq!(out[1], 5i16 << 12);
    }

    #[test]
    fn decode_8bit_group_mono_and_stereo() {
        let mut group = [0u8; SOUND_GROUP_SIZE];
        group[4] = 0x00; // filter 0, shift 0
        group[5] = 0x00;
        group[16] = 0x10; // +16 << 8 = 4096
        group[17] = 0xF0; // -16 << 8 = -4096
        let mut state = XaState::new();
        let mut out = [0i16; 112];
        decode_sound_group_8bit(&group, true, &mut state, &mut out);
        assert_eq!(out[0], 4096);
        assert_eq!(out[1], -4096);
    }

    #[cfg(feature = "alloc")]
    #[test]
    fn riff_cdxa_unwrap_and_xa_stream_demux() {
        fn make_raw_xa_sector(file: u8, channel: u8, coding: u8) -> [u8; RAW_SECTOR_SIZE] {
            let mut raw = [0u8; RAW_SECTOR_SIZE];
            raw[..12].copy_from_slice(&SYNC_PATTERN);
            raw[15] = 2;
            let sub = Subheader::new(
                file,
                channel,
                SUBMODE_AUDIO | SUBMODE_FORM2 | SUBMODE_REALTIME,
                coding,
            );
            raw[16..24].copy_from_slice(&sub.to_bytes());
            for g in 0..SOUND_GROUPS_PER_SECTOR {
                let base = 24 + g * SOUND_GROUP_SIZE;
                raw[base..base + 16].fill(0x0C);
            }
            raw
        }

        let mut sectors = Vec::new();
        sectors.extend_from_slice(&make_raw_xa_sector(1, 0, 0x01));
        sectors.extend_from_slice(&make_raw_xa_sector(1, 1, 0x04));
        sectors.extend_from_slice(&make_raw_xa_sector(1, 0, 0x01));

        // Wrap in RIFF/CDXA
        let data_size = sectors.len() as u32;
        let riff_size = (RIFF_CDXA_HEADER_SIZE as u32 - 8) + data_size;
        let mut wrapped = Vec::with_capacity(RIFF_CDXA_HEADER_SIZE + sectors.len());
        wrapped.extend_from_slice(b"RIFF");
        wrapped.extend_from_slice(&riff_size.to_le_bytes());
        wrapped.extend_from_slice(b"CDXA");
        wrapped.extend_from_slice(b"fmt ");
        wrapped.extend_from_slice(&16u32.to_le_bytes());
        wrapped.extend_from_slice(&[0u8; 16]);
        wrapped.extend_from_slice(b"data");
        wrapped.extend_from_slice(&data_size.to_le_bytes());
        wrapped.extend_from_slice(&sectors);

        assert_eq!(unwrap_riff_cdxa(&wrapped).unwrap(), sectors.as_slice());

        let stream = XaStream::parse(&wrapped).unwrap();
        assert_eq!(stream.mode(), SectorMode::Mode2_2352);
        assert_eq!(stream.sector_count(), 3);
        let infos = stream.streams();
        assert_eq!(infos.len(), 2);
        assert_eq!(infos[0].sector_count, 2);
        assert_eq!(infos[1].sector_count, 1);

        let (coding0, pcm0) = stream.decode(1, 0).unwrap();
        assert!(coding0.stereo);
        assert_eq!(coding0.sample_rate_hz, 37_800);
        assert_eq!(pcm0.len(), 2 * 4032);
        assert!(pcm0.iter().all(|&s| s == 0));

        let (coding1, pcm1) = stream.decode(1, 1).unwrap();
        assert!(!coding1.stereo);
        assert_eq!(coding1.sample_rate_hz, 18_900);
        assert_eq!(pcm1.len(), 4032);
    }
}
