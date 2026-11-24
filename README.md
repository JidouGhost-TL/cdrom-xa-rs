# cdrom-xa

Pure-Rust, zero-dependency CD-ROM and CD-ROM XA library implementing Yellow Book sector framing, EDC/ECC error-correction math, CUE sheet parsing, ISO-9660 filesystem traversal, XA-ADPCM decoding, and non-destructive ISO-9660 / CD-ROM XA disc repacking.

- **Zero dependencies** (`[dependencies]` and `[dev-dependencies]` are empty)
- **`#![forbid(unsafe_code)]`**
- **`no_std` compatible** (`default = ["std"]`, optional `alloc` feature)
- **MSRV `1.70`** (Edition 2021)
- **Platform-agnostic** — works with any Yellow Book / CD-ROM XA optical disc image

## Installation

```toml
[dependencies]
cdrom-xa = "0.1"
```

For `no_std` environments:

```toml
# Bare no_std (zero allocations: error, sector, xa sound-group decoding & RIFF/CDXA unwrapping)
cdrom-xa = { version = "0.1", default-features = false }

# no_std + alloc (adds cue, iso, repack, XaDecoder, XaStream)
cdrom-xa = { version = "0.1", default-features = false, features = ["alloc"] }
```

## Modules

- `sector` — Physical sector framing (`Mode1_2352`, `Mode2_2352`, `Mode2_2336`, `Cooked2048`), MSF/LBA/BCD addressing, CD-ROM XA subheaders, Yellow Book CRC-32 EDC (`0xD8018001`), and Reed-Solomon P/Q ECC over GF(2^8) (`0x11D`).
- `cue` — Multi-track / multi-file CUE sheet parser (`FILE`, `TRACK`, `INDEX`, `PREGAP`, `POSTGAP`).
- `iso` — ISO-9660 Primary Volume Descriptor, directory tree walk with CD-ROM XA System Use attributes, cooked and raw-sector file extraction.
- `xa` — 4-bit and 8-bit CD-ROM XA ADPCM sound-group decoder, interleaved Mode 2 Form 2 audio demuxer (`XaStream`), and `RIFF`/`CDXA` container unwrapper.
- `repack` — Non-destructive ISO-9660 / CD-ROM XA image repacker supporting all four `SectorMode` layouts (`Mode1_2352`, `Mode2_2352`, `Mode2_2336`, `Cooked2048`), preserving untouched sectors (including Form 2 audio/video) byte-for-byte while regenerating EDC/ECC on modified and appended sectors.

## Usage

### Reading an ISO-9660 image from a CUE/BIN or ISO

```rust
use cdrom_xa::iso::IsoImage;

fn main() -> cdrom_xa::Result<()> {
    let image = IsoImage::open("disc.cue")?;
    println!("Volume ID: {}", image.pvd().volume_id);

    for record in image.records()? {
        println!("{} ({} bytes @ LBA {})", record.path, record.size, record.lba);
    }

    if let Some(entry) = image.find("/README.TXT")? {
        let bytes = image.read_file(&entry)?;
        println!("Read {} bytes", bytes.len());
    }

    Ok(())
}
```

### Building, verifying, and regenerating Mode 2 Form 1 EDC/ECC

```rust
use cdrom_xa::sector::{
    build_mode2_form1_sector, regenerate_mode2_form1_edc_ecc, verify_mode2_form1_edc_ecc,
    Subheader, COOKED_SECTOR_SIZE, SUBMODE_DATA, SUBMODE_EOF, SUBMODE_EOR,
};

fn main() -> cdrom_xa::Result<()> {
    let user_data = [0x42u8; COOKED_SECTOR_SIZE];
    let subheader = Subheader {
        file: 0,
        channel: 0,
        submode: SUBMODE_DATA | SUBMODE_EOR | SUBMODE_EOF,
        coding: 0,
    };

    let mut raw = build_mode2_form1_sector(16, &user_data, subheader)?;
    verify_mode2_form1_edc_ecc(&raw)?;

    // Modify user data and regenerate EDC + Reed-Solomon P/Q ECC:
    raw[24] ^= 0xFF;
    assert!(verify_mode2_form1_edc_ecc(&raw).is_err());
    regenerate_mode2_form1_edc_ecc(&mut raw);
    verify_mode2_form1_edc_ecc(&raw)?;

    Ok(())
}
```

### Decoding an interleaved CD-ROM XA ADPCM stream

```rust
use cdrom_xa::xa::XaStream;

fn main() -> cdrom_xa::Result<()> {
    let raw_bytes = std::fs::read("audio.xa")?;
    let stream = XaStream::from_vec(raw_bytes)?;

    for info in stream.streams() {
        let (coding, pcm) = stream.decode(info.key.file, info.key.channel)?;
        println!(
            "file={} ch={}: {} Hz, {} ch, {} samples",
            info.key.file,
            info.key.channel,
            coding.sample_rate_hz,
            coding.channels(),
            pcm.len()
        );
    }

    Ok(())
}
```

### Repacking a CD-ROM / CD-ROM XA disc image

```rust
use cdrom_xa::repack::IsoRepacker;

fn main() -> cdrom_xa::Result<()> {
    let mut repacker = IsoRepacker::open("disc.cue")?;
    repacker.replace("/DATA/SCRIPT.BIN", vec![0x55u8; 4096]);

    let mut out = std::fs::File::create("patched.bin")?;
    let report = repacker.write_to(&mut out)?;
    println!(
        "Replaced {} files ({} relocated), {} -> {} sectors",
        report.replaced_files, report.relocated_files, report.original_sectors, report.final_sectors
    );

    Ok(())
}
```

## Specification Notes & Caveats

- **Mode 2 Form 2 files (`XA` audio/video):** `IsoImage::read_file` reads cooked 2048-byte sectors and returns `Error::InvalidIso` if a sector is Mode 2 Form 2 (2324-byte payload). Use `IsoImage::read_file_raw_sectors` (or `IsoImage::read_sector_raw`) for Form 2 / interleaved XA files, then pass the raw sectors to `XaStream::from_vec`.
- **Mode 2 Form 2 EDC verification:** The CD-ROM XA specification allows a Form 2 sector's 4-byte EDC field to be zeroed (`0x0000_0000`). `verify_mode2_form2_edc` strictly checks the computed CRC-32 against the stored bytes, whereas `verify_mode2_form2_edc_allow_zero` accepts an all-zero EDC field.
- **Repacker layouts:** `IsoRepacker` supports all four `SectorMode` layouts (`Mode1_2352`, `Mode2_2352`, `Mode2_2336`, and `Cooked2048`).
- **Multi-track single-BIN `.cue` images:** When `IsoImage::open` opens a `.cue` sheet whose data track shares a single `.bin` file with subsequent tracks (e.g., CD-DA audio), the mounted slice is clamped to the data track (`IsoImage::has_trailing_tracks() == true`). `IsoRepacker` supports **in-place** replacements that fit within the file's existing sector span, but rejects replacements that require relocating/appending sectors with `Error::InvalidIso` (which would overwrite the following tracks).
- **ISO-9660 Path Table invariant on repack:** `IsoRepacker` only replaces existing files; it never adds, removes, or moves directories, so the existing ISO-9660 L-type and M-type Path Tables remain valid without modification.

## Feature Flags

- `std` (default): Enables `std::io` / filesystem helpers (`IsoImage::open`, `IsoImage::from_reader`, `IsoImage::from_reader_with_mode`, `IsoRepacker::open`, `IsoRepacker::write_to`), `Error::Io` / `From<std::io::Error>`, and `std::error::Error`. Implies `alloc`.
- `alloc`: Enables heap-backed modules (`cue`, `iso`, `repack`, `XaDecoder`, `XaStream`) in `no_std` environments.
- `--no-default-features`: Bare `no_std` core (`error`, `sector`, `xa` sound-group decoder and `unwrap_riff_cdxa`) with zero allocations.

## License

Licensed under either of [Apache License, Version 2.0](LICENSE-APACHE) or [MIT License](LICENSE-MIT) at your option.
