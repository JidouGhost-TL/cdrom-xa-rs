# cdrom-xa

Pure-Rust, zero-dependency CD-ROM and CD-ROM XA library implementing Yellow Book sector framing, EDC/ECC error-correction math, CUE sheet parsing, ISO-9660 filesystem traversal, XA-ADPCM decoding, and non-destructive Mode 2/2352 disc repacking.

- **Zero dependencies** (`[dependencies]` and `[dev-dependencies]` are empty)
- **`#![forbid(unsafe_code)]`**
- **`no_std` compatible** (`default = ["std"]`, optional `alloc` feature)
- **Platform-agnostic** — works with any Yellow Book / CD-ROM XA optical disc image

## Modules

- `sector` — Physical sector framing (`Mode1_2352`, `Mode2_2352`, `Mode2_2336`, `Cooked2048`), MSF/LBA/BCD addressing, CD-ROM XA subheaders, Yellow Book CRC-32 EDC (`0xD8018001`), and Reed-Solomon P/Q ECC over GF(2^8) (`0x11D`).
- `cue` — Multi-track / multi-file CUE sheet parser (`FILE`, `TRACK`, `INDEX`, `PREGAP`, `POSTGAP`).
- `iso` — ISO-9660 Primary Volume Descriptor, directory tree walk with CD-ROM XA System Use attributes, cooked and raw-sector file extraction.
- `xa` — 4-bit and 8-bit CD-ROM XA ADPCM sound-group decoder, interleaved Mode 2 Form 2 audio demuxer (`XaStream`), and `RIFF`/`CDXA` container unwrapper.
- `repack` — Non-destructive ISO-9660 / Mode 2/2352 image repacker preserving untouched sectors (including Form 2 audio/video) byte-for-byte while regenerating EDC/ECC on modified and appended sectors.

## Usage

### Reading an ISO-9660 image from a CUE/BIN or ISO

```rust,no_run
use cdrom_xa::iso::IsoImage;

let image = IsoImage::open("disc.cue")?;
println!("Volume ID: {}", image.pvd().volume_id);

for record in image.records()? {
    println!("{} ({} bytes @ LBA {})", record.path, record.size, record.lba);
}

if let Some(entry) = image.find("/README.TXT")? {
    let bytes = image.read_file(&entry)?;
    println!("Read {} bytes", bytes.len());
}
# Ok::<(), cdrom_xa::Error>(())
```

### Building, verifying, and regenerating Mode 2 Form 1 EDC/ECC

```rust
use cdrom_xa::sector::{
    build_mode2_form1_sector, regenerate_mode2_form1_edc_ecc, verify_mode2_form1_edc_ecc,
    Subheader, COOKED_SECTOR_SIZE, SUBMODE_DATA, SUBMODE_EOF, SUBMODE_EOR,
};

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
# Ok::<(), cdrom_xa::Error>(())
```

### Decoding an interleaved CD-ROM XA ADPCM stream

```rust,no_run
use cdrom_xa::xa::XaStream;

let raw_bytes = std::fs::read("audio.xa")?;
let stream = XaStream::parse(&raw_bytes)?;

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
# Ok::<(), cdrom_xa::Error>(())
```

### Repacking a Mode 2/2352 disc image

```rust,no_run
use cdrom_xa::repack::IsoRepacker;

let mut repacker = IsoRepacker::open("disc.cue")?;
repacker.replace("/DATA/SCRIPT.BIN", vec![0x55u8; 4096]);

let mut out = std::fs::File::create("patched.bin")?;
let report = repacker.write_to(&mut out)?;
println!(
    "Replaced {} files ({} relocated), {} -> {} sectors",
    report.replaced_files, report.relocated_files, report.original_sectors, report.final_sectors
);
# Ok::<(), cdrom_xa::Error>(())
```

## Feature Flags

- `std` (default): Enables `std::io` / filesystem helpers (`IsoImage::open`, `IsoImage::from_reader`, `IsoRepacker::open`, `IsoRepacker::write_to`, `std::error::Error`). Implies `alloc`.
- `alloc`: Enables heap-backed modules (`cue`, `iso`, `repack`, `XaDecoder`, `XaStream`) in `no_std` environments.
- `--no-default-features`: Bare `no_std` core (`error`, `sector`, `xa` sound-group decoder and `unwrap_riff_cdxa`) with zero allocations.

## License

Licensed under either of [Apache License, Version 2.0](LICENSE-APACHE) or [MIT License](LICENSE-MIT) at your option.
