//! Decompresses every packet of a recording (wire framing) into `<out_dir>/f<NNN>.raw`.
//!
//! `cargo run -p scancraft-driver-revopoint --example dump_frames -- <recording.bin> <out_dir> [max_frames]`

use scancraft_driver_revopoint::media::PacketParser;
use scancraft_driver_revopoint::quicklz::QuickLzDecompressor;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let input = args.get(1).expect("usage: dump_frames <recording.bin> <out_dir> [max_frames]");
    let out = std::path::PathBuf::from(args.get(2).expect("out_dir"));
    let max: usize = args.get(3).and_then(|s| s.parse().ok()).unwrap_or(usize::MAX);
    std::fs::create_dir_all(&out).unwrap();
    let mut parser = PacketParser::default();
    parser.push(&std::fs::read(input).unwrap());
    let mut qlz = QuickLzDecompressor::new();
    let mut buf = vec![0u8; 8 << 20];
    let (mut n, mut errors) = (0usize, 0usize);
    while let Some(packet) = parser.next_packet() {
        if n >= max {
            break;
        }
        match qlz.decompress(&packet, &mut buf) {
            Ok(len) => std::fs::write(out.join(format!("f{n:03}.raw")), &buf[..len]).unwrap(),
            Err(e) => {
                errors += 1;
                eprintln!("packet {n}: {e}");
            }
        }
        n += 1;
    }
    println!("{n} packets, {errors} decode errors");
}
