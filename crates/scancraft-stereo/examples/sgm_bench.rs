//! Matcher timing on a synthetic speckle pair:
//! `cargo run --release -p scancraft-stereo --example sgm_bench -- [width] [height] [disparities] [diagonals 0|1]`

use scancraft_stereo::{SgmMatcher, SgmParams};
use std::time::Instant;

fn main() {
    env_logger::init();
    let args: Vec<String> = std::env::args().collect();
    let w: usize = args.get(1).and_then(|s| s.parse().ok()).unwrap_or(640);
    let h: usize = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(512);
    let d: usize = args.get(3).and_then(|s| s.parse().ok()).unwrap_or(192);
    let diag = args.get(4).map(|s| s != "0").unwrap_or(true);

    let mut s = 1u32;
    let left: Vec<u8> = (0..w * h)
        .map(|_| {
            s ^= s << 13;
            s ^= s >> 17;
            s ^= s << 5;
            if s % 6 == 0 { 200 } else { 20 + (s % 30) as u8 }
        })
        .collect();
    let shift = 40;
    let right: Vec<u8> = (0..w * h).map(|i| if i % w + shift < w { left[i + shift] } else { 0 }).collect();

    let mut m = SgmMatcher::new(SgmParams { min_disparity: 0, num_disparities: d, diagonal_paths: diag, ..Default::default() });
    for i in 0..6 {
        let t = Instant::now();
        let out = m.compute(&left, &right, w, h);
        println!("run {i}: {:.1} ms, {} valid", t.elapsed().as_secs_f64() * 1e3, out.valid_count());
    }
}
