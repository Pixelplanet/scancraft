//! Is a captured IR pair row-aligned (rectified)? For each vertical offset `dy` of the right
//! image, finds every textured left pixel's best census match along the row (any disparity in a
//! wide range) and reports the mean best cost per image region. A rectified pair has its minimum
//! at `dy = 0` everywhere; an unrectified one shows a region-dependent offset.
//! `cargo run --release -p scancraft-driver-revopoint --example vcsel_rowcheck -- <capture.bin> [frame] [dmin] [dmax]`

use scancraft_driver_revopoint::{FrameDecoder, FrameLayout, PacketParser};
use scancraft_stereo::census::{census_9x7, texture_mask};

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let capture = std::fs::read(&args[1]).expect("capture");
    let pick: usize = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(5);
    let dmin: i64 = args.get(3).and_then(|s| s.parse().ok()).unwrap_or(-320);
    let dmax: i64 = args.get(4).and_then(|s| s.parse().ok()).unwrap_or(320);
    let mut parser = PacketParser::default();
    parser.push(&capture);
    let layout = if std::env::var("SRC").is_ok_and(|v| v == "depthir") { FrameLayout::DepthIrPair } else { FrameLayout::IrPair };
    let mut decoder = FrameDecoder::new(layout, 640, 512);
    let mut k = 0;
    let frame = loop {
        let p = parser.next_packet().expect("frame index beyond capture");
        if let Ok(f) = decoder.decode(&p) {
            if k == pick {
                break f;
            }
            k += 1;
        }
    };
    let (w, h) = (640usize, 512usize);
    let (mut cl, mut cr, mut tex) = (Vec::new(), Vec::new(), Vec::new());
    census_9x7(&frame.ir_left.data, w, h, &mut cl);
    census_9x7(&frame.ir_right.data, w, h, &mut cr);
    texture_mask(&frame.ir_left.data, w, h, 3.0, &mut tex);

    // 3x3 grid of regions.
    println!("mean best census cost per region (rows: image thirds top..bottom, cols: left..right)");
    for dy in -8i64..=8 {
        let mut acc = [[(0u64, 0u64); 3]; 3];
        let mut disp_hist = std::collections::HashMap::<i64, u32>::new();
        for y in (16..h - 16).step_by(3) {
            let yr = y as i64 + dy;
            for x in (8..w - 8).step_by(3) {
                if !tex[y * w + x] {
                    continue;
                }
                let c = cl[y * w + x];
                let mut best = (u32::MAX, 0);
                for d in dmin..=dmax {
                    let xr = x as i64 - d;
                    if xr < 4 || xr >= w as i64 - 4 {
                        continue;
                    }
                    let cost = (c ^ cr[yr as usize * w + xr as usize]).count_ones();
                    if cost < best.0 {
                        best = (cost, d);
                    }
                }
                if best.0 == u32::MAX {
                    continue;
                }
                let cell = &mut acc[y * 3 / h][x * 3 / w];
                cell.0 += best.0 as u64;
                cell.1 += 1;
                if best.0 < 12 {
                    *disp_hist.entry(best.1 / 8 * 8).or_default() += 1;
                }
            }
        }
        let row: Vec<String> = acc
            .iter()
            .map(|r| r.iter().map(|&(s, n)| format!("{:5.2}", s as f64 / n.max(1) as f64)).collect::<Vec<_>>().join(" "))
            .collect();
        let mut top: Vec<(i64, u32)> = disp_hist.into_iter().collect();
        top.sort_by_key(|&(_, n)| std::cmp::Reverse(n));
        println!("dy {dy:+3}: {} | strong-match disparities (bin 8): {:?}", row.join(" | "), &top[..top.len().min(4)]);
    }
}
