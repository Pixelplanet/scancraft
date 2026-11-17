//! Train a Gaussian splat on the synthetic object scan and write the 3DGS `.ply`.
//!
//! ```text
//! cargo run --release -p scancraft-splat --example train_splat -- [--iters N] [--out DIR] [--res WxH] [--sh N] [--export]
//! ```
//!
//! Every 8th view is held out; PSNR/SSIM on those views (and on a sample of training views) is
//! printed at the end. `--export` also writes the inputs as a nerfstudio + COLMAP dataset.
//! Outputs go to `--out` (default `target/splat-demo`): `splat.ply`, `heldout_*.png`
//! (ground truth | render side by side), `dataset/` with `--export`.

use scancraft_splat::{SplatJob, SplatTrainConfig, export, synthetic};
use std::path::PathBuf;
use std::time::Instant;

fn main() {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info,wgpu_core=warn,wgpu_hal=warn,naga=warn,cubecl=warn"))
        .init();
    let mut iters = 3000u32;
    let mut out = PathBuf::from("target/splat-demo");
    let (mut w, mut h) = (640u32, 480u32);
    let mut sh = 3u32;
    let mut do_export = false;
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        match a.as_str() {
            "--iters" => iters = args.next().and_then(|v| v.parse().ok()).expect("--iters N"),
            "--out" => out = args.next().map(PathBuf::from).expect("--out DIR"),
            "--sh" => sh = args.next().and_then(|v| v.parse().ok()).expect("--sh N"),
            "--res" => {
                let v = args.next().expect("--res WxH");
                let (a, b) = v.split_once('x').expect("--res WxH");
                (w, h) = (a.parse().unwrap(), b.parse().unwrap());
            }
            "--export" => do_export = true,
            other => panic!("unknown argument {other}"),
        }
    }

    let scene = synthetic::SceneConfig { width: w, height: h, focal: 760.0 * w as f32 / 640.0, ..Default::default() };
    let t = Instant::now();
    let dataset = synthetic::dataset(&scene);
    println!("synthetic scene: {} views {}x{}, {} cloud points, rendered in {:.2?}", dataset.images.len(), w, h, dataset.init_cloud.points.len(), t.elapsed());

    if do_export {
        let dir = out.join("dataset");
        export::export_dataset(&dataset, &dir, &Default::default()).expect("export");
        println!("exported nerfstudio + COLMAP dataset to {}", dir.display());
    }

    let cfg = SplatTrainConfig { iterations: iters, sh_degree: sh, eval_every: Some(8), progress_every: 250, ..Default::default() };
    let eval_every = cfg.eval_every.unwrap();
    let mut job = SplatJob::spawn(dataset.clone(), cfg);
    let mut last = 0;
    while !job.is_finished() {
        std::thread::sleep(std::time::Duration::from_millis(100));
        if let Some(p) = job.poll() {
            if p.iteration != last {
                last = p.iteration;
                println!(
                    "  {:?} {}/{}  splats {}  loss {}  {:.1?}",
                    p.phase,
                    p.iteration,
                    p.total_iterations,
                    p.num_splats,
                    p.loss.map_or("-".into(), |l| format!("{l:.4}")),
                    p.elapsed
                );
            }
        }
    }
    let splat = job.join().expect("training failed");
    let r = &splat.report;
    println!(
        "trained {} iters in {:.2?} (prep {:.2?}); {} train / {} held-out views; init {} of {} points covered -> {} splats; final {} splats",
        r.iterations,
        r.train_time,
        r.prepare_time,
        r.train_views,
        r.eval_views,
        r.init_points_covered,
        r.init_points_in,
        r.init_splats,
        r.final_splats
    );
    if let Some(e) = r.eval {
        println!("held-out views: PSNR {:.2} dB  SSIM {:.4}  ({} views)", e.psnr, e.ssim, e.views);
    }
    if let Some(e) = r.train_eval {
        println!("training views: PSNR {:.2} dB  SSIM {:.4}  ({} views)", e.psnr, e.ssim, e.views);
    }

    std::fs::create_dir_all(&out).unwrap();
    let ply = out.join("splat.ply");
    splat.save_ply(&ply).expect("ply");
    println!("wrote {} ({} bytes)", ply.display(), std::fs::metadata(&ply).unwrap().len());

    // Side-by-side ground truth | render for a few held-out views, plus an independent PSNR on
    // the object (mask) pixels.
    for (i, img) in dataset.images.iter().enumerate().filter(|(i, _)| i % eval_every == eval_every / 2).take(3) {
        let t = Instant::now();
        let render = splat.render_view(img, [0.0, 0.0, 0.0]).unwrap();
        let dt = t.elapsed();
        let rgb: Vec<u8> = render.pixels().flat_map(|p| [p[0], p[1], p[2]]).collect();
        let psnr_all = scancraft_splat::psnr_rgb8(&img.rgb, &rgb, None);
        let psnr_obj = scancraft_splat::psnr_rgb8(&img.rgb, &rgb, img.mask.as_deref());
        let mut both = image::RgbImage::new(img.width * 2, img.height);
        image::imageops::replace(&mut both, &image::RgbImage::from_raw(img.width, img.height, img.rgb.clone()).unwrap(), 0, 0);
        image::imageops::replace(&mut both, &image::RgbImage::from_raw(img.width, img.height, rgb).unwrap(), img.width as i64, 0);
        let path = out.join(format!("heldout_{i:02}.png"));
        both.save(&path).unwrap();
        println!("  {} PSNR {psnr_all:.2} dB (object only {psnr_obj:.2} dB), render {dt:.1?} -> {}", img.name, path.display());
    }
}
