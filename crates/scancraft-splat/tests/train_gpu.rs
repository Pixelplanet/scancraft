//! End-to-end training on the synthetic scene. Needs a GPU, so it is ignored by default:
//! `cargo test -p scancraft-splat --test train_gpu -- --ignored`.

use scancraft_splat::{Phase, SplatError, SplatJob, SplatTrainConfig, synthetic};

fn small_scene() -> scancraft_splat::SplatDataset {
    synthetic::dataset(&synthetic::SceneConfig { width: 320, height: 240, focal: 380.0, views_per_ring: 12, init_points: 20_000, ..Default::default() })
}

#[test]
#[ignore = "needs a GPU"]
fn trains_and_generalises_to_held_out_views() {
    let ds = small_scene();
    let cfg = SplatTrainConfig { iterations: 1000, sh_degree: 1, eval_every: Some(6), progress_every: 100, ..Default::default() };
    let mut job = SplatJob::spawn(ds.clone(), cfg);
    let mut phases = Vec::new();
    while !job.is_finished() {
        std::thread::sleep(std::time::Duration::from_millis(50));
        if let Some(p) = job.poll() {
            if phases.last() != Some(&p.phase) {
                phases.push(p.phase);
            }
        }
    }
    let splat = job.join().unwrap();
    let r = &splat.report;
    let eval = r.eval.unwrap();
    println!("{r:#?}");
    assert!(eval.psnr > 24.0, "held-out PSNR {}", eval.psnr);
    // The underside is never seen, so the coverage filter removes part of the cloud.
    assert!(r.init_points_covered < r.init_points_in && r.init_points_covered > r.init_points_in / 2);
    assert!(phases.contains(&Phase::Training));

    // The PLY is a standard 3DGS file in mm.
    let ply = splat.to_ply_bytes().unwrap();
    let header = String::from_utf8_lossy(&ply[..ply.len().min(2000)]).to_string();
    for field in ["property float x", "f_dc_0", "f_rest_0", "opacity", "scale_0", "rot_0", "end_header"] {
        assert!(header.contains(field), "{field} missing in {header}");
    }

    // Rendering from a held-out camera reproduces the image.
    let img = &ds.images[3];
    let render = splat.render_view(img, [0.0; 3]).unwrap();
    let rgb: Vec<u8> = render.pixels().flat_map(|p| [p[0], p[1], p[2]]).collect();
    assert!(scancraft_splat::psnr_rgb8(&img.rgb, &rgb, None) > 24.0);
}

#[test]
#[ignore = "needs a GPU"]
fn cancellation_stops_training() {
    let job = SplatJob::spawn(small_scene(), SplatTrainConfig { iterations: 100_000, ..Default::default() });
    std::thread::sleep(std::time::Duration::from_secs(3));
    job.cancel();
    assert!(matches!(job.join(), Err(SplatError::Cancelled)));
}
