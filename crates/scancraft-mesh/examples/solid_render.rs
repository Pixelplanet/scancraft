//! Software renderer for checking solid meshes (orthographic, z-buffer, PNG output).
//!
//! cargo run --release -p scancraft-mesh --example solid_render -- <mesh.ply> <out.png> [key=value ...]
//!
//! Keys: `dir=x,y,z` view direction (into the screen), `up=x,y,z`, `center=x,y,z` and `half=w`
//! (zoom: half width of the view in mm; default: whole mesh), `size=W` (pixels, default 1400),
//! `clip=px,py,pz,nx,ny,nz` (keep only the half space n·(x−p) ≤ 0, so cut walls show their
//! inside), `slab=t` (with clip: keep only |n·(x−p)| ≤ t), `samples=path.ply` (colour the mesh by
//! distance to the nearest measured sample: grey < 0.3 mm, yellow → red up to 1.5 mm, blue beyond),
//! `origin=1` (colour by the vertex colours of the PLY, e.g. the origin tags of `solid_ply`),
//! `grazing=1` (low raking light, to show bumps).
//!
//! Faces seen from behind (inside of the material, visible through a cut) are drawn dark red.

use glam::Vec3;
use std::collections::HashMap;
use std::io::{Read, Write};

/// Binary PLY mesh: positions, normals, optional colours (any vertex property layout of float /
/// uchar properties), triangles.
fn read_mesh(path: &str) -> (Vec<Vec3>, Vec<Vec3>, Option<Vec<[f32; 3]>>, Vec<[u32; 3]>) {
    let mut d = Vec::new();
    std::fs::File::open(path).unwrap().read_to_end(&mut d).unwrap();
    let end = d.windows(11).position(|w| w == b"end_header
").unwrap() + 11;
    let h = String::from_utf8_lossy(&d[..end]).to_string();
    let (mut nv, mut nf) = (0usize, 0usize);
    let mut props: Vec<(String, usize)> = Vec::new();
    let mut in_vertex = false;
    for l in h.lines() {
        let t: Vec<&str> = l.split_whitespace().collect();
        match t.as_slice() {
            ["element", "vertex", n] => {
                nv = n.parse().unwrap();
                in_vertex = true;
            }
            ["element", "face", n] => {
                nf = n.parse().unwrap();
                in_vertex = false;
            }
            ["property", ty, name] if in_vertex => props.push((name.to_string(), if *ty == "uchar" { 1 } else { 4 })),
            _ => {}
        }
    }
    let stride: usize = props.iter().map(|p| p.1).sum();
    let off = |n: &str| {
        let mut o = 0;
        for (name, s) in &props {
            if name == n {
                return Some(o);
            }
            o += s;
        }
        None
    };
    let f = |o: usize| f32::from_le_bytes(d[o..o + 4].try_into().unwrap());
    let (ox, onx, ored) = (off("x").unwrap(), off("nx").unwrap(), off("red"));
    let mut pos = Vec::with_capacity(nv);
    let mut nor = Vec::with_capacity(nv);
    let mut col = ored.map(|_| Vec::with_capacity(nv));
    for i in 0..nv {
        let o = end + i * stride;
        pos.push(Vec3::new(f(o + ox), f(o + ox + 4), f(o + ox + 8)));
        nor.push(Vec3::new(f(o + onx), f(o + onx + 4), f(o + onx + 8)));
        if let (Some(c), Some(r)) = (col.as_mut(), ored) {
            c.push([d[o + r] as f32 / 255.0, d[o + r + 1] as f32 / 255.0, d[o + r + 2] as f32 / 255.0]);
        }
    }
    let mut tris = Vec::with_capacity(nf);
    let base = end + nv * stride;
    for i in 0..nf {
        let o = base + i * 13 + 1;
        let g = |k: usize| i32::from_le_bytes(d[o + 4 * k..o + 4 * k + 4].try_into().unwrap()) as u32;
        tris.push([g(0), g(1), g(2)]);
    }
    (pos, nor, col, tris)
}

/// Measured samples (grey points) of a samples.ply written by the solid_ply example.
fn read_samples(path: &str) -> Vec<Vec3> {
    let mut d = Vec::new();
    std::fs::File::open(path).unwrap().read_to_end(&mut d).unwrap();
    let end = d.windows(11).position(|w| w == b"end_header\n").unwrap() + 11;
    let h = String::from_utf8_lossy(&d[..end]).to_string();
    let nv: usize = h.lines().find(|l| l.starts_with("element vertex")).unwrap().split_whitespace().last().unwrap().parse().unwrap();
    let f = |o: usize| f32::from_le_bytes(d[o..o + 4].try_into().unwrap());
    (0..nv)
        .filter_map(|i| {
            let o = end + i * 27;
            (d[o + 24] == 180).then(|| Vec3::new(f(o), f(o + 4), f(o + 8)))
        })
        .collect()
}

fn vec3(s: &str) -> Vec3 {
    let v: Vec<f32> = s.split(',').map(|x| x.parse().unwrap()).collect();
    Vec3::new(v[0], v[1], v[2])
}

fn png(path: &str, w: usize, h: usize, rgb: &[u8]) {
    fn crc(data: &[u8]) -> u32 {
        let mut c = 0xffff_ffffu32;
        for &b in data {
            c ^= b as u32;
            for _ in 0..8 {
                c = if c & 1 != 0 { 0xedb8_8320 ^ (c >> 1) } else { c >> 1 };
            }
        }
        !c
    }
    let mut raw = Vec::with_capacity((w * 3 + 1) * h);
    for y in 0..h {
        raw.push(0u8);
        raw.extend_from_slice(&rgb[y * w * 3..(y + 1) * w * 3]);
    }
    // zlib stream of stored blocks.
    let mut z = vec![0x78u8, 0x01];
    for (i, chunk) in raw.chunks(65535).enumerate() {
        let last = (i + 1) * 65535 >= raw.len();
        z.push(last as u8);
        z.extend_from_slice(&(chunk.len() as u16).to_le_bytes());
        z.extend_from_slice(&(!(chunk.len() as u16)).to_le_bytes());
        z.extend_from_slice(chunk);
    }
    let (mut a, mut b) = (1u32, 0u32);
    for &x in &raw {
        a = (a + x as u32) % 65521;
        b = (b + a) % 65521;
    }
    z.extend_from_slice(&((b << 16) | a).to_be_bytes());
    let mut out = vec![0x89, b'P', b'N', b'G', 0x0d, 0x0a, 0x1a, 0x0a];
    let mut chunk = |ty: &[u8], data: &[u8]| {
        out.extend_from_slice(&(data.len() as u32).to_be_bytes());
        let mut c = ty.to_vec();
        c.extend_from_slice(data);
        out.extend_from_slice(&c);
        out.extend_from_slice(&crc(&c).to_be_bytes());
    };
    let mut ihdr = Vec::new();
    ihdr.extend_from_slice(&(w as u32).to_be_bytes());
    ihdr.extend_from_slice(&(h as u32).to_be_bytes());
    ihdr.extend_from_slice(&[8, 2, 0, 0, 0]);
    chunk(b"IHDR", &ihdr);
    chunk(b"IDAT", &z);
    chunk(b"IEND", &[]);
    std::fs::File::create(path).unwrap().write_all(&out).unwrap();
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let (mesh_path, out) = (&args[1], &args[2]);
    let kv: HashMap<String, String> =
        args[3..].iter().filter_map(|a| a.split_once('=').map(|(k, v)| (k.to_string(), v.to_string()))).collect();
    let (pos, nor, vcol, tris) = read_mesh(mesh_path);
    let by_origin = kv.get("origin").is_some() && vcol.is_some();
    let dir = kv.get("dir").map(|s| vec3(s)).unwrap_or(Vec3::NEG_Z).normalize();
    let up0 = kv.get("up").map(|s| vec3(s)).unwrap_or(Vec3::Y);
    let right = dir.cross(up0).normalize();
    let up = right.cross(dir).normalize();
    let size: usize = kv.get("size").map(|s| s.parse().unwrap()).unwrap_or(1400);
    let clip = kv.get("clip").map(|s| {
        let v: Vec<f32> = s.split(',').map(|x| x.parse().unwrap()).collect();
        (Vec3::new(v[0], v[1], v[2]), Vec3::new(v[3], v[4], v[5]).normalize())
    });
    let slab: Option<f32> = kv.get("slab").map(|s| s.parse().unwrap());
    let keep = |p: Vec3| match clip {
        Some((c, n)) => {
            let s = n.dot(p - c);
            match slab {
                Some(t) => s.abs() <= t,
                None => s <= 0.0,
            }
        }
        None => true,
    };

    // Per-vertex distance to the measured samples (optional).
    let dist: Option<Vec<f32>> = kv.get("samples").map(|sp| {
        let s = read_samples(sp);
        let cell = 1.0f32;
        let mut g: HashMap<(i32, i32, i32), Vec<u32>> = HashMap::new();
        for (i, p) in s.iter().enumerate() {
            let c = (*p / cell).floor().as_ivec3();
            g.entry((c.x, c.y, c.z)).or_default().push(i as u32);
        }
        pos.iter()
            .map(|p| {
                let c = (*p / cell).floor().as_ivec3();
                let mut best = f32::MAX;
                for dz in -2..=2 {
                    for dy in -2..=2 {
                        for dx in -2..=2 {
                            if let Some(v) = g.get(&(c.x + dx, c.y + dy, c.z + dz)) {
                                for &i in v {
                                    best = best.min(s[i as usize].distance_squared(*p));
                                }
                            }
                        }
                    }
                }
                best.sqrt()
            })
            .collect()
    });

    let vis: Vec<usize> = (0..pos.len()).filter(|&i| keep(pos[i])).collect();
    let (center, half) = match (kv.get("center"), kv.get("half")) {
        (Some(c), Some(h)) => (vec3(c), h.parse::<f32>().unwrap()),
        _ => {
            let (mut lo, mut hi) = (glam::Vec2::splat(f32::MAX), glam::Vec2::splat(f32::MIN));
            for &i in &vis {
                let q = glam::Vec2::new(pos[i].dot(right), pos[i].dot(up));
                lo = lo.min(q);
                hi = hi.max(q);
            }
            let c2 = (lo + hi) * 0.5;
            let mid: Vec3 = right * c2.x + up * c2.y;
            (mid, 0.52 * (hi - lo).max_element())
        }
    };
    let (w, h) = (size, size);
    let scale = w as f32 / (2.0 * half);
    let mut zb = vec![f32::MAX; w * h];
    let mut img = vec![255u8; w * h * 3];
    let grazing = kv.get("grazing").is_some();
    let l1 = if grazing { (-dir * 0.25 + up * 0.97).normalize() } else { (-dir + up * 0.6 + right * 0.4).normalize() };
    let l2 = (-dir - right * 0.5).normalize();
    let proj = |p: Vec3| {
        let q = p - center;
        (q.dot(right) * scale + w as f32 * 0.5, h as f32 * 0.5 - q.dot(up) * scale, q.dot(dir))
    };
    for t in &tris {
        let (a, b, c) = (pos[t[0] as usize], pos[t[1] as usize], pos[t[2] as usize]);
        if !(keep(a) && keep(b) && keep(c)) {
            continue;
        }
        let (pa, pb, pc) = (proj(a), proj(b), proj(c));
        let fnorm = (b - a).cross(c - a);
        let front = fnorm.dot(dir) < 0.0;
        let minx = pa.0.min(pb.0).min(pc.0).floor().max(0.0) as i32;
        let maxx = pa.0.max(pb.0).max(pc.0).ceil().min(w as f32 - 1.0) as i32;
        let miny = pa.1.min(pb.1).min(pc.1).floor().max(0.0) as i32;
        let maxy = pa.1.max(pb.1).max(pc.1).ceil().min(h as f32 - 1.0) as i32;
        if minx > maxx || miny > maxy {
            continue;
        }
        let area = (pb.0 - pa.0) * (pc.1 - pa.1) - (pc.0 - pa.0) * (pb.1 - pa.1);
        if area.abs() < 1e-12 {
            continue;
        }
        for y in miny..=maxy {
            for x in minx..=maxx {
                let (px, py) = (x as f32 + 0.5, y as f32 + 0.5);
                let w0 = ((pb.0 - px) * (pc.1 - py) - (pc.0 - px) * (pb.1 - py)) / area;
                let w1 = ((pc.0 - px) * (pa.1 - py) - (pa.0 - px) * (pc.1 - py)) / area;
                let w2 = 1.0 - w0 - w1;
                if w0 < -1e-4 || w1 < -1e-4 || w2 < -1e-4 {
                    continue;
                }
                let z = w0 * pa.2 + w1 * pb.2 + w2 * pc.2;
                let k = y as usize * w + x as usize;
                if z >= zb[k] {
                    continue;
                }
                zb[k] = z;
                let n = (nor[t[0] as usize] * w0 + nor[t[1] as usize] * w1 + nor[t[2] as usize] * w2).normalize_or_zero();
                let col: [f32; 3] = if front {
                    let s = 0.18 + 0.62 * n.dot(l1).max(0.0) + 0.25 * n.dot(l2).max(0.0);
                    let base = match &dist {
                        _ if by_origin => {
                            let c = vcol.as_ref().unwrap();
                            let (a, b, cc) = (c[t[0] as usize], c[t[1] as usize], c[t[2] as usize]);
                            [0, 1, 2].map(|k| a[k] * w0 + b[k] * w1 + cc[k] * w2)
                        }
                        Some(dv) => {
                            let d = dv[t[0] as usize] * w0 + dv[t[1] as usize] * w1 + dv[t[2] as usize] * w2;
                            if d < 0.3 {
                                [0.85, 0.86, 0.88]
                            } else if d < 1.5 {
                                let u = (d - 0.3) / 1.2;
                                [1.0, 0.9 - 0.75 * u, 0.2]
                            } else {
                                [0.3, 0.5, 1.0]
                            }
                        }
                        None => [0.82, 0.84, 0.88],
                    };
                    [base[0] * s, base[1] * s, base[2] * s]
                } else {
                    let s = 0.35 + 0.45 * (-n).dot(l1).max(0.0);
                    [0.75 * s, 0.12 * s, 0.1 * s]
                };
                for ch in 0..3 {
                    img[k * 3 + ch] = (col[ch].clamp(0.0, 1.0) * 255.0) as u8;
                }
            }
        }
    }
    png(out, w, h, &img);
    println!("wrote {out} ({w}x{h}, {:.3} mm/px)", 1.0 / scale);
}
