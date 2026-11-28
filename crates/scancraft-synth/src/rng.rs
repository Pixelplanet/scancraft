//! Small deterministic random numbers (SplitMix64). Every random choice is derived from the
//! scene seed and the index of what it is for, so results do not depend on thread scheduling.

#[derive(Debug, Clone)]
pub struct Rng(u64);

/// Mixes several numbers into one seed.
pub fn mix(parts: &[u64]) -> u64 {
    let mut h = 0x9e37_79b9_7f4a_7c15u64;
    for &p in parts {
        h ^= p.wrapping_add(0x9e37_79b9_7f4a_7c15).wrapping_add(h << 6).wrapping_add(h >> 2);
        h = splitmix(h);
    }
    h
}

fn splitmix(mut z: u64) -> u64 {
    z = z.wrapping_add(0x9e37_79b9_7f4a_7c15);
    z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
    z ^ (z >> 31)
}

impl Rng {
    pub fn new(seed: u64) -> Self {
        Rng(seed)
    }

    pub fn for_parts(parts: &[u64]) -> Self {
        Rng(mix(parts))
    }

    pub fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9e37_79b9_7f4a_7c15);
        splitmix(self.0)
    }

    /// Uniform in [0, 1).
    pub fn uniform(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }

    pub fn range(&mut self, lo: f64, hi: f64) -> f64 {
        lo + (hi - lo) * self.uniform()
    }

    /// Standard normal (Box-Muller).
    pub fn gauss(&mut self) -> f64 {
        let u1 = self.uniform().max(1e-300);
        let u2 = self.uniform();
        (-2.0 * u1.ln()).sqrt() * (std::f64::consts::TAU * u2).cos()
    }
}

/// Smooth value noise in [0, 1] on a lattice of `cell` mm (for patchy dropout).
pub fn value_noise(seed: u64, x: f64, y: f64, z: f64, cell: f64) -> f64 {
    let (x, y, z) = (x / cell, y / cell, z / cell);
    let (ix, iy, iz) = (x.floor(), y.floor(), z.floor());
    let (fx, fy, fz) = (x - ix, y - iy, z - iz);
    let s = |t: f64| t * t * (3.0 - 2.0 * t);
    let (sx, sy, sz) = (s(fx), s(fy), s(fz));
    let corner = |dx: i64, dy: i64, dz: i64| {
        let h = mix(&[seed, (ix as i64 + dx) as u64, (iy as i64 + dy) as u64, (iz as i64 + dz) as u64]);
        (h >> 11) as f64 / (1u64 << 53) as f64
    };
    let lerp = |a: f64, b: f64, t: f64| a + (b - a) * t;
    let x00 = lerp(corner(0, 0, 0), corner(1, 0, 0), sx);
    let x10 = lerp(corner(0, 1, 0), corner(1, 1, 0), sx);
    let x01 = lerp(corner(0, 0, 1), corner(1, 0, 1), sx);
    let x11 = lerp(corner(0, 1, 1), corner(1, 1, 1), sx);
    lerp(lerp(x00, x10, sy), lerp(x01, x11, sy), sz)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn deterministic_and_roughly_normal() {
        let mut a = Rng::for_parts(&[1, 2, 3]);
        let mut b = Rng::for_parts(&[1, 2, 3]);
        assert_eq!(a.next_u64(), b.next_u64());
        let mut r = Rng::new(7);
        let n = 20000;
        let xs: Vec<f64> = (0..n).map(|_| r.gauss()).collect();
        let mean = xs.iter().sum::<f64>() / n as f64;
        let var = xs.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / n as f64;
        assert!(mean.abs() < 0.03 && (var - 1.0).abs() < 0.05, "{mean} {var}");
        let v = value_noise(5, 1.3, 2.7, 0.0, 4.0);
        assert!((0.0..=1.0).contains(&v));
    }
}
