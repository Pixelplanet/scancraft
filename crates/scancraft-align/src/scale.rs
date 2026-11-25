//! Scale correction from hand-measured lengths: a scan imported in unknown units (a
//! photogrammetry export, a STL in inches) is scaled by comparing distances the user measured
//! on the part with the same distances in the model.

/// The scale (factor: model units -> mm) fitted to measured/true length pairs.
#[derive(Debug, Clone, PartialEq)]
pub struct ScaleEstimate {
    /// model * factor = mm.
    pub factor: f32,
    /// true / measured per pair.
    pub factors: Vec<f32>,
    /// Largest |factor_i / factor - 1| over the pairs; 0 for one pair.
    pub spread: f32,
}

/// Least-squares scale through the origin: `factor = sum(true*measured) / sum(measured^2)`, so
/// long lengths weigh more. Non-positive and non-finite lengths are ignored; None when no pair
/// is left.
pub fn scale_from_lengths(pairs: &[(f32, f32)]) -> Option<ScaleEstimate> {
    let good: Vec<(f64, f64)> = pairs.iter().filter(|&&(m, t)| m.is_finite() && t.is_finite() && m > 0.0 && t > 0.0).map(|&(m, t)| (m as f64, t as f64)).collect();
    if good.is_empty() {
        return None;
    }
    let num: f64 = good.iter().map(|(m, t)| m * t).sum();
    let den: f64 = good.iter().map(|(m, _)| m * m).sum();
    let factor = num / den;
    let factors: Vec<f32> = good.iter().map(|(m, t)| (t / m) as f32).collect();
    let spread = if good.len() < 2 { 0.0 } else { factors.iter().map(|f| (*f as f64 / factor - 1.0).abs()).fold(0.0, f64::max) };
    Some(ScaleEstimate { factor: factor as f32, factors, spread: spread as f32 })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn three_lengths_with_errors_give_the_factor() {
        // True scale 1.5 (the model is in 2/3 mm units); +-1 % per length.
        let pairs = [(100.0, 151.0), (40.0, 59.4), (210.0, 312.0)];
        let e = scale_from_lengths(&pairs).expect("estimate");
        eprintln!("scale: factor {:.4}, spread {:.3} %, factors {:?}", e.factor, e.spread * 100.0, e.factors);
        assert!((e.factor - 1.5).abs() / 1.5 < 0.01, "factor {}", e.factor);
        assert!(e.spread > 0.005 && e.spread < 0.03, "spread {}", e.spread);
    }

    #[test]
    fn one_pair_gives_its_ratio() {
        let e = scale_from_lengths(&[(25.4, 25.4)]).expect("estimate");
        assert!((e.factor - 1.0).abs() < 1e-6 && e.spread == 0.0 && e.factors.len() == 1);
        let e = scale_from_lengths(&[(10.0, 35.0)]).expect("estimate");
        assert!((e.factor - 3.5).abs() < 1e-6 && e.spread == 0.0);
    }

    #[test]
    fn bad_lengths_are_ignored_and_long_lengths_dominate() {
        // One accurate 10x pair and one short pair 20 % off: the long one wins.
        let e = scale_from_lengths(&[(0.0, 5.0), (-1.0, 9.0), (f32::NAN, 3.0), (1000.0, 1500.0), (10.0, 18.0)]).expect("estimate");
        eprintln!("scale mixed: factor {:.4}, spread {:.3}", e.factor, e.spread);
        assert!((e.factor - 1.5).abs() / 1.5 < 0.01, "factor {}", e.factor);
        assert_eq!(scale_from_lengths(&[(0.0, 5.0), (-2.0, 1.0)]), None);
    }
}
