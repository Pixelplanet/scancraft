//! Importing external scans (PLY, OBJ, STL) as static passes.
//!
//! [`import_scan`] reads a file, picks the format from its content (falling back to the
//! extension), and returns an [`ImportedScan`] whose points always have normals when possible:
//! from the file, else from the mesh faces (area-weighted), else estimated (see
//! [`crate::normals`]).

use crate::normals::{self, NormalEstimation};
use crate::{obj, ply, stl, IoError};
use scancraft_core::{ColorRgba, Mesh, Point3D, PointCloud, Triangle};
use std::path::Path;

/// File formats that [`import_scan`] reads.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ScanFormat {
    Ply,
    Obj,
    Stl,
}

/// Where an imported scan's normals came from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NormalSource {
    /// No normals (empty scan, or estimation switched off).
    None,
    /// Stored in the file.
    File,
    /// Derived from the mesh faces.
    Faces,
    /// Estimated from the point neighbourhoods.
    Estimated,
}

/// A guess at the file's length unit. The formats carry no unit; ScanCraft works in mm.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UnitsHint {
    /// Size fits millimetres (largest extent 3 mm .. 20 m).
    Millimetres,
    /// Largest extent below 3 units: almost certainly metres (an object under 3 mm is not a
    /// realistic scan).
    Metres,
    /// Empty, or too large to judge (could be micrometres, or a scene in mm).
    Unknown,
}

impl UnitsHint {
    /// Guesses the unit from the largest bounding-box extent.
    pub fn from_extent(max_extent: f32) -> Self {
        if max_extent <= 0.0 || !max_extent.is_finite() {
            Self::Unknown
        } else if max_extent < 3.0 {
            Self::Metres
        } else if max_extent <= 20_000.0 {
            Self::Millimetres
        } else {
            Self::Unknown
        }
    }

    /// Factor that converts the file's values to millimetres (1 when unknown).
    pub fn to_mm(self) -> f32 {
        match self {
            Self::Metres => 1000.0,
            Self::Millimetres | Self::Unknown => 1.0,
        }
    }
}

/// Settings for [`import_scan_with`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImportOptions {
    /// Estimate normals for clouds that have neither normals nor faces.
    pub estimate_normals: bool,
    /// Neighbours and orientation for that estimate.
    pub estimation: NormalEstimation,
}

impl Default for ImportOptions {
    fn default() -> Self {
        Self { estimate_normals: true, estimation: NormalEstimation::default() }
    }
}

impl ImportOptions {
    /// Options that never run the (slower) normal estimation.
    pub fn without_estimation() -> Self {
        Self { estimate_normals: false, ..Self::default() }
    }
}

/// A scan read from a file. All per-point arrays have the length of `points`.
#[derive(Debug, Clone, PartialEq)]
pub struct ImportedScan {
    pub format: ScanFormat,
    pub points: Vec<[f32; 3]>,
    /// Unit normals; `[0, 0, 0]` marks a point whose normal could not be found.
    pub normals: Option<Vec<[f32; 3]>>,
    pub normal_source: NormalSource,
    /// RGBA colours, when the file has them.
    pub colors: Option<Vec<[u8; 4]>>,
    /// Triangles (polygons fan-triangulated), when the file is a mesh.
    pub faces: Option<Vec<[u32; 3]>>,
    pub units: UnitsHint,
}

/// Colour given to points of a file without colours (same as the older loaders).
const DEFAULT_COLOR: ColorRgba = ColorRgba { r: 200, g: 205, b: 215, a: 255 };

impl ImportedScan {
    pub(crate) fn empty(format: ScanFormat) -> Self {
        Self {
            format,
            points: Vec::new(),
            normals: None,
            normal_source: NormalSource::None,
            colors: None,
            faces: None,
            units: UnitsHint::Unknown,
        }
    }

    pub fn len(&self) -> usize {
        self.points.len()
    }

    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }

    /// Axis-aligned bounds `(min, max)` of the finite points, or `None` if there are none.
    pub fn bounds(&self) -> Option<([f32; 3], [f32; 3])> {
        let mut min = [f32::INFINITY; 3];
        let mut max = [f32::NEG_INFINITY; 3];
        for p in self.points.iter().filter(|p| normals::is_finite(**p)) {
            for a in 0..3 {
                min[a] = min[a].min(p[a]);
                max[a] = max[a].max(p[a]);
            }
        }
        (min[0] <= max[0]).then_some((min, max))
    }

    /// Multiplies all coordinates by `factor` (for example [`UnitsHint::to_mm`]).
    pub fn scale(&mut self, factor: f32) {
        for p in &mut self.points {
            *p = p.map(|v| v * factor);
        }
        if factor < 0.0 {
            self.flip_normals();
        }
    }

    /// Reverses every normal. The sign of estimated normals on an open, one-sided surface (the
    /// inside of a shell, say) is a guess; this fixes a wrong guess.
    pub fn flip_normals(&mut self) {
        if let Some(n) = &mut self.normals {
            n.iter_mut().for_each(|v| *v = v.map(|c| -c));
        }
    }

    /// The points as a ScanCraft cloud (missing normals `[0, 0, 1]`, missing colours grey).
    pub fn to_point_cloud(&self) -> PointCloud {
        PointCloud::from_points((0..self.points.len()).map(|i| self.point(i)).collect())
    }

    /// The scan as a mesh, if it has faces.
    pub fn to_mesh(&self) -> Option<Mesh> {
        let faces = self.faces.as_ref()?;
        let vertices = (0..self.points.len()).map(|i| self.point(i)).collect();
        let triangles = faces.iter().map(|f| Triangle::new(f[0], f[1], f[2])).collect();
        Some(Mesh::from_geometry(vertices, triangles))
    }

    fn point(&self, i: usize) -> Point3D {
        let normal = match &self.normals {
            Some(n) if n[i] != [0.0; 3] => n[i],
            _ => [0.0, 0.0, 1.0],
        };
        let color = self.colors.as_ref().map_or(DEFAULT_COLOR, |c| {
            let [r, g, b, a] = c[i];
            ColorRgba::new(r, g, b, a)
        });
        Point3D { position: self.points[i], normal, color, confidence: 1.0 }
    }
}

/// Imports a scan with default options (normals estimated when the file has none).
pub fn import_scan<P: AsRef<Path>>(path: P) -> Result<ImportedScan, IoError> {
    import_scan_with(path, &ImportOptions::default())
}

/// Imports a PLY, OBJ or STL file. The format comes from the content (PLY magic, STL size or
/// `solid ... facet`), else from the extension, else OBJ is tried for text files.
pub fn import_scan_with<P: AsRef<Path>>(path: P, options: &ImportOptions) -> Result<ImportedScan, IoError> {
    let path = path.as_ref();
    let bytes = std::fs::read(path)?;
    let ext = path.extension().and_then(|e| e.to_str()).map(|e| e.to_ascii_lowercase());
    import_bytes(&bytes, ext.as_deref(), options)
}

/// Like [`import_scan_with`] for a file already in memory; `extension` is a lower-case hint.
pub fn import_bytes(bytes: &[u8], extension: Option<&str>, options: &ImportOptions) -> Result<ImportedScan, IoError> {
    let scan = match detect_format(bytes, extension)? {
        ScanFormat::Ply => ply::parse_ply(bytes)?,
        ScanFormat::Stl => stl::parse_stl(bytes)?,
        ScanFormat::Obj => obj::parse_obj(bytes)?,
    };
    Ok(finish(scan, options))
}

/// Picks the format from the content first, then from the extension.
pub fn detect_format(bytes: &[u8], extension: Option<&str>) -> Result<ScanFormat, IoError> {
    if bytes.starts_with(b"ply") && matches!(bytes.get(3), Some(b'\n' | b'\r')) {
        return Ok(ScanFormat::Ply);
    }
    if stl::looks_like_stl(bytes) {
        return Ok(ScanFormat::Stl);
    }
    match extension {
        Some("ply") => Err(IoError::Parse("PLY: missing 'ply' magic".into())),
        Some("stl") => Ok(ScanFormat::Stl), // the STL parser reports what is wrong
        Some("obj") => Ok(ScanFormat::Obj),
        _ if obj::looks_like_obj(bytes) => Ok(ScanFormat::Obj),
        _ => Err(IoError::Unsupported("not a PLY, OBJ or STL file".into())),
    }
}

/// Completes a parsed scan: drops non-finite points of bare clouds, checks file normals,
/// fills missing normals from faces or by estimation, and guesses the unit.
pub(crate) fn finish(mut scan: ImportedScan, options: &ImportOptions) -> ImportedScan {
    if scan.faces.is_none() {
        drop_non_finite(&mut scan);
    }
    if let Some(n) = scan.normals.take() {
        scan.normals = usable_file_normals(n);
        if scan.normals.is_none() {
            log::warn!("imported normals are mostly zero or invalid; deriving new ones");
            scan.normal_source = NormalSource::None;
        }
    }
    match (&scan.normals, &scan.faces) {
        (Some(_), Some(faces)) => {
            // Fill zero normals (unknown) from the faces.
            let from_faces = normals::vertex_normals_from_faces(&scan.points, faces);
            let n = scan.normals.as_mut().expect("checked");
            for (v, f) in n.iter_mut().zip(from_faces) {
                if *v == [0.0; 3] {
                    *v = f;
                }
            }
        }
        (None, Some(faces)) if !faces.is_empty() => {
            scan.normals = Some(normals::vertex_normals_from_faces(&scan.points, faces));
            scan.normal_source = NormalSource::Faces;
        }
        (None, _) if options.estimate_normals && !scan.points.is_empty() => {
            scan.normals = Some(normals::estimate_normals(&scan.points, &options.estimation));
            scan.normal_source = NormalSource::Estimated;
        }
        _ => {}
    }
    scan.units = scan.bounds().map_or(UnitsHint::Unknown, |(min, max)| {
        UnitsHint::from_extent((0..3).map(|a| max[a] - min[a]).fold(0.0, f32::max))
    });
    scan
}

/// Removes points with NaN or infinite coordinates (some exporters write invalid pixels).
fn drop_non_finite(scan: &mut ImportedScan) {
    let keep: Vec<bool> = scan.points.iter().map(|p| normals::is_finite(*p)).collect();
    let dropped = keep.iter().filter(|k| !**k).count();
    if dropped == 0 {
        return;
    }
    log::info!("import: dropped {dropped} points with invalid coordinates");
    fn retain<T>(v: &mut Vec<T>, keep: &[bool]) {
        let mut i = 0;
        v.retain(|_| {
            i += 1;
            keep[i - 1]
        });
    }
    retain(&mut scan.points, &keep);
    if let Some(n) = &mut scan.normals {
        retain(n, &keep);
    }
    if let Some(c) = &mut scan.colors {
        retain(c, &keep);
    }
}

/// Normalises file normals. Returns `None` when more than half are zero or invalid (some
/// exporters write all-zero normals); otherwise bad ones become `[0, 0, 0]`.
fn usable_file_normals(mut n: Vec<[f32; 3]>) -> Option<Vec<[f32; 3]>> {
    let mut bad = 0usize;
    for v in &mut n {
        *v = normals::normalize_or_zero(*v);
        if *v == [0.0; 3] {
            bad += 1;
        }
    }
    (bad * 2 <= n.len()).then_some(n)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{save_mesh_obj, save_mesh_ply_binary, save_mesh_stl_ascii, save_mesh_stl_binary, save_ply_binary};

    fn temp(name: &str) -> std::path::PathBuf {
        std::env::temp_dir().join(format!("scancraft_import_{}_{name}", std::process::id()))
    }

    /// A closed tetrahedron with outward winding.
    fn tetra() -> Mesh {
        let mut m = Mesh::new();
        for p in [[0.0, 0.0, 0.0], [10.0, 0.0, 0.0], [0.0, 10.0, 0.0], [0.0, 0.0, 10.0]] {
            m.add_vertex(Point3D::new(p).with_color(ColorRgba::new(10, 20, 30, 255)));
        }
        for [a, b, c] in [[0, 2, 1], [0, 1, 3], [0, 3, 2], [1, 2, 3]] {
            m.add_triangle(a, b, c);
        }
        m.recompute_normals();
        m
    }

    /// Checks that every vertex normal of the tetrahedron points away from its centre.
    fn assert_outward(scan: &ImportedScan) {
        let n = scan.normals.as_ref().expect("normals");
        let c = [2.5f32, 2.5, 2.5];
        for (p, n) in scan.points.iter().zip(n) {
            let d = [p[0] - c[0], p[1] - c[1], p[2] - c[2]];
            assert!(d[0] * n[0] + d[1] * n[1] + d[2] * n[2] > 0.0, "normal {n:?} at {p:?}");
        }
    }

    #[test]
    fn mesh_round_trips_through_every_format() {
        let mesh = tetra();
        let cases: [(&str, fn(&Mesh, &std::path::Path) -> Result<(), IoError>, ScanFormat); 4] = [
            ("m.ply", |m, p| save_mesh_ply_binary(m, p), ScanFormat::Ply),
            ("m.obj", |m, p| save_mesh_obj(m, p), ScanFormat::Obj),
            ("m.stl", |m, p| save_mesh_stl_binary(m, p), ScanFormat::Stl),
            ("a.stl", |m, p| save_mesh_stl_ascii(m, p), ScanFormat::Stl),
        ];
        for (name, save, format) in cases {
            let path = temp(name);
            save(&mesh, &path).unwrap();
            let scan = import_scan(&path).unwrap();
            let _ = std::fs::remove_file(&path);
            assert_eq!(scan.format, format, "{name}");
            assert_eq!(scan.points.len(), 4, "{name}: STL vertices must be welded");
            assert_eq!(scan.faces.as_ref().unwrap().len(), 4, "{name}");
            for p in &mesh.vertices {
                assert!(scan.points.contains(&p.position), "{name}: {p:?} missing");
            }
            assert_outward(&scan);
            assert_eq!(scan.units, UnitsHint::Millimetres);
            let m = scan.to_mesh().unwrap();
            assert_eq!(m.triangles.len(), 4);
        }
    }

    #[test]
    fn cloud_without_normals_gets_estimated_ones() {
        // A sphere of radius 40 mm, saved without normals.
        let n = 5_000;
        let golden = std::f32::consts::PI * (3.0 - 5f32.sqrt());
        let points: Vec<Point3D> = (0..n)
            .map(|i| {
                let y = 1.0 - 2.0 * (i as f32 + 0.5) / n as f32;
                let r = (1.0 - y * y).sqrt();
                let t = golden * i as f32;
                Point3D::new([40.0 * r * t.cos(), 40.0 * y, 40.0 * r * t.sin()])
            })
            .collect();
        let path = temp("cloud.ply");
        save_ply_binary(&PointCloud::from_points(points), &path).unwrap();
        let scan = import_scan(&path).unwrap();
        let plain = import_scan_with(&path, &ImportOptions::without_estimation()).unwrap();
        let _ = std::fs::remove_file(&path);

        assert_eq!(scan.len(), n);
        assert_eq!(scan.normal_source, NormalSource::Estimated);
        assert!(scan.colors.is_some());
        let normals = scan.normals.as_ref().unwrap();
        for (p, nrm) in scan.points.iter().zip(normals) {
            let cos = (p[0] * nrm[0] + p[1] * nrm[1] + p[2] * nrm[2]) / 40.0;
            assert!(cos > 0.9, "normal {nrm:?} at {p:?}");
        }
        assert_eq!(plain.normal_source, NormalSource::None);
        assert!(plain.normals.is_none());

        let mut flipped = scan.clone();
        flipped.flip_normals();
        assert_eq!(flipped.normals.unwrap()[0], normals[0].map(|c| -c));
    }

    #[test]
    fn invalid_points_dropped_and_zero_normals_replaced() {
        let ply = b"ply\nformat ascii 1.0\nelement vertex 3\nproperty float x\nproperty float y\n\
property float z\nproperty float nx\nproperty float ny\nproperty float nz\nend_header\n\
0 0 0 0 0 0\nnan 1 1 0 0 1\n1 1 1 0 0 0\n";
        let scan = import_bytes(ply, Some("ply"), &ImportOptions::without_estimation()).unwrap();
        assert_eq!(scan.len(), 2);
        // Both remaining normals were zero: treated as missing, not estimated here.
        assert_eq!(scan.normal_source, NormalSource::None);
        assert!(scan.normals.is_none());
    }

    #[test]
    fn units_hint_and_scaling() {
        assert_eq!(UnitsHint::from_extent(0.4), UnitsHint::Metres);
        assert_eq!(UnitsHint::from_extent(400.0), UnitsHint::Millimetres);
        assert_eq!(UnitsHint::from_extent(1e6), UnitsHint::Unknown);
        assert_eq!(UnitsHint::from_extent(f32::NAN), UnitsHint::Unknown);
        let ply = b"ply\nformat ascii 1.0\nelement vertex 2\nproperty float x\nproperty float y\n\
property float z\nend_header\n0 0 0\n0.4 0.1 0.2\n";
        let mut scan = import_bytes(ply, None, &ImportOptions::default()).unwrap();
        assert_eq!(scan.units, UnitsHint::Metres);
        scan.scale(scan.units.to_mm());
        assert_eq!(scan.points[1], [400.0, 100.0, 200.0]);
    }

    #[test]
    fn format_detection() {
        assert_eq!(detect_format(b"ply\nformat ascii 1.0\n", Some("txt")).unwrap(), ScanFormat::Ply);
        assert_eq!(detect_format(b"solid x\n facet normal 0 0 1\n", None).unwrap(), ScanFormat::Stl);
        assert_eq!(detect_format(b"# c\nv 1 2 3\n", None).unwrap(), ScanFormat::Obj);
        assert_eq!(detect_format(b"anything", Some("obj")).unwrap(), ScanFormat::Obj);
        assert!(detect_format(b"\x00\x01binary junk", None).is_err());
        assert!(detect_format(b"hello", Some("ply")).is_err());
        assert!(import_bytes(b"", Some("stl"), &ImportOptions::default()).is_err());
    }
}
