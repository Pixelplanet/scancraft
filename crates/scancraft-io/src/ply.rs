use crate::import::{ImportedScan, NormalSource, ScanFormat};
use crate::IoError;
use byteorder::{BigEndian, ByteOrder, LittleEndian, WriteBytesExt};
use scancraft_core::{Mesh, PointCloud};
use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::Path;

/// Export point cloud to binary little-endian PLY file
pub fn save_ply_binary<P: AsRef<Path>>(cloud: &PointCloud, path: P) -> Result<(), IoError> {
    let file = File::create(path)?;
    let mut w = BufWriter::new(file);

    writeln!(w, "ply")?;
    writeln!(w, "format binary_little_endian 1.0")?;
    writeln!(w, "comment ScanCraft Point Cloud Export")?;
    writeln!(w, "element vertex {}", cloud.points.len())?;
    writeln!(w, "property float x")?;
    writeln!(w, "property float y")?;
    writeln!(w, "property float z")?;
    writeln!(w, "property uchar red")?;
    writeln!(w, "property uchar green")?;
    writeln!(w, "property uchar blue")?;
    writeln!(w, "property uchar alpha")?;
    writeln!(w, "end_header")?;

    for p in &cloud.points {
        w.write_f32::<LittleEndian>(p.position[0])?;
        w.write_f32::<LittleEndian>(p.position[1])?;
        w.write_f32::<LittleEndian>(p.position[2])?;
        w.write_u8(p.color.r)?;
        w.write_u8(p.color.g)?;
        w.write_u8(p.color.b)?;
        w.write_u8(p.color.a)?;
    }

    w.flush()?;
    Ok(())
}

/// Export triangle mesh to binary little-endian PLY file
pub fn save_mesh_ply_binary<P: AsRef<Path>>(mesh: &Mesh, path: P) -> Result<(), IoError> {
    let file = File::create(path)?;
    let mut w = BufWriter::new(file);

    writeln!(w, "ply")?;
    writeln!(w, "format binary_little_endian 1.0")?;
    writeln!(w, "comment ScanCraft Mesh Export")?;
    writeln!(w, "element vertex {}", mesh.vertices.len())?;
    writeln!(w, "property float x")?;
    writeln!(w, "property float y")?;
    writeln!(w, "property float z")?;
    writeln!(w, "property float nx")?;
    writeln!(w, "property float ny")?;
    writeln!(w, "property float nz")?;
    writeln!(w, "property uchar red")?;
    writeln!(w, "property uchar green")?;
    writeln!(w, "property uchar blue")?;
    writeln!(w, "property uchar alpha")?;
    writeln!(w, "element face {}", mesh.triangles.len())?;
    writeln!(w, "property list uchar int vertex_indices")?;
    writeln!(w, "end_header")?;

    for v in &mesh.vertices {
        w.write_f32::<LittleEndian>(v.position[0])?;
        w.write_f32::<LittleEndian>(v.position[1])?;
        w.write_f32::<LittleEndian>(v.position[2])?;
        w.write_f32::<LittleEndian>(v.normal[0])?;
        w.write_f32::<LittleEndian>(v.normal[1])?;
        w.write_f32::<LittleEndian>(v.normal[2])?;
        w.write_u8(v.color.r)?;
        w.write_u8(v.color.g)?;
        w.write_u8(v.color.b)?;
        w.write_u8(v.color.a)?;
    }

    for t in &mesh.triangles {
        w.write_u8(3)?; // 3 vertices per face
        w.write_i32::<LittleEndian>(t.indices[0] as i32)?;
        w.write_i32::<LittleEndian>(t.indices[1] as i32)?;
        w.write_i32::<LittleEndian>(t.indices[2] as i32)?;
    }

    w.flush()?;
    Ok(())
}


/// Load a PLY file: ASCII or binary (either byte order), point cloud or mesh, any property
/// layout. Missing normals are derived from faces when there are faces; missing normals of a
/// bare cloud stay `[0, 0, 1]` (use [`crate::import_scan`] to estimate them).
pub fn load_ply<P: AsRef<Path>>(path: P) -> Result<(PointCloud, Option<Mesh>), IoError> {
    let bytes = std::fs::read(path)?;
    let scan = crate::import::finish(parse_ply(&bytes)?, &crate::ImportOptions::without_estimation());
    Ok((scan.to_point_cloud(), scan.to_mesh()))
}

/// PLY body encodings.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Encoding {
    Ascii,
    BinaryLe,
    BinaryBe,
}

/// PLY scalar property types.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Scalar {
    I8,
    U8,
    I16,
    U16,
    I32,
    U32,
    F32,
    F64,
}

impl Scalar {
    fn parse(name: &str) -> Option<Self> {
        Some(match name {
            "char" | "int8" => Self::I8,
            "uchar" | "uint8" => Self::U8,
            "short" | "int16" => Self::I16,
            "ushort" | "uint16" => Self::U16,
            "int" | "int32" => Self::I32,
            "uint" | "uint32" => Self::U32,
            "float" | "float32" => Self::F32,
            "double" | "float64" => Self::F64,
            _ => return None,
        })
    }

    fn size(self) -> usize {
        match self {
            Self::I8 | Self::U8 => 1,
            Self::I16 | Self::U16 => 2,
            Self::I32 | Self::U32 | Self::F32 => 4,
            Self::F64 => 8,
        }
    }

    #[inline]
    fn read<E: ByteOrder>(self, b: &[u8]) -> f64 {
        match self {
            Self::I8 => b[0] as i8 as f64,
            Self::U8 => b[0] as f64,
            Self::I16 => E::read_i16(b) as f64,
            Self::U16 => E::read_u16(b) as f64,
            Self::I32 => E::read_i32(b) as f64,
            Self::U32 => E::read_u32(b) as f64,
            Self::F32 => E::read_f32(b) as f64,
            Self::F64 => E::read_f64(b),
        }
    }

    /// Converts a colour channel of this type to 0..=255 (floats are 0..1, 16 bit is 0..65535).
    fn to_channel(self, v: f64) -> u8 {
        let v = match self {
            Self::F32 | Self::F64 => v * 255.0,
            Self::U16 | Self::I16 => v / 257.0,
            _ => v,
        };
        v.round().clamp(0.0, 255.0) as u8
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum PropKind {
    Scalar(Scalar),
    List { count: Scalar, item: Scalar },
}

#[derive(Debug, Clone)]
struct Property {
    name: String,
    kind: PropKind,
}

#[derive(Debug, Clone)]
struct Element {
    name: String,
    count: usize,
    props: Vec<Property>,
}

impl Element {
    /// Index of the first property with one of these names.
    fn prop(&self, names: &[&str]) -> Option<usize> {
        names.iter().find_map(|n| self.props.iter().position(|p| p.name == *n))
    }

    /// Record size in bytes when every property is a scalar.
    fn fixed_size(&self) -> Option<usize> {
        self.props
            .iter()
            .map(|p| match p.kind {
                PropKind::Scalar(s) => Some(s.size()),
                PropKind::List { .. } => None,
            })
            .sum()
    }

    /// Smallest possible binary record size in bytes (lists empty).
    fn min_size(&self) -> usize {
        self.props
            .iter()
            .map(|p| match p.kind {
                PropKind::Scalar(s) => s.size(),
                PropKind::List { count, .. } => count.size(),
            })
            .sum()
    }

    /// Item type of a list property, or the type of a scalar one.
    fn scalar_type(&self, i: usize) -> Scalar {
        match self.props[i].kind {
            PropKind::Scalar(s) => s,
            PropKind::List { item, .. } => item,
        }
    }
}

struct Header {
    encoding: Encoding,
    elements: Vec<Element>,
    body_start: usize,
}

fn parse_err(msg: impl Into<String>) -> IoError {
    IoError::Parse(format!("PLY: {}", msg.into()))
}

fn truncated(msg: impl Into<String>) -> IoError {
    IoError::Truncated(format!("PLY: {}", msg.into()))
}

fn parse_header(bytes: &[u8]) -> Result<Header, IoError> {
    if !bytes.starts_with(b"ply") {
        return Err(parse_err("missing 'ply' magic"));
    }
    // The header is short; do not scan a whole broken file for it.
    let window = &bytes[..bytes.len().min(1 << 20)];
    let end = find(window, b"end_header").ok_or_else(|| truncated("header has no end_header"))?;
    let body_start = match bytes[end..].iter().position(|&b| b == b'\n') {
        Some(nl) => end + nl + 1,
        None => bytes.len(),
    };
    let text = std::str::from_utf8(&bytes[..end]).map_err(|_| parse_err("header is not text"))?;

    let mut encoding = None;
    let mut elements: Vec<Element> = Vec::new();
    for (n, line) in text.lines().enumerate().skip(1) {
        let mut t = line.split_ascii_whitespace();
        let bad = || parse_err(format!("bad header line {}: '{}'", n + 1, line.trim()));
        match t.next() {
            Some("format") => {
                encoding = Some(match t.next() {
                    Some("ascii") => Encoding::Ascii,
                    Some("binary_little_endian") => Encoding::BinaryLe,
                    Some("binary_big_endian") => Encoding::BinaryBe,
                    _ => return Err(bad()),
                })
            }
            Some("element") => {
                let name = t.next().ok_or_else(bad)?.to_string();
                let count = t.next().and_then(|c| c.parse().ok()).ok_or_else(bad)?;
                elements.push(Element { name, count, props: Vec::new() });
            }
            Some("property") => {
                let el = elements.last_mut().ok_or_else(bad)?;
                let ty = t.next().ok_or_else(bad)?;
                let kind = if ty == "list" {
                    let count = t.next().and_then(Scalar::parse).ok_or_else(bad)?;
                    let item = t.next().and_then(Scalar::parse).ok_or_else(bad)?;
                    PropKind::List { count, item }
                } else {
                    PropKind::Scalar(Scalar::parse(ty).ok_or_else(bad)?)
                };
                let name = t.next().ok_or_else(bad)?.to_string();
                el.props.push(Property { name, kind });
            }
            _ => {} // comment, obj_info, blank lines
        }
    }
    let encoding = encoding.ok_or_else(|| parse_err("no format line"))?;
    Ok(Header { encoding, elements, body_start })
}

fn find(hay: &[u8], needle: &[u8]) -> Option<usize> {
    hay.windows(needle.len()).position(|w| w == needle)
}

/// Sequential reader of PLY values, binary or ASCII.
trait Source {
    fn value(&mut self, ty: Scalar) -> Result<f64, IoError>;
    /// Skips `n` records of the all-scalar element `el`.
    fn skip_fixed(&mut self, el: &Element, n: usize) -> Result<(), IoError>;
    /// The most records of `min_size` bytes each that could still follow.
    fn max_records(&self, min_size: usize) -> usize;
}

struct Binary<'a, E> {
    bytes: &'a [u8],
    pos: usize,
    order: std::marker::PhantomData<E>,
}

impl<'a, E> Binary<'a, E> {
    fn new(bytes: &'a [u8]) -> Self {
        Self { bytes, pos: 0, order: std::marker::PhantomData }
    }
}

impl<E: ByteOrder> Source for Binary<'_, E> {
    #[inline]
    fn value(&mut self, ty: Scalar) -> Result<f64, IoError> {
        let end = self.pos + ty.size();
        let b = self.bytes.get(self.pos..end).ok_or_else(|| truncated("body ends early"))?;
        self.pos = end;
        Ok(ty.read::<E>(b))
    }

    fn skip_fixed(&mut self, el: &Element, n: usize) -> Result<(), IoError> {
        let len = el.fixed_size().unwrap_or(0).saturating_mul(n);
        if len > self.bytes.len() - self.pos {
            return Err(truncated(format!("element '{}' ends early", el.name)));
        }
        self.pos += len;
        Ok(())
    }

    fn max_records(&self, min_size: usize) -> usize {
        (self.bytes.len() - self.pos) / min_size.max(1)
    }
}

struct Ascii<'a> {
    tokens: std::str::SplitAsciiWhitespace<'a>,
    len: usize,
}

impl Source for Ascii<'_> {
    #[inline]
    fn value(&mut self, _ty: Scalar) -> Result<f64, IoError> {
        let t = self.tokens.next().ok_or_else(|| truncated("body ends early"))?;
        t.parse().map_err(|_| parse_err(format!("bad number '{t}'")))
    }

    fn skip_fixed(&mut self, el: &Element, n: usize) -> Result<(), IoError> {
        for _ in 0..n.saturating_mul(el.props.len()) {
            self.tokens.next().ok_or_else(|| truncated(format!("element '{}' ends early", el.name)))?;
        }
        Ok(())
    }

    fn max_records(&self, _min_size: usize) -> usize {
        // Every record holds at least one token and a separator.
        self.len / 2 + 1
    }
}

/// Parses a PLY file into an [`ImportedScan`] (normals and colours only if the file has them).
pub(crate) fn parse_ply(bytes: &[u8]) -> Result<ImportedScan, IoError> {
    let header = parse_header(bytes)?;
    let body = &bytes[header.body_start.min(bytes.len())..];
    match header.encoding {
        Encoding::BinaryLe => read_body(&header, &mut Binary::<LittleEndian>::new(body)),
        Encoding::BinaryBe => read_body(&header, &mut Binary::<BigEndian>::new(body)),
        Encoding::Ascii => {
            let text = std::str::from_utf8(body).map_err(|_| parse_err("ASCII body is not text"))?;
            read_body(&header, &mut Ascii { tokens: text.split_ascii_whitespace(), len: text.len() })
        }
    }
}

fn read_body<S: Source>(header: &Header, src: &mut S) -> Result<ImportedScan, IoError> {
    let mut scan = ImportedScan::empty(ScanFormat::Ply);
    let mut have_vertices = false;
    for el in &header.elements {
        // Reject impossible counts before allocating anything for them.
        if el.count > src.max_records(el.min_size()) {
            return Err(truncated(format!("element '{}' claims {} records, file too short", el.name, el.count)));
        }
        match el.name.as_str() {
            "vertex" if !have_vertices => {
                read_vertices(el, src, &mut scan)?;
                have_vertices = true;
            }
            "face" if scan.faces.is_none() => scan.faces = Some(read_faces(el, src)?),
            _ if el.fixed_size().is_some() => src.skip_fixed(el, el.count)?,
            _ => {
                let mut items = Vec::new();
                for _ in 0..el.count {
                    read_record(el, src, &mut [], None, &mut items)?;
                }
            }
        }
    }
    if !have_vertices {
        return Err(parse_err("no vertex element"));
    }
    if let Some(faces) = &scan.faces {
        check_faces(faces, scan.points.len())?;
    }
    Ok(scan)
}

/// Reads one record. Scalars land in `values` (by property index, when it is long enough);
/// the items of the list property `list_prop`, if given, land in `items`.
#[inline]
fn read_record<S: Source>(
    el: &Element,
    src: &mut S,
    values: &mut [f64],
    list_prop: Option<usize>,
    items: &mut Vec<f64>,
) -> Result<(), IoError> {
    for (i, p) in el.props.iter().enumerate() {
        match p.kind {
            PropKind::Scalar(ty) => {
                let v = src.value(ty)?;
                if let Some(slot) = values.get_mut(i) {
                    *slot = v;
                }
            }
            PropKind::List { count, item } => {
                let n = src.value(count)?;
                if !(0.0..=65_536.0).contains(&n) {
                    return Err(parse_err(format!("bad list length {n} in '{}'", el.name)));
                }
                let keep = list_prop == Some(i);
                if keep {
                    items.clear();
                }
                for _ in 0..n as usize {
                    let v = src.value(item)?;
                    if keep {
                        items.push(v);
                    }
                }
            }
        }
    }
    Ok(())
}

fn read_vertices<S: Source>(el: &Element, src: &mut S, scan: &mut ImportedScan) -> Result<(), IoError> {
    let [Some(px), Some(py), Some(pz)] = [el.prop(&["x"]), el.prop(&["y"]), el.prop(&["z"])] else {
        return Err(parse_err("vertex element has no x, y, z"));
    };
    let normal = match [["nx", "normal_x"], ["ny", "normal_y"], ["nz", "normal_z"]].map(|n| el.prop(&n)) {
        [Some(a), Some(b), Some(c)] => Some([a, b, c]),
        _ => None,
    };
    let color = match [["red", "diffuse_red", "r"], ["green", "diffuse_green", "g"], ["blue", "diffuse_blue", "b"]]
        .map(|n| el.prop(&n))
    {
        [Some(a), Some(b), Some(c)] => Some([a, b, c]),
        _ => None,
    };
    let alpha = el.prop(&["alpha", "diffuse_alpha", "a"]);

    scan.points.reserve_exact(el.count);
    let mut normals = normal.map(|_| Vec::with_capacity(el.count));
    let mut colors = color.map(|_| Vec::with_capacity(el.count));
    let mut values = vec![0.0f64; el.props.len()];
    let mut items = Vec::new();
    for _ in 0..el.count {
        read_record(el, src, &mut values, None, &mut items)?;
        scan.points.push([values[px] as f32, values[py] as f32, values[pz] as f32]);
        if let (Some(idx), Some(out)) = (normal, normals.as_mut()) {
            out.push(idx.map(|i| values[i] as f32));
        }
        if let (Some(idx), Some(out)) = (color, colors.as_mut()) {
            let [r, g, b] = idx.map(|i| el.scalar_type(i).to_channel(values[i]));
            let a = alpha.map_or(255, |i| el.scalar_type(i).to_channel(values[i]));
            out.push([r, g, b, a]);
        }
    }
    if normals.is_some() {
        scan.normal_source = NormalSource::File;
    }
    scan.normals = normals;
    scan.colors = colors;
    Ok(())
}

fn read_faces<S: Source>(el: &Element, src: &mut S) -> Result<Vec<[u32; 3]>, IoError> {
    let list = el
        .prop(&["vertex_indices", "vertex_index"])
        .filter(|&i| matches!(el.props[i].kind, PropKind::List { .. }))
        .ok_or_else(|| parse_err("face element has no vertex_indices list"))?;
    let mut faces = Vec::with_capacity(el.count);
    let mut items = Vec::with_capacity(8);
    for _ in 0..el.count {
        read_record(el, src, &mut [], Some(list), &mut items)?;
        triangulate_fan(&items, &mut faces)?;
    }
    Ok(faces)
}

/// Splits a polygon (vertex indices) into a triangle fan.
pub(crate) fn triangulate_fan(poly: &[f64], out: &mut Vec<[u32; 3]>) -> Result<(), IoError> {
    if poly.iter().any(|&v| !(0.0..=u32::MAX as f64).contains(&v) || v.fract() != 0.0) {
        return Err(parse_err(format!("bad face index in {poly:?}")));
    }
    for i in 1..poly.len().saturating_sub(1) {
        out.push([poly[0] as u32, poly[i] as u32, poly[i + 1] as u32]);
    }
    Ok(())
}

/// Rejects faces that refer to vertices outside `0..vertex_count`.
pub(crate) fn check_faces(faces: &[[u32; 3]], vertex_count: usize) -> Result<(), IoError> {
    match faces.iter().flatten().find(|&&i| i as usize >= vertex_count) {
        Some(i) => Err(IoError::Parse(format!("face refers to vertex {i}, but there are only {vertex_count}"))),
        None => Ok(()),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use scancraft_core::Point3D;

    const XYZ: &str = "property float x\nproperty float y\nproperty float z\n";

    fn ascii_ply(body: &str, vertex_props: &str, count: usize, faces: Option<usize>) -> Vec<u8> {
        let mut s = format!("ply\nformat ascii 1.0\ncomment test\nelement vertex {count}\n{vertex_props}");
        if let Some(f) = faces {
            s += &format!("element face {f}\nproperty list uchar int vertex_indices\n");
        }
        s += "end_header\n";
        s += body;
        s.into_bytes()
    }

    #[test]
    fn ascii_cloud_with_normals_and_float_colours() {
        let props = format!(
            "{XYZ}property float nx\nproperty float ny\nproperty float nz\n\
             property float red\nproperty float green\nproperty float blue\n"
        );
        let body = "1 2 3 0 0 2 1 0 0.5\n4 5 6 0 1 0 0 1 0\n";
        let scan = parse_ply(&ascii_ply(body, &props, 2, None)).unwrap();
        assert_eq!(scan.points, vec![[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]);
        assert_eq!(scan.normals.as_ref().unwrap()[0], [0.0, 0.0, 2.0]);
        assert_eq!(scan.colors.as_ref().unwrap()[0], [255, 0, 128, 255]);
        assert_eq!(scan.normal_source, NormalSource::File);
    }

    #[test]
    fn ascii_mesh_with_quad_and_extra_elements() {
        let mut text = String::from(
            "ply\r\nformat ascii 1.0\r\nelement vertex 4\r\nproperty double x\r\nproperty double y\r\n\
             property double z\r\nelement face 1\r\nproperty uchar flags\r\n\
             property list uchar uint vertex_index\r\nelement edge 1\r\nproperty int vertex1\r\n\
             property int vertex2\r\nend_header\r\n",
        );
        text += "0 0 0\r\n1 0 0\r\n1 1 0\r\n0 1 0\r\n7 4 0 1 2 3\r\n0 1\r\n";
        let scan = parse_ply(text.as_bytes()).unwrap();
        assert_eq!(scan.points.len(), 4);
        assert_eq!(scan.faces.unwrap(), vec![[0, 1, 2], [0, 2, 3]]);
        assert!(scan.normals.is_none());
    }

    #[test]
    fn big_endian_binary_with_skipped_element() {
        let mut b = b"ply\nformat binary_big_endian 1.0\nelement camera 1\nproperty float k\n\
element vertex 2\nproperty short x\nproperty short y\nproperty short z\n\
property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n"
            .to_vec();
        b.extend_from_slice(&1.5f32.to_be_bytes());
        for (p, c) in [([1i16, -2, 3], [10u8, 20, 30]), ([4, 5, -6], [40, 50, 60])] {
            for v in p {
                b.extend_from_slice(&v.to_be_bytes());
            }
            b.extend_from_slice(&c);
        }
        let scan = parse_ply(&b).unwrap();
        assert_eq!(scan.points, vec![[1.0, -2.0, 3.0], [4.0, 5.0, -6.0]]);
        assert_eq!(scan.colors.unwrap()[1], [40, 50, 60, 255]);
    }

    #[test]
    fn truncated_and_malformed_files_are_errors() {
        let cloud = PointCloud::from_points(vec![Point3D::new([1.0, 2.0, 3.0]); 100]);
        let path = std::env::temp_dir().join(format!("scancraft_io_trunc_{}.ply", std::process::id()));
        save_ply_binary(&cloud, &path).unwrap();
        let bytes = std::fs::read(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        assert!(parse_ply(&bytes).is_ok());
        for cut in [bytes.len() - 1, bytes.len() / 2, 40, 3] {
            assert!(parse_ply(&bytes[..cut]).is_err(), "cut at {cut} accepted");
        }
        assert!(matches!(parse_ply(&bytes[..bytes.len() - 7]), Err(IoError::Truncated(_))));

        // A huge claimed count fails fast instead of allocating.
        let huge = format!("ply\nformat binary_little_endian 1.0\nelement vertex 4000000000\n{XYZ}end_header\n");
        assert!(matches!(parse_ply(huge.as_bytes()), Err(IoError::Truncated(_))));

        let short = ascii_ply("1 2 3\n4 5\n", XYZ, 2, None);
        assert!(matches!(parse_ply(&short), Err(IoError::Truncated(_))));
        let bad_index = ascii_ply("0 0 0\n1 0 0\n0 1 0\n3 0 1 5\n", XYZ, 3, Some(1));
        assert!(parse_ply(&bad_index).is_err());
        let bad_number = ascii_ply("0 0 zero\n", XYZ, 1, None);
        assert!(matches!(parse_ply(&bad_number), Err(IoError::Parse(_))));
        assert!(parse_ply(b"ply\nformat ascii 1.0\nelement vertex 1\nproperty float q\nend_header\n1\n").is_err());
        assert!(parse_ply(b"not a ply").is_err());
    }
}
