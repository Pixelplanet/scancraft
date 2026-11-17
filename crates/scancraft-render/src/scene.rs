//! The 3D viewport renderer: point clouds, meshes and the ground grid drawn with wgpu inside
//! egui's render pass (an `egui_wgpu` paint callback).
//!
//! Geometry lives on the GPU between frames. Each layer carries a `generation`: a new generation
//! uploads the layer again; the same generation with more points uploads only the new ones,
//! so a growing live scan costs one small write per frame.

use std::sync::Arc;

use bytemuck::{Pod, Zeroable};
use egui_wgpu::wgpu;
use glam::{Mat4, Vec3};
use scancraft_core::{Mesh, PointCloud};

use crate::camera::Camera;

/// One point on the GPU: position, packed normal, colour (gamma space). 20 bytes.
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Pod, Zeroable)]
pub struct GpuPoint {
    pub position: [f32; 3],
    pub normal: [i8; 4],
    pub color: [u8; 4],
}

impl GpuPoint {
    pub fn from_point(p: &scancraft_core::Point3D) -> Self {
        let n = |v: f32| (v.clamp(-1.0, 1.0) * 127.0).round() as i8;
        GpuPoint { position: p.position, normal: [n(p.normal[0]), n(p.normal[1]), n(p.normal[2]), 0], color: [p.color.r, p.color.g, p.color.b, 255] }
    }
}

/// Convert a whole cloud.
pub fn gpu_points(cloud: &PointCloud) -> Vec<GpuPoint> {
    cloud.points.iter().map(GpuPoint::from_point).collect()
}

/// One mesh vertex on the GPU. 28 bytes.
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Pod, Zeroable)]
pub struct GpuVertex {
    pub position: [f32; 3],
    pub normal: [f32; 3],
    pub color: [u8; 4],
}

/// One grid line end.
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Pod, Zeroable)]
struct GridVertex {
    position: [f32; 3],
    color: [u8; 4],
}

/// A mesh ready for upload: vertices, triangle indices and unique edge indices.
#[derive(Debug, Clone, Default)]
pub struct GpuMesh {
    pub vertices: Vec<GpuVertex>,
    pub indices: Vec<u32>,
    pub edges: Vec<u32>,
}

impl GpuMesh {
    pub fn from_mesh(mesh: &Mesh) -> Self {
        let vertices = mesh
            .vertices
            .iter()
            .map(|v| GpuVertex { position: v.position, normal: v.normal, color: [v.color.r, v.color.g, v.color.b, 255] })
            .collect();
        let mut indices = Vec::with_capacity(mesh.triangles.len() * 3);
        let mut edges = std::collections::HashSet::with_capacity(mesh.triangles.len() * 2);
        let n = mesh.vertices.len() as u32;
        for tri in &mesh.triangles {
            let [a, b, c] = tri.indices;
            if a >= n || b >= n || c >= n {
                continue;
            }
            indices.extend_from_slice(&[a, b, c]);
            for (x, y) in [(a, b), (b, c), (c, a)] {
                edges.insert((x.min(y), x.max(y)));
            }
        }
        let mut edge_list: Vec<(u32, u32)> = edges.into_iter().collect();
        edge_list.sort_unstable();
        let edges = edge_list.into_iter().flat_map(|(a, b)| [a, b]).collect();
        GpuMesh { vertices, indices, edges }
    }
}

/// How points are coloured.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum PointShading {
    /// The colour each point carries.
    Vertex = 0,
    /// The body colour under a headlight.
    #[default]
    Lit = 1,
    /// A ramp over height.
    Height = 2,
    /// Normal directions as colours.
    Normals = 3,
}

/// How meshes are coloured.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum MeshShading {
    #[default]
    Lit = 0,
    Height = 1,
    Vertex = 2,
}

/// Points to draw this frame.
#[derive(Clone)]
pub struct PointLayer {
    pub generation: u64,
    pub points: Arc<Vec<GpuPoint>>,
    /// Draw over the model in the points' own colour, larger (a selection highlight).
    pub overlay: bool,
}

/// A mesh to draw this frame.
#[derive(Clone)]
pub struct MeshLayer {
    pub generation: u64,
    pub mesh: Arc<GpuMesh>,
    pub faces: bool,
    pub edges: bool,
}

/// The ground grid: square, centred, at height `z`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GridSpec {
    pub center: [f32; 2],
    pub z: f32,
    pub half_extent: f32,
    pub spacing: f32,
    pub major_every: u32,
    pub minor: [u8; 4],
    pub major: [u8; 4],
    pub axis_x: [u8; 4],
    pub axis_y: [u8; 4],
}

impl GridSpec {
    /// A grid under a bounding box (or round the origin with no geometry) in tidy steps.
    pub fn around(bbox: Option<&scancraft_core::BoundingBox>, minor: [u8; 4], major: [u8; 4], axis_x: [u8; 4], axis_y: [u8; 4]) -> Self {
        let (center, z, radius) = match bbox.filter(|b| !b.is_empty()) {
            Some(b) => {
                let c = b.center();
                (Vec3::new(c.x, c.y, b.min.z), b.min.z, (b.max - b.min).truncate().length().max(40.0))
            }
            None => (Vec3::new(0.0, 250.0, 0.0), 0.0, 250.0),
        };
        let spacing = nice_step(radius / 12.0);
        let half_extent = (radius * 2.0 / spacing).ceil() * spacing;
        let center = [(center.x / spacing).round() * spacing, (center.y / spacing).round() * spacing];
        GridSpec { center, z, half_extent, spacing, major_every: 10, minor, major, axis_x, axis_y }
    }

    fn lines(&self) -> Vec<GridVertex> {
        let mut out = Vec::new();
        let n = (self.half_extent / self.spacing).round() as i32;
        let [cx, cy] = self.center;
        let fade = |i: i32, c: [u8; 4]| {
            let f = 1.0 - (i.unsigned_abs() as f32 / n.max(1) as f32).powf(2.0);
            [c[0], c[1], c[2], (c[3] as f32 * f.max(0.0)) as u8]
        };
        for i in -n..=n {
            let x = cx + i as f32 * self.spacing;
            let y = cy + i as f32 * self.spacing;
            let gi_x = (x / self.spacing).round() as i64;
            let gi_y = (y / self.spacing).round() as i64;
            let col_x = if x.abs() < self.spacing * 0.01 { self.axis_y } else if gi_x % self.major_every as i64 == 0 { self.major } else { self.minor };
            let col_y = if y.abs() < self.spacing * 0.01 { self.axis_x } else if gi_y % self.major_every as i64 == 0 { self.major } else { self.minor };
            // Line of constant x, running along y.
            for (j0, j1) in [(-n, 0), (0, n)] {
                out.push(GridVertex { position: [x, cy + j0 as f32 * self.spacing, self.z], color: fade(i.abs().max(j0.abs()), col_x) });
                out.push(GridVertex { position: [x, cy + j1 as f32 * self.spacing, self.z], color: fade(i.abs().max(j1.abs()), col_x) });
                out.push(GridVertex { position: [cx + j0 as f32 * self.spacing, y, self.z], color: fade(i.abs().max(j0.abs()), col_y) });
                out.push(GridVertex { position: [cx + j1 as f32 * self.spacing, y, self.z], color: fade(i.abs().max(j1.abs()), col_y) });
            }
        }
        out
    }
}

/// The smallest of 1, 2, 5 × 10ⁿ at or above `v`.
pub fn nice_step(v: f32) -> f32 {
    let v = v.max(1e-3);
    let p = 10f32.powf(v.log10().floor());
    for m in [1.0, 2.0, 5.0, 10.0] {
        if m * p >= v {
            return m * p;
        }
    }
    10.0 * p
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Pod, Zeroable)]
struct Uniforms {
    view_proj: [[f32; 4]; 4],
    eye: [f32; 4],
    viewport: [f32; 4],
    ramp: [f32; 4],
    body: [f32; 4],
    edge: [f32; 4],
}

/// Everything one frame of the viewport needs. Built by the app and handed to egui as a paint
/// callback.
#[derive(Clone)]
pub struct SceneFrame {
    pub view_proj: Mat4,
    pub eye: Vec3,
    pub viewport_px: [f32; 2],
    pub point_size_px: f32,
    pub point_shading: PointShading,
    pub mesh_shading: MeshShading,
    pub height_range: [f32; 2],
    pub body: [f32; 3],
    pub edge: [f32; 4],
    pub points: Vec<PointLayer>,
    pub mesh: Option<MeshLayer>,
    pub grid: Option<GridSpec>,
}

impl SceneFrame {
    pub fn new(camera: &Camera, viewport_px: [f32; 2]) -> Self {
        SceneFrame {
            view_proj: camera.view_projection(),
            eye: camera.eye,
            viewport_px,
            point_size_px: 2.0,
            point_shading: PointShading::Lit,
            mesh_shading: MeshShading::Lit,
            height_range: [0.0, 100.0],
            body: [152.0 / 255.0, 156.0 / 255.0, 162.0 / 255.0],
            edge: [16.0 / 255.0, 16.0 / 255.0, 18.0 / 255.0, 1.0],
            points: Vec::new(),
            mesh: None,
            grid: None,
        }
    }

    /// The egui shape that draws this frame into `rect`.
    pub fn into_shape(self, rect: egui::Rect) -> egui::Shape {
        egui::Shape::Callback(egui_wgpu::Callback::new_paint_callback(rect, self))
    }
}

struct PointSlot {
    generation: u64,
    len: usize,
    capacity: usize,
    buffer: Option<wgpu::Buffer>,
}

impl PointSlot {
    fn new() -> Self {
        PointSlot { generation: u64::MAX, len: 0, capacity: 0, buffer: None }
    }

    fn sync(&mut self, device: &wgpu::Device, queue: &wgpu::Queue, layer: &PointLayer) {
        let size = std::mem::size_of::<GpuPoint>();
        let n = layer.points.len();
        let incremental = layer.generation == self.generation && n >= self.len && n <= self.capacity && self.buffer.is_some();
        if incremental {
            if n > self.len {
                queue.write_buffer(self.buffer.as_ref().unwrap(), (self.len * size) as u64, bytemuck::cast_slice(&layer.points[self.len..n]));
            }
        } else {
            let capacity = (n.max(4096) * 3 / 2).next_power_of_two();
            if capacity > self.capacity || self.buffer.is_none() || capacity < self.capacity / 4 {
                self.buffer = Some(device.create_buffer(&wgpu::BufferDescriptor {
                    label: Some("scancraft_points"),
                    size: (capacity * size) as u64,
                    usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
                    mapped_at_creation: false,
                }));
                self.capacity = capacity;
            }
            if n > 0 {
                queue.write_buffer(self.buffer.as_ref().unwrap(), 0, bytemuck::cast_slice(&layer.points[..]));
            }
        }
        self.generation = layer.generation;
        self.len = n;
    }
}

/// GPU state kept in egui-wgpu's callback resources.
pub struct SceneRenderer {
    uniform_buffer: wgpu::Buffer,
    bind_group: wgpu::BindGroup,
    points_pipeline: wgpu::RenderPipeline,
    overlay_pipeline: wgpu::RenderPipeline,
    mesh_pipeline: wgpu::RenderPipeline,
    edges_pipeline: wgpu::RenderPipeline,
    grid_pipeline: wgpu::RenderPipeline,
    point_slots: Vec<PointSlot>,
    mesh_generation: u64,
    mesh_vertices: Option<wgpu::Buffer>,
    mesh_indices: Option<(wgpu::Buffer, u32)>,
    mesh_edges: Option<(wgpu::Buffer, u32)>,
    grid_spec: Option<GridSpec>,
    grid: Option<(wgpu::Buffer, u32)>,
    // What to draw this frame (set in prepare).
    draw_points: Vec<(usize, bool)>,
    draw_faces: bool,
    draw_edges: bool,
    draw_grid: bool,
}

impl SceneRenderer {
    pub fn new(device: &wgpu::Device, target_format: wgpu::TextureFormat, depth_format: Option<wgpu::TextureFormat>, msaa_samples: u32) -> Self {
        let module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("scancraft_scene"),
            source: wgpu::ShaderSource::Wgsl(std::borrow::Cow::Borrowed(include_str!("scene.wgsl"))),
        });
        let uniform_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("scancraft_scene_uniforms"),
            size: std::mem::size_of::<Uniforms>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("scancraft_scene_layout"),
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX | wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Buffer { ty: wgpu::BufferBindingType::Uniform, has_dynamic_offset: false, min_binding_size: None },
                count: None,
            }],
        });
        let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("scancraft_scene_bind_group"),
            layout: &layout,
            entries: &[wgpu::BindGroupEntry { binding: 0, resource: uniform_buffer.as_entire_binding() }],
        });
        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("scancraft_scene_pipeline_layout"),
            bind_group_layouts: &[Some(&layout)],
            immediate_size: 0,
        });
        let depth = |write: bool| {
            depth_format.map(|format| wgpu::DepthStencilState {
                format,
                depth_write_enabled: Some(write),
                depth_compare: Some(wgpu::CompareFunction::LessEqual),
                stencil: wgpu::StencilState::default(),
                bias: wgpu::DepthBiasState::default(),
            })
        };
        let premultiplied = wgpu::BlendState {
            color: wgpu::BlendComponent { src_factor: wgpu::BlendFactor::One, dst_factor: wgpu::BlendFactor::OneMinusSrcAlpha, operation: wgpu::BlendOperation::Add },
            alpha: wgpu::BlendComponent { src_factor: wgpu::BlendFactor::One, dst_factor: wgpu::BlendFactor::OneMinusSrcAlpha, operation: wgpu::BlendOperation::Add },
        };
        let make = |label: &str,
                    vs: &str,
                    fs: &str,
                    buffers: &[Option<wgpu::VertexBufferLayout>],
                    topology: wgpu::PrimitiveTopology,
                    write: bool,
                    blend: Option<wgpu::BlendState>| {
            device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                label: Some(label),
                layout: Some(&pipeline_layout),
                vertex: wgpu::VertexState { module: &module, entry_point: Some(vs), buffers, compilation_options: Default::default() },
                primitive: wgpu::PrimitiveState { topology, ..Default::default() },
                depth_stencil: depth(write),
                multisample: wgpu::MultisampleState { count: msaa_samples.max(1), mask: !0, alpha_to_coverage_enabled: false },
                fragment: Some(wgpu::FragmentState {
                    module: &module,
                    entry_point: Some(fs),
                    targets: &[Some(wgpu::ColorTargetState { format: target_format, blend, write_mask: wgpu::ColorWrites::ALL })],
                    compilation_options: Default::default(),
                }),
                multiview_mask: None,
                cache: None,
            })
        };
        let point_attrs = wgpu::vertex_attr_array![0 => Float32x3, 1 => Snorm8x4, 2 => Unorm8x4];
        let mesh_attrs = wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32x3, 2 => Unorm8x4];
        let grid_attrs = wgpu::vertex_attr_array![0 => Float32x3, 1 => Unorm8x4];
        let point_layout = wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<GpuPoint>() as u64,
            step_mode: wgpu::VertexStepMode::Instance,
            attributes: &point_attrs,
        };
        let mesh_layout = wgpu::VertexBufferLayout { array_stride: std::mem::size_of::<GpuVertex>() as u64, step_mode: wgpu::VertexStepMode::Vertex, attributes: &mesh_attrs };
        let grid_layout = wgpu::VertexBufferLayout { array_stride: std::mem::size_of::<GridVertex>() as u64, step_mode: wgpu::VertexStepMode::Vertex, attributes: &grid_attrs };
        let points_pipeline = make("scancraft_points", "vs_points", "fs_points", &[Some(point_layout.clone())], wgpu::PrimitiveTopology::TriangleList, true, None);
        let overlay_pipeline = make("scancraft_points_overlay", "vs_points_overlay", "fs_points", &[Some(point_layout)], wgpu::PrimitiveTopology::TriangleList, true, None);
        let mesh_pipeline = make("scancraft_mesh", "vs_mesh", "fs_mesh", &[Some(mesh_layout.clone())], wgpu::PrimitiveTopology::TriangleList, true, None);
        let edges_pipeline = make("scancraft_edges", "vs_edges", "fs_edges", &[Some(mesh_layout)], wgpu::PrimitiveTopology::LineList, false, Some(premultiplied));
        let grid_pipeline = make("scancraft_grid", "vs_grid", "fs_grid", &[Some(grid_layout)], wgpu::PrimitiveTopology::LineList, false, Some(premultiplied));
        SceneRenderer {
            uniform_buffer,
            bind_group,
            points_pipeline,
            overlay_pipeline,
            mesh_pipeline,
            edges_pipeline,
            grid_pipeline,
            point_slots: Vec::new(),
            mesh_generation: u64::MAX,
            mesh_vertices: None,
            mesh_indices: None,
            mesh_edges: None,
            grid_spec: None,
            grid: None,
            draw_points: Vec::new(),
            draw_faces: false,
            draw_edges: false,
            draw_grid: false,
        }
    }

    fn prepare_frame(&mut self, device: &wgpu::Device, queue: &wgpu::Queue, frame: &SceneFrame) {
        let uniforms = Uniforms {
            view_proj: frame.view_proj.to_cols_array_2d(),
            eye: [frame.eye.x, frame.eye.y, frame.eye.z, 1.0],
            viewport: [frame.viewport_px[0].max(1.0), frame.viewport_px[1].max(1.0), frame.point_size_px, frame.point_shading as u32 as f32],
            ramp: [frame.height_range[0], frame.height_range[1], frame.mesh_shading as u32 as f32, 0.0],
            body: [frame.body[0], frame.body[1], frame.body[2], 1.0],
            edge: frame.edge,
        };
        queue.write_buffer(&self.uniform_buffer, 0, bytemuck::bytes_of(&uniforms));

        while self.point_slots.len() < frame.points.len() {
            self.point_slots.push(PointSlot::new());
        }
        self.draw_points.clear();
        for (i, layer) in frame.points.iter().enumerate() {
            self.point_slots[i].sync(device, queue, layer);
            if self.point_slots[i].len > 0 {
                self.draw_points.push((i, layer.overlay));
            }
        }

        self.draw_faces = false;
        self.draw_edges = false;
        if let Some(layer) = &frame.mesh {
            if layer.generation != self.mesh_generation {
                use wgpu::util::DeviceExt;
                let m = &layer.mesh;
                self.mesh_vertices = (!m.vertices.is_empty()).then(|| {
                    device.create_buffer_init(&wgpu::util::BufferInitDescriptor { label: Some("scancraft_mesh_vertices"), contents: bytemuck::cast_slice(&m.vertices), usage: wgpu::BufferUsages::VERTEX })
                });
                self.mesh_indices = (!m.indices.is_empty()).then(|| {
                    (
                        device.create_buffer_init(&wgpu::util::BufferInitDescriptor { label: Some("scancraft_mesh_indices"), contents: bytemuck::cast_slice(&m.indices), usage: wgpu::BufferUsages::INDEX }),
                        m.indices.len() as u32,
                    )
                });
                self.mesh_edges = (!m.edges.is_empty()).then(|| {
                    (
                        device.create_buffer_init(&wgpu::util::BufferInitDescriptor { label: Some("scancraft_mesh_edges"), contents: bytemuck::cast_slice(&m.edges), usage: wgpu::BufferUsages::INDEX }),
                        m.edges.len() as u32,
                    )
                });
                self.mesh_generation = layer.generation;
            }
            self.draw_faces = layer.faces && self.mesh_vertices.is_some() && self.mesh_indices.is_some();
            self.draw_edges = layer.edges && self.mesh_vertices.is_some() && self.mesh_edges.is_some();
        }

        self.draw_grid = false;
        if let Some(spec) = frame.grid {
            if self.grid_spec != Some(spec) {
                use wgpu::util::DeviceExt;
                let lines = spec.lines();
                self.grid = (!lines.is_empty()).then(|| {
                    (
                        device.create_buffer_init(&wgpu::util::BufferInitDescriptor { label: Some("scancraft_grid"), contents: bytemuck::cast_slice(&lines), usage: wgpu::BufferUsages::VERTEX }),
                        lines.len() as u32,
                    )
                });
                self.grid_spec = Some(spec);
            }
            self.draw_grid = self.grid.is_some();
        }
    }

    fn paint_frame(&self, render_pass: &mut wgpu::RenderPass<'static>) {
        render_pass.set_bind_group(0, &self.bind_group, &[]);
        if self.draw_grid {
            if let Some((buffer, count)) = &self.grid {
                render_pass.set_pipeline(&self.grid_pipeline);
                render_pass.set_vertex_buffer(0, buffer.slice(..));
                render_pass.draw(0..*count, 0..1);
            }
        }
        if self.draw_faces {
            if let (Some(v), Some((i, n))) = (&self.mesh_vertices, &self.mesh_indices) {
                render_pass.set_pipeline(&self.mesh_pipeline);
                render_pass.set_vertex_buffer(0, v.slice(..));
                render_pass.set_index_buffer(i.slice(..), wgpu::IndexFormat::Uint32);
                render_pass.draw_indexed(0..*n, 0, 0..1);
            }
        }
        if self.draw_edges {
            if let (Some(v), Some((i, n))) = (&self.mesh_vertices, &self.mesh_edges) {
                render_pass.set_pipeline(&self.edges_pipeline);
                render_pass.set_vertex_buffer(0, v.slice(..));
                render_pass.set_index_buffer(i.slice(..), wgpu::IndexFormat::Uint32);
                render_pass.draw_indexed(0..*n, 0, 0..1);
            }
        }
        // Overlays last, so they sit on top of the model.
        let order = self.draw_points.iter().filter(|(_, o)| !o).chain(self.draw_points.iter().filter(|(_, o)| *o));
        for &(i, overlay) in order {
            let slot = &self.point_slots[i];
            if let Some(buffer) = &slot.buffer {
                render_pass.set_pipeline(if overlay { &self.overlay_pipeline } else { &self.points_pipeline });
                render_pass.set_vertex_buffer(0, buffer.slice(..(slot.len * std::mem::size_of::<GpuPoint>()) as u64));
                render_pass.draw(0..6, 0..slot.len as u32);
            }
        }
    }
}

impl egui_wgpu::CallbackTrait for SceneFrame {
    fn prepare(
        &self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        _screen_descriptor: &egui_wgpu::ScreenDescriptor,
        _egui_encoder: &mut wgpu::CommandEncoder,
        resources: &mut egui_wgpu::CallbackResources,
    ) -> Vec<wgpu::CommandBuffer> {
        if let Some(renderer) = resources.get_mut::<SceneRenderer>() {
            renderer.prepare_frame(device, queue, self);
        }
        Vec::new()
    }

    fn paint(&self, _info: egui::PaintCallbackInfo, render_pass: &mut wgpu::RenderPass<'static>, resources: &egui_wgpu::CallbackResources) {
        if let Some(renderer) = resources.get::<SceneRenderer>() {
            renderer.paint_frame(render_pass);
        }
    }
}

/// Install the renderer into egui-wgpu's callback resources.
pub fn install(render_state: &egui_wgpu::RenderState, depth_format: Option<wgpu::TextureFormat>, msaa_samples: u32) {
    let renderer = SceneRenderer::new(&render_state.device, render_state.target_format, depth_format, msaa_samples);
    render_state.renderer.write().callback_resources.insert(renderer);
}

#[cfg(test)]
mod tests {
    use super::*;
    use scancraft_core::{Point3D, Triangle};

    #[test]
    fn nice_steps_are_1_2_5() {
        assert_eq!(nice_step(0.7), 1.0);
        assert_eq!(nice_step(1.2), 2.0);
        assert_eq!(nice_step(3.0), 5.0);
        assert_eq!(nice_step(7.0), 10.0);
        assert_eq!(nice_step(42.0), 50.0);
    }

    #[test]
    fn mesh_edges_are_unique() {
        let v = |x: f32, y: f32| Point3D::new([x, y, 0.0]);
        let mesh = Mesh::from_geometry(vec![v(0.0, 0.0), v(1.0, 0.0), v(1.0, 1.0), v(0.0, 1.0)], vec![Triangle::new(0, 1, 2), Triangle::new(0, 2, 3), Triangle::new(0, 9, 2)]);
        let gpu = GpuMesh::from_mesh(&mesh);
        assert_eq!(gpu.indices.len(), 6, "out-of-range triangle dropped");
        assert_eq!(gpu.edges.len() / 2, 5);
    }

    #[test]
    fn points_pack_normals() {
        let p = Point3D::new([1.0, 2.0, 3.0]).with_normal([0.0, 0.0, 1.0]);
        let g = GpuPoint::from_point(&p);
        assert_eq!(g.normal, [0, 0, 127, 0]);
        assert_eq!(std::mem::size_of::<GpuPoint>(), 20);
        assert_eq!(std::mem::size_of::<GpuVertex>(), 28);
    }

    #[test]
    fn grid_sits_under_the_geometry() {
        let bbox = scancraft_core::BoundingBox::from_arrays([-50.0, 150.0, 10.0], [50.0, 350.0, 90.0]);
        let g = GridSpec::around(Some(&bbox), [1; 4], [2; 4], [3; 4], [4; 4]);
        assert_eq!(g.z, 10.0);
        assert!(g.half_extent >= 100.0);
        assert!(!g.lines().is_empty());
    }
}
