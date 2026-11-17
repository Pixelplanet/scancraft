// ScanCraft viewport shaders: point sprites, lit meshes, mesh edges and the ground grid.
// Output is gamma-space colour (egui-wgpu renders into a non-sRGB target).

struct Uniforms {
    view_proj: mat4x4<f32>,
    eye: vec4<f32>,
    // x, y: viewport size in pixels; z: point size in pixels; w: point shading mode.
    viewport: vec4<f32>,
    // x: low z, y: high z of the height ramp; z: mesh shading mode; w: unused.
    ramp: vec4<f32>,
    // Body colour (gamma space) for lit shading.
    body: vec4<f32>,
    // Edge colour.
    edge: vec4<f32>,
};

@group(0) @binding(0) var<uniform> u: Uniforms;

fn height_ramp(z: f32) -> vec3<f32> {
    let t = clamp((z - u.ramp.x) / max(u.ramp.y - u.ramp.x, 1e-3), 0.0, 1.0);
    // Calm blue -> teal -> green -> amber ramp.
    let c0 = vec3<f32>(0.23, 0.42, 0.78);
    let c1 = vec3<f32>(0.18, 0.66, 0.70);
    let c2 = vec3<f32>(0.45, 0.76, 0.40);
    let c3 = vec3<f32>(0.93, 0.70, 0.30);
    if (t < 0.333) {
        return mix(c0, c1, t / 0.333);
    } else if (t < 0.666) {
        return mix(c1, c2, (t - 0.333) / 0.333);
    }
    return mix(c2, c3, (t - 0.666) / 0.334);
}

fn headlight(normal: vec3<f32>, position: vec3<f32>) -> f32 {
    let n = normalize(normal);
    let to_eye = normalize(u.eye.xyz - position);
    // Two-sided: scan normals may face either way.
    let d = abs(dot(n, to_eye));
    // A second, fixed key light from above keeps relief readable head-on.
    let key = abs(dot(n, normalize(vec3<f32>(0.35, -0.45, 0.82))));
    return 0.28 + 0.55 * d + 0.22 * key;
}

// ---------------------------------------------------------------- points

struct PointIn {
    @location(0) position: vec3<f32>,
    @location(1) normal: vec4<f32>,
    @location(2) color: vec4<f32>,
};

struct PointOut {
    @builtin(position) clip: vec4<f32>,
    @location(0) corner: vec2<f32>,
    @location(1) color: vec3<f32>,
};

@vertex
fn vs_points(@builtin(vertex_index) vi: u32, p: PointIn) -> PointOut {
    var corners = array<vec2<f32>, 6>(
        vec2<f32>(-1.0, -1.0), vec2<f32>(1.0, -1.0), vec2<f32>(1.0, 1.0),
        vec2<f32>(-1.0, -1.0), vec2<f32>(1.0, 1.0), vec2<f32>(-1.0, 1.0)
    );
    let corner = corners[vi];
    var out: PointOut;
    let clip = u.view_proj * vec4<f32>(p.position, 1.0);
    let size = u.viewport.z;
    let offset = corner * size / u.viewport.xy * clip.w;
    out.clip = vec4<f32>(clip.xy + offset, clip.z, clip.w);
    out.corner = corner;
    let mode = u32(u.viewport.w);
    var color = p.color.rgb;
    if (mode == 1u) {
        color = u.body.rgb * headlight(p.normal.xyz, p.position);
    } else if (mode == 2u) {
        color = height_ramp(p.position.z) * headlight(p.normal.xyz, p.position);
    } else if (mode == 3u) {
        color = normalize(p.normal.xyz + vec3<f32>(1e-5)) * 0.5 + vec3<f32>(0.5);
    }
    out.color = color;
    return out;
}

@fragment
fn fs_points(in: PointOut) -> @location(0) vec4<f32> {
    if (dot(in.corner, in.corner) > 1.0) {
        discard;
    }
    return vec4<f32>(in.color, 1.0);
}

// ---------------------------------------------------------------- mesh

struct MeshIn {
    @location(0) position: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) color: vec4<f32>,
};

struct MeshOut {
    @builtin(position) clip: vec4<f32>,
    @location(0) world: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) color: vec3<f32>,
};

@vertex
fn vs_mesh(m: MeshIn) -> MeshOut {
    var out: MeshOut;
    out.clip = u.view_proj * vec4<f32>(m.position, 1.0);
    out.world = m.position;
    out.normal = m.normal;
    out.color = m.color.rgb;
    return out;
}

@fragment
fn fs_mesh(in: MeshOut) -> @location(0) vec4<f32> {
    // Face normal from screen-space derivatives: crisp facets, robust to bad vertex normals.
    let face = normalize(cross(dpdx(in.world), dpdy(in.world)));
    let mode = u32(u.ramp.z);
    var base = u.body.rgb;
    if (mode == 1u) {
        base = height_ramp(in.world.z);
    } else if (mode == 2u) {
        base = in.color;
    }
    return vec4<f32>(base * headlight(face, in.world), 1.0);
}

@vertex
fn vs_edges(m: MeshIn) -> @builtin(position) vec4<f32> {
    var clip = u.view_proj * vec4<f32>(m.position, 1.0);
    // Pull edges slightly towards the eye so they win over their own faces.
    clip.z = clip.z - 2e-5 * clip.w;
    return clip;
}

@fragment
fn fs_edges() -> @location(0) vec4<f32> {
    return u.edge;
}

// ---------------------------------------------------------------- grid

struct GridIn {
    @location(0) position: vec3<f32>,
    @location(1) color: vec4<f32>,
};

struct GridOut {
    @builtin(position) clip: vec4<f32>,
    @location(0) color: vec4<f32>,
};

@vertex
fn vs_grid(g: GridIn) -> GridOut {
    var out: GridOut;
    out.clip = u.view_proj * vec4<f32>(g.position, 1.0);
    out.color = g.color;
    return out;
}

@fragment
fn fs_grid(in: GridOut) -> @location(0) vec4<f32> {
    // Premultiplied alpha.
    return vec4<f32>(in.color.rgb * in.color.a, in.color.a);
}

// ---------------------------------------------------------------- overlay points

// Points drawn over the model in their own colour (a selected frame), larger and pulled towards
// the eye so they read on top of the surface they belong to.
@vertex
fn vs_points_overlay(@builtin(vertex_index) vi: u32, p: PointIn) -> PointOut {
    var corners = array<vec2<f32>, 6>(
        vec2<f32>(-1.0, -1.0), vec2<f32>(1.0, -1.0), vec2<f32>(1.0, 1.0),
        vec2<f32>(-1.0, -1.0), vec2<f32>(1.0, 1.0), vec2<f32>(-1.0, 1.0)
    );
    let corner = corners[vi];
    var out: PointOut;
    let clip = u.view_proj * vec4<f32>(p.position, 1.0);
    // Alpha 255 (a highlighted frame): 1.6 x the point size; otherwise alpha / 32 x (marker discs).
    let a = p.color.a * 255.0;
    let mult = select(a / 32.0, 1.6, a > 254.5);
    let size = u.viewport.z * mult;
    let offset = corner * size / u.viewport.xy * clip.w;
    out.clip = vec4<f32>(clip.xy + offset, clip.z - 4e-4 * clip.w, clip.w);
    out.corner = corner;
    out.color = p.color.rgb;
    return out;
}
