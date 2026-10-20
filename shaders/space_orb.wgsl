// space_orb.wgsl
//
// 00d_better_shaders patch
//
// Shader modes (carried in instance.material[0] as an integer bit-cast):
//   0 = FLAT           — base color only, no lighting (old behaviour)
//   1 = PHONG          — diffuse + ambient + specular, base-color tinted
//   2 = PHONG_TEXTURE  — Phong + 2-D array texture sample (planet maps)
//   3 = EARTH          — full Earth pass: Phong + texture + night-side dimming
//                        + specular ocean highlight via a per-fragment ocean mask
//   4 = WIREFRAME      — edge-highlight overlay (polygon mode fill + tinted dark)
//   5 = DEPTH_VIZ      — debug: depth encoded as rainbow gradient
//   6 = GL_DIFFUSE     — clone of old SDL/OpenGL vertex-lit diffuse shader
//
// The mode index is packed into material[0] as an f32-cast u32.
// material[1] = texture layer index (u32 cast)
// material[2] = roughness / specular exponent scale  (0.0–1.0)
// material[3] = reserved / padding

// ────────────────────────────────────────────────────────────── bind groups ──

struct Uniforms {
    view_proj: mat4x4<f32>,
    // Packed extra uniforms (extended from the old single-matrix layout).
    // [0..3]  = camera world position (x,y,z,pad)
    // [4..7]  = sun world position   (x,y,z,pad)
    // [8..11] = time (seconds), render flags, pad, pad
    camera_world_pos: vec4<f32>,
    sun_world_pos:    vec4<f32>,
    time_flags:       vec4<f32>,  // x=time_s, y=flags(bitmask), z=pad, w=pad
}

@group(0) @binding(0)
var<uniform> u: Uniforms;

struct InstanceData {
    // xyz = camera/view-space centre (km), w = visual_radius (km)
    center_radius: vec4<f32>,
    // base colour RGBA
    color:         vec4<f32>,
    // x = shader_mode (u32 as f32), y = tex_layer (u32 as f32),
    // z = roughness, w = reserved
    material:      vec4<f32>,

    // object-local inertial basis, expressed in camera/view-space.
    // These keep asymmetric STL meshes stationary in their local frame while
    // the camera rotates around them.
    local_x:       vec4<f32>,
    local_y:       vec4<f32>,
    local_z:       vec4<f32>,
}

@group(1) @binding(0)
var<storage, read> instances: array<InstanceData>;

@group(2) @binding(0) var tex_array:  texture_2d_array<f32>;
@group(2) @binding(1) var tex_sampler: sampler;

// ──────────────────────────────────────────────────────────── vertex stage ──

struct VertexIn {
    @location(0) position: vec3<f32>,
    @location(1) normal:   vec3<f32>,
    @location(2) uv:       vec2<f32>,
    @builtin(instance_index) inst_idx: u32,
}

struct VertexOut {
    @builtin(position) clip_pos:    vec4<f32>,
    @location(0)       world_pos:   vec3<f32>,  // camera/view-space position; name kept for runtime-shader compatibility
    @location(1)       world_norm:  vec3<f32>,  // local/view normal; name kept for runtime-shader compatibility
    @location(2)       uv:          vec2<f32>,
    @location(3)       base_color:  vec4<f32>,
    // flat (no interpolation) integers carried as floats
    @location(4)       shader_mode: f32,
    @location(5)       tex_layer:   f32,
    @location(6)       roughness:   f32,
}

@vertex
fn vs_main(in: VertexIn) -> VertexOut {
    let inst = instances[in.inst_idx];

    let center = inst.center_radius.xyz;
    let radius = inst.center_radius.w;

    // Local-inertial-object-frame draw:
    //   position is local unit-mesh/object space
    //   radius scales the local object
    //   center is already camera/view-space, computed on CPU in f64
    //   local_x/y/z rotate local STL vertices into camera/view-space
    //
    // This keeps imported asymmetric meshes, such as the pyramid, stationary in
    // their local inertial frame while the camera orbits around them.
    let local_view_pos =
        in.position.x * inst.local_x.xyz +
        in.position.y * inst.local_y.xyz +
        in.position.z * inst.local_z.xyz;

    let view_pos = local_view_pos * radius + center;

    let world_norm = normalize(
        in.normal.x * inst.local_x.xyz +
        in.normal.y * inst.local_y.xyz +
        in.normal.z * inst.local_z.xyz
    );

    var out: VertexOut;
    out.clip_pos   = u.view_proj * vec4<f32>(view_pos, 1.0);
    out.world_pos  = view_pos;
    out.world_norm = world_norm;
    out.uv         = in.uv;
    out.base_color = inst.color;
    out.shader_mode = inst.material.x;
    out.tex_layer   = inst.material.y;
    out.roughness   = inst.material.z;
    return out;
}

// ──────────────────────────────────────────────────────────── helpers ──────

const AMBIENT: f32 = 0.12;

fn phong_diffuse(world_norm: vec3<f32>, world_pos: vec3<f32>, sun_pos: vec3<f32>) -> f32 {
    let to_sun = normalize(sun_pos - world_pos);
    return max(dot(world_norm, to_sun), 0.0);
}

fn phong_specular(
    world_norm: vec3<f32>,
    world_pos:  vec3<f32>,
    sun_pos:    vec3<f32>,
    cam_pos:    vec3<f32>,
    exponent:   f32,
) -> f32 {
    let to_sun = normalize(sun_pos - world_pos);
    let to_cam = normalize(cam_pos - world_pos);
    let half_v = normalize(to_sun + to_cam);
    return pow(max(dot(world_norm, half_v), 0.0), exponent);
}

// Crude ocean mask: returns 1.0 for fragments that look ocean-blue-ish.
// For the real Earth texture this picks out the water regions well enough
// without needing a separate spec-mask texture.
fn ocean_mask(col: vec3<f32>) -> f32 {
    let blueness  = col.b - max(col.r, col.g);
    return clamp(blueness * 4.0, 0.0, 1.0);
}

// ──────────────────────────────────────────────────────────── fragment stage

@fragment
fn fs_main(in: VertexOut) -> @location(0) vec4<f32> {
    // Consolidated shader modes:
    //   0 = flat
    //   1 = smooth  (safe Lambert)
    //   2 = texture (future real texture path; currently aliases to smooth)
    //
    // Removed old broken modes:
    //   Phong, Earth, fake Wireframe, DepthViz, GlDiffuse.
    //
    // Texture mode intentionally does NOT sample tex_array yet. The next patch
    // should wire a real texture-loading path only into mode 2.

    var mode: u32 = 1u;
    if in.shader_mode >= 0.0 && in.shader_mode <= 2.0 {
        mode = u32(in.shader_mode);
    }

    let base_col = vec4<f32>(
        clamp(in.base_color.rgb, vec3<f32>(0.0), vec3<f32>(1.0)),
        clamp(in.base_color.a, 0.0, 1.0)
    );

    if mode == 0u {
        return base_col;
    }

    let n = normalize(in.world_norm);

    // u.sun_world_pos.xyz is camera/view-space in the current renderer.
    // Use it instead of a constant light vector so camera orbit/drag has
    // visible lighting feedback on centered planets.
    let to_sun = normalize(u.sun_world_pos.xyz - in.world_pos);
    let lambert = 0.25 + 0.75 * max(dot(n, to_sun), 0.0);

    // mode 1 smooth
    if mode == 1u {
        return vec4<f32>(base_col.rgb * lambert, base_col.a);
    }

    // mode 2 texture placeholder.
    // For now it is intentionally the same as smooth. Real texture sampling
    // goes here later, not in any legacy shader path.
    if mode == 2u {
        return vec4<f32>(base_col.rgb * lambert, base_col.a);
    }

    return vec4<f32>(1.0, 0.0, 1.0, 1.0);
}
