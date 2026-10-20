//! Reusable WGSL sources. The host owns GPU resources and bind groups.
//! Vertex coordinates use Hagi world-to-view bases and WebGPU 0..1 clip depth.
pub const MESH_WGSL: &str = include_str!("../shaders/space_orb.wgsl");
pub const ORBIT_LINES_WGSL: &str = include_str!("../shaders/orbit_lines.wgsl");
