//! hagi: Hardware Accelerated Graphical Interface helpers.
//!
//! This crate lives outside `sols` so it can be reused by core_front_space,
//! core_front_earth, core_front_helios, Makepad/PADS, WebGPU scene hosts,
//! and future renderer frontends.

pub mod camera;
pub mod input;
pub mod kinematics;
pub mod scene;

pub use camera::{ClipPlanes, SeenCamera};
pub use input::{CameraCommand, CameraControlMode};
pub use kinematics::R3Kinematics;
pub use scene::{SceneObject, SeenSceneController};
