use glam::DVec3;

use crate::camera::SeenCamera;
use crate::input::{CameraCommand, CameraControlMode};
use crate::kinematics::R3Kinematics;

#[derive(Debug)]
pub struct SceneObject {
    pub name: String,
    pub kinematics: R3Kinematics,
    pub visual_radius: f64,
}

impl SceneObject {
    pub fn new(name: impl Into<String>, position: DVec3, visual_radius: f64) -> Self {
        Self {
            name: name.into(),
            kinematics: R3Kinematics::with_position(position),
            visual_radius,
        }
    }
}

#[derive(Debug, Clone)]
pub struct SeenSceneController {
    pub camera: SeenCamera,
    pub mode: CameraControlMode,
    pub selected_target: Option<String>,
}

impl Default for SeenSceneController {
    fn default() -> Self {
        Self {
            camera: SeenCamera::default(),
            mode: CameraControlMode::ObjectOrbit,
            selected_target: None,
        }
    }
}

impl SeenSceneController {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn set_mode(&mut self, mode: CameraControlMode) {
        self.mode = mode;
    }

    pub fn select_target(&mut self, target: impl Into<String>) {
        self.selected_target = Some(target.into());
    }

    pub fn apply_command(&mut self, command: CameraCommand) {
        match command {
            CameraCommand::MoveLocal { delta } => self.camera.move_camera_local(delta),
            CameraCommand::MoveWorld { delta } => self.camera.move_world(delta),
            CameraCommand::MoveTo { point } => self.camera.move_to(point),
            CameraCommand::LookUp { radians } => self.camera.look_up(radians),
            CameraCommand::LookRight { radians } => self.camera.look_right(radians),
            CameraCommand::Spin { radians } => self.camera.spin(radians),
            CameraCommand::LookAt { point } => self.camera.look_at(point),
            CameraCommand::OrbitFocusPixels {
                focus,
                object_radius,
                dx_pixels,
                dy_pixels,
                viewport_width,
                viewport_height,
            } => self.camera.orbit_focus_pixels_on_sphere(
                focus,
                object_radius,
                dx_pixels,
                dy_pixels,
                viewport_width,
                viewport_height,
            ),
            CameraCommand::ZoomLocal { amount } => {
                // Compatibility name: this is no longer a fake lens/object zoom.
                // It is pure camera translation along quaternion forward.
                self.camera.move_forward(amount);
            }
        }
    }

    pub fn frame_target_close_orbit(&mut self, target_position: DVec3, radius: f64) {
        let radius = radius.max(1e-6);
        self.camera.position = target_position + DVec3::new(0.0, 0.0, radius);
        self.camera.look_at(target_position);
    }

    pub fn apply_drag(
        &mut self,
        target_position: Option<DVec3>,
        target_radius: Option<f64>,
        dx_pixels: f64,
        dy_pixels: f64,
        viewport_width: f64,
        viewport_height: f64,
    ) {
        match self.mode {
            CameraControlMode::Free => {
                // Free camera drag is a pure view rotation in camera-local space.
                // Do not move the camera, orbit a focus, touch object transforms,
                // scale objects, or change FOV/clip planes.
                self.camera.rotate_view_pixels_in_place(
                    dx_pixels,
                    dy_pixels,
                    viewport_width,
                    viewport_height,
                );
            }
            CameraControlMode::ObjectOrbit => {
                if let Some(focus) = target_position {
                    self.camera.orbit_focus_pixels_on_sphere(
                        focus,
                        target_radius.unwrap_or(1.0),
                        dx_pixels,
                        dy_pixels,
                        viewport_width,
                        viewport_height,
                    );
                }
            }
        }
    }
}

#[cfg(test)]
mod zoom_semantics_tests {
    use super::*;
    use glam::DQuat;

    #[test]
    fn zoom_local_moves_camera_forward_without_scaling_scene() {
        let mut controller = SeenSceneController::new();
        controller.camera.position = DVec3::new(1.0, 2.0, 3.0);
        controller.camera.orientation =
            DQuat::from_axis_angle(DVec3::Y, std::f64::consts::FRAC_PI_2).normalize();

        let before = controller.camera.position;
        let forward = controller.camera.forward();
        let fov_before = controller.camera.fov_y_degrees;
        let clip_before = controller.camera.clip;

        controller.apply_command(CameraCommand::ZoomLocal { amount: 1000.0 });

        assert!((controller.camera.position - (before + forward * 1000.0)).length() < 1.0e-9);
        assert_eq!(controller.camera.fov_y_degrees, fov_before);
        assert_eq!(controller.camera.clip, clip_before);
        assert!((controller.camera.orientation.length_squared() - 1.0).abs() < 1.0e-12);
    }
}

#[cfg(test)]
mod free_camera_drag_tests {
    use super::*;

    #[test]
    fn free_drag_rotates_only_orientation_not_position() {
        let mut controller = SeenSceneController::new();
        controller.set_mode(CameraControlMode::Free);
        controller.camera.position = DVec3::new(123.0, -456.0, 789.0);
        controller.camera.fov_y_degrees = 60.0;
        controller.camera.set_clip(3.0, 9_000_000.0);

        let pos_before = controller.camera.position;
        let fov_before = controller.camera.fov_y_degrees;
        let clip_before = controller.camera.clip;
        let forward_before = controller.camera.forward();

        controller.apply_drag(None, None, 120.0, -60.0, 1200.0, 800.0);

        assert!((controller.camera.position - pos_before).length() < 1.0e-12);
        assert_eq!(controller.camera.fov_y_degrees, fov_before);
        assert_eq!(controller.camera.clip, clip_before);
        assert!((controller.camera.orientation.length_squared() - 1.0).abs() < 1.0e-12);
        assert!(controller.camera.forward().dot(forward_before) < 0.999);
    }
}
