use glam::{DMat4, DQuat, DVec3};

use crate::kinematics::normalized_or;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ClipPlanes {
    pub near: f64,
    pub far: f64,
}

impl ClipPlanes {
    pub fn new(near: f64, far: f64) -> Self {
        let near = near.max(1e-6);
        let far = far.max(near * 16.0);
        Self { near, far }
    }
}

impl Default for ClipPlanes {
    fn default() -> Self {
        Self {
            near: 1.0,
            far: 1024.0,
        }
    }
}

/// Quaternion camera preserving the useful old `SdlGlScene` controls:
///
/// - local movement is transformed by current orientation
/// - look_up/look_right/spin pre-multiply orientation quaternions
/// - look_at rotates the current forward vector onto the target vector
#[derive(Debug, Clone)]
pub struct SeenCamera {
    pub position: DVec3,
    pub orientation: DQuat,
    pub fov_y_degrees: f64,
    pub clip: ClipPlanes,
}

impl Default for SeenCamera {
    fn default() -> Self {
        Self {
            position: DVec3::ZERO,
            orientation: DQuat::IDENTITY,
            fov_y_degrees: 64.0,
            clip: ClipPlanes::default(),
        }
    }
}

impl SeenCamera {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn with_position(position: DVec3) -> Self {
        Self {
            position,
            ..Self::default()
        }
    }

    pub fn set_clip(&mut self, near: f64, far: f64) {
        self.clip = ClipPlanes::new(near, far);
    }

    pub fn right(&self) -> DVec3 {
        self.orientation * DVec3::X
    }

    pub fn up(&self) -> DVec3 {
        self.orientation * DVec3::Y
    }

    /// Default camera forward is -Z, matching the old OpenGL camera convention.
    pub fn forward(&self) -> DVec3 {
        self.orientation * -DVec3::Z
    }

    /// Move in camera coordinates.
    pub fn move_camera_local(&mut self, delta: DVec3) {
        self.position += self.orientation * delta;
    }

    /// Move along the current quaternion-derived camera forward vector.
    ///
    /// Positive distance moves in the direction the camera is facing.
    /// Negative distance moves backward. This changes camera position only;
    /// it never changes FOV, clip planes, object radius, model matrix, or scale.
    pub fn move_forward(&mut self, distance: f64) {
        if !distance.is_finite() || distance == 0.0 {
            return;
        }

        self.position += self.forward() * distance;
    }

    /// Move directly in world/spatial coordinates.
    pub fn move_world(&mut self, delta: DVec3) {
        self.position += delta;
    }

    pub fn move_to(&mut self, point: DVec3) {
        self.position = point;
    }

    /// Pitch around camera-local X/right style axis, preserving old sign.
    pub fn look_up(&mut self, radians: f64) {
        let q = DQuat::from_axis_angle(DVec3::X, -radians);
        self.orientation = (q.normalize() * self.orientation).normalize();
    }

    /// Yaw around Y axis, preserving old function behavior.
    pub fn look_right(&mut self, radians: f64) {
        let q = DQuat::from_axis_angle(DVec3::Y, radians);
        self.orientation = (q.normalize() * self.orientation).normalize();
    }

    /// Roll/spin around Z axis, preserving old function behavior.
    pub fn spin(&mut self, radians: f64) {
        let q = DQuat::from_axis_angle(DVec3::Z, radians);
        self.orientation = (q.normalize() * self.orientation).normalize();
    }

    /// Free-camera viewport drag.
    ///
    /// This rotates only the camera orientation in the camera's own local frame.
    /// It never changes position, target/focus, FOV, clip planes, object radius,
    /// model matrices, or scale. This is the free-camera equivalent of turning
    /// your head: the pointing vector changes, the camera origin does not.
    ///
    /// A full viewport drag maps to approximately the visible field of view,
    /// not to TAU/PI. That prevents a tiny mouse drag from spinning the camera
    /// completely away from the object being inspected.
    pub fn rotate_view_pixels_in_place(
        &mut self,
        dx_pixels: f64,
        dy_pixels: f64,
        viewport_width: f64,
        viewport_height: f64,
    ) {
        let w = viewport_width.max(1.0);
        let h = viewport_height.max(1.0);

        if !dx_pixels.is_finite() || !dy_pixels.is_finite() {
            return;
        }

        let fov_y = self
            .fov_y_degrees
            .to_radians()
            .clamp(1.0e-6, std::f64::consts::PI - 1.0e-6);
        let aspect = w / h;
        let fov_x = 2.0 * (aspect * (0.5 * fov_y).tan()).atan();

        // Screen convention:
        // - dragging right turns view right.
        // - dragging down pitches view down.
        //
        // Camera-local convention:
        // +X is right, +Y is up, -Z is forward.
        let yaw = -dx_pixels / w * fov_x;
        let pitch = -dy_pixels / h * fov_y;

        let yaw_q = DQuat::from_axis_angle(DVec3::Y, yaw);
        let pitch_q = DQuat::from_axis_angle(DVec3::X, pitch);

        // Post-multiply: local-frame rotation. This is the important bit.
        self.orientation = (self.orientation * yaw_q * pitch_q).normalize();
    }

    /// Rotate camera so forward points at `point`.
    pub fn look_at(&mut self, point: DVec3) {
        let to_target = point - self.position;
        let target_forward = normalized_or(to_target, self.forward());
        let current_forward = normalized_or(self.forward(), -DVec3::Z);

        let dot = current_forward.dot(target_forward).clamp(-1.0, 1.0);

        if dot > 1.0 - 1e-12 {
            return;
        }

        let q = if dot < -1.0 + 1e-12 {
            let axis = normalized_or(current_forward.cross(DVec3::Y), DVec3::X);
            DQuat::from_axis_angle(axis, std::f64::consts::PI)
        } else {
            let axis = normalized_or(current_forward.cross(target_forward), DVec3::Y);
            let angle = dot.acos();
            DQuat::from_axis_angle(axis, angle)
        };

        self.orientation = (q * self.orientation).normalize();
    }

    /// Legacy pixel orbit wrapper. Uses a unit sphere if the caller does not
    /// know object scale. Final object-mode callers should pass the radius.
    pub fn orbit_focus_pixels(
        &mut self,
        focus: DVec3,
        dx_pixels: f64,
        dy_pixels: f64,
        viewport_width: f64,
        viewport_height: f64,
    ) {
        self.orbit_focus_pixels_on_sphere(
            focus,
            1.0,
            dx_pixels,
            dy_pixels,
            viewport_width,
            viewport_height,
        );
    }

    /// Object-mode drag mapped through a spherical object.
    ///
    /// hagi is intentionally object-agnostic: the scene host gives it a focus
    /// point and a visual radius. In SOLS object mode the scene host first
    /// converts the camera into the target-centered coordinate frame, so hagi
    /// can treat the selected object as the center of the world.
    ///
    /// Near the surface, a screen-pixel drag becomes physical distance on the
    /// focus plane, then arc length on the sphere. Far away, a full viewport
    /// drag is capped at PI radians, so dragging across a whole visible planet
    /// maxes at a 180 degree roll around it.
    pub fn orbit_focus_pixels_on_sphere(
        &mut self,
        focus: DVec3,
        object_radius: f64,
        dx_pixels: f64,
        dy_pixels: f64,
        viewport_width: f64,
        viewport_height: f64,
    ) {
        let w = viewport_width.max(1.0);
        let h = viewport_height.max(1.0);
        let object_radius = object_radius.abs().max(1e-6);

        let offset = self.position - focus;
        let camera_distance = offset.length();

        if !camera_distance.is_finite() || camera_distance <= 1e-9 {
            return;
        }

        let fov_y = self
            .fov_y_degrees
            .to_radians()
            .clamp(1.0e-6, std::f64::consts::PI - 1.0e-6);
        let aspect = w / h;
        let world_per_pixel_y = 2.0 * camera_distance * (0.5 * fov_y).tan() / h;
        let world_per_pixel_x = world_per_pixel_y * aspect;

        let close_rad_x = world_per_pixel_x / object_radius;
        let close_rad_y = world_per_pixel_y / object_radius;

        let far_cap_x = std::f64::consts::PI / w;
        let far_cap_y = std::f64::consts::PI / h;

        let radians_per_px_x = close_rad_x.min(far_cap_x).max(1.0e-12);
        let radians_per_px_y = close_rad_y.min(far_cap_y).max(1.0e-12);

        // Pointer motion is treated like grabbing the projected sphere surface
        // and rolling the camera along that surface arc.
        //
        // dx moves along camera-right on the image, which is a rotation around
        // camera-up. dy moves along camera-up/down, which is a rotation around
        // camera-right. The sign convention here is the opposite of the old
        // world-Z yaw path, which is why this fixes the reversed-feeling orbit.
        let right_arc = self.up() * (-dx_pixels * radians_per_px_x);
        let up_arc = self.right() * (-dy_pixels * radians_per_px_y);
        let arc = right_arc + up_arc;
        let angle = arc.length();

        if !angle.is_finite() || angle <= 1.0e-12 {
            return;
        }

        let axis = normalized_or(arc, self.up());
        let q = DQuat::from_axis_angle(axis, angle);

        let mut new_offset = q * offset;
        new_offset = normalized_or(new_offset, offset) * camera_distance;

        self.position = focus + new_offset;
        self.look_at(focus);
    }

    pub fn orbit_focus_radians(&mut self, focus: DVec3, yaw: f64, pitch: f64) {
        let offset = self.position - focus;
        let radius = offset.length();

        if !radius.is_finite() || radius <= 1e-9 {
            return;
        }

        // SOLS orbital space is XY-plane / Z-up. Horizontal screen drag should
        // orbit around the selected body in that same orbital plane. The broken
        // modern path used world Y here, which made dx behave like a vertical
        // tumble for the SOLS coordinate system instead of the old planet-orbit
        // rotation.
        let yaw_axis = DVec3::Z;
        let yaw_q = DQuat::from_axis_angle(yaw_axis, yaw);

        // Pitch after yaw, around the yawed camera-right axis, so dy still feels
        // like screen-space pitch while keeping the selected body as the focus.
        let right_after_yaw = normalized_or(yaw_q * self.right(), DVec3::X);
        let pitch_q = DQuat::from_axis_angle(right_after_yaw, pitch);

        let mut new_offset = pitch_q * yaw_q * offset;
        new_offset = normalized_or(new_offset, offset) * radius;

        self.position = focus + new_offset;
        self.look_at(focus);
    }

    pub fn view_matrix(&self) -> DMat4 {
        let world_from_camera = DMat4::from_rotation_translation(self.orientation, self.position);
        world_from_camera.inverse()
    }

    pub fn projection_matrix(&self, width: f64, height: f64) -> DMat4 {
        let aspect = if height.abs() > 1e-9 {
            width / height
        } else {
            1.0
        };

        DMat4::perspective_rh(
            self.fov_y_degrees.to_radians(),
            aspect,
            self.clip.near,
            self.clip.far,
        )
    }

    pub fn view_projection_matrix(&self, width: f64, height: f64) -> DMat4 {
        self.projection_matrix(width, height) * self.view_matrix()
    }
}

#[cfg(test)]
mod object_orbit_tests {
    use super::*;

    #[test]
    fn object_drag_full_width_caps_at_half_turn() {
        let mut camera = SeenCamera::with_position(DVec3::new(0.0, -10_000.0, 0.0));
        camera.look_at(DVec3::ZERO);

        camera.orbit_focus_pixels_on_sphere(DVec3::ZERO, 1_000.0, 800.0, 0.0, 800.0, 600.0);

        let distance = camera.position.length();
        assert!((distance - 10_000.0).abs() < 1.0e-6);
        assert!(camera.position.y > 9_999.0);
    }

    #[test]
    fn object_drag_keeps_focus_locked() {
        let mut camera = SeenCamera::with_position(DVec3::new(0.0, -20_000.0, 5_000.0));
        camera.look_at(DVec3::ZERO);

        camera.orbit_focus_pixels_on_sphere(DVec3::ZERO, 6_371.0, 25.0, -10.0, 1280.0, 720.0);

        assert!(camera.forward().dot((-camera.position).normalize()) > 0.999999);
    }
}

#[cfg(test)]
mod quaternion_forward_zoom_tests {
    use super::*;

    fn close_vec(a: DVec3, b: DVec3) -> bool {
        (a - b).length() < 1.0e-9
    }

    #[test]
    fn move_forward_uses_quaternion_forward_vector() {
        let mut camera = SeenCamera::default();
        camera.position = DVec3::new(10.0, 20.0, 30.0);
        camera.orientation =
            DQuat::from_axis_angle(DVec3::Y, std::f64::consts::FRAC_PI_2).normalize();

        let before = camera.position;
        let forward = camera.forward();

        camera.move_forward(1234.5);

        assert!(close_vec(camera.position, before + forward * 1234.5));
        assert!((camera.orientation.length_squared() - 1.0).abs() < 1.0e-12);
    }

    #[test]
    fn move_forward_does_not_mutate_lens_or_clip() {
        let mut camera = SeenCamera::default();
        camera.fov_y_degrees = 42.0;
        camera.set_clip(7.0, 123456.0);

        let orientation_before = camera.orientation;
        let fov_before = camera.fov_y_degrees;
        let clip_before = camera.clip;

        camera.move_forward(-99.0);

        assert_eq!(camera.orientation, orientation_before);
        assert_eq!(camera.fov_y_degrees, fov_before);
        assert_eq!(camera.clip, clip_before);
    }
}
