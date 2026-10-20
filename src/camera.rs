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
/// - orientation maps WORLD to VIEW (the original SdlGlScene convention)
/// - local movement is transformed by the inverse orientation
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
        self.orientation.inverse() * DVec3::X
    }

    pub fn up(&self) -> DVec3 {
        self.orientation.inverse() * DVec3::Y
    }

    /// Default camera forward is -Z, matching the old OpenGL camera convention.
    pub fn forward(&self) -> DVec3 {
        self.orientation.inverse() * -DVec3::Z
    }

    /// Move in camera coordinates.
    pub fn move_camera_local(&mut self, delta: DVec3) {
        self.position += self.orientation.inverse() * delta;
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

        // Invert the camera-to-world increment; stored quaternion is world-to-view.
        self.orientation = (pitch_q.inverse() * yaw_q.inverse() * self.orientation).normalize();
    }

    /// Rotate camera so forward points at `point`.
    pub fn look_at(&mut self, point: DVec3) {
        let to_target = point - self.position;
        if !to_target.is_finite() || to_target.length_squared() < 1e-24 { return; }
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

        self.orientation = (self.orientation * q.inverse()).normalize();
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
        let world_per_pixel_y = 2.0 * camera_distance * (0.5 * fov_y).tan() / h;
        let world_per_pixel_x = world_per_pixel_y; // square screen pixels; aspect is already in horizontal FOV

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
        DMat4::from_quat(self.orientation) * DMat4::from_translation(-self.position)
    }

    /// OpenGL depth -1..1, matching the original C++ engine.
    pub fn projection_opengl(&self, width: f64, height: f64) -> DMat4 {
        DMat4::perspective_rh_gl(self.fov_y_degrees.to_radians(),
            width.max(1.0) / height.max(1.0), self.clip.near, self.clip.far)
    }

    /// WebGPU depth 0..1; same lens as OpenGL with explicit depth conversion.
    pub fn projection_matrix(&self, width: f64, height: f64) -> DMat4 {
        let gl = self.projection_opengl(width, height);
        let depth = DMat4::from_cols(
            glam::DVec4::X, glam::DVec4::Y,
            glam::DVec4::new(0.0,0.0,0.5,0.0),
            glam::DVec4::new(0.0,0.0,0.5,1.0));
        depth * gl
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

impl SeenCamera {
    pub fn ray_from_screen(
        &self,
        cursor_x: f64,
        cursor_y: f64,
        viewport_width: f64,
        viewport_height: f64,
    ) -> Option<(DVec3, DVec3)> {
        if viewport_width <= 1.0
            || viewport_height <= 1.0
            || !cursor_x.is_finite()
            || !cursor_y.is_finite()
        {
            return None;
        }

        let ndc_x = ((cursor_x / viewport_width) * 2.0) - 1.0;
        let ndc_y = 1.0 - ((cursor_y / viewport_height) * 2.0);

        let aspect = viewport_width / viewport_height;
        let tan_half_fov = (self.fov_y_degrees.to_radians() * 0.5).tan();

        let local_dir = DVec3::new(ndc_x * aspect * tan_half_fov, ndc_y * tan_half_fov, -1.0);

        if local_dir.length_squared() <= 0.0 {
            return None;
        }

        let world_dir = (self.orientation.inverse() * local_dir.normalize()).normalize();

        Some((self.position, world_dir))
    }


    pub fn world_to_view(&self, point: DVec3) -> DVec3 {
        self.orientation * (point - self.position)
    }
    pub fn view_basis_f32(&self) -> ([f32;4],[f32;4],[f32;4]) {
        let v = |axis: DVec3| { let v=self.orientation*axis; [v.x as f32,v.y as f32,v.z as f32,0.0] };
        (v(DVec3::X),v(DVec3::Y),v(DVec3::Z))
    }
    pub fn projection_with_clip(&self, width:f64, height:f64, near:f64, far:f64) -> DMat4 {
        let mut camera=self.clone(); camera.set_clip(near,far); camera.projection_matrix(width,height)
    }
    pub fn translate_local(&mut self, delta:DVec3) { self.move_camera_local(delta); }
    pub fn translate_world(&mut self, delta:DVec3) { self.move_world(delta); }
    pub fn translate_forward(&mut self, distance:f64) { self.move_forward(distance); }
    pub fn move_xyz_world(&mut self, x:f64,y:f64,z:f64) { self.move_world(DVec3::new(x,y,z)); }
    pub fn move_dxdydz_camera(&mut self,x:f64,y:f64,z:f64) { self.move_camera_local(DVec3::new(x,y,-z)); }
    pub fn move_towards_point(&mut self, target:DVec3, distance:f64) {
        let d=target-self.position; if d.is_finite() && d.length_squared()>0.0 { self.position+=d.normalize()*distance; }
    }
    pub fn rotate_towards_point(&mut self, target:DVec3) { self.look_at(target); }
    pub fn look_at_quat(&mut self, target:DVec3) { self.look_at(target); }
    pub fn set_follow_view(&mut self, target:DVec3, distance:f64) {
        self.position=target+DVec3::new(0.0,-distance,distance*0.25); self.look_at(target);
    }
    pub fn rotate_around_point(&mut self, target:DVec3, yaw:f64,pitch:f64) { self.orbit_focus_radians(target,yaw,pitch); }
    pub fn orbit_quat_about_point(&mut self,target:DVec3,yaw:f64,pitch:f64) { self.orbit_focus_radians(target,yaw,pitch); }
    pub fn rotate_local_axis(&mut self, axis:DVec3, radians:f64) {
        if axis.is_finite() && axis.length_squared()>0.0 && radians.is_finite() {
            self.orientation=(DQuat::from_axis_angle(axis.normalize(),-radians)*self.orientation).normalize();
        }
    }
    pub fn rotate_world_axis(&mut self, axis:DVec3, radians:f64) {
        if axis.is_finite() && axis.length_squared()>0.0 && radians.is_finite() {
            self.orientation=(self.orientation*DQuat::from_axis_angle(axis.normalize(),-radians)).normalize();
        }
    }
    pub fn roll_about_forward(&mut self, radians:f64) { self.rotate_local_axis(-DVec3::Z,radians); }
    pub fn normalize_rotation(&mut self) {
        let n=self.orientation.length_squared();
        self.orientation=if n.is_finite() && n>1e-24 { self.orientation.normalize() } else { DQuat::IDENTITY };
    }
    pub fn rotation_is_unit_quat(&self)->bool { (self.orientation.length_squared()-1.0).abs()<1e-9 }
    /// Argument is WORLD TO VIEW, never camera-to-world.
    pub fn set_rotation_quat(&mut self,q:DQuat) { self.orientation=q; self.normalize_rotation(); }
    pub fn as_xyzw(&self)->[f64;4] { self.orientation.to_array() }
}

impl SeenCamera {
    /// Screen projection and apparent size share the exact lens used by mesh drawing.
    pub fn project_sphere(&self, world:DVec3, radius:f64, width:f64,height:f64)->Option<(f64,f64,f64,f64)> {
        if width<=0.0 || height<=0.0 {return None;}
        let view=self.world_to_view(world);let depth=-view.z;
        if !depth.is_finite() || depth<=self.clip.near || depth>=self.clip.far {return None;}
        let p=self.projection_matrix(width,height)*view.extend(1.0);
        let ndc=p.truncate()/p.w;
        if !ndc.is_finite(){return None;}
        let diameter=radius.max(0.0)*height/(depth*(self.fov_y_degrees.to_radians()*0.5).tan());
        Some((ndc.x,ndc.y,depth,diameter))
    }
    /// Host supplies scene bounds; Hagi owns framing geometry and orientation.
    pub fn frame_bounds(&mut self, center:DVec3, radius:f64, direction:DVec3, margin:f64, min_distance:f64,max_distance:f64) {
        let distance=(radius.max(0.0)*margin/(self.fov_y_degrees.to_radians()*0.5).tan().max(0.01))
            .clamp(min_distance,max_distance);
        self.position=center+direction.normalize_or_zero()*distance;
        self.look_at(center);
    }
}

impl ClipPlanes {
    pub fn bounded(near:f64,far:f64,max_ratio:f64)->Self {
        let near=if near.is_finite(){near.max(0.001)}else{0.001};
        let far=if far.is_finite(){far.max(near+1.0).max(near*16.0)}else{near*16.0};
        Self {near,far:far.min(near*max_ratio.max(1000.0))}
    }
    /// Inputs are already view-space center depth and radius. Never transform twice.
    pub fn from_view_spheres(spheres:impl IntoIterator<Item=(f64,f64)>, fallback:Self,max_ratio:f64)->Self {
        let mut near=f64::INFINITY;let mut far=0.0_f64;
        for (depth,radius) in spheres {
            if !depth.is_finite() || depth<=0.0 || !radius.is_finite(){continue;}
            let radius=radius.abs().max(1.0);
            near=near.min((depth-radius*4.0).max(depth*0.25));
            far=far.max(depth+radius*4.0);
        }
        if near.is_finite() && far>0.0 {Self::bounded(near,far,max_ratio)}
        else {Self::bounded(fallback.near,fallback.far,max_ratio)}
    }
    pub fn around_focus(depth:f64,radius:f64,max_ratio:f64)->Self {
        let near=(depth-radius*8.0).max((depth*0.001).max(0.001));
        Self::bounded(near,(depth+radius*8.0).max(near*16.0),max_ratio)
    }
}

impl SeenCamera {
    pub fn place_looking_at(&mut self,target:DVec3,offset:DVec3) {self.move_to(target+offset);self.look_at(target);}
    pub fn animate_orbit(&mut self,target:DVec3,radius:f64,height:f64,angle:f64) {
        self.place_looking_at(target,DVec3::new(angle.cos()*radius,angle.sin()*radius,height));
    }
}
