use glam::{DQuat, DVec3};

/// Reusable 3D kinematic state.
///
/// This intentionally uses quaternion orientation instead of Euler
/// rotation slots.
#[derive(Debug)]
pub struct R3Kinematics {
    pub position: DVec3,
    pub velocity: DVec3,
    pub orientation: DQuat,
    pub angular_velocity_axis: DVec3,
    pub angular_velocity_rad_s: f64,
    pub color: [f32; 4],
}

impl Default for R3Kinematics {
    fn default() -> Self {
        Self {
            position: DVec3::ZERO,
            velocity: DVec3::ZERO,
            orientation: DQuat::IDENTITY,
            angular_velocity_axis: DVec3::Z,
            angular_velocity_rad_s: 0.0,
            color: [0.0, 0.0, 0.0, 1.0],
        }
    }
}

impl R3Kinematics {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn with_position(position: DVec3) -> Self {
        Self {
            position,
            ..Self::default()
        }
    }

    pub fn with_color(mut self, color: [f32; 4]) -> Self {
        self.color = color;
        self
    }

    pub fn integrate_basic(&mut self, dt_seconds: f64) {
        self.position += self.velocity * dt_seconds;

        if self.angular_velocity_rad_s != 0.0 {
            let axis = normalized_or(self.angular_velocity_axis, DVec3::Z);
            let dq = DQuat::from_axis_angle(axis, self.angular_velocity_rad_s * dt_seconds);
            self.orientation = (dq * self.orientation).normalize();
        }
    }

    pub fn set_orientation(&mut self, orientation: DQuat) {
        self.orientation = orientation.normalize();
    }
}

pub fn normalized_or(v: DVec3, fallback: DVec3) -> DVec3 {
    let len = v.length();
    if len.is_finite() && len > 1e-12 {
        v / len
    } else {
        fallback
    }
}
