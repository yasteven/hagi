use glam::DVec3;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CameraControlMode {
    /// World/free camera mode: camera owns its world position/orientation.
    Free,

    /// Object camera mode: the scene host gives hagi a target-centered frame
    /// and hagi treats the selected object as the center of the world.
    ObjectOrbit,
}

impl CameraControlMode {
    pub fn from_mode_name(mode_name: &str) -> Self {
        let m = mode_name.trim().to_ascii_lowercase();

        if m == "free"
            || m == "free-mode"
            || m == "free-camera"
            || m == "world"
            || m == "world-mode"
        {
            Self::Free
        } else {
            Self::ObjectOrbit
        }
    }

    pub fn as_mode_name(self) -> &'static str {
        match self {
            Self::Free => "world-mode",
            Self::ObjectOrbit => "object-mode",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CameraCommand {
    MoveLocal {
        delta: DVec3,
    },
    MoveWorld {
        delta: DVec3,
    },
    MoveTo {
        point: DVec3,
    },
    LookUp {
        radians: f64,
    },
    LookRight {
        radians: f64,
    },
    Spin {
        radians: f64,
    },
    LookAt {
        point: DVec3,
    },
    OrbitFocusPixels {
        focus: DVec3,
        object_radius: f64,
        dx_pixels: f64,
        dy_pixels: f64,
        viewport_width: f64,
        viewport_height: f64,
    },
    /// Compatibility name for wheel/zoom controls.
    ///
    /// Positive `amount` moves forward along the quaternion camera forward vector.
    /// Negative `amount` moves backward. This is camera translation only; it must
    /// not mutate object/model scale, FOV, clip planes, or any Euler state.
    ZoomLocal {
        amount: f64,
    },
}
