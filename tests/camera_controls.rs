use approx::assert_relative_eq;
use glam::DVec3;
use hagi::{CameraCommand, CameraControlMode, R3Kinematics, SeenCamera, SeenSceneController};

fn assert_vec3_close(a: DVec3, b: DVec3) {
    assert_relative_eq!(a.x, b.x, epsilon = 1e-9);
    assert_relative_eq!(a.y, b.y, epsilon = 1e-9);
    assert_relative_eq!(a.z, b.z, epsilon = 1e-9);
}

#[test]
fn r3_kinematics_uses_quaternion_orientation() {
    let k = R3Kinematics::default();

    assert_vec3_close(k.position, DVec3::ZERO);
    assert_vec3_close(k.velocity, DVec3::ZERO);

    let forward = k.orientation * -DVec3::Z;
    assert_vec3_close(forward, -DVec3::Z);
}

#[test]
fn move_camera_local_default_forward_matches_old_camera_convention() {
    let mut cam = SeenCamera::default();

    cam.move_camera_local(DVec3::new(0.0, 0.0, -10.0));

    assert_vec3_close(cam.position, DVec3::new(0.0, 0.0, -10.0));
}

#[test]
fn move_world_ignores_orientation() {
    let mut cam = SeenCamera::default();

    cam.look_right(std::f64::consts::FRAC_PI_2);
    cam.move_world(DVec3::new(1.0, 2.0, 3.0));

    assert_vec3_close(cam.position, DVec3::new(1.0, 2.0, 3.0));
}

#[test]
fn look_right_rotates_forward_vector() {
    let mut cam = SeenCamera::default();

    cam.look_right(std::f64::consts::FRAC_PI_2);

    let f = cam.forward();
    assert_relative_eq!(f.length(), 1.0, epsilon = 1e-9);
    assert_relative_eq!(f.x.abs(), 1.0, epsilon = 1e-9);
    assert_relative_eq!(f.y, 0.0, epsilon = 1e-9);
    assert_relative_eq!(f.z.abs(), 0.0, epsilon = 1e-9);
}

#[test]
fn look_up_rotates_forward_vector() {
    let mut cam = SeenCamera::default();

    cam.look_up(std::f64::consts::FRAC_PI_2);

    let f = cam.forward();
    assert_relative_eq!(f.length(), 1.0, epsilon = 1e-9);
    assert_relative_eq!(f.x, 0.0, epsilon = 1e-9);
    assert_relative_eq!(f.y.abs(), 1.0, epsilon = 1e-9);
    assert_relative_eq!(f.z.abs(), 0.0, epsilon = 1e-9);
}

#[test]
fn spin_rolls_camera_right_and_up_axes() {
    let mut cam = SeenCamera::default();

    let right_before = cam.right();
    let up_before = cam.up();

    cam.spin(std::f64::consts::FRAC_PI_2);

    let right_after = cam.right();
    let up_after = cam.up();

    assert!(right_before.dot(right_after).abs() < 1e-9);
    assert!(up_before.dot(up_after).abs() < 1e-9);
}

#[test]
fn look_at_points_forward_at_target() {
    let mut cam = SeenCamera::with_position(DVec3::new(0.0, 0.0, 10.0));
    let target = DVec3::ZERO;

    cam.look_at(target);

    let expected = (target - cam.position).normalize();
    assert_vec3_close(cam.forward().normalize(), expected);
}

#[test]
fn object_orbit_preserves_radius() {
    let mut cam = SeenCamera::with_position(DVec3::new(0.0, 0.0, 10.0));
    let focus = DVec3::ZERO;

    cam.look_at(focus);

    let before = cam.position.distance(focus);
    cam.orbit_focus_pixels(focus, 480.0, 0.0, 960.0, 540.0);
    let after = cam.position.distance(focus);

    assert_relative_eq!(before, after, epsilon = 1e-9);

    let expected_forward = (focus - cam.position).normalize();
    assert_vec3_close(cam.forward().normalize(), expected_forward);
}

#[test]
fn controller_free_drag_changes_orientation_not_position() {
    let mut ctl = SeenSceneController::new();
    ctl.set_mode(CameraControlMode::Free);

    let pos_before = ctl.camera.position;
    let forward_before = ctl.camera.forward();

    ctl.apply_drag(None, None, 100.0, 0.0, 1000.0, 500.0);

    assert_vec3_close(ctl.camera.position, pos_before);
    assert!(ctl.camera.forward().dot(forward_before) < 0.999);
}

#[test]
fn controller_object_orbit_drag_moves_position_and_looks_at_focus() {
    let focus = DVec3::ZERO;
    let mut ctl = SeenSceneController::new();
    ctl.set_mode(CameraControlMode::ObjectOrbit);
    ctl.camera.move_to(DVec3::new(0.0, 0.0, 10.0));
    ctl.camera.look_at(focus);

    let before = ctl.camera.position;
    ctl.apply_drag(Some(focus), Some(1.0), 250.0, 0.0, 1000.0, 500.0);

    assert!(ctl.camera.position.distance(before) > 1e-6);

    let expected_forward = (focus - ctl.camera.position).normalize();
    assert_vec3_close(ctl.camera.forward().normalize(), expected_forward);
}

#[test]
fn apply_command_works_for_basic_camera_commands() {
    let mut ctl = SeenSceneController::new();

    ctl.apply_command(CameraCommand::MoveWorld {
        delta: DVec3::new(1.0, 2.0, 3.0),
    });
    assert_vec3_close(ctl.camera.position, DVec3::new(1.0, 2.0, 3.0));

    ctl.apply_command(CameraCommand::LookAt { point: DVec3::ZERO });

    let expected_forward = (DVec3::ZERO - ctl.camera.position).normalize();
    assert_vec3_close(ctl.camera.forward().normalize(), expected_forward);
}
