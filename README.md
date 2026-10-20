# hagi
Helicopter Around Given I's

Like, "eye's", or I (me), moving around in 3D space.

Reusable camera and kinematkcs for 3D gui scenes
for Makepad/PADS frontends, WebGPU scene hosts, and
other GPU scene projects.

Current scope:

```text
[x] quaternion R3Kinematics
[x] SeenCamera
[x] old SDL/OpenGL camera control semantics ported to Rust
[x] local camera movement
[x] world movement
[x] look_up / look_right / spin
[x] look_at
[x] object orbit with radius preservation
[x] unit tests
```

Run:

```bash
cargo test
```

<!-- hagi-camera-conventions-20261020 -->
## Camera conventions

`SeenCamera.orientation` maps world to view. The view matrix is
`R(q) * T(-position)`; camera-local movement uses the inverse quaternion.
This follows the original SDL/OpenGL camera convention. OpenGL projection is
available directly; WebGPU projection remaps depth from [-1,1] to [0,1].

Hagi owns camera movement, projection, clip fitting, navigation geometry,
animations and device-independent touch gestures. The host supplies scene
objects, units, targets, speed policy and GPU resources. Reusable mesh and
orbit WGSL are exposed through `hagi::shaders`.

`touch::TouchCamera` accepts full snapshots with stable finger IDs:
one finger pans, two-finger movement looks, pinch translates forward/backward,
and twist rolls. Pinch does not change field of view or scene scale.
Finger-count changes, cancellation and viewport resizing reset the baseline.

Run `cargo test --locked` to check camera, matrix and gesture behavior.
