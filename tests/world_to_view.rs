use glam::{DMat4,DQuat,DVec2,DVec3};
use hagi::{SeenCamera,touch::{Contact,TouchCamera}};
fn close(a:DVec3,b:DVec3){assert!((a-b).length()<1e-8,"{a:?} != {b:?}");}
#[test]fn cpp_pitch_yaw_roll_and_row_vector_movement(){
 let mut c=SeenCamera::default();c.look_right(0.4);c.look_up(-0.2);c.spin(0.3);
 let q=DQuat::from_rotation_z(0.3)*DQuat::from_rotation_x(0.2)*DQuat::from_rotation_y(0.4);
 assert!(c.orientation.dot(q).abs()>1.0-1e-12);
 // C++ row-vector delta * R is R-transpose * column-vector delta.
 let d=DVec3::new(2.0,-3.0,-5.0);c.move_camera_local(d);
 close(c.position,DMat4::from_quat(q).transpose().transform_vector3(d));
 close(c.world_to_view(c.position+c.forward()*7.0),DVec3::new(0.0,0.0,-7.0));
}
#[test]fn pyramid_floating_origin_matches_cpp_matrix_chain(){
 let mut c=SeenCamera::with_position(DVec3::new(5000.0,-2000.0,700.0));
 c.look_right(0.7);c.look_up(-0.3);c.spin(0.1);c.set_clip(0.1,1e8);
 let center=c.position+c.forward()*100.0;let local=DVec3::new(1.0,3.0,2.0);
 let split=c.world_to_view(center)+c.orientation*local;
 let chain=c.view_matrix().transform_point3(center+local);close(split,chain);
 let gl=c.projection_opengl(1600.0,900.0)*chain.extend(1.0);
 let gpu=c.projection_matrix(1600.0,900.0)*split.extend(1.0);
 assert!((gl.x/gl.w-gpu.x/gpu.w).abs()<1e-10);
 assert!((gl.y/gl.w-gpu.y/gpu.w).abs()<1e-10);
 assert!(((gl.z/gl.w+1.0)*0.5-gpu.z/gpu.w).abs()<1e-10);
 let q=c.orientation;let _=c.projection_matrix(900.0,1600.0);assert_eq!(q,c.orientation);
}
#[test]fn look_behind_and_same_position_stay_finite(){
 let mut c=SeenCamera::default();c.look_at(DVec3::Z);close(c.forward(),DVec3::Z);
 let q=c.orientation;c.look_at(c.position);assert_eq!(q,c.orientation);
}
fn p(id:i64,x:f64,y:f64)->Contact{Contact{id,position:DVec2::new(x,y)}}
#[test]fn gesture_ids_rebase_and_pan_then_pinch(){
 let mut c=SeenCamera::default();let mut t=TouchCamera::default();let v=DVec2::splat(1000.0);
 t.update(&mut c,&[p(5,100.0,100.0)],v,100.0,None);
 t.update(&mut c,&[p(5,200.0,100.0)],v,100.0,None);close(c.position,DVec3::new(-10.0,0.0,0.0));
 let before=c.position;
 t.update(&mut c,&[p(5,200.0,100.0),p(9,400.0,100.0)],v,100.0,None);close(c.position,before);
 t.update(&mut c,&[p(9,450.0,100.0),p(5,150.0,100.0)],v,100.0,None);close(c.position,before+DVec3::new(0.0,0.0,-20.0));
 let before=c.position;t.update(&mut c,&[p(9,450.0,100.0)],v,100.0,None);close(c.position,before);
 t.cancel();t.update(&mut c,&[p(9,900.0,900.0)],v,100.0,None);close(c.position,before);
}
#[test]fn twist_preserves_position_and_forward(){
 let mut c=SeenCamera::default();let mut t=TouchCamera::default();let v=DVec2::splat(1000.0);
 t.update(&mut c,&[p(1,400.0,500.0),p(2,600.0,500.0)],v,100.0,None);
 t.update(&mut c,&[p(1,500.0,400.0),p(2,500.0,600.0)],v,100.0,None);
 close(c.position,DVec3::ZERO);close(c.forward(),-DVec3::Z);close(c.up(),-DVec3::X);
}
#[test]fn clip_fitting_reads_view_depth_once(){
 let clip=hagi::ClipPlanes::from_view_spheres([(100.0,2.0)],hagi::ClipPlanes::default(),1e6);
 assert_eq!(clip.near,92.0);assert!(clip.far>100.0);
 // Translating the entire scene and camera does not change these view depths.
 for offset in [DVec3::ZERO,DVec3::new(1e10,1e10,1e10)] {
  let mut c=SeenCamera::with_position(offset);c.look_right(0.6);
  let depth=-c.world_to_view(offset+c.forward()*100.0).z;
  let fit=hagi::ClipPlanes::from_view_spheres([(depth,2.0)],c.clip,1e6);
  assert!((fit.near-clip.near).abs()<1e-5);
 }
}
