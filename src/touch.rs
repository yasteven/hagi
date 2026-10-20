//! Device-independent full-contact snapshots. No SDL, DOM, network, or SOLS dependencies.
use glam::{DVec2, DVec3};
use crate::SeenCamera;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Contact { pub id: i64, pub position: DVec2 }

#[derive(Debug, Clone, Default)]
pub struct TouchCamera {
    contacts: Vec<Contact>,
    viewport: DVec2,
}
impl TouchCamera {
    pub fn cancel(&mut self) { self.contacts.clear(); }
    /// Call for each ordered snapshot; addition/removal/resize establishes a new baseline.
    /// `travel` is world units per full-screen drag, selected by the scene host.
    /// Target lock only applies after translation; two-finger looking remains available.
    pub fn update(&mut self, camera:&mut SeenCamera, contacts:&[Contact], viewport:DVec2,
                  travel:f64, target_lock:Option<DVec3>) {
        if contacts.len()>10 || !viewport.is_finite() || viewport.min_element()<=0.0 ||
            !travel.is_finite() || travel<0.0 || contacts.iter().any(|p| !p.position.is_finite()) {
            self.cancel(); return;
        }
        let mut next=contacts.to_vec(); next.sort_by_key(|p|p.id);
        if next.windows(2).any(|p|p[0].id==p[1].id) { self.cancel(); return; }
        let same=viewport==self.viewport && next.len()==self.contacts.len() &&
            next.iter().zip(&self.contacts).all(|(a,b)|a.id==b.id);
        if same && !next.is_empty() {
            if next.len()==1 {
                let d=(next[0].position-self.contacts[0].position)/viewport;
                camera.move_camera_local(DVec3::new(-d.x*travel,d.y*travel,0.0));
                if let Some(target)=target_lock {camera.look_at(target);}
            } else if next.len()==2 {
                let before=(self.contacts[0].position+self.contacts[1].position)*0.5;
                let after=(next[0].position+next[1].position)*0.5;
                let d=(after-before)/viewport;
                let a=self.contacts[1].position-self.contacts[0].position;
                let b=next[1].position-next[0].position;
                // Dolly distance is proportional to pinch, independent of screen location.
                if a.length()>1.0 && b.length()>1.0 {
                    let pinch=(b.length()-a.length())/viewport.min_element();
                    if pinch.abs()>1e-12 {
                        camera.move_forward(pinch*travel*2.0);
                        if let Some(target)=target_lock {camera.look_at(target);}
                    }
                    let twist=(a.x*b.y-a.y*b.x).atan2(a.dot(b));
                    camera.spin(-twist);
                }
                // Preserve the old SDL signs and local world-to-view multiplication.
                camera.look_right(-std::f64::consts::PI*d.x/4.0);
                camera.look_up(std::f64::consts::PI*d.y/4.0);
            }
        }
        self.contacts=next; self.viewport=viewport;
    }
}
