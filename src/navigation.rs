//! Scene-independent navigation geometry. Coordinates use the host's consistent unit.
use glam::DVec3;
pub fn clip_translation(start_km:DVec3,requested_end_km:DVec3,
    spheres:impl IntoIterator<Item=(DVec3,f64)>)->DVec3 {
    let travel = requested_end_km - start_km;
    let a = travel.length_squared();
    if !a.is_finite() || a <= 1.0e-18 {
        return start_km;
    }

    let mut first_hit: Option<(f64, DVec3, f64)> = None;

    for (center_km,radius_km) in spheres {
        let radius_km=radius_km.max(0.001);
        let from_center = start_km - center_km;
        let start_distance = from_center.length();

        if !start_distance.is_finite() {
            continue;
        }

        if start_distance < radius_km {
            let outward = if from_center.length_squared() > 1.0e-18 {
                from_center.normalize()
            } else {
                (-travel).normalize_or_zero()
            };
            return center_km + outward * radius_km;
        }

        if start_distance <= radius_km + 1.0e-9 && from_center.dot(travel) < 0.0 {
            let candidate = (0.0, center_km, radius_km);
            if first_hit.map(|old| candidate.0 < old.0).unwrap_or(true) {
                first_hit = Some(candidate);
            }
            continue;
        }

        let b = 2.0 * from_center.dot(travel);
        let c = from_center.length_squared() - radius_km * radius_km;
        let discriminant = b * b - 4.0 * a * c;
        if discriminant < 0.0 {
            continue;
        }

        let sqrt_discriminant = discriminant.sqrt();
        let denominator = 2.0 * a;
        let t_enter = (-b - sqrt_discriminant) / denominator;
        if !(1.0e-12..=1.0).contains(&t_enter) {
            continue;
        }

        let candidate = (t_enter, center_km, radius_km);
        if first_hit.map(|old| candidate.0 < old.0).unwrap_or(true) {
            first_hit = Some(candidate);
        }
    }

    let Some((t, center_km, radius_km)) = first_hit else {
        return requested_end_km;
    };

    let intersection = start_km + travel * t;
    let outward = (intersection - center_km).normalize_or_zero();
    center_km + outward * radius_km
}
