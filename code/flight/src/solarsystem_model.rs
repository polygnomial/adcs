use crate::constants::*;
use crate::linear_algebra::vec3;
use libm::{floorf, sinf, cosf, atan2f, asinf};


pub fn sun_unit_vector(jd: f32) -> Result<vec3, ()>{
    // days from J2000.0
    let D: f32 = jd - 2451545.0;
    // mean anomaly
    let g: f32 = ((357.529 + 0.98560028*D) % 360.0)/180.0*PI_F32;
    // mean longitude
    let q: f32 = ((280.459 + 0.98564736*D) % 360.0)/180.0*PI_F32;
    // geocentric apparent ecliptic longitude
    let λ_sun: f32 = ((q + 1.915*sinf(g) + 0.020*sinf(2.0*g)) % 360.0) / 180.0*PI_F32;
    // obliquity of ecliptic
    let ε = 23.439 - (0.00000036*D);

    let boresight =vec3::new(
        cosf(λ_sun),
        cosf(ε)*sinf(λ_sun),
        sinf(ε)*sinf(λ_sun),
    );
    Ok(boresight)
}
// https://aa.usno.navy.mil/faq/sun_approx
pub fn sun_position(jd: f32) -> Result<vec3, () >{
    // days from J2000.0
    let D: f32 = jd - 2451545.0;
    // mean anomaly
    let g: f32 = ((357.529 + 0.98560028*D) % 360.0)/180.0*PI_F32;
    // mean longitude
    let q_deg: f32 = (280.459 + 0.98564736*D) % 360.0;
    // geocentric apparent ecliptic longitude
    let λ_sun: f32 = ((q_deg + 1.915*sinf(g) + 0.020*sinf(2.0*g)) % 360.0) / 180.0*PI_F32;
    // obliquity of ecliptic
    let ε = (23.439 - (0.00000036*D)) / 180.0*PI_F32;

    let boresight = vec3::new(
        cosf(λ_sun),
        cosf(ε)*sinf(λ_sun),
        sinf(ε)*sinf(λ_sun),
    );

    // distance from sun in km
    let R: f32 = (1.00014 - 0.01671*cosf(g) - 0.00014*cosf(2.0*g)) * KM_PER_AU;
    // sun unit vector
    let position = boresight * R;
    Ok(position)
}