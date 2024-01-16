use libm::acos;

// TODO move to a CONFIG file
const R_PHOTODIODE: f32 = 3900.0; // ohms
const LUX_SOLAR: f32 = 133358.4; // lux
const ID_PER_LUX_TYPICAL: f32 = 6.3  * 1e-9; // amps per lux
const ID_PER_LUX_MIN: f32 = 5.0  * 1e-9; // amps per lux
const ID_PER_LUX_FROM_CALIBRATION: f32 = 5.0  * 1e-9; // amps per lux, from taking the photodiode outside and pointing it at the sun
const DEGREES_PER_RADIAN: f32 = 180.0 / 3.14159265358979;
const MOUNT_ANGLE: f32 = 30.0; // degrees
const HALF_ANGLE_OF_PHOTODIODE: f32 = 70.0; // degrees
const PHOTODIODE_PAIR_DENOM: f32 = 2.0 * R_PHOTODIODE * ID_PER_LUX_TYPICAL;


pub fn voltage_to_angle(v_meas: u16) -> Result<f32, &'static str> {
    let i_meas = (v_meas as f32) / R_PHOTODIODE;
    let angle: f32 = acos(i_meas as f64 / ID_PER_LUX_TYPICAL as f64) as f32;
    if angle >= 0.0 && angle <= HALF_ANGLE_OF_PHOTODIODE {
        Ok(angle as f32)
    } else {
        Err("Invalid angle estimation")
    }
}


pub fn photodiode_pair(v1: u16, v2: u16) -> Result<f32, &'static str> {
    let angle: f32 = acos(((v2 - v1) as f32 / PHOTODIODE_PAIR_DENOM) as f64) as f32;
    if angle >= 0.0 && angle <= HALF_ANGLE_OF_PHOTODIODE{
        Ok(angle)
    } else {
        Err("Invalid angle estimation")
    }
}