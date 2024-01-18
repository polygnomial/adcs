use crate::linear_algebra::{vec3, mat3, quat};
// TODO create warning if near singularity

pub fn triad(R1: vec3, R2: vec3, r1: vec3, r2: vec3) -> Result<quat, ()>{
    let r1_hat = r1.normalize();
    let r2_hat = r2.normalize();
    let R1_hat = R1.normalize();
    let R2_hat = R2.normalize();

    let b1 = r1_hat;
    let b2 = r1_hat.cross(&r2_hat).normalize();
    let b3 = b1.cross(&b2);

    let B = mat3::new(
        b1.x, b2.x, b3.x,
        b1.y, b2.y, b3.y,
        b1.z, b2.z, b3.z,
    );

    let a1 = R1_hat;
    let a2 = R1_hat.cross(&R2_hat).normalize();
    let a3 = a1.cross(&a2);

    let A = mat3::new(
        a1.x, a2.x, a3.x,
        a1.y, a2.y, a3.y,
        a1.z, a2.z, a3.z,
    );

    let C = A * B.transpose();
    let q = quat::from_matrix(&C);
    Ok(q)
}