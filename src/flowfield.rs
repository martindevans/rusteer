extern crate nalgebra;
use self::nalgebra::{ Vec3 };

pub trait FlowfieldBase {
    fn sample(&self, location: Vec3<f32>) -> Vec3<f32>;
}
