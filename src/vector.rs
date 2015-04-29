extern crate nalgebra;
use self::nalgebra::{ Vec3, Norm };

use std::num::{ Float };

pub trait VectorHelper {
    fn truncate_length(&self, max_length: f32) -> Vec3<f32>;
}

impl VectorHelper for Vec3<f32> {
    fn truncate_length(&self, max_length: f32) -> Vec3<f32> {
        let m_sqr = max_length * max_length;
        let l_sqr = self.sqnorm();

        if l_sqr < m_sqr {
            return self.clone();
        }

        return self.clone() * (max_length / (l_sqr).sqrt());
    }
}
