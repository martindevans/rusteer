extern crate nalgebra;
use self::nalgebra::{ Vec3 };

pub trait PathwayBase {
    fn map_point_to_path(&self, point: Vec3<f32>) -> (Vec3<f32>, f32);

    fn map_path_distance_to_point(&self, distance: f32) -> Vec3<f32>;

    fn map_point_to_path_distance(&self, point: Vec3<f32>) -> f32;
}
