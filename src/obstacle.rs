extern crate nalgebra;
use self::nalgebra::{ Vec3 };

use vehicle::{ VehicleBase };

pub trait ObstacleBase {
    fn steer_to_avoid(&self, vehicle: &VehicleBase, min_collision_time: f32) -> Vec3<f32>;

    fn next_intersection(&self, vehicle: &VehicleBase) -> Option<f32>;
}
