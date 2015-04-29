extern crate nalgebra;
use self::nalgebra::{ Vec3 };

use vehicle::{ VehicleBase };

pub trait AnnotationBase {
    fn is_enabled() -> bool;

    fn enable(enabled: bool);

    fn line(start : Vec3<f32>, end: Vec3<f32>, color: Vec3<f32>, opacity: f32);

    fn circle_xz(radius: f32, center: Vec3<f32>, color: Vec3<f32>, opacity: f32, segments: i32);

    fn disk_xz(radius: f32, center: Vec3<f32>, color: Vec3<f32>, opacity: f32, segments: i32);

    fn circle_3d(radius: f32, center: Vec3<f32>, axis: Vec3<f32>, color: Vec3<f32>, opacity: f32, segments: i32);

    fn disk_3d(radius: f32, center: Vec3<f32>, axis: Vec3<f32>, color: Vec3<f32>, opacity: f32, segments: i32);

    fn avoid_obstacle(min_distance_to_collision: f32) {
    }

    fn path_following(future: Vec3<f32>, on_path: Vec3<f32>, target: Vec3<f32>, outside: f32) {
    }

    fn avoid_close_neighbour(other: &VehicleBase, additional_distance: f32) {
    }

    fn avoid_neighbour(threat: &VehicleBase, steer: f32, our_future: Vec3<f32>, threat_future: Vec3<f32>) {
    }

    fn velocity_acceleration(vehicle: &VehicleBase, max_acceleration_length: f32, max_velocity_length: f32) {
    }
}
