extern crate nalgebra;
use self::nalgebra::{ Vec3 };

use annotation::{ AnnotationBase };
use vector::{ VectorHelper };
use flowfield::{ FlowfieldBase };
use pathway::{ PathwayBase };

pub trait VehicleBase {
    //These 4 should be in ILocalSpaceBasis (in C# terms), do we need such a trait?
    fn side(&self) -> Vec3<f32>;
    fn up(&self) -> Vec3<f32>;
    fn forward(&self) -> Vec3<f32>;
    fn position(&self) -> Vec3<f32>;

    fn mass(&self) -> f32;

    fn radius(&self) -> f32;

    fn velocity(&self) -> Vec3<f32>;

    fn acceleration(&self) -> Vec3<f32>;

    fn speed(&self) -> f32;

    fn predict_position(&self, predictionTime : f32) -> Vec3<f32>;

    fn max_force(&self) -> f32;

    fn max_speed(&self) -> f32;

    fn steer_for_wander(&self, dt: f32, wander_side: &mut f32, wander_up: &mut f32, annotations: &AnnotationBase) {

    }

    fn steer_for_flee(&self, dt: f32, target: Vec3<f32>, max_speed: f32, annotations: &AnnotationBase) -> Vec3<f32> {
        let offset = self.position() - target;
        let desired_velocity = offset.truncate_length(max_speed);
        return desired_velocity - self.velocity();
    }

    fn steer_for_seek(&self, dt: f32, target: Vec3<f32>, max_speed: f32, annotations: &AnnotationBase) -> Vec3<f32> {
        let offset = target - self.position();
        let desired_velocity = offset.truncate_length(max_speed);
        return desired_velocity - self.velocity();
    }

    fn steer_for_arrival(&self, dt: f32, target: Vec3<f32>, max_speed: f32, slowing_distance: f32, annotations: &AnnotationBase) -> Vec3<f32> {
        panic!();
    }

    fn steer_to_follow_flow_field(&self, dt: f32, &FlowfieldBase, max_speed: f32, prediction_distance: f32, annotations: &AnnotationBase) -> Vec3<f32> {
        panic!();
    }

    fn steer_to_stay_on_path(&self, prediction_time: f32, path: &PathwayBase, max_speed: f32, annotations: &AnnotationBase) -> Vec3<f32> {
        panic!();
    }
}

/* pub struct SimpleVehicle {
    position: Vec3<f32>,
}

impl VehicleBase for SimpleVehicle {
    fn mass(&self) -> f32 {
        return 1.0;
    }
} */
