extern crate nalgebra;
use self::nalgebra::{ Vec3 };

use annotation::{ AnnotationBase };
use vector::{ VectorHelper };
use flowfield::{ FlowfieldBase };
use pathway::{ PathwayBase };
use obstacle::{ ObstacleBase };
use random::{ scalar_random_walk };

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

    fn steer_for_wander(&self, dt: f32, wander_side: &mut f32, wander_up: &mut f32, annotations: &AnnotationBase) -> Vec3<f32>  {
        //Wander (how much we're wandering in side/up directions is stored and modified slightly through multiple runs)
        let speed = 12.0 * dt;
        *wander_side = scalar_random_walk(*wander_side, speed, -1.0, 1.0);
        *wander_up = scalar_random_walk(*wander_up, speed, -1.0, 1.0);

        //Calculate wander amount
        return self.side() * *wander_side + self.up() * *wander_up;
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

    fn steer_to_follow_flow_field(&self, dt: f32, flowfield: &FlowfieldBase, max_speed: f32, prediction_time: f32, annotations: &AnnotationBase) -> Vec3<f32> {
        let future_pos = self.predict_position(prediction_time);
        let flow = flowfield.sample(future_pos);
        return self.velocity() - flow.truncate_length(max_speed);
    }

    fn steer_to_stay_on_path(&self, prediction_time: f32, path: &PathwayBase, max_speed: f32, annotations: &AnnotationBase) -> Vec3<f32> {
        panic!();
    }

    fn steer_to_follow_path(&self, direction: bool, prediction_time: f32, path: &PathwayBase, max_speed: f32, annotations: &AnnotationBase) -> Vec3<f32> {
        panic!();
    }

    fn steer_to_avoid_obstacle(&self, min_collision_time: f32, obstacle: &ObstacleBase, annotations: &AnnotationBase) -> Vec3<f32> where Self: Sized+VehicleBase {
        return obstacle.steer_to_avoid(self, min_collision_time);
    }

    fn steer_to_avoid_obstacles(&self, min_collision_time: f32, obstacles: &[&ObstacleBase], annotations: &AnnotationBase) -> Vec3<f32> {
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
