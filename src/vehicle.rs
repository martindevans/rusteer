extern crate nalgebra;
use self::nalgebra::{ Vec3, Dot, Norm  };

use annotation::{ AnnotationBase };
use vector::{ VectorHelper };
use flowfield::{ FlowfieldBase };
use pathway::{ PathwayBase };
use obstacle::{ ObstacleBase };
use random::{ scalar_random_walk };

pub trait LocalSpaceBasis {
    fn side(&self) -> Vec3<f32>;

    fn up(&self) -> Vec3<f32>;

    fn forward(&self) -> Vec3<f32>;

    fn position(&self) -> Vec3<f32>;
}

pub trait VehicleBase : LocalSpaceBasis {
    fn mass(&self) -> f32;

    fn radius(&self) -> f32;

    fn velocity(&self) -> Vec3<f32>;

    fn acceleration(&self) -> Vec3<f32>;

    fn speed(&self) -> f32;

    fn predict_position(&self, prediction_time : f32) -> Vec3<f32>;

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

    fn steer_for_flee(&self, target: Vec3<f32>, max_speed: f32, annotations: &AnnotationBase) -> Vec3<f32> {
        let offset = self.position() - target;
        let desired_velocity = offset.truncate_length(max_speed);
        return desired_velocity - self.velocity();
    }

    fn steer_for_seek(&self, target: Vec3<f32>, max_speed: f32, annotations: &AnnotationBase) -> Vec3<f32> {
        let offset = target - self.position();
        let desired_velocity = offset.truncate_length(max_speed);
        return desired_velocity - self.velocity();
    }

    fn steer_for_arrival(&self, target: Vec3<f32>, max_speed: f32, slowing_distance: f32, annotations: &AnnotationBase) -> Vec3<f32> {
        let offset = target - self.position();
        let distance = offset.norm();
        let ramped_speed = max_speed * (distance / slowing_distance);
        let clipped_speed = ramped_speed.min(max_speed);

        //this line is reordered from original (offset * x instead of x * offset) because rust complains about the other order
        //Basic arithmetic operator overloads aren't direction insensitive? :'(
        let desired_velocity = offset * (clipped_speed / distance);

        return desired_velocity - self.velocity();
    }

    fn steer_to_follow_flow_field(&self, flowfield: &FlowfieldBase, max_speed: f32, prediction_time: f32, annotations: &AnnotationBase) -> Vec3<f32> {
        let future_pos = self.predict_position(prediction_time);
        let flow = flowfield.sample(future_pos);
        return self.velocity() - flow.truncate_length(max_speed);
    }

    fn steer_to_stay_on_path(&self, prediction_time: f32, path: &PathwayBase, max_speed: f32, annotations: &AnnotationBase) -> Vec3<f32> {
        // Predict out future position
        let future_position = self.predict_position(prediction_time);

        // Find the point on the path nearest to predicted position
        let (on_path, tangent, outside) = path.map_point_to_path(future_position);

        // If our predicted future position was in the path, return zero steering.
        if (outside < 0.0) {
            return Vec3::<f32>::new(0.0, 0.0, 0.0)
        }

        // Our predicted future position was outside the path, need to steer towards it.
        // Use onPath projection of futurePosition as seek target
        annotations.path_following(future_position, on_path, on_path, outside);
        return self.steer_for_seek(on_path, max_speed, annotations);
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

    fn steer_for_separation(&self, max_distance: f32, cos_max_angle: f32, others: &mut Iterator<Item=&VehicleBase>, annotations: &AnnotationBase) -> Vec3<f32> where Self: Sized+VehicleBase {
        let mut steering = Vec3::<f32>::new(0.0, 0.0, 0.0);
        let mut neighbours = 0;

        for other in others {
            if is_in_boid_neighbourhood(self, other, self.radius() * 3.0, max_distance, cos_max_angle) {
                // add in steering contribution
                // (opposite of the offset direction, divided once by distance
                // to normalize, divided another time to get 1/d falloff)
                let offset = other.position() - self.position();
                let distance_sqr = offset.dot(&offset);
                steering = steering + (offset / -distance_sqr);

                // count neighbors
                neighbours += 1;
            }
        }

        // divide by neighbors, then normalize to pure direction
        if neighbours > 0 {
            steering = steering / (neighbours as f32);
            steering.normalize();
        }

        return steering;
    }

    fn steer_to_avoid_close_neighbours(&self, min_distance: f32, others: &mut Iterator<Item=&VehicleBase>, annotations: &AnnotationBase) -> Vec3<f32> {
        panic!();
    }

    fn steer_for_alignment(&self, max_distance: f32, cos_max_angle: f32, others: &mut Iterator<Item=&VehicleBase>, annotations: &AnnotationBase) -> Vec3<f32> where Self: Sized+VehicleBase {
        panic!();
    }

    fn steer_for_cohesion(&self, max_distance: f32, cos_max_angle: f32, others: &mut Iterator<Item=&VehicleBase>, annotations: &AnnotationBase) -> Vec3<f32> where Self: Sized+VehicleBase {
        panic!();
    }

    fn steer_for_pursuit(&self, quarry: &VehicleBase, max_prediction_time: f32, max_speed: f32, annotations: &AnnotationBase) -> Vec3<f32> {
        panic!();
    }

    fn steer_for_evasion(&self, menace: &VehicleBase, max_prediction_time: f32, max_speed: f32, annotations: &AnnotationBase) -> Vec3<f32> where Self: Sized+VehicleBase {
        let offset = menace.position() - self.position();
        let distance = offset.norm();

        let rough_time = distance / menace.speed();
        let prediction_time = if rough_time > max_prediction_time { max_prediction_time } else { rough_time };

        let target = menace.predict_position(prediction_time);

        return self.steer_for_flee(target, max_speed, annotations);
    }

    fn steer_for_target_speed(&self, target_speed: f32, max_force: f32, annotations: &AnnotationBase) -> Vec3<f32> where Self: Sized+VehicleBase {
        let speed_error = target_speed - self.speed();

        return self.forward() * speed_error.max(-max_force).min(max_force);
    }

    fn steer_to_avoid_neighbours(&self, min_time_to_collision: f32, others: &mut Iterator<Item=&VehicleBase>, annotations: &AnnotationBase) -> Vec3<f32> where Self: Sized+VehicleBase {
        panic!();
    }
}

fn is_in_boid_neighbourhood(vehicle: &VehicleBase, other: &VehicleBase, min_distance: f32, max_distance: f32, cos_max_angle: f32) -> bool {
    panic!();
}

pub struct SimpleVehicle {
    position: Vec3<f32>,
    mass: f32,
    radius: f32,
    speed: f32,
    max_force: f32,
    max_speed: f32
}

impl LocalSpaceBasis for SimpleVehicle {
    fn side(&self) -> Vec3<f32> {
        panic!();
    }

    fn up(&self) -> Vec3<f32> {
        panic!();
    }

    fn forward(&self) -> Vec3<f32> {
        panic!();
    }

    fn position(&self) -> Vec3<f32> {
        return self.position;
    }
}

impl VehicleBase for SimpleVehicle {
    fn mass(&self) -> f32 {
        return self.mass;
    }

    fn radius(&self) -> f32 {
        return self.radius;
    }

    fn velocity(&self) -> Vec3<f32> {
        return self.forward() * self.speed;
    }

    fn acceleration(&self) -> Vec3<f32> {
        panic!();
    }

    fn speed(&self) -> f32 {
        return self.speed;
    }

    fn predict_position(&self, prediction_time : f32) -> Vec3<f32> {
        return self.position + (self.velocity() * prediction_time)
    }

    fn max_force(&self) -> f32 {
        return self.max_force;
    }

    fn max_speed(&self) -> f32 {
        return self.max_speed;
    }
}
