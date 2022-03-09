extern crate vecmath;

use vecmath::{
    Vector2, 
    vec2_add, 
    vec2_sub, 
    vec2_scale, 
    vec2_mul
};

use crate::my_maths::rotate;
use crate::impulse::Impulse; 

#[derive(Copy, Clone)]
pub struct PhysicsRect {
    pub pos: Vector2<f32>,
    pub size: Vector2<f32>,
    pub velocity: Vector2<f32>,
    pub angle: f32,
    pub angular_velocity: f32,
    pub mass: f32,
    // temp/status values
    pub inertia: f32, // angular inertia constant
    pub impulse: Impulse,
    pub temp_impulse: Impulse, // Why?
}

impl PhysicsRect {
    pub fn new(
        pos: Vector2<f32>,
        size: Vector2<f32>,
        velocity: Vector2<f32>,
        angle: f32,
        angular_velocity: f32,
        mass: f32,
    ) -> PhysicsRect {
        let inertia = mass * (size[0] * size[0] + size[1] * size[1]) / 12.0;

        let rect = PhysicsRect {
            pos: pos,
            size: size,
            velocity: velocity,
            angle: angle,
            angular_velocity: angular_velocity,
            mass: mass,
            inertia: inertia,
            impulse: Impulse {
                linear: [0.0, 0.0],
                angular: 0.0,
            },
            temp_impulse: Impulse {
                linear: [0.0, 0.0],
                angular: 0.0,
            },
        };

        return rect;
    }

    pub fn move_p(&mut self, diff: Vector2<f32>) {
        self.pos = vec2_add(self.pos, diff);
    }

    pub fn reset_impulse(&mut self) {
        self.impulse = Impulse {
            linear: [0.0, 0.0],
            angular: 0.0,
        };
    }

    pub fn reset_temp_impulse(&mut self) {
        self.temp_impulse = Impulse {
            linear: [0.0, 0.0],
            angular: 0.0,
        };
    }

    pub fn get_total_impulse(&self) -> Impulse {
        return Impulse {
            linear: vec2_add(self.impulse.linear, self.temp_impulse.linear),
            angular: self.impulse.angular + self.temp_impulse.angular,
        };
    }

    pub fn get_impulse_velocity(&self) -> Vector2<f32> { 
        // The units do not work out here
        // velocity has units [m/s] and impulse [kg.m/s]
        return vec2_add(self.velocity, self.impulse.linear);
    }

    pub fn get_impulse_velocity_dt(&self, dt: f32) -> Vector2<f32> { // The units do not work out here
        // The units do not work out here
        // velocity has units [m/s] and impulse [kg.m/s]
        return vec2_scale(vec2_add(self.velocity, self.impulse.linear), dt);
    }

    pub fn get_impulse_angular_velocity(&self) -> f32 {
        // The units don't seem to work here as well
        // angular velocity has units [/s] and angular momentum [kg.m.m/s]
        return self.angular_velocity + self.impulse.angular;
    }

    pub fn get_center(&self) -> Vector2<f32> {
        // This only works if the cubes are not rotating
        // You will want to implement something using sines and cosines
        return [
            self.pos[0] + self.size[0] / 2.0,
            self.pos[1] + self.size[1] / 2.0,
        ];
    }

    pub fn get_impulse_center(&self, dt: f32) -> Vector2<f32> {
        return vec2_add(
            self.get_center(),
            vec2_scale(self.get_impulse_velocity(), dt),
        );
    }

    pub fn get_impulse_radius(&self, pos: Vector2<f32>, dt: f32) -> Vector2<f32> {
        return vec2_sub(pos, self.get_impulse_center(dt));
    }

    // top left, top right, bottom right, bottom left
    pub fn get_corners(&self) -> [Vector2<f32>; 4] {
        // This only works if your re
        return [
            [self.pos[0], self.pos[1]],
            [self.pos[0] + self.size[0], self.pos[1]],
            [self.pos[0] + self.size[0], self.pos[1] + self.size[1]],
            [self.pos[0], self.pos[1] + self.size[1]],
        ];
    }

    // Takes in the corners and then rotates them about a point known as the center of mass
    // depreciated because it does not take the new position of the rect into account
    pub fn _get_rotated_corners(&self, angle: f32) -> [Vector2<f32>; 4] {
        let corners = self.get_corners();

        let mut result = corners;

        let center = [
            self.pos[0] + self.size[0] / 2.0,
            self.pos[1] + self.size[1] / 2.0,
        ];

        for i in 0..corners.len() {
            result[i] = rotate(corners[i], center, angle);
        }

        return result;
    }

    // calculated the corners of a rectangle given impulse and rotation
    pub fn get_impulse_corners(&self, dt: f32) -> [Vector2<f32>; 4] {
        let mut corners = self.get_corners();

        let angle = self.angle + self.angular_velocity * self.impulse.angular * dt;
        let center = self.get_impulse_center(dt);

        for i in 0..4 {
            corners[i] = vec2_add(
                corners[i],
                vec2_scale(vec2_mul(self.velocity, self.impulse.linear), dt),
            );
            corners[i] = rotate(corners[i], center, angle);
        }

        return corners;
    }
}
