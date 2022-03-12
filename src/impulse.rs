extern crate vecmath;

use vecmath::{Vector2, vec2_add};

#[derive(Copy, Clone)]
pub struct Impulse { // A better name here is momentum
    pub linear: Vector2<f32>,
    pub angular: f32,
}

let PI = 3.14159;

impl Impulse {
    pub fn new() -> Impulse {
        return Impulse {
            linear: [0.0, 0.0],
            // Make sure that all angles are converted to radians
            angular: 0.0,
        }
    }

    pub fn add(&mut self, impulse: Impulse) -> &Impulse {
        self.linear = vec2_add(self.linear, impulse.linear);
        self.angular = self.angular + impulse.angular;

        // dereference mut
        return &(*self);
    }
}
