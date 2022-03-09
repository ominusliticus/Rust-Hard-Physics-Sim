extern crate vecmath;

pub use vecmath::{
    Vector2,
    vec2_add,
    vec2_sub,
    vec2_normalized,
    vec2_scale,
    vec2_dot,
    vec2_len
};


use crate::phys_rect::PhysicsRect;


pub fn _min(num1: f32, num2: f32) -> f32 {
    if num1 < num2 {
        return num1;
    }

    return num2;
}

pub fn _max(num1: f32, num2: f32) -> f32 {
    if num1 > num2 {
        return num1;
    }

    return num2;
}

pub fn find_max_f32(list: [f32; 4]) -> usize {
    let mut max = 0;

    for i in 1..4 {
        if list[max] < list[i] {
            max = i;
        }
    }

    return max;
}

pub fn find_min_f32(list: [f32; 4]) -> usize {
    let mut min = 0;

    for i in 1..4 {
        if list[min] > list[i] {
            min = i;
        }
    }

    return min;
}


pub fn vec2_mat2_mul(vec: Vector2<f32>, mat: [Vector2<f32>; 2]) -> Vector2<f32> {
    let mut result = [0.0, 0.0];

    result[0] = vec[0] * mat[0][0] + vec[1] * mat[1][0];
    result[1] = vec[0] * mat[0][1] + vec[1] * mat[1][1];

    return result;
}

pub fn rotate(point: Vector2<f32>, origin: Vector2<f32>, angle: f32) -> Vector2<f32> {
    let rotation_matrix = [
        [angle.cos() as f32, angle.sin() as f32],
        [-1.0 * angle.sin() as f32, angle.cos() as f32],
    ];

    let point_local = vec2_sub(point, origin);
    let rotated_point = vec2_mat2_mul(point_local, rotation_matrix);

    return vec2_add(rotated_point, origin);
}

pub fn project(pos: Vector2<f32>, line: [Vector2<f32>; 2]) -> f32 {
    let norm_line = vec2_normalized(vec2_sub(line[1], line[0]));

    // move point into a normal position
    let mut proj = vec2_sub(pos, line[0]);

    // project the point onto the line
    proj = vec2_scale(norm_line, vec2_dot(proj, norm_line));

    // transform the projections onto a single axis
    let mut out = (proj[0] * proj[0] + proj[1] * proj[1]).sqrt();

    // account for normal direction
    let dot = vec2_dot(proj, norm_line);
    if dot < 0.0 {
        out = out * -1.0;
    }

    return out;
}

pub fn project_rect_line(line: [Vector2<f32>; 2], rect: &PhysicsRect, dt: f32) -> [f32; 4] {
    let mut projection = rect.get_impulse_corners(dt);
    let norm_line = vec2_normalized(vec2_sub(line[1], line[0]));

    let mut out = [0.0, 0.0, 0.0, 0.0];
    for i in 0..4 {
        out[i] = project(projection[i], line);
    }

    return out;
}

pub fn vec2_dist(vec1: Vector2<f32>, vec2: Vector2<f32>) -> f32 {
    return vec2_len(vec2_sub(vec1, vec2));
}

pub fn abs(n: f32) -> f32 {
    if n < 0.0 {
        return n * -1.0;
    }

    return n;
}

pub fn find_normal_from_pos(rect: &PhysicsRect, pos: Vector2<f32>, dt: f32) -> Vector2<f32> {
    let mut corners = rect.get_impulse_corners(dt);

    // find the two corners that are closest to the point
    // BUBBLE SORT!!!!
    for i in 0..4 {
        for j in 0..4 - 1 - i {
            if vec2_dist(corners[j], pos) < vec2_dist(corners[j + 1], pos) {
                corners.swap(j, j + 1);
            }
        }
    }

    // calculate the normal from the two corners
    let line = vec2_normalized(vec2_sub(corners[0], corners[1]));
    let normal = [-1.0 * line[1], line[0]];
    let dot = vec2_dot(vec2_normalized(vec2_sub(pos, corners[0])), normal);

    // scale the norm in the right direction
    return vec2_scale(normal, dot / abs(dot));
}
