
use crate::phys_rect::PhysicsRect;
use crate::impulse::Impulse;
use crate::my_maths::{
    abs,
    project_rect_line,
    find_min_f32,
    find_max_f32,
    find_normal_from_pos,
    project
};

use vecmath::{
    Vector2,
    vec2_add,
    vec2_sub,
    vec2_normalized,
    vec2_scale,
    vec2_dot,
    vec2_len,
    vec2_mul,
    vec2_cross
};


pub fn checkAxisOverlap(
    line: [Vector2<f32>; 2],
    rect_a: &PhysicsRect,
    rect_b: &PhysicsRect,
    dt: f32,
) -> bool {
    let project_a = project_rect_line(line, rect_a, dt);
    let project_b = project_rect_line(line, rect_b, dt);

    // Find the intervals of both along the axis
    let interval_a_i = [find_min_f32(project_a), find_max_f32(project_a)];
    let interval_b_i = [find_min_f32(project_b), find_max_f32(project_b)];
    let interval_a = [project_a[interval_a_i[0]], project_a[interval_a_i[1]]];
    let interval_b = [project_b[interval_b_i[0]], project_b[interval_b_i[1]]];

    //println!("{} {}", project_a[0], project_a[1]);

    let mut overlap = false;
    // Case 1 (left overlap case)
    if interval_a[0] <= interval_b[0] && interval_b[0] <= interval_a[1] {
        overlap = true;
    }
    // Case 2 (right overlap case)
    if interval_a[0] <= interval_b[1] && interval_b[1] <= interval_a[1] {
        overlap = true;
    }
    // Case 3 (The max and min bound the entire side)
    if interval_a[0] >= interval_b[0] && interval_a[1] <= interval_b[1] {
        overlap = true;
    }

    return overlap;
}


// returns (If the rects collided, pos error, coordinate of points of collision, normal vector according to rect A)
pub fn check_collision(
    rect_a: &PhysicsRect,
    rect_b: &PhysicsRect,
    dt: f32,
) -> (
    bool,
    [f32; 2],
    [bool; 2],
    [Vector2<f32>; 2],
    [Vector2<f32>; 2],
) {
    let corners = rect_a.get_impulse_corners(dt);
    let corners_b = rect_b.get_impulse_corners(dt);

    // process the overlap of all axis
    let mut no_overlap = [false, false, false, false];
    no_overlap[0] = checkAxisOverlap([corners[0], corners[1]], rect_a, rect_b, dt);
    no_overlap[1] = checkAxisOverlap([corners[1], corners[2]], rect_a, rect_b, dt);
    no_overlap[2] = checkAxisOverlap([corners_b[0], corners_b[1]], rect_b, rect_a, dt);
    no_overlap[3] = checkAxisOverlap([corners_b[1], corners_b[2]], rect_b, rect_a, dt);

    // if there is no overlap then just return base info
    for overlap in no_overlap {
        if !overlap {
            return (
                false,
                [0.0, 0.0],
                [false, false],
                [[0.0, 0.0], [0.0, 0.0]],
                [[0.0, 0.0], [0.0, 0.0]],
            );
        }
    }

    // Any code past this point assumes that the rectangles are overlapping

    // calculate the contact points (Basic implementation that does not first put the contact point on the outside of the rect)
    // identify the shortest length between the two rects to identify the normal rect (can be both if they are resting flat ontop of one another)
    // 
    // You have a function called vec2_dist that does precisely this 
    // This is an example of a code pattern that you use multiple times 
    // in this code, so its a perfect example of trivial refactoring
    let mut shortest_length_a = vec2_len(vec2_sub(rect_a.get_impulse_center(dt), corners_b[0]));
    let mut shortest_length_b = vec2_len(vec2_sub(rect_b.get_impulse_center(dt), corners[0]));
    let mut shortest_corner_a = corners_b[0];
    let mut shortest_corner_b = corners[0];

    for i in 1..4 {
        let mut length = vec2_len(vec2_sub(rect_a.get_impulse_center(dt), corners_b[i]));
        if shortest_length_a > length {
            shortest_length_a = length;
            shortest_corner_a = corners_b[i];
        }

        length = vec2_len(vec2_sub(rect_b.get_impulse_center(dt), corners[i]));
        if shortest_length_b > length {
            shortest_length_b = length;
            shortest_corner_b = corners[i];
        }
    }

    // Compare the lengths
    let mut error = [0.0, 0.0];
    let mut normals = [[0.0, 0.0], [0.0, 0.0]];
    let mut points = [[0.0, 0.0], [0.0, 0.0]];
    let mut contacts = [false, false];

    let min_dist = 0.001;

    // Note: This accounts for multi-contact collisions
    // Rect B is norm and a-corner is the contact
    if shortest_length_a > shortest_length_b
        || abs(shortest_length_a - shortest_length_b) < min_dist
    {
        points[0] = shortest_corner_b;
        normals[0] = find_normal_from_pos(rect_b, rect_a.get_impulse_center(dt), dt);
        contacts[0] = true;

        // Here is another example of a set of functions which call identically
        // in this file, and is a good example of trivial refactorization.
        let p = vec2_add(
            rect_b.get_impulse_center(dt),
            vec2_mul(normals[0], vec2_scale(rect_a.size, 0.5)),
        );
        let line = [p, vec2_add(p, normals[0])];
        error[0] = project(points[0], line);
    }
    // Rect A is norm and b-corner is the contact
    if shortest_length_a < shortest_length_b
        || abs(shortest_length_a - shortest_length_b) < min_dist
    {
        points[1] = shortest_corner_a;
        normals[1] = find_normal_from_pos(rect_a, rect_b.get_impulse_center(dt), dt);
        contacts[1] = true;

        let p = vec2_add(
            rect_a.get_impulse_center(dt),
            vec2_mul(normals[1], vec2_scale(rect_a.size, 0.5)),
        );
        let line = [p, vec2_add(p, normals[1])];
        error[1] = project(points[1], line);
    }

    // default error
    return (true, error, contacts, points, normals);
}

/**
pub fn apply_double_normal_constraint(
    pos1: Vector2<f32>,
    velocity1: Vector2<f32>,
    angular_velocity1: f32,
    radius1: Vector2<f32>,
    center1: Vector2<f32>,
    inertia1: f32,
    mass1: f32,
    pos2: Vector2<f32>,
    velocity2: Vector2<f32>,
    angular_velocity2: f32,
    radius2: Vector2<f32>,
    center2: Vector2<f32>,
    inertia2: f32,
    mass2: f32,
    normal: Vector2<f32>,
    error: f32,
    bias: f32,
    dt: f32,
) -> (Impulse, Impulse) {
    // tangential velocity
    let r_vec1 = vec2_normalized(vec2_sub(pos1, center1));
    let tan_vel1 = vec2_len(radius1) * angular_velocity1;
    let tan_vec1 = [-1.0 * r_vec1[1] * tan_vel1, r_vec1[0] * tan_vel1];

    let r_vec2 = vec2_normalized(vec2_sub(pos2, center2));
    let tan_vel2 = vec2_len(radius2) * angular_velocity2;
    let tan_vec2 = [-1.0 * r_vec2[1] * tan_vel2, r_vec2[0] * tan_vel2];

    let calc_vel1 = vec2_add(velocity1, tan_vec1);
    let calc_vel2 = vec2_add(velocity2, tan_vec2);

    // apply biased contact constraint
    let c_d1 = vec2_dot(calc_vel1, normal) + bias * (error / dt);
    let c_d2 = vec2_dot(calc_vel2, normal) + bias * (error / dt);

    //print_vec2(velocity);
    //print_vec2(tan_vec);
    //println!("{}", angular_velocity);
    //println!("{}", tan_cross);
    //println!("{}", c_d);

    let cross1 = vec2_cross(radius1, normal);
    let m_eff1 = mass1 + inertia1 * cross1 * cross1;

    let cross2 = vec2_cross(radius2, normal);
    let m_eff2 = mass2 + inertia2 * cross2 * cross2;

    // next plug in the massive equation you solved for

    let lambda = -m_eff * c_d;

    //println!("{}", m_eff);

    let out = Impulse {
        linear: vec2_scale(normal, lambda),
        angular: (lambda * cross),
    };

    return out;
}
*/

pub fn apply_normal_constraint(
    pos: Vector2<f32>,
    velocity: Vector2<f32>,
    angular_velocity: f32,
    radius: Vector2<f32>,
    center: Vector2<f32>,
    inertia: f32,
    mass: f32,
    normal: Vector2<f32>,
    error: f32,
    bias: f32,
    inertial_impulse: Vector2<f32>,
    dt: f32,
) -> Impulse {
    // tangential velocity
    let r_vec = vec2_normalized(vec2_sub(pos, center));
    let tan_vel = vec2_len(radius) * angular_velocity;
    let tan_vec = [-1.0 * r_vec[1] * tan_vel, r_vec[0] * tan_vel];

    let calc_vel = vec2_add(vec2_add(velocity, tan_vec), vec2_scale(inertial_impulse, 1.0 / mass));
    // apply biased contact constraint
    let c_d = vec2_dot(calc_vel, normal) + bias * (error / dt);

    let cross = vec2_cross(radius, normal);
    let m_eff = 1.0 / (mass + inertia * cross * cross);

    let lambda = -m_eff * c_d;

    //println!("{}", m_eff);

    let out = Impulse {
        linear: vec2_scale(normal, lambda),
        angular: (lambda * cross),
    };

    return out;
}

pub fn rect_constraint(rects: [&mut PhysicsRect; 2], size: usize, iter: usize, dt: f32) {
    let bias = 0.0;

    // rest all base impulses
    for i in 0..size {
        rects[i].reset_impulse();
    }

    for iter_count in 0..iter {
        // reset all temp impulses
        // calculate constants for processing
        for i in 0..size {
            rects[i].reset_temp_impulse();
        }

        // compare all rects with one another
        for i in 0..size {
            for j in (i + 1)..size {
                let (collided, error, contacts, points, normals) =
                    check_collision(&rects[i], &rects[j], dt);

                if collided {
                    for c in 0..contacts.len() {
                        if contacts[c as usize] {
                            let radius_a = rects[i].get_impulse_radius(points[c as usize], dt);
                            let center_a = rects[i].get_impulse_center(dt);
                            let radius_b = rects[j].get_impulse_radius(points[c as usize], dt);
                            let center_b = rects[j].get_impulse_center(dt);

                            let mut normal_a = normals[c as usize];
                            let mut normal_b = normals[c as usize];

                            // flip the normals for corresponding rects
                            //if c == 0 {
                            //    normal_b = vec2_scale(normal_b, -1.0);
                            //}
                            //else if c == 1 {
                            //    normal_a = vec2_scale(normal_a, -1.0);
                            //}

                            // calculate the relative inertial impulses of each rect on the point
                            let r_vec_a = vec2_normalized(vec2_sub(points[c], center_a));
                            let tan_vel_a = vec2_len(radius_a) * (rects[i].angular_velocity + rects[i].impulse.angular);
                            let tan_vec_a = [r_vec_a[1] * tan_vel_a, -1.0 * r_vec_a[0] * tan_vel_a];
                            let inertial_impulse_a = vec2_scale(vec2_add(rects[i].get_impulse_velocity(), tan_vec_a), rects[i].inertia);

                            let r_vec_b = vec2_normalized(vec2_sub(points[c], center_b));
                            let tan_vel_b = vec2_len(radius_b) * (rects[j].angular_velocity + rects[j].impulse.angular);
                            let tan_vec_b = [r_vec_b[1] * tan_vel_b, -1.0 * r_vec_b[0] * tan_vel_b];
                            let inertial_impulse_b = vec2_scale(vec2_add(rects[j].get_impulse_velocity(), tan_vec_b), rects[j].inertia);

                            // find impulses
                            let new_impulse_a = apply_normal_constraint(
                                points[c as usize],
                                rects[i].get_impulse_velocity(),
                                rects[i].angular_velocity + rects[i].impulse.angular,
                                radius_a,
                                center_a,
                                rects[i].inertia,
                                rects[i].mass,
                                normal_a,
                                error[c],
                                bias,
                                inertial_impulse_b,
                                //[0.0, 0.0],
                                dt,
                            );

                            let new_impulse_b = apply_normal_constraint(
                                points[c as usize],
                                rects[j].get_impulse_velocity(),
                                rects[j].angular_velocity + rects[j].impulse.angular,
                                radius_b,
                                center_b,
                                rects[j].inertia,
                                rects[j].mass,
                                normal_b,
                                error[c],
                                bias,
                                inertial_impulse_a,
                                //[0.0, 0.0],
                                dt,
                            );

                            rects[i].temp_impulse.add(new_impulse_a);
                            rects[j].temp_impulse.add(new_impulse_b);
                        }
                    }
                }
            }
        }

        // apply temp impulse to the main impulse
        for i in 0..size {
            rects[i].impulse.add(rects[i].temp_impulse);
        }
    }

    // apply impulse to velocity
    for i in 0..size {
        if vec2_len(rects[i].impulse.linear) > 0.0 {
            rects[i].velocity;
        }
        rects[i].velocity = rects[i].get_impulse_velocity();
        rects[i].angular_velocity = rects[i].get_impulse_angular_velocity();
    }
}

// apply the screen bound constraint on velocity
pub fn screen_bound_constraint(rect: &mut PhysicsRect, dt: f32) {
    // find How much they are overlapping
    let normals = [
        [0.0, -1.0],
        [-1.0, 0.0],
        [0.0, 1.0],
        [1.0, 0.0],
    ];

    let walls = [
        6.0,
        8.0,
        0.0,
        0.0,
    ];

    rect.reset_impulse();

    for s in 0..4 {
        for i in 0..1 {
            // apply for each corner of the square
            rect.reset_temp_impulse();

            let corners = rect.get_impulse_corners(dt);

            let center = rect.get_impulse_center(dt);

            let mut corner_count = 0;

            for corner in corners {
                //println!("{} {}", radius[0], radius[1]);
                let radius = rect.get_impulse_radius(corner, dt);

                // set surface
                let surface = vec2_add(vec2_scale(normals[s], -walls[s]), [corner[0] * ((1.0 + (s as f32)) % 2.0), corner[1] * ((s as f32) % 2.0)]);


                let error = vec2_dot(normals[s], vec2_sub(corner, surface));
                //println!("error of {}, {} {}, {} {}", error, new_pos[0], new_pos[1], new_copy[0], new_copy[1]);

                if error < 0.0 {
                    //println!("error of {}, {} {}, {} {}", error, new_pos[0], new_pos[1], new_copy[0], new_copy[1]);

                    // Process the impulse calculation
                    let new_impulse = apply_normal_constraint(
                        corner,
                        rect.get_impulse_velocity(),
                        rect.angular_velocity + rect.impulse.angular,
                        radius,
                        center,
                        rect.inertia,
                        rect.mass,
                        normals[s],
                        0.0,
                        0.0,
                        [0.0, 0.0],
                        dt,
                    );

                    rect.temp_impulse.add(new_impulse);

                    //println!("corner: {} error: {} angular velocity: {} angular impulse: {}", corner_count, error, rect.angularVelocity, temp_angular_impulse);
                }

                corner_count = corner_count + 1;
            }

            // apply temp impulse to the main impulse
            rect.impulse.add(rect.temp_impulse);
        }
    }

    if rect.impulse.linear[1] != 0.0 {
        //println!("velocity: {}, {} impulse: {}, {}", rect.velocity[0], rect.velocity[1], impulse[0], impulse[1]);
    }

    rect.velocity = rect.get_impulse_velocity();
    rect.angular_velocity = rect.get_impulse_angular_velocity();
}

pub fn apply_constraints(rects: [&mut PhysicsRect; 2], size: usize, iter: usize, dt: f32) {
    // rest all base impulses
    for i in 0..size {
        screen_bound_constraint(rects[i], dt);
    }

    rect_constraint(rects, size, iter, dt);

    for i in 0..size {
        for j in (i + 1)..size {
            
        }
    }
}

