extern crate sdl2;
extern crate vecmath;

use sdl2::event::Event;
use sdl2::keyboard::Keycode;
use sdl2::pixels::{Color, PixelFormatEnum};
//use std::time::Duration;
use sdl2::mouse::MouseState;
use sdl2::rect::{Point, Rect};

use vecmath::{
    vec2_add, vec2_cross, vec2_dot, vec2_len, vec2_mul, vec2_neg, vec2_normalized, vec2_scale,
    vec2_sub, Vector2,
};

struct RenderRect<'a> {
    pos: Vector2<i32>,
    size: Vector2<u32>,
    angle: f32,
    color: Color,
    texture: sdl2::render::Texture<'a>,
}

impl<'a> RenderRect<'a> {
    pub fn new(
        pos: Vector2<i32>,
        size: Vector2<u32>,
        angle: f32,
        color: Color,
        creator: &sdl2::render::TextureCreator<sdl2::video::WindowContext>,
    ) -> Result<RenderRect, String> {
        let texture = creator
            .create_texture_target(PixelFormatEnum::RGBA8888, size[0], size[1])
            .map_err(|e| e.to_string())?;

        let rect = RenderRect {
            pos: pos,
            size: size,
            angle: angle,
            color: color,
            texture: texture,
        };

        Ok(rect)
    }

    pub fn render(&mut self, canvas: &mut sdl2::render::Canvas<sdl2::video::Window>) {
        render_rect(
            self.color,
            self.angle,
            &mut self.texture,
            canvas,
            self.pos,
            self.size,
        );
    }
}

#[derive(Copy, Clone)]
struct Impulse {
    linear: Vector2<f32>,
    angular: f32,
}

impl Impulse {
    pub fn new() -> Impulse {
        return Impulse {
            linear: [0.0, 0.0],
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

#[derive(Copy, Clone)]
struct PhysicsRect {
    pos: Vector2<f32>,
    size: Vector2<f32>,
    velocity: Vector2<f32>,
    angle: f32,
    angular_velocity: f32,
    mass: f32,
    // temp/status values
    inertia: f32, // angular inertia constant
    impulse: Impulse,
    temp_impulse: Impulse,
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
        return vec2_add(self.velocity, self.impulse.linear);
    }

    pub fn get_impulse_velocity_dt(&self, dt: f32) -> Vector2<f32> {
        return vec2_scale(vec2_add(self.velocity, self.impulse.linear), dt);
    }

    pub fn get_impulse_angular_velocity(&self) -> f32 {
        return self.angular_velocity + self.impulse.angular;
    }

    pub fn get_center(&self) -> Vector2<f32> {
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
    fn get_corners(&self) -> [Vector2<f32>; 4] {
        return [
            [self.pos[0], self.pos[1]],
            [self.pos[0] + self.size[0], self.pos[1]],
            [self.pos[0] + self.size[0], self.pos[1] + self.size[1]],
            [self.pos[0], self.pos[1] + self.size[1]],
        ];
    }

    // Takes in the corners and then rotates them about a point known as the center of mass
    // depreciated because it does not take the new position of the rect into account
    fn _get_rotated_corners(&self, angle: f32) -> [Vector2<f32>; 4] {
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
    fn get_impulse_corners(&self, dt: f32) -> [Vector2<f32>; 4] {
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

fn vec2_mat2_mul(vec: Vector2<f32>, mat: [Vector2<f32>; 2]) -> Vector2<f32> {
    let mut result = [0.0, 0.0];

    result[0] = vec[0] * mat[0][0] + vec[1] * mat[1][0];
    result[1] = vec[0] * mat[0][1] + vec[1] * mat[1][1];

    return result;
}

fn rotate(point: Vector2<f32>, origin: Vector2<f32>, angle: f32) -> Vector2<f32> {
    let rotation_matrix = [
        [angle.cos() as f32, angle.sin() as f32],
        [-1.0 * angle.sin() as f32, angle.cos() as f32],
    ];

    let point_local = vec2_sub(point, origin);
    let rotated_point = vec2_mat2_mul(point_local, rotation_matrix);

    return vec2_add(rotated_point, origin);
}

fn sync_rp_rect(render: &mut RenderRect, physics: &PhysicsRect) {
    render.pos = [
        (physics.pos[0] * 100.0) as i32,
        (physics.pos[1] * 100.0) as i32,
    ];
    render.size = [
        (physics.size[0] * 100.0) as u32,
        (physics.size[1] * 100.0) as u32,
    ];
    render.angle = physics.angle;
}

fn print_vec2(vec: Vector2<f32>) {
    println!("Vector: {}, {}", vec[0], vec[1]);
}

fn _min(num1: f32, num2: f32) -> f32 {
    if num1 < num2 {
        return num1;
    }

    return num2;
}

fn _max(num1: f32, num2: f32) -> f32 {
    if num1 > num2 {
        return num1;
    }

    return num2;
}

fn project(pos: Vector2<f32>, line: [Vector2<f32>; 2]) -> f32 {
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

fn project_rect_line(line: [Vector2<f32>; 2], rect: &PhysicsRect, dt: f32) -> [f32; 4] {
    let mut projection = rect.get_impulse_corners(dt);
    let norm_line = vec2_normalized(vec2_sub(line[1], line[0]));

    let mut out = [0.0, 0.0, 0.0, 0.0];
    for i in 0..4 {
        out[i] = project(projection[i], line);
    }

    return out;
}

fn find_max_f32(list: [f32; 4]) -> usize {
    let mut max = 0;

    for i in 1..4 {
        if list[max] < list[i] {
            max = i;
        }
    }

    return max;
}

fn find_min_f32(list: [f32; 4]) -> usize {
    let mut min = 0;

    for i in 1..4 {
        if list[min] > list[i] {
            min = i;
        }
    }

    return min;
}

fn checkAxisOverlap(
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

fn vec2_dist(vec1: Vector2<f32>, vec2: Vector2<f32>) -> f32 {
    return vec2_len(vec2_sub(vec1, vec2));
}

fn abs(n: f32) -> f32 {
    if n < 0.0 {
        return n * -1.0;
    }

    return n;
}

fn find_normal_from_pos(rect: &PhysicsRect, pos: Vector2<f32>, dt: f32) -> Vector2<f32> {
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

// returns (If the rects collided, pos error, coordinate of points of collision, normal vector according to rect A)
fn check_collision(
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

fn apply_double_normal_constraint(
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

fn apply_normal_constraint(
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

fn rect_constraint(rects: [&mut PhysicsRect; 2], size: usize, iter: usize, dt: f32) {
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
fn screen_bound_constraint(rect: &mut PhysicsRect, dt: f32) {
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

fn apply_constraints(rects: [&mut PhysicsRect; 2], size: usize, iter: usize, dt: f32) {
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

fn render_rect<'a>(
    color: Color,
    angle: f32,
    texture: &mut sdl2::render::Texture<'a>,
    canvas: &mut sdl2::render::Canvas<sdl2::video::Window>,
    pos: Vector2<i32>,
    size: Vector2<u32>,
) {
    canvas.set_draw_color(color);
    canvas
        .with_texture_canvas(texture, |texture_canvas| {
            texture_canvas.clear();
            texture_canvas.set_draw_color(color);
            texture_canvas
                .fill_rect(Rect::new(pos[0], pos[1], size[0], size[1]))
                .expect("could not fill rect");
        })
        .map_err(|e| e.to_string());

    //println!("{} {} {} {}", pos[0], pos[1], size[0], size[1]);
    let dst = Some(Rect::new(pos[0], pos[1], size[0], size[1]));
    canvas.copy_ex(
        &texture,
        None,
        dst,
        (angle as f64) * (180.0 / 3.1415926),
        Some(Point::new((size[0] as i32) / 2, (size[1] as i32) / 2)),
        false,
        false,
    );
}

fn clear_screen(canvas: &mut sdl2::render::Canvas<sdl2::video::Window>) {
    canvas.set_draw_color(Color::RGBA(0, 0, 0, 255));
    canvas.clear();
}

fn main() -> Result<(), String> {
    let sdl_context = sdl2::init()?;
    let video_subsystem = sdl_context.video()?;
    let window = video_subsystem
        .window("rust-sdl2 resource-manager demo", 800, 600)
        .position_centered()
        .build()
        .map_err(|e| e.to_string())?;
    let mut canvas = window
        .into_canvas()
        .software()
        .build()
        .map_err(|e| e.to_string())?;
    let creator = canvas.texture_creator();

    let mut rect = RenderRect::new(
        [150, 300],
        [100, 100],
        0.0,
        Color::RGBA(255, 0, 0, 255),
        &creator,
    )
    .unwrap();
    let mut p_rect = PhysicsRect::new(
        [1.5, 4.0],
        [1.0, 1.0],
        [0.0, 0.0],
        //1.41 / 2.0,
        0.0,
        //3.14159265358979323846264338327950 / 4.0,
        0.0,
        1.0,
    );

    let mut rect_b = RenderRect::new(
        [450, 300],
        [100, 100],
        0.0,
        Color::RGBA(255, 0, 0, 255),
        &creator,
    )
    .unwrap();
    let mut p_rect_b = PhysicsRect::new(
        [4.5, 4.0],
        [1.0, 1.0],
        [0.0, 0.0],
        //1.41 / 2.0,
        0.0,
        // 3.14159265358979 / 4.0,
        0.0,
        1.0,
    );

    // Normal marker line
    let mut norm_rect = RenderRect::new(
        [50, 50],
        [3, 100],
        3.1415926,
        Color::RGBA(255, 255, 255, 255),
        &creator,
    )
    .unwrap();

    // white collision dots
    let mut col_rect = RenderRect::new(
        [0, 0],
        [3, 3],
        0.0,
        Color::RGBA(255, 255, 255, 255),
        &creator,
    )
    .unwrap();
    let mut col_rect_b = RenderRect::new(
        [0, 0],
        [3, 3],
        0.0,
        Color::RGBA(255, 255, 255, 255),
        &creator,
    )
    .unwrap();

    let dt = 0.001;

    let mut last_mouse_pos = [0.0, 0.0];
    let mut mouse_velocity = [0.0, 0.0];
    let mouse_velocity_decay = 0.8;

    let mut grabbed_a = false;
    let mut grabbed_b = false;

    'mainloop: loop {
        for event in sdl_context.event_pump()?.poll_iter() {
            match event {
                // Key A
                Event::KeyDown {
                    repeat: false,
                    keycode: Some(Keycode::A),
                    ..
                } => {
                    p_rect.angle = p_rect.angle + 3.141592653589792328 / 6.0;
                }

                // Key D
                Event::KeyDown {
                    repeat: false,
                    keycode: Some(Keycode::D),
                    ..
                } => {
                    p_rect_b.angle = p_rect_b.angle + 3.141592653589792328 / 6.0;
                }

                // Key SPACE
                Event::KeyDown {
                    repeat: false,
                    keycode: Some(Keycode::Space),
                    ..
                } => {
                    p_rect.velocity = [0.0, 0.0];
                    p_rect.angular_velocity = 0.0;
                    p_rect_b.velocity = [0.0, 0.0];
                    p_rect_b.angular_velocity = 0.0;
                }

                // Exit
                Event::KeyDown {
                    keycode: Some(Keycode::Escape),
                    ..
                }
                | Event::Quit { .. } => break 'mainloop,
                _ => {}
            }
        }

        // Mouse Controls
        let mouse = MouseState::new(&sdl_context.event_pump().unwrap());
        let mouse_pos = [mouse.x() as f32, mouse.y() as f32];
        //println!("Mouse at x: {}, y: {}", mouse.x(), mouse.y());

        // move rectangles
        if mouse.left() || mouse.right() {
            mouse_velocity = vec2_scale(mouse_velocity, mouse_velocity_decay);
            mouse_velocity = vec2_add(vec2_scale(vec2_sub(mouse_pos, last_mouse_pos), 1.0 - mouse_velocity_decay), mouse_velocity);
        }

        // p_rect_a
        if mouse.left()
            && vec2_len(vec2_sub(p_rect.get_center(), vec2_scale(mouse_pos, 0.01)))
                < p_rect.size[0] / 2.0
        {
            //println!("Mouse at x: {}, y: {}", mouse.x(), mouse.y());
            p_rect.move_p(vec2_scale(vec2_sub(mouse_pos, last_mouse_pos), 0.01));

            grabbed_a = true;
        } 
        // set false and give speed
        else if grabbed_a {
            p_rect.velocity = vec2_scale(mouse_velocity, 0.01 * (1.0/dt));
            grabbed_a = false;
        }

        // p_rect_b
        if mouse.right()
            && vec2_len(vec2_sub(p_rect_b.get_center(), vec2_scale(mouse_pos, 0.01)))
                < p_rect_b.size[0] / 2.0
        {
            //println!("Mouse at x: {}, y: {}", mouse.x(), mouse.y());
            p_rect_b.move_p(vec2_scale(vec2_sub(mouse_pos, last_mouse_pos), 0.01));

            grabbed_b = true;
        } 
        // set false and give speed
        else if grabbed_b {
            p_rect_b.velocity = vec2_scale(mouse_velocity, 0.01 * (1.0/dt));
            grabbed_b = false;
        }

        // End Mouse processing
        last_mouse_pos = mouse_pos;

        // Apply Forces
        // p_rect_b.angle = p_rect_b.angle + 1.0 * dt;

        if !(grabbed_a || grabbed_b) {
            // apply gravity
            p_rect.velocity = vec2_add(p_rect.velocity, [0.0, 9.8 * dt]);
            p_rect_b.velocity = vec2_add(p_rect_b.velocity, [0.0, 9.8 * dt]);

            //p_rect.velocity[0] = 0.1;
            //p_rect_b.velocity[0] = -0.1;

            // constrain velocity

            apply_constraints([&mut p_rect, &mut p_rect_b], 2, 100, dt);

            // apply motion
            p_rect.pos = vec2_add(p_rect.pos, vec2_scale(p_rect.velocity, dt));
            p_rect.angle = p_rect.angle + p_rect.angular_velocity * dt as f32;

            p_rect_b.pos = vec2_add(p_rect_b.pos, vec2_scale(p_rect_b.velocity, dt));
            p_rect_b.angle = p_rect_b.angle + p_rect_b.angular_velocity * dt as f32;
        }

        //print_vec2(p_rect.pos);

        // set blue if collides
        //println!("{}", vec2_len(vec2_sub(p_rect.pos, p_rect_b.pos)));
        if (vec2_len(vec2_sub(p_rect.pos, p_rect_b.pos)) < 10.0) {
            let (collided, error, contacts, points, normals) =
                check_collision(&p_rect, &p_rect_b, dt);
            //let (collided_b, erro_b, point_b, normal_b) = check_collision(&p_rect_b, &p_rect, dt);
            if collided {
                rect_b.color = Color::RGBA(0, 0, 255, 255);
                col_rect.pos = [(points[0][0] * 100.0) as i32, (points[0][1] * 100.0) as i32];
                col_rect_b.pos = [(points[1][0] * 100.0) as i32, (points[1][1] * 100.0) as i32];
                //println!("error: {}", error);

                norm_rect.angle = normals[0][1].atan2(normals[0][0]) + 3.1415926 / 2.0;
            } else {
                rect_b.color = Color::RGBA(0, 255, 0, 255);
            }
        } else {
            rect_b.color = Color::RGBA(255, 0, 0, 255);
        }

        // sync rects
        sync_rp_rect(&mut rect, &p_rect);
        sync_rp_rect(&mut rect_b, &p_rect_b);

        clear_screen(&mut canvas);
        rect.render(&mut canvas);
        rect_b.render(&mut canvas);

        col_rect.render(&mut canvas);
        col_rect_b.render(&mut canvas);

        //norm_rect.render(&mut canvas);

        canvas.present();

        // Normalize the loop speed (Count how long has passed for computation and then sleep the rest of the time per frame)
        // Delay Creation tool
        //::std::thread::sleep(Duration::new(0, 1_000_000_000u32 / 30));
    }

    Ok(())
}
