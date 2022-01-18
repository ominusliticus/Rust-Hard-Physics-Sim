extern crate sdl2;
extern crate vecmath;

use std::cmp;
use sdl2::event::Event;
use sdl2::keyboard::Keycode;
use sdl2::pixels::{Color, PixelFormatEnum};
//use std::time::Duration;
use sdl2::rect::{Point, Rect};

struct RenderRect<'a> {
    pos: vecmath::Vector2<i32>,
    size: vecmath::Vector2<u32>,
    angle: f64,
    color: Color,
    texture: sdl2::render::Texture<'a>,
}

impl<'a> RenderRect<'a>{
    pub fn new(
        pos: vecmath::Vector2<i32>, 
        size: vecmath::Vector2<u32>,
        angle: f64,
        color: Color,
        creator: &sdl2::render::TextureCreator<sdl2::video::WindowContext
        >) -> Result<RenderRect, String> {

        let mut texture = creator
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
        render_rect(self.color, self.angle, &mut self.texture, canvas, self.pos, self.size);
    }
}

struct PhysicsRect {
    pos: vecmath::Vector2<f32>,
    size: vecmath::Vector2<f32>,
    velocity: vecmath::Vector2<f32>,
    angle: f64,
    angularVelocity: f64,
    mass: f32,
}

fn vec2_mat2_mul(vec: vecmath::Vector2<f32>, mat: [vecmath::Vector2<f32>; 2]) -> vecmath::Vector2<f32> {
    let mut result = [0.0, 0.0];

    result[0] = vec[0] * mat[0][0] + vec[1] * mat[1][0];
    result[1] = vec[0] * mat[0][1] + vec[1] * mat[1][1];

    return result;
}

fn rotate(point: vecmath::Vector2<f32>, origin: vecmath::Vector2<f32>, angle: f64) -> vecmath::Vector2<f32> {
    let rotation_matrix = [
        [angle.cos() as f32, angle.sin() as f32],
        [-1.0 * angle.sin() as f32, angle.cos() as f32],
    ];

    let point_local = vecmath::vec2_sub(point, origin);
    let rotated_point = vec2_mat2_mul(point_local, rotation_matrix);

    return vecmath::vec2_add(rotated_point, origin);
}

// top left, top right, bottom right, bottom left
fn getCorners(rect: &PhysicsRect) -> [vecmath::Vector2<f32>; 4] {
    return [
        [rect.pos[0], rect.pos[1]],
        [rect.pos[0] + rect.size[0], rect.pos[1]],
        [rect.pos[0] + rect.size[0], rect.pos[1] + rect.size[1]],
        [rect.pos[0], rect.pos[1] + rect.size[1]],
    ];
}

// Takes in the corners and then rotates them about a point known as the center of mass
fn calcRotatedCorners(rect: &PhysicsRect, angle: f64, center: vecmath::Vector2<f32>) -> [vecmath::Vector2<f32>; 4] {
    let mut corners = getCorners(rect);

    let mut result = corners;

    for i in 0..corners.len() {
        result[i] = rotate(corners[i], center, angle);
    }

    return result;
}

fn syncRPRect(render: &mut RenderRect, physics: &PhysicsRect) {
    render.pos = [(physics.pos[0] * 100.0) as i32, (physics.pos[1] * 100.0) as i32];
    render.size = [(physics.size[0] * 100.0) as u32, (physics.size[1] * 100.0) as u32];
    render.angle = physics.angle;
}

fn print_vec2(vec: vecmath::Vector2<f32>) {
    println!("Vector: {}, {}", vec[0], vec[1]);
}

fn min(num1: f32, num2: f32) -> f32 {
    if num1 < num2 {
        return num1;
    }

    return num2;
}

fn max(num1: f32, num2: f32) -> f32 {
    if num1 > num2 {
        return num1;
    }

    return num2;
}

// apply the screen bound constraint on velocity
fn screenBoundConstraint(rect: &mut PhysicsRect, dt: f32) {
    // apply for each corner of the square
    // just the bottom left is being processed for now
    let corners = getCorners(&rect);

    let center = [rect.pos[0] + rect.size[0] / 2.0, rect.pos[1] + rect.size[1] / 2.0];

    // find How much they are overlapping
    let normal = [0.0, -1.0];

    // calculate rotational inertia based on M(A^2 + B^2) / 12
    let inertia = rect.mass * (rect.size[0] * rect.size[0] + rect.size[1] * rect.size[1]) / 12.0;

    // current impulses
    let mut impulse = [0.0, 0.0];
    let mut angular_impulse = 0.0;

    for i in 0..1 {
        let mut temp_impulse = [0.0, 0.0];
        let mut temp_angular_impulse = 0.0;

        let mut corner_count = 0;

        for corner in corners {
            //println!("{} {}", radius[0], radius[1]);

            let mut new_pos = vecmath::vec2_add(corner, vecmath::vec2_scale(vecmath::vec2_add(rect.velocity, impulse), dt));
            let new_center = vecmath::vec2_add(center, vecmath::vec2_scale(vecmath::vec2_add(rect.velocity, impulse), dt));
            new_pos = rotate(new_pos, new_center, rect.angle + (rect.angularVelocity + angular_impulse) * (dt as f64));

            let radius = vecmath::vec2_sub(new_pos, new_center);

            let error = vecmath::vec2_dot(normal, vecmath::vec2_sub(new_pos, [new_pos[0], 6.0]));
            
            //println!("error of {}, {} {}, {} {}", error, new_pos[0], new_pos[1], new_copy[0], new_copy[1]);
            
            if error < 0.0 {
                //println!("error of {}, {} {}, {} {}", error, new_pos[0], new_pos[1], new_copy[0], new_copy[1]);

                // tangential velocity
                let r_vec = vecmath::vec2_normalized(vecmath::vec2_sub(new_pos, new_center));
                let tan_vel = (vecmath::vec2_len(radius) * (rect.angularVelocity + angular_impulse) as f32);
                let tan_vec = [r_vec[1] * tan_vel, -1.0 * r_vec[0] * tan_vel];
                //let tan_cross = vecmath::vec2_cross(tan_vec, radius);

                let c_d = vecmath::vec2_dot(vecmath::vec2_sub(rect.velocity, tan_vec), normal);

                print_vec2(rect.velocity);
                //print_vec2(tan_vec);
                //println!("{}", rect.angularVelocity);
                //println!("{}", tan_cross);
                println!("{}", c_d);
                println!("Corner: {}", corner_count);

                let cross = vecmath::vec2_cross(radius, normal);
                let m_eff = 1.0 / (rect.mass + inertia * cross * cross);

                let lambda = -m_eff * c_d;

                //println!("{}", m_eff);

                temp_impulse = vecmath::vec2_add(temp_impulse, vecmath::vec2_scale(normal, lambda));
                temp_angular_impulse = temp_angular_impulse + (lambda * cross) as f64;
                //temp_impulse = vecmath::vec2_add(temp_impulse, vecmath::vec2_scale(normal, lambda));

                //println!("corner: {} error: {} angular velocity: {} angular impulse: {}", corner_count, error, rect.angularVelocity, temp_angular_impulse);
            }

            corner_count = corner_count + 1;
        }

        // apply temp impulse to the main impulse
        impulse = vecmath::vec2_add(impulse, temp_impulse);
        angular_impulse = angular_impulse + temp_angular_impulse;
    }

    if impulse[1] != 0.0 {
        //println!("velocity: {}, {} impulse: {}, {}", rect.velocity[0], rect.velocity[1], impulse[0], impulse[1]);
    }

    rect.velocity = vecmath::vec2_add(rect.velocity, impulse);
    rect.angularVelocity = rect.angularVelocity + angular_impulse;
    //println!("{}", angular_impulse);
}

fn render_rect<'a> (color: Color, angle: f64, texture: &mut sdl2::render::Texture<'a>, canvas: &mut sdl2::render::Canvas<sdl2::video::Window>, pos: vecmath::Vector2<i32>, size: vecmath::Vector2<u32>) {
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
        angle * (180.0 / 3.1415926),
        Some(Point::new((size[0] as i32) / 2, (size[1] as i32) / 2)),
        false,
        false,
    );
    
}

fn clearScreen(canvas: &mut sdl2::render::Canvas<sdl2::video::Window>) {
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

    let mut rect = RenderRect::new([350, 300], [100, 100], 0.0, Color::RGBA(255, 0, 0, 255), &creator).unwrap();
    let mut p_rect = PhysicsRect {
        pos: [3.5, 4.0],
        size: [1.0, 1.0],
        velocity: [0.0, 0.0],
        angle: 1.41 / 2.0,
        //angle: 0.0,
        //angle: 3.14159265358979 / 4.0,
        angularVelocity: 0.0,
        mass: 1.0,
    };

    let dt = 0.001;

    'mainloop: loop {
        for event in sdl_context.event_pump()?.poll_iter() {
            match event {
                Event::KeyDown {
                    keycode: Some(Keycode::Escape),
                    ..
                }
                | Event::Quit { .. } => break 'mainloop,
                _ => {}
            }
        }
        //rect.angle = (rect.angle + 0.5) % 360.;

        // apply gravity
        p_rect.velocity = vecmath::vec2_add(p_rect.velocity, [0.0, 9.8 * dt]);

        // constrain velocity
        screenBoundConstraint(&mut p_rect, dt);

        // apply motion
        p_rect.pos = vecmath::vec2_add(p_rect.pos, vecmath::vec2_scale(p_rect.velocity, dt));
        p_rect.angle = p_rect.angle + p_rect.angularVelocity * dt as f64;

        //print_vec2(p_rect.pos);

        // sync rects
        syncRPRect(&mut rect, &p_rect);

        clearScreen(&mut canvas);
        
        rect.render(&mut canvas);

        canvas.present();

        // Normalize the loop speed (Count how long has passed for computation and then sleep the rest of the time per frame)
        // Delay Creation tool
        //::std::thread::sleep(Duration::new(0, 1_000_000_000u32 / 30));

    }

    Ok(())
}
