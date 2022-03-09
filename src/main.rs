#![warn(clippy::all, clippy::pedantic)]

extern crate sdl2;
extern crate vecmath;

// ALL the sdl code should go in its own file
use sdl2::event::Event;
use sdl2::keyboard::Keycode;
use sdl2::pixels::Color;
//use std::time::Duration;
use sdl2::mouse::MouseState;

use vecmath::{
    Vector2,
    vec2_add,
    vec2_sub,
    vec2_scale
};

// include all modules so other files can see then
mod collision_dectection;
mod impulse;
mod phys_rect;
mod render_rect;
mod my_maths;

use crate::collision_dectection::{apply_constraints, check_collision};
use crate::phys_rect::PhysicsRect;
use crate::render_rect::RenderRect;
use crate::my_maths::vec2_len;

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
