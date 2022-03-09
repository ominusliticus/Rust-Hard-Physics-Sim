extern crate sdl2;
extern crate vecmath;

use vecmath::Vector2;
use sdl2::pixels::{Color, PixelFormatEnum};
use sdl2::rect::{Point, Rect};

pub struct RenderRect<'a> {
    pub pos: Vector2<i32>,
    pub size: Vector2<u32>,
    pub angle: f32,
    pub color: Color,
    pub texture: sdl2::render::Texture<'a>,
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

pub fn render_rect<'a>(
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
