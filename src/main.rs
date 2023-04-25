use gl_lib::na::{Vector2, geometry::Rotation};
use gl_lib::sdl2::{render::*, event::{Event}, rect, pixels};
use gl_lib::helpers;
use gl_lib::gl;
use gl_lib::imode_gui::*;
use gl_lib::imode_gui::drawer2d::*;
use gl_lib::color::Color;

type V2 = Vector2::<f64>;

#[derive(Debug)]
struct Body {
    center: V2,
    limbs: Vec::<Limb>,
    limb_target_offset: V2
}

#[derive(Debug)]
struct Limb {
    upper_leg_len : f64,
    lower_leg_len : f64,
    state: LimbState,
    target_pos: V2,
}


#[derive(Debug)]
enum LimbState {
    Grounded,
    Moving
}


fn main() {

    let sdl_setup = helpers::setup_sdl().unwrap();
    let window = sdl_setup.window;
    let sdl = sdl_setup.sdl;
    let viewport = sdl_setup.viewport;
    let gl = &sdl_setup.gl;

    let mut widget_setup = helpers::setup_widgets(gl).unwrap();

    let mut drawer2D = Drawer2D {
        gl: gl,
        viewport: viewport,
        tr: &mut widget_setup.text_renderer,
        rounded_rect_shader: &mut widget_setup.rounded_rect_shader,
        render_square: &widget_setup.render_square,
        circle_shader: &mut widget_setup.circle_shader
    };


    let mut ui = Ui::new(drawer2D);

    let y_center = (viewport.h  / 2 - 30) as f64;

    let mut body = Body {
        center: V2::new(50.0, y_center),
        limb_target_offset: V2::new(80.0, 100.0),
        limbs: vec! [
            Limb {
                upper_leg_len: 80.0,
                lower_leg_len: 60.0,
                state: LimbState::Grounded,
                target_pos: V2::new(0.0, y_center + 100.0)
            },
            Limb {
                upper_leg_len: 80.0,
                lower_leg_len: 60.0,
                state: LimbState::Grounded,
                target_pos: V2::new(80.0, y_center + 100.0)
            }]
    };

    let mut event_pump = sdl.event_pump().unwrap();

    unsafe {
        gl.ClearColor(1.0, 1.0, 1.0, 1.0);
    }

    let mut sim = false;
    let mut angle = 0.0;
    loop {

        // rendering
        unsafe {
            gl.Clear(gl::COLOR_BUFFER_BIT | gl::DEPTH_BUFFER_BIT);
        }

        let vel = V2::new(1.0, 0.0);

        if sim {
            simulate(&mut body, vel, 1.0);
        }
        update_limbs(&mut body);

        handle_ui(&mut ui, &mut event_pump);

        if ui.button("Sim") {
            sim = !sim;
        };


        // Draw body

        draw_body(&mut ui, &body, angle);

        ui.slider(&mut body.limbs[0].target_pos.x, 0.0, 400.0);

        ui.slider(&mut body.center.x, 0.0, 400.0);

        ui.slider(&mut angle, 0.0, std::f32::consts::PI * 2.0);


        if ui.button("Reset") {
            body.center.x = 15.0;
        };


        window.gl_swap_window();

    }
}

fn draw_body(ui: &mut Ui, body: &Body, angle: f32) {

    let body_color = Color::Rgb(30,240,30);
    let target_color = Color::Rgb(200,30,30);
    let knee_color = Color::Rgb(30,30,200);

    let foot_color = Color::Rgb(200,30,200);



    draw_with_center(ui, body.center, 30, body_color);


    ui.drawer2D.line(body.center.x as i32, body.center.y as i32,  150.0, 20, angle, foot_color);

    for limb in &body.limbs {
        draw_with_center(ui, limb.target_pos, 20, target_color);

        let (knee_pos, foot_pos) = calc_knee_and_foot_pos(ui, body.center, limb) ;

        draw_with_center(ui, knee_pos, 12, knee_color);

        draw_with_center(ui, foot_pos, 12, foot_color);
    }
}

fn calc_knee_and_foot_pos(ui: &mut Ui, body_pos: V2, limb: &Limb) -> (V2, V2) {

    // A is body corner a is opposite of that, so lower leg
    // B is knee corner, b is dist from body to target
    // C is target, c is opposite of that, so upper leg
    let a = limb.lower_leg_len;
    let b = (body_pos - limb.target_pos).magnitude();
    let c = limb.upper_leg_len;

    let total_len = a + b;
    if b > total_len {
        let dir = (limb.target_pos - body_pos).normalize();
        return (dir * a + body_pos, dir * total_len as f64 + body_pos);
    }

    let mut alpha = f64::acos((b*b + c*c - a*a) / (2.0 * b * c));
    let beta = f64::acos((a*a + c*c - b*b) / (2.0 * a * c));


    let s = f64::asin((limb.target_pos.x - body_pos.x) / b);
    let target_a = std::f64::consts::PI/2.0- alpha - s;
    let rot = Rotation::<f64, 2>::new(target_a);

    let mut l = V2::new(100.0, 0.0);

    l = rot * l;


    let knee_pos = body_pos + l;

    let a_1 = (knee_pos - body_pos).magnitude();
    let b_1 = (knee_pos - limb.target_pos).magnitude();

    let c_1 = a_1 + b_1;

    (knee_pos, limb.target_pos)
}



fn draw_with_center(ui: &mut Ui, center: V2, width: i32, color: Color) {
    let w_half = width/2;

    ui.drawer2D.rounded_rect_color(center.x as i32 - w_half, center.y as i32 - w_half, width, width, color);
}




fn handle_ui(ui: &mut Ui, event_pump: &mut gl_lib::sdl2::EventPump) {

    ui.consume_events(event_pump);

}


fn simulate(body: &mut Body, velocity: V2, dt: f64) {
    body.center = body.center + velocity * dt;
}

fn update_limbs(body: &mut Body) {
    for limb in &mut body.limbs {
        let dist = (body.center - limb.target_pos).norm();
        if dist > limb.upper_leg_len + limb.lower_leg_len {
            limb.target_pos = body.center + body.limb_target_offset;
        }
    }
}
