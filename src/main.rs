use gl_lib::na::{Vector2, geometry::{Rotation, Translation2, Point2}};
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
    joint0: Joint,
    joint1: Joint,
    state: LimbState,
    target_pos: V2,
}

#[derive(Debug, Copy, Clone)]
struct Joint {
    angle: f64,
    len: f64
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
        center: V2::new(150.0, y_center),
        limb_target_offset: V2::new(80.0, 100.0),
        limbs: vec! [
            Limb {
                joint0: Joint {
                    angle: 1.57,
                    len: 80.0
                },
                joint1: Joint {
                    angle: 1.57,
                    len: 60.0
                },
                state: LimbState::Grounded,
                target_pos: V2::new(0.0, y_center + 100.0)
            },

            Limb {
                joint0: Joint {
                    angle: 0.0,
                    len: 80.0
                },
                joint1: Joint {
                    angle: 0.0,
                    len: 60.0
                },
                state: LimbState::Grounded,
                target_pos: V2::new(80.0, y_center + 100.0)
            }

            ]
    };

    let mut event_pump = sdl.event_pump().unwrap();

    unsafe {
        gl.ClearColor(1.0, 1.0, 1.0, 1.0);
    }

    let mut sim = false;
    let mut update = true;
    loop {

        // rendering
        unsafe {
            gl.Clear(gl::COLOR_BUFFER_BIT | gl::DEPTH_BUFFER_BIT);
        }

        let vel = V2::new(1.0, 0.0);

        if sim {
            simulate(&mut body, vel, 1.0);
        }

        if update {
            update_limbs(&mut body);
        }

        handle_ui(&mut ui, &mut event_pump);

        if ui.button("Sim") {
            sim = !sim;
        };

        if ui.button("Update") {
            update = !update;
        };


        ui.label("limb0 target x:");
        ui.slider(&mut body.limbs[0].target_pos.x, body.center.x + -200.0, body.center.x + 200.0);

        ui.label("Body X:");
        ui.slider(&mut body.center.x, 0.0, 400.0);


        ui.newline();
        ui.label(&format!("Joint0 a({:.2}):", body.limbs[0].joint0.angle));

        ui.slider(&mut body.limbs[0].joint0.angle, 0.0, std::f64::consts::PI);

        ui.newline();
        ui.label(&format!("Joint1 a({:.2}):", body.limbs[0].joint1.angle));
        ui.slider(&mut body.limbs[0].joint1.angle, 0.0, std::f64::consts::PI);

        ui.newline();
        if ui.button("Reset") {
            body.center.x = 15.0;
        };

        // Draw body

        draw_body(&mut ui, &body);

        window.gl_swap_window();

    }
}


fn draw_body(ui: &mut Ui, body: &Body) {

    let body_color = Color::Rgb(30,240,30);

    draw_with_center(ui, body.center, 30, body_color);

    for limb in &body.limbs {
        draw_limb(ui, body.center, &limb);

    }
}


fn forward_kinematics_local(limb: &Limb) -> (V2, V2) {

    // angle for joints
    let rot0 = Rotation::<f64, 2>::new(limb.joint0.angle);
    let rot1 = Rotation::<f64, 2>::new(limb.joint1.angle);

    // from knee to foot tranlstion
    let x2 = V2::new(limb.joint1.len, 0.0);

    // knee to foot with rotation
    let x1 = rot1 * x2 + V2::new(limb.joint0.len, 0.0);

    let foot = rot0 * x1;

    let knee = rot0 * V2::new(limb.joint0.len, 0.0);

    (knee, foot)

}


fn draw_limb(ui: &mut Ui, body_center: V2, limb: &Limb) {

    let target_color = Color::Rgb(200,30,30);
    let knee_color = Color::Rgb(30,30,200);
    let foot_color = Color::Rgb(200,30,200);
    let leg_color =  Color::Rgb(0, 0, 0);

    let (knee, foot) = forward_kinematics_local(limb);

    let knee_pos = knee + body_center;

    let foot_pos = foot + body_center;


    draw_with_center(ui, knee_pos, 20, knee_color);

    draw_with_center(ui, foot_pos, 20, foot_color);

    draw_with_center(ui, limb.target_pos, 10, target_color);

/*    ui.newline();
    ui.label(&format!("Body: {:.2?} {:.2?} ", body_center, (body_center - knee_pos).magnitude()));

    ui.newline();
    ui.label(&format!("Knee: {:.2?}", (knee_pos , (knee_pos - foot_pos).magnitude())));

    ui.newline();
    ui.label(&format!("Foot: {:.2?}", foot_pos));

    ui.newline();
    ui.label(&format!("Mouse: {:.2?}", ui.mouse_pos));
*/

    ui.drawer2D.line(body_center.x as i32, body_center.y as i32, knee_pos.x as i32, knee_pos.y as i32, 5);

    ui.drawer2D.line(knee_pos.x as i32, knee_pos.y as i32, foot_pos.x as i32, foot_pos.y as i32, 5);


}


fn update_joint_angles(body_pos: V2, limb: &mut Limb) {

    // A is body corner a is opposite of that, so lower leg
    // B is knee corner, b is dist from body to target
    // C is target, c is opposite of that, so upper leg

    let a = limb.joint1.len;
    let b = (body_pos - limb.target_pos).magnitude();
    let c = limb.joint0.len;

    let x = body_pos.x - limb.target_pos.x;
    let y = body_pos.y - limb.target_pos.y;

    let total_len = a + c;
    if b > total_len {
        // TODO: hip angle should be pointing towards target, knee angle straight.
        limb.joint0.angle = std::f64::consts::PI - f64::asin((limb.target_pos.x - body_pos.x) / b);
        limb.joint1.angle = 0.0;
        return;
    }

    // find internal angles in triangle
    let mut alpha = f64::acos((b*b + c*c - a*a) / (2.0 * b * c));
    let beta = f64::acos((a*a + c*c - b*b) / (2.0 * a * c));

    // for kinematics we need the outside angles
    // for B it is simply Pi - beta
    // for A we find s which is angle body to target line and the plan.
    // we need to rotate 90 degrees as default, and subtract s and alpha
    // Not quite sure why, but seems to work well, most likely since acos has a limit rangex

    let mut s = f64::asin((limb.target_pos.x - body_pos.x) / b);
    let mut target_a = std::f64::consts::PI/2.0 - s - alpha;

    if x < 0.0 {

    }

    limb.joint0.angle = target_a;
    limb.joint1.angle = std::f64::consts::PI - beta;

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
        if dist > limb.joint0.len + limb.joint1.len {
            limb.target_pos = body.center + body.limb_target_offset;
        }

        update_joint_angles(body.center, limb);
    }
}
