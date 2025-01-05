use std::f32::consts::PI;
use std::{f32::consts::FRAC_PI_2, num::NonZeroUsize};

use macroquad::prelude::*;
// use macroquad::rand;
use macroquad::ui::{hash, root_ui};
use nalgebra::Vector2;
use rapier2d::prelude::*;

use blobs::blob::*;
use blobs::build::*;
use blobs::gfx::Drawable;
use blobs::phys::Physics;

fn init(
    size: f32,
    num_splits: u32,
    num_layers: u32,
    shell_size: u32,
    center_gap: f32,
    layer_gap: f32,
    log_radius: f32,
    impulse: f32,
) -> (Physics, Vec<LayeredBlob>) {
    let mut phys = Physics::default();
    // phys.integration_parameters.contact_damping_ratio = 1.0e2;
    // phys.integration_parameters.num_solver_iterations = NonZeroUsize::new(4).unwrap();
    phys.integration_parameters
        .num_internal_stabilization_iterations = 20;
    // phys.integration_parameters.max_ccd_substeps = 50;

    let mut blobs = Vec::<LayeredBlob>::new();
    for i in 0..num_splits {
        for j in 0..num_splits {
            let center = vector![
                size * (i as f32 / num_splits as f32) - size / 2.0 + size / 2.0 / num_splits as f32,
                size * (j as f32 / num_splits as f32) - size / 2.0 + size / 2.0 / num_splits as f32
            ];
            let mut builder = LayeredBlobBuilder {
                center,
                center_gap,
                radius: (10.0_f32).powf(log_radius),
                num_layers,
                layer_gap,
                layer_size: NonZeroUsize::new(shell_size as usize)
                    .unwrap_or(NonZeroUsize::new(12).unwrap()),
                external_node_builder: BlobBodyBuilder {
                    body_builder: RigidBodyBuilder::dynamic(),
                    collider_builder: ColliderBuilder::cuboid(1.0, 1.0e-1),
                },
                ..Default::default()
            };
            let blob = builder.build(&mut phys);

            let ang = rand::gen_range(-PI, PI);
            let imp = Vector2::new(ang.cos(), ang.sin()).normalize() * blob.mass(&phys) * impulse;
            blob.apply_impulse(imp, &mut phys);
            blobs.push(blob);
        }
    }

    // Add walls
    let mut walls: Vec<ColliderHandle> = Vec::new();
    let wall_builder = ColliderBuilder::cuboid(size, 1.0e-3)
        .friction(0.0)
        .restitution(1.0);
    // bottom
    let collider = wall_builder
        .clone()
        .translation(vector![-size / 2.0, -size / 2.0]);
    let handle = phys.colliders.insert(collider);
    walls.push(handle);
    // top
    let collider = wall_builder
        .clone()
        .translation(vector![-size / 2.0, size / 2.0]);
    let handle = phys.colliders.insert(collider);
    walls.push(handle);
    // left
    let collider = wall_builder
        .clone()
        .rotation(-FRAC_PI_2)
        .translation(vector![-size / 2.0, -size / 2.0]);
    let handle = phys.colliders.insert(collider);
    walls.push(handle);
    // right
    let collider = wall_builder
        .clone()
        .rotation(FRAC_PI_2)
        .translation(vector![size / 2.0, -size / 2.0]);
    let handle = phys.colliders.insert(collider);
    walls.push(handle);

    (phys, blobs)
}

#[macroquad::main("_floating_")]
async fn main() {
    // Dimensions
    let size: f32 = 20.0;
    let gap: f32 = 0.0;
    request_new_screen_size(800.0, 800.0);
    let dialog_size = vec2(200., 200.);
    let dialog_position = vec2(0.0, 0.0);
    let camera = Camera2D::from_display_rect(Rect {
        x: -size / 2.0 - gap,
        y: -size / 2.0 - gap,
        w: size + gap * 2.0,
        h: size + gap * 2.0,
    });
    set_camera(&camera);

    // Simulation
    let mut num_splits = 1;
    let mut num_layers = 1;
    let mut shell_size = 12;
    let mut log_radius = -0.5;
    let mut center_gap = 4.0;
    let mut layer_gap = 1.0;
    let mut impulse = 0.5;
    let mut run = true;
    let mut step_size = 10;
    let (mut phys, mut blobs) = init(
        size, num_splits, num_layers, shell_size, center_gap, layer_gap, log_radius, impulse,
    );

    loop {
        if run {
            phys.step();
        }

        clear_background(BLACK);

        // ui
        root_ui().window(hash!(), dialog_position, dialog_size, |ui| {
            ui.drag(hash!(), "splits", Some((1, 12)), &mut num_splits);
            ui.drag(hash!(), "shell size", Some((1, 24)), &mut shell_size);
            ui.drag(hash!(), "num layers", Some((1, 4)), &mut num_layers);
            ui.slider(hash!(), "center gap", 0.1..10.0, &mut center_gap);
            ui.slider(hash!(), "layer gap", 0.1..3.0, &mut layer_gap);
            ui.slider(hash!(), "log radius", -2f32..1f32, &mut log_radius);
            ui.slider(hash!(), "impulse", 0.0f32..5.0f32, &mut impulse);
            ui.drag(hash!(), "step size", Some((0, 50)), &mut step_size);
            if ui.button(None, "reset") {
                let (p, b) = init(
                    size, num_splits, num_layers, shell_size, center_gap, layer_gap, log_radius,
                    impulse,
                );
                phys = p;
                blobs = b;
            }
            ui.same_line(50.0);
            if ui.button(None, "stop") {
                run = false
            }
            ui.same_line(100.0);
            if ui.button(None, "start") {
                run = true
            }
            ui.same_line(150.0);
            if ui.button(None, "step") {
                run = false;
                for _ in 0..step_size {
                    phys.step();
                }
            }
        });

        // scene
        for blob in blobs.iter() {
            blob.draw(&mut phys);
        }

        next_frame().await
    }
}
