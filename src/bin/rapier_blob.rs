use std::{
    f32::consts::{FRAC_PI_2, PI},
    num::NonZeroUsize,
};

use itertools::Itertools;
use macroquad::prelude::*;
use macroquad::rand;
use macroquad::ui::{hash, root_ui};
use nalgebra::Vector2;
use rapier2d::prelude::*;

#[derive(Default)]
struct Physics {
    bodies: RigidBodySet,
    colliders: ColliderSet,
    impulse_joints: ImpulseJointSet,
    multibody_joints: MultibodyJointSet,
    integration_parameters: IntegrationParameters,
    physics_pipeline: PhysicsPipeline,
    island_manager: IslandManager,
    broad_phase: DefaultBroadPhase,
    narrow_phase: NarrowPhase,
    ccd_solver: CCDSolver,
    query_pipeline: QueryPipeline,
    gravity: Vector2<Real>,
}

impl Physics {
    fn step(&mut self) {
        let physics_hooks = ();
        let event_handler = ();
        self.physics_pipeline.step(
            &self.gravity,
            &self.integration_parameters,
            &mut self.island_manager,
            &mut self.broad_phase,
            &mut self.narrow_phase,
            &mut self.bodies,
            &mut self.colliders,
            &mut self.impulse_joints,
            &mut self.multibody_joints,
            &mut self.ccd_solver,
            Some(&mut self.query_pipeline),
            &physics_hooks,
            &event_handler,
        );
    }
}

struct Blob {
    center: RigidBodyHandle,
    shell: Vec<RigidBodyHandle>,
    radius: Real,
}
impl Blob {
    fn new(
        center: Vector2<Real>,
        radius: Real,
        shell_size: u32,
        stiffness: f32,
        damping: f32,
        gap: f32,
        world: &mut Physics,
    ) -> Self {
        // central ball
        let central_body = RigidBodyBuilder::dynamic()
            .translation(center)
            .lock_rotations();
        let handle_center = world.bodies.insert(central_body);
        let collider = ColliderBuilder::ball(radius)
            .friction(0.0)
            .restitution(1.0)
            .density(1.0);
        world
            .colliders
            .insert_with_parent(collider, handle_center, &mut world.bodies);

        // Shell
        // let stiffness = 150.0;
        // let damping = 2.0;
        let shell_step = PI * 2.0 / shell_size as f32;
        let base_distance = (1.5 + gap) * radius;
        let mut shells = Vec::<RigidBodyHandle>::new();
        for i in 0..shell_size {
            let x = (shell_step * i as f32).cos();
            let y = (shell_step * i as f32).sin();
            let offset = Vector2::new(x, y) * base_distance;
            let shell = RigidBodyBuilder::dynamic()
                .translation(center + offset)
                .lock_rotations();
            let handle_shell = world.bodies.insert(shell);
            let collider = ColliderBuilder::ball(radius * 0.5)
                .friction(0.0)
                .restitution(1.0)
                .density(1.0);
            world
                .colliders
                .insert_with_parent(collider, handle_shell, &mut world.bodies);
            let joint = SpringJointBuilder::new(base_distance, stiffness, damping);
            world
                .impulse_joints
                .insert(handle_center, handle_shell, joint, true);
            shells.push(handle_shell);
        }
        // connect adjacent shell nodes
        let bodies = &mut world.bodies;
        for (h1, h2) in shells.iter().circular_tuple_windows() {
            let b1 = &bodies[*h1];
            let b2 = &bodies[*h2];
            // let dist = distance(b1.position().translation, b1.position().translation);
            let dist = (b1.position().translation.to_homogeneous()
                - b2.position().translation.to_homogeneous())
            .magnitude();
            let joint = SpringJointBuilder::new(dist, stiffness, damping);
            world.impulse_joints.insert(*h1, *h2, joint, true);
        }

        Blob {
            center: handle_center,
            shell: shells,
            radius,
        }
    }

    fn draw(&self, phys: &mut Physics) {
        let bodies = &mut phys.bodies;
        let ball_body = &bodies[self.center];
        for (h1, h2) in self.shell.iter().circular_tuple_windows() {
            let b1 = &bodies[*h1];
            let b2 = &bodies[*h2];
            draw_line(
                b1.position().translation.vector.x,
                b1.position().translation.vector.y,
                b2.position().translation.vector.x,
                b2.position().translation.vector.y,
                5.0e-2,
                LIGHTGRAY,
            );
        }
        for handle in self.shell.iter() {
            let shell_body = &bodies[*handle];
            draw_line(
                ball_body.position().translation.vector.x,
                ball_body.position().translation.vector.y,
                shell_body.position().translation.vector.x,
                shell_body.position().translation.vector.y,
                5.0e-2,
                LIGHTGRAY,
            );
            draw_circle(
                shell_body.position().translation.vector.x,
                shell_body.position().translation.vector.y,
                self.radius * 0.5,
                SKYBLUE,
            );
        }
        draw_circle(
            ball_body.position().translation.vector.x,
            ball_body.position().translation.vector.y,
            self.radius,
            BLUE,
        );
    }

    fn total_mass(&self, phys: &Physics) -> f32 {
        let bodies = &phys.bodies;
        bodies[self.center].mass() + self.shell.iter().map(|it| bodies[*it].mass()).sum::<Real>()
    }
}

fn init(
    size: f32,
    num_splits: u32,
    shell_size: u32,
    log_radius: f32,
    impulse: f32,
) -> (Physics, Vec<Blob>) {
    let mut phys = Physics::default();
    phys.integration_parameters.contact_damping_ratio = 1.0e-1;
    phys.integration_parameters.num_solver_iterations = NonZeroUsize::new(10).unwrap();
    let mut blobs = Vec::<Blob>::new();
    for i in 0..num_splits {
        for j in 0..num_splits {
            let blob = Blob::new(
                vector![
                    size * (i as f32 / num_splits as f32) - size / 2.0
                        + size / 2.0 / num_splits as f32,
                    size * (j as f32 / num_splits as f32) - size / 2.0
                        + size / 2.0 / num_splits as f32
                ],
                10.0_f32.powf(log_radius),
                shell_size,
                100.0,
                2.0,
                1.0,
                &mut phys,
            );
            let ang = rand::gen_range(-PI, PI);
            let imp =
                Vector2::new(ang.cos(), ang.sin()).normalize() * blob.total_mass(&phys) * impulse;
            let ball_body = &mut phys.bodies[blob.center];
            ball_body.reset_forces(true);
            ball_body.reset_torques(true);
            ball_body.apply_impulse(imp, true);
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
    let size: f32 = 50.0;
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
    let mut num_splits = 6;
    let mut shell_size = 12;
    let mut log_radius = 0.0;
    let mut impulse = 1.0;
    let (mut phys, mut blobs) = init(size, num_splits, shell_size, log_radius, impulse);

    loop {
        phys.step();

        clear_background(BLACK);

        // ui
        root_ui().window(hash!(), dialog_position, dialog_size, |ui| {
            ui.drag(hash!(), "splits", Some((1, 12)), &mut num_splits);
            ui.drag(hash!(), "shell size", Some((6, 24)), &mut shell_size);
            ui.slider(hash!(), "log radius", -2f32..1f32, &mut log_radius);
            ui.slider(hash!(), "impulse", 0.5f32..15.0f32, &mut impulse);
            if ui.button(None, "reset") {
                let (p, b) = init(size, num_splits, shell_size, log_radius, impulse);
                phys = p;
                blobs = b;
            }
        });

        // scene
        for blob in blobs.iter() {
            blob.draw(&mut phys);
        }

        next_frame().await
    }
}
