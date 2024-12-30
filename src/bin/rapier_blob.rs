use std::f32::consts::FRAC_PI_2;

use macroquad::prelude::*;
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
    radius: Real,
}
impl Blob {
    fn new(center: Vector2<Real>, radius: Real, world: &mut Physics) -> Self {
        // central ball
        let central_body = RigidBodyBuilder::dynamic().translation(center);
        let handle_center = world.bodies.insert(central_body);
        let collider = ColliderBuilder::ball(radius)
            .friction(0.0)
            .restitution(1.0)
            .mass(1.0);
        world
            .colliders
            .insert_with_parent(collider, handle_center, &mut world.bodies);

        Blob {
            center: handle_center,
            radius,
        }
    }

    fn draw(&self, phys: &mut Physics) {
        let ball_body = &mut phys.bodies[self.center];
        draw_circle(
            ball_body.position().translation.vector.x,
            ball_body.position().translation.vector.y,
            self.radius,
            BLUE,
        );
    }
}

#[macroquad::main("_floating_")]
async fn main() {
    // Dimensions
    let size: f32 = 20.0;
    let gap: f32 = 0.0;
    request_new_screen_size(800.0, 800.0);
    let camera = Camera2D::from_display_rect(Rect {
        x: -size / 2.0 - gap,
        y: -size / 2.0 - gap,
        w: size + gap * 2.0,
        h: size + gap * 2.0,
    });
    set_camera(&camera);

    // Physics
    let mut phys = Physics::default();

    // Blob
    let blob = Blob::new(vector![0.0, 0.0], 5.0e-1, &mut phys);
    let ball_body = &mut phys.bodies[blob.center];
    ball_body.reset_forces(true);
    ball_body.reset_torques(true);
    ball_body.apply_impulse(vector![-3.0, 5.0], true);

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

    loop {
        phys.step();

        clear_background(BLACK);

        blob.draw(&mut phys);

        next_frame().await
    }
}
