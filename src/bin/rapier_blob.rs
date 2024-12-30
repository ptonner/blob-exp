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
}
impl Blob {
    fn new(center: Vector2<Real>, radius: Real, world: &mut Physics) -> Self {
        // central ball
        let central_body = RigidBodyBuilder::dynamic().translation(center);
        let handle_center = world.bodies.insert(central_body);
        let collider = ColliderBuilder::ball(radius).restitution(1.0);
        world
            .colliders
            .insert_with_parent(collider, handle_center, &mut world.bodies);

        Blob {
            center: handle_center,
        }
    }
}

#[macroquad::main("_floating_")]
async fn main() {
    // Dimensions
    request_new_screen_size(800.0, 800.0);
    let w: f32 = 800.0;
    let h: f32 = 800.0;
    // let w = screen_width();
    // let h = screen_height();
    let scale = w.min(h);
    let size = 7.0 * scale / 8.0;
    let gap = scale / 16.0;

    // Physics
    let mut phys = Physics::default();
    dbg!(&phys.integration_parameters.dt);
    // TODO: what are units here / what is the right value?
    phys.integration_parameters.set_inv_dt(1.0);
    let blob = Blob::new(vector![w / 2.0, h / 2.0], 20.0, &mut phys);

    // Add walls
    let mut walls: Vec<ColliderHandle> = Vec::new();
    // bottom
    let collider = ColliderBuilder::cuboid(size, 5.0)
        .translation(vector![gap, gap])
        .friction(0.0)
        .restitution(1.0)
        .build();
    let handle = phys.colliders.insert(collider);
    walls.push(handle);
    // top
    let collider = ColliderBuilder::cuboid(size, 5.0)
        .translation(vector![gap, scale - gap])
        .friction(0.0)
        .restitution(1.0)
        .build();
    let handle = phys.colliders.insert(collider);
    walls.push(handle);
    // left
    let collider = ColliderBuilder::cuboid(size, 5.0)
        .translation(vector![gap, gap])
        .rotation(FRAC_PI_2)
        .friction(0.0)
        .restitution(1.0)
        .build();
    let handle = phys.colliders.insert(collider);
    walls.push(handle);
    // right
    let collider = ColliderBuilder::cuboid(size, 5.0)
        .translation(vector![scale - gap, gap])
        .rotation(FRAC_PI_2)
        .friction(0.0)
        .restitution(1.0)
        .build();
    let handle = phys.colliders.insert(collider);
    walls.push(handle);

    // Initial state
    let ball_body = &mut phys.bodies[blob.center];
    ball_body.reset_forces(true);
    ball_body.reset_torques(true);
    ball_body.apply_impulse(vector![-10000.0, 0.0], true);

    loop {
        phys.step();

        let ball_body = &mut phys.bodies[blob.center];
        clear_background(BLACK);

        draw_circle(
            ball_body.position().translation.vector.x,
            ball_body.position().translation.vector.y,
            15.0,
            BLUE,
        );

        for wall in walls.iter() {
            let body = &phys.colliders[*wall];
            draw_rectangle_ex(
                body.position().translation.x,
                body.position().translation.y,
                size,
                5.0,
                DrawRectangleParams {
                    offset: vec2(0.0, 0.0),
                    rotation: body.position().rotation.angle(),
                    color: RED,
                },
            )
        }

        next_frame().await
    }
}
