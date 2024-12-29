use macroquad::prelude::*;
use nalgebra::Vector2;
use rapier2d::prelude::*;

#[derive(Default)]
struct World {
    bodies: RigidBodySet,
    colliders: ColliderSet,
    impulse_joints: ImpulseJointSet,
    multibody_joints: MultibodyJointSet,
}

struct Blob {
    center: RigidBodyHandle,
}
impl Blob {
    fn new(center: Vector2<Real>, radius: Real, world: &mut World) -> Self {
        // central ball
        let central_body = RigidBodyBuilder::dynamic().translation(center);
        let handle_center = world.bodies.insert(central_body);
        let collider = ColliderBuilder::ball(radius);
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
    let w = screen_width();
    let h = screen_height();
    let mut world = World::default();
    let blob = Blob::new(vector![w / 2.0, h / 2.0], 20.0, &mut world);

    /* Create other structures necessary for the simulation. */
    let gravity = vector![0.0, 0.0];
    let mut integration_parameters = IntegrationParameters::default();
    integration_parameters.set_inv_dt(60.0);
    let mut physics_pipeline = PhysicsPipeline::new();
    let mut island_manager = IslandManager::new();
    let mut broad_phase = DefaultBroadPhase::new();
    let mut narrow_phase = NarrowPhase::new();
    let mut ccd_solver = CCDSolver::new();
    let mut query_pipeline = QueryPipeline::new();
    let physics_hooks = ();
    let event_handler = ();

    loop {
        physics_pipeline.step(
            &gravity,
            &integration_parameters,
            &mut island_manager,
            &mut broad_phase,
            &mut narrow_phase,
            &mut world.bodies,
            &mut world.colliders,
            &mut world.impulse_joints,
            &mut world.multibody_joints,
            &mut ccd_solver,
            Some(&mut query_pipeline),
            &physics_hooks,
            &event_handler,
        );

        let ball_body = &world.bodies[blob.center];

        clear_background(BLACK);

        draw_circle(
            ball_body.position().translation.vector.x,
            screen_height() - ball_body.position().translation.vector.y,
            15.0,
            BLUE,
        );

        next_frame().await
    }
}
