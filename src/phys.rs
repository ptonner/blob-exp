use nalgebra::Vector2;
use rapier2d::prelude::*;

use crate::blob::{BlobBodyBuilder, BlobPhysics, Body, Joint};

/// A generic physics simulation backed by rapier2d
#[derive(Default)]
pub struct Physics {
    pub bodies: RigidBodySet,
    pub colliders: ColliderSet,
    pub impulse_joints: ImpulseJointSet,
    pub multibody_joints: MultibodyJointSet,
    pub integration_parameters: IntegrationParameters,
    pub physics_pipeline: PhysicsPipeline,
    pub island_manager: IslandManager,
    pub broad_phase: DefaultBroadPhase,
    pub narrow_phase: NarrowPhase,
    pub ccd_solver: CCDSolver,
    pub query_pipeline: QueryPipeline,
    pub gravity: Vector2<Real>,
}

impl Physics {
    pub fn step(&mut self) {
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

impl BlobPhysics for Physics {
    fn add_body(&mut self, builder: &BlobBodyBuilder, location: Vector2<Real>, angle: f32) -> Body {
        let body_builder = builder.body_builder.clone().translation(location);
        let collider_builder = builder.collider_builder.clone().rotation(angle);
        let body = body_builder.build();
        let handle = self.bodies.insert(body);
        self.colliders
            .insert_with_parent(collider_builder.build(), handle, &mut self.bodies);
        return handle;
    }
    fn get_body(&self, node: &Body) -> &RigidBody {
        &self.bodies[*node]
    }
    fn get_body_mut(&mut self, node: &Body) -> &mut RigidBody {
        &mut self.bodies[*node]
    }
    fn get_collider(&self, node: &ColliderHandle) -> &Collider {
        &self.colliders[*node]
    }
    fn get_collider_mut(&mut self, node: &ColliderHandle) -> &mut Collider {
        &mut self.colliders[*node]
    }
    fn get_colliders(&self, node: &Body) -> Vec<&Collider> {
        let body = self.get_body(node);
        body.colliders()
            .into_iter()
            .map(|c| self.get_collider(c))
            .collect()
    }
    fn add_joint(&mut self, node1: &Body, node2: &Body, joint: GenericJoint) -> Joint {
        self.impulse_joints.insert(*node1, *node2, joint, true)
    }
    fn get_joint(&self, joint: &Joint) -> &ImpulseJoint {
        self.impulse_joints.get(*joint).unwrap()
    }
}
