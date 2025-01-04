use nalgebra::Vector2;
use rapier2d::prelude::*;

use crate::blob::{BlobNode, BlobNodeBuilder, BlobPhysics};

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
    fn add_node(&mut self, builder: &BlobNodeBuilder) -> BlobNode {
        let body = builder.body_builder.build();
        let handle = self.bodies.insert(body);
        self.colliders.insert_with_parent(
            builder.collider_builder.build(),
            handle,
            &mut self.bodies,
        );
        return handle;
    }
    fn get_body(&self, node: &BlobNode) -> &RigidBody {
        &self.bodies[*node]
    }
    fn get_body_mut(&mut self, node: &BlobNode) -> &mut RigidBody {
        &mut self.bodies[*node]
    }
    fn get_collider(&self, node: &ColliderHandle) -> &Collider {
        &self.colliders[*node]
    }
    fn get_collider_mut(&mut self, node: &ColliderHandle) -> &mut Collider {
        &mut self.colliders[*node]
    }
    fn get_colliders(&self, node: &BlobNode) -> Vec<&Collider> {
        let body = self.get_body(node);
        body.colliders()
            .into_iter()
            .map(|c| self.get_collider(c))
            .collect()
    }
    fn add_joint(&mut self, node1: &BlobNode, node2: &BlobNode, joint: GenericJoint) {
        self.impulse_joints.insert(*node1, *node2, joint, true);
    }
}
