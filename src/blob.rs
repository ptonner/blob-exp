use macroquad::prelude::*;
use nalgebra::Vector2;
use rapier2d::prelude::*;

/// The base blob component, with many connected bodies forming a single blob
pub type Body = RigidBodyHandle;

/// A joint connecting two bodies
pub type Joint = ImpulseJointHandle;

/// A blob-like physical object, made of rigid bodies and joints
pub trait BlobLike {
    /// All bodies in the blob
    fn all_bodies(&self) -> Vec<Body>;
    /// Add an impulse to the blob
    fn apply_impulse<T: BlobPhysics>(&self, imp: Vector2<Real>, phys: &mut T) {
        for node in self.all_bodies() {
            let ball_body = phys.get_body_mut(&node);
            ball_body.apply_impulse(imp, true);
        }
    }
    /// Total mass of the blob, calculated from all bodies
    fn mass<T: BlobPhysics>(&self, phys: &T) -> f32 {
        self.all_bodies()
            .iter()
            .map(|node| phys.get_body(node).mass())
            .sum::<Real>()
    }
}

/// Trait for physics object compatible with blob simulations
pub trait BlobPhysics {
    /// Build a body and add to physics
    ///
    /// - `location`: the location of the body
    /// - `angle`: the orientation of the body, relative to the center
    fn add_body(&mut self, builder: &BlobBodyBuilder, location: Vector2<Real>, angle: f32) -> Body;

    fn add_joint(&mut self, node1: &Body, node2: &Body, joint: GenericJoint) -> Joint;
    fn get_body(&self, node: &Body) -> &RigidBody;
    fn get_body_mut(&mut self, node: &Body) -> &mut RigidBody;
    fn get_collider(&self, node: &ColliderHandle) -> &Collider;
    fn get_collider_mut(&mut self, node: &ColliderHandle) -> &mut Collider;
    fn get_colliders(&self, node: &Body) -> Vec<&Collider>;
    fn get_joint(&self, joint: &Joint) -> &ImpulseJoint;
}

// TODO: make into a trait?
/// Builder for blob bodies
pub struct BlobBodyBuilder {
    pub body_builder: RigidBodyBuilder,
    pub collider_builder: ColliderBuilder,
}

// impl BlobBodyBuilder {
//     /// Build a body and add to physics
//     ///
//     /// - `location`: the location of the body
//     /// - `angle`: the orientation of the body, relative to the center
//     pub fn build<T: BlobPhysics>(&mut self, location: Vector2<Real>, angle: f32) -> Body {
//         self.body_builder = self.body_builder.clone().translation(location);
//         self.collider_builder = self.collider_builder.clone().rotation(angle);
//         phys.add_node(&self)
//     }
// }

impl Default for BlobBodyBuilder {
    fn default() -> Self {
        Self {
            body_builder: RigidBodyBuilder::dynamic(),
            collider_builder: ColliderBuilder::ball(1.0)
                .friction(0.0)
                .restitution(1.0)
                .mass(1.0),
        }
    }
}

pub trait BlobBuilder {
    type BlobType: BlobLike;
    /// Construct a blob-like object in the provided physics system
    fn build<T: BlobPhysics>(&mut self, phys: &mut T) -> Self::BlobType;
}
