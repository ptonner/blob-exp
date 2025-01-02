use std::{f32::consts::PI, num::NonZeroUsize};

use itertools::Itertools;
use macroquad::prelude::*;
use nalgebra::Vector2;
use rapier2d::prelude::*;

use crate::phys::Physics;

/// The base blob component, with many connected nodes forming a single blob
type BlobNode = RigidBodyHandle;

/// Draw a node based on its colliders
fn draw_node_from_colliders(node: &BlobNode, phys: &Physics, color: Color) {
    let body = phys.get_body(node);
    for c in body.colliders() {
        let col = &phys.colliders[*c];
        let shape = col.shape();
        match shape.as_typed_shape() {
            TypedShape::Ball(ball) => {
                draw_circle(
                    body.position().translation.vector.x,
                    body.position().translation.vector.y,
                    ball.radius,
                    color,
                );
            }
            _ => todo!(),
        }
    }
}

impl Physics {
    fn get_body(&self, node: &BlobNode) -> &RigidBody {
        &self.bodies[*node]
    }
    fn get_body_mut(&mut self, node: &BlobNode) -> &mut RigidBody {
        &mut self.bodies[*node]
    }
}

/// A single blob layer
type Layer = Vec<BlobNode>;

/// A joint connecting two nodes
type Joint = (BlobNode, BlobNode);

pub struct Blob {
    /// The central node of the blob
    center: Option<BlobNode>,
    /// Blob layers surrounding the center
    layers: Vec<Layer>,
    /// Joints between nodes
    joints: Vec<Joint>,
}

impl Blob {
    pub fn all_nodes(&self) -> Vec<BlobNode> {
        let layer_nodes = self.layers.clone().into_iter().flatten();
        if let Some(c) = self.center {
            layer_nodes.chain(vec![c].into_iter()).collect()
        } else {
            layer_nodes.collect()
        }
    }

    pub fn total_mass(&self, phys: &Physics) -> f32 {
        let mut mass = self
            .layers
            .iter()
            .map(|layer| {
                layer
                    .iter()
                    .map(|node| phys.get_body(node).mass())
                    .sum::<Real>()
            })
            .sum();
        if let Some(c) = self.center {
            mass += phys.get_body(&c).mass();
        }
        return mass;
    }

    pub fn apply_impulse(&self, imp: Vector2<Real>, phys: &mut Physics) {
        for node in self.all_nodes() {
            let ball_body = phys.get_body_mut(&node);
            ball_body.apply_impulse(imp, true);
        }
    }

    pub fn draw(&self, phys: &Physics) {
        for (n1, n2) in self.joints.iter() {
            let b1 = phys.get_body(&n1);
            let b2 = phys.get_body(&n2);
            draw_line(
                b1.position().translation.vector.x,
                b1.position().translation.vector.y,
                b2.position().translation.vector.x,
                b2.position().translation.vector.y,
                5.0e-2,
                LIGHTGRAY,
            );
        }

        for layer in self.layers.iter() {
            for node in layer.iter() {
                draw_node_from_colliders(node, phys, SKYBLUE);
            }
        }

        if let Some(c) = self.center {
            draw_node_from_colliders(&c, phys, BLUE);
        }
    }
}

/// A prismatic spring joint for building blob joints with valid motion along a
/// single axis.
pub struct PrismaticSpringJoint {
    pub data: GenericJoint,
}

impl PrismaticSpringJoint {
    /// Creates a new prismatic spring joint allowing only relative translations along the specified direction.
    ///
    /// This direction is expressed in the local-space of both rigid-bodies.
    ///
    /// Spring properties are controled via `stiffness` and `damping`.
    ///
    /// NB: A higher value of `num_internal_stabilization_iterations` than the default is recommended for integration parameters when using this joint (i.e. >= 20).
    pub fn new(direction: Vector<Real>, stiffness: Real, damping: Real) -> Self {
        let axis = UnitVector::new_normalize(direction);
        let data = GenericJointBuilder::new(JointAxesMask::LOCKED_PRISMATIC_AXES)
            // Prismatic construction
            .local_axis1(axis)
            .local_axis2(axis)
            // Spring construction
            .coupled_axes(JointAxesMask::LIN_AXES)
            .motor_position(JointAxis::LinX, direction.magnitude(), stiffness, damping)
            .motor_model(JointAxis::LinX, MotorModel::ForceBased)
            .build();
        Self { data }
    }
}

impl From<PrismaticSpringJoint> for GenericJoint {
    fn from(val: PrismaticSpringJoint) -> GenericJoint {
        val.data
    }
}

pub struct BlobNodeBuilder {
    pub body_builder: RigidBodyBuilder,
    pub collider_builder: ColliderBuilder,
}

impl BlobNodeBuilder {
    fn build(&self, center: Vector2<Real>, phys: &mut Physics) -> BlobNode {
        let body = self.body_builder.clone().translation(center).build();
        let handle = phys.bodies.insert(body);
        phys.colliders
            .insert_with_parent(self.collider_builder.build(), handle, &mut phys.bodies);
        return handle;
    }
}

impl Default for BlobNodeBuilder {
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

/// Specification for a blob joint
pub enum BlobJointSpec {
    SpringJoint { stiffness: f32, damping: f32 },
    PrismaticSpringJoint { stiffness: f32, damping: f32 },
}

/// The builder object for blobs
pub struct BlobBuilder {
    /// The node radius
    pub radius: f32,
    /// Whether to include a blob center node
    pub include_center: bool,
    /// Center of the initial blob position
    pub center: Vector2<Real>,
    /// The number of nodes in each layer
    pub layer_size: NonZeroUsize,
    /// The number of layers in the blob
    pub num_layers: u32,
    /// Gap between center and the first layer
    pub center_gap: f32,
    /// Gap between layers of the blob
    pub layer_gap: f32,
    /// The radial joint specification
    pub radial_joint_spec: BlobJointSpec,
    /// The circumferential joint specification
    pub circumferential_joint_spec: BlobJointSpec,
    /// Whether to include circumferential joints in the blob
    pub include_circumferential_joints: bool,
    /// The number of cross layer connections
    pub cross_layer_connections: usize,
    /// External node builder
    pub external_node_builder: BlobNodeBuilder,
    /// Internal node builder
    pub internal_node_builder: BlobNodeBuilder,
}

impl Default for BlobBuilder {
    fn default() -> Self {
        Self {
            center: Vector2::new(0.0, 0.0),
            radius: 1.0,
            include_center: true,
            num_layers: 1,
            layer_size: NonZeroUsize::new(12).unwrap(),
            center_gap: 1.0,
            layer_gap: 0.1,
            radial_joint_spec: BlobJointSpec::PrismaticSpringJoint {
                stiffness: 10.0,
                damping: 10.0,
            },
            circumferential_joint_spec: BlobJointSpec::SpringJoint {
                stiffness: 10.0,
                damping: 10.0,
            },
            include_circumferential_joints: false,
            cross_layer_connections: 1,
            external_node_builder: BlobNodeBuilder::default(),
            internal_node_builder: BlobNodeBuilder::default(),
        }
    }
}

impl BlobBuilder {
    // TODO: modifying field methods

    // Construction operations
    // fn build_node(&self, center: Vector2<Real>, phys: &mut Physics) -> BlobNode {
    //     let body = RigidBodyBuilder::dynamic().translation(center);
    //     let handle = phys.bodies.insert(body);
    //     let collider = ColliderBuilder::ball(self.radius)
    //         // TODO: make these parameters part of the builder
    //         .friction(0.0)
    //         .restitution(1.0)
    //         .mass(1.0);
    //     phys.colliders
    //         .insert_with_parent(collider, handle, &mut phys.bodies);
    //     return handle;
    // }

    fn connect_nodes(
        &self,
        node1: BlobNode,
        node2: BlobNode,
        spec: &BlobJointSpec,
        phys: &mut Physics,
    ) {
        let b1 = phys.get_body(&node1);
        let b2 = phys.get_body(&node2);

        let direction = Vector::from_homogeneous(
            (b2.position().translation.vector - b1.position().translation.vector).to_homogeneous(),
        )
        .unwrap();
        let joint: GenericJoint = match spec {
            BlobJointSpec::SpringJoint { stiffness, damping } => {
                // let axis = b1.position().translation.to_homogeneous()
                //     - b2.position().translation.to_homogeneous();
                // let dist = axis.magnitude();
                SpringJointBuilder::new(direction.magnitude(), *stiffness, *damping)
                    .build()
                    .into()
            }
            BlobJointSpec::PrismaticSpringJoint { stiffness, damping } => {
                PrismaticSpringJoint::new(direction, *stiffness, *damping).into()
            }
        };

        phys.impulse_joints.insert(node1, node2, joint, true);
    }
    pub fn build(&self, phys: &mut Physics) -> Blob {
        // build center
        let mut center = None;
        if self.include_center {
            // center = Some(self.build_node(self.center, phys));
            center = Some(self.internal_node_builder.build(self.center, phys));
        }

        // build layers
        let mut joints = Vec::<Joint>::new();
        let mut layers = Vec::<Layer>::new();
        let mut distance = self.center_gap;
        // place in equal spaced circle
        let shell_step = PI * 2.0 / self.layer_size.get() as f32;
        for l in 1..=self.num_layers {
            let mut layer = Layer::new();

            // build layer
            for i in 0..self.layer_size.into() {
                let x = (shell_step * i as f32).cos();
                let y = (shell_step * i as f32).sin();
                let offset = Vector2::new(x, y) * distance + self.center;
                let node = if l == self.num_layers {
                    self.external_node_builder.build(offset, phys)
                } else {
                    self.internal_node_builder.build(offset, phys)
                };
                // layer.push(self.build_node(offset, phys));
                layer.push(node);
            }

            // if first layer and there is a center, connect them
            if center.is_some() && layers.len() == 0 {
                let c = center.unwrap();
                for n in layer.iter() {
                    self.connect_nodes(c, *n, &self.radial_joint_spec, phys);
                    joints.push((c, *n));
                }
            }

            // connect adjacent nodes
            if self.include_circumferential_joints {
                for (h1, h2) in layer.iter().circular_tuple_windows() {
                    self.connect_nodes(*h1, *h2, &self.circumferential_joint_spec, phys);
                    joints.push((*h1, *h2));
                }
            }
            // connect to previous layer if present
            if let Some(prev_layer) = layers.last() {
                for offset in 0..self.cross_layer_connections {
                    for (p, n) in layer.iter().zip(prev_layer.iter().cycle().skip(offset)) {
                        self.connect_nodes(*p, *n, &self.circumferential_joint_spec, phys);
                        joints.push((*p, *n));
                    }
                }
                // // connect to direct parent
                // for (p, n) in layer.iter().zip(prev_layer) {
                //     self.connect_nodes(*p, *n, phys);
                //     joints.push((*p, *n));
                // }
                // // connect to one forward parent
                // for (p, n) in layer.iter().zip(prev_layer.iter().cycle().skip(1)) {
                //     self.connect_nodes(*p, *n, phys);
                //     joints.push((*p, *n));
                // }
            }

            // prep for next layer
            layers.push(layer);
            distance += self.layer_gap;
        }
        Blob {
            center,
            layers,
            joints,
        }
    }
}
