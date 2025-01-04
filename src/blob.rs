use std::{
    f32::consts::{FRAC_PI_2, PI},
    num::NonZeroUsize,
};

use itertools::Itertools;
use macroquad::prelude::*;
use nalgebra::Vector2;
use rapier2d::prelude::*;

/// The base blob component, with many connected nodes forming a single blob
pub type BlobNode = RigidBodyHandle;

/// Trait for physics object compatible with blob simulations
pub trait BlobPhysics {
    fn add_node(&mut self, builder: &BlobNodeBuilder) -> BlobNode;
    fn add_joint(&mut self, node1: &BlobNode, node2: &BlobNode, joint: GenericJoint);
    fn get_body(&self, node: &BlobNode) -> &RigidBody;
    fn get_body_mut(&mut self, node: &BlobNode) -> &mut RigidBody;
    fn get_collider(&self, node: &ColliderHandle) -> &Collider;
    fn get_collider_mut(&mut self, node: &ColliderHandle) -> &mut Collider;
    fn get_colliders(&self, node: &BlobNode) -> Vec<&Collider>;
}

/// A single blob layer
type Layer = Vec<BlobNode>;

/// A joint connecting two nodes
type Joint = (BlobNode, BlobNode);

pub struct Blob {
    /// The central node of the blob
    pub center: Option<BlobNode>,
    /// Blob layers surrounding the center
    pub layers: Vec<Layer>,
    /// Joints between nodes
    pub joints: Vec<Joint>,
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

    pub fn total_mass<T: BlobPhysics>(&self, phys: &T) -> f32 {
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

    pub fn apply_impulse<T: BlobPhysics>(&self, imp: Vector2<Real>, phys: &mut T) {
        for node in self.all_nodes() {
            let ball_body = phys.get_body_mut(&node);
            ball_body.apply_impulse(imp, true);
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
    fn build<T: BlobPhysics>(
        &mut self,
        center: Vector2<Real>,
        angle: f32,
        phys: &mut T,
    ) -> BlobNode {
        self.body_builder = self.body_builder.clone().translation(center);
        self.collider_builder = self.collider_builder.clone().rotation(angle);
        phys.add_node(&self)
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
    fn connect_nodes<T: BlobPhysics>(
        &self,
        node1: BlobNode,
        node2: BlobNode,
        spec: &BlobJointSpec,
        phys: &mut T,
    ) {
        let b1 = phys.get_body(&node1);
        let b2 = phys.get_body(&node2);

        let direction = Vector::from_homogeneous(
            (b2.position().translation.vector - b1.position().translation.vector).to_homogeneous(),
        )
        .unwrap();
        let joint: GenericJoint = match spec {
            BlobJointSpec::SpringJoint { stiffness, damping } => {
                SpringJointBuilder::new(direction.magnitude(), *stiffness, *damping)
                    .build()
                    .into()
            }
            BlobJointSpec::PrismaticSpringJoint { stiffness, damping } => {
                PrismaticSpringJoint::new(direction, *stiffness, *damping).into()
            }
        };

        phys.add_joint(&node1, &node2, joint);
    }

    pub fn build<T: BlobPhysics>(&mut self, phys: &mut T) -> Blob {
        // build center
        let mut center = None;
        if self.include_center {
            // center = Some(self.build_node(self.center, phys));
            center = Some(self.internal_node_builder.build(self.center, 0.0, phys));
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
                let angle = shell_step * i as f32;
                let x = angle.cos();
                let y = angle.sin();
                let offset = Vector2::new(x, y) * distance + self.center;
                // this angle is oriented towards the center
                let local_angle = angle + FRAC_PI_2;
                let node = if l == self.num_layers {
                    self.external_node_builder.build(offset, local_angle, phys)
                } else {
                    self.internal_node_builder.build(offset, local_angle, phys)
                };
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
