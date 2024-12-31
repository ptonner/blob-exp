use std::{f32::consts::PI, num::NonZeroUsize};

use itertools::Itertools;
use macroquad::prelude::*;
use nalgebra::Vector2;
use rapier2d::prelude::*;

use crate::phys::Physics;

/// The base blob component, with many connected nodes forming a single blob
type BlobNode = RigidBodyHandle;
// #[derive(Clone, Copy)]
// pub struct BlobNode {
//     body: RigidBodyHandle,
//     radius: Real,
// }

fn draw_node(node: &BlobNode, phys: &Physics) {
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
                    BLUE,
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
                draw_node(node, phys);
            }
        }

        if let Some(c) = self.center {
            draw_node(&c, phys);
        }
    }
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
    /// Stiffness of the blob springs
    pub stiffness: f32,
    /// Damping of the blob springs
    pub damping: f32,
    /// Gap between center and the first layer
    pub center_gap: f32,
    /// Gap between layers of the blob
    pub layer_gap: f32,
}

impl Default for BlobBuilder {
    fn default() -> Self {
        Self {
            center: Vector2::new(0.0, 0.0),
            radius: 1.0,
            include_center: true,
            num_layers: 1,
            layer_size: NonZeroUsize::new(12).unwrap(),
            stiffness: 100.0,
            damping: 1.0,
            center_gap: 1.0,
            layer_gap: 0.1,
        }
    }
}

impl BlobBuilder {
    // TODO: modifying field methods
    // Construction operations
    fn build_node(&self, center: Vector2<Real>, phys: &mut Physics) -> BlobNode {
        let body = RigidBodyBuilder::dynamic().translation(center);
        let handle = phys.bodies.insert(body);
        let collider = ColliderBuilder::ball(self.radius)
            // TODO: make these parameters part of the builder
            .friction(0.0)
            .restitution(1.0)
            .mass(1.0);
        phys.colliders
            .insert_with_parent(collider, handle, &mut phys.bodies);
        return handle;
    }
    fn connect_nodes(&self, node1: BlobNode, node2: BlobNode, phys: &mut Physics) {
        // let bodies = &mut phys.bodies;
        let b1 = phys.get_body(&node1);
        let b2 = phys.get_body(&node2);
        // let b1 = &bodies[node1];
        // let b2 = &bodies[node2];
        let dist = (b1.position().translation.to_homogeneous()
            - b2.position().translation.to_homogeneous())
        .magnitude();
        let joint = SpringJointBuilder::new(dist, self.stiffness, self.damping);
        phys.impulse_joints.insert(node1, node2, joint, true);
    }
    pub fn build(&self, phys: &mut Physics) -> Blob {
        // build center
        let mut center = None;
        if self.include_center {
            center = Some(self.build_node(self.center, phys));
        }

        // build layers
        let mut joints = Vec::<Joint>::new();
        let mut layers = Vec::<Layer>::new();
        let mut distance = self.center_gap;
        // place in equal spaced circle
        let shell_step = PI * 2.0 / self.layer_size.get() as f32;
        for _l in 0..self.num_layers {
            let mut layer = Layer::new();

            // build layer
            for i in 0..self.layer_size.into() {
                let x = (shell_step * i as f32).cos();
                let y = (shell_step * i as f32).sin();
                let offset = Vector2::new(x, y) * distance + self.center;
                layer.push(self.build_node(offset, phys));
            }

            // connect adjacent nodes
            for (h1, h2) in layer.iter().circular_tuple_windows() {
                self.connect_nodes(*h1, *h2, phys);
                joints.push((*h1, *h2));
            }

            // if first layer and there is a center, connect them
            if center.is_some() && layers.len() == 0 {
                let c = center.unwrap();
                for n in layer.iter() {
                    self.connect_nodes(c, *n, phys);
                    joints.push((c, *n));
                }
            }

            // connect to previous layer if present
            if layers.len() > 0 {
                let prev_layer = layers.last().unwrap();
                // connect to direct parent
                for (p, n) in layer.iter().zip(prev_layer) {
                    self.connect_nodes(*p, *n, phys);
                    joints.push((*p, *n));
                }
                // connect to one forward parent
                for (p, n) in layer.iter().zip(prev_layer.iter().cycle().skip(1)) {
                    self.connect_nodes(*p, *n, phys);
                    joints.push((*p, *n));
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