use std::{f32::consts::PI, num::NonZeroUsize};

use itertools::Itertools;
use macroquad::prelude::*;
use nalgebra::Vector2;
use rapier2d::prelude::*;

use crate::phys::Physics;

/// The base blob component, with many connected nodes forming a single blob
// type BlobNode = RigidBodyHandle;
#[derive(Clone, Copy)]
struct BlobNode {
    body: RigidBodyHandle,
    radius: Real,
}

impl BlobNode {
    fn get_body<'a>(&self, phys: &'a Physics) -> &'a RigidBody {
        &phys.bodies[self.body]
    }
    fn get_body_mut<'a>(&self, phys: &'a mut Physics) -> &'a mut RigidBody {
        &mut phys.bodies[self.body]
    }
}

/// A single blob layer
type Layer = Vec<BlobNode>;

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
    pub fn total_mass(&self, phys: &Physics) -> f32 {
        let mut mass = self
            .layers
            .iter()
            .map(|layer| {
                layer
                    .iter()
                    .map(|node| node.get_body(&phys).mass())
                    .sum::<Real>()
            })
            .sum();
        if let Some(c) = self.center {
            mass += c.get_body(&phys).mass();
        }
        return mass;
    }
    pub fn apply_impulse(&self, imp: Vector2<Real>, phys: &mut Physics) {
        if let Some(c) = self.center {
            let ball_body = c.get_body_mut(phys);
            // ball_body.reset_forces(true);
            // ball_body.reset_torques(true);
            ball_body.apply_impulse(imp, true);
            return;
        }
        todo!();
    }
    pub fn draw(&self, phys: &Physics) {
        // let bodies = &mut phys.bodies;
        for (n1, n2) in self.joints.iter() {
            let b1 = n1.get_body(phys);
            let b2 = n2.get_body(phys);
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
                let body = node.get_body(&phys);
                draw_circle(
                    body.position().translation.vector.x,
                    body.position().translation.vector.y,
                    node.radius,
                    SKYBLUE,
                );
            }
        }

        if let Some(c) = self.center {
            let body = c.get_body(&phys);
            draw_circle(
                body.position().translation.vector.x,
                body.position().translation.vector.y,
                c.radius,
                BLUE,
            );
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
        return BlobNode {
            body: handle,
            radius: self.radius,
        };
    }
    fn connect_nodes(&self, node1: BlobNode, node2: BlobNode, phys: &mut Physics) {
        // let bodies = &mut phys.bodies;
        let b1 = node1.get_body(phys);
        let b2 = node2.get_body(phys);
        // let b1 = &bodies[node1];
        // let b2 = &bodies[node2];
        let dist = (b1.position().translation.to_homogeneous()
            - b2.position().translation.to_homogeneous())
        .magnitude();
        let joint = SpringJointBuilder::new(dist, self.stiffness, self.damping);
        phys.impulse_joints
            .insert(node1.body, node2.body, joint, true);
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
