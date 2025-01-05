use std::{
    f32::consts::{FRAC_PI_2, PI},
    num::NonZeroUsize,
};

use itertools::Itertools;
use macroquad::prelude::*;
use nalgebra::Vector2;
use rapier2d::prelude::*;

use crate::blob::*;
use crate::gfx::*;
use crate::joint::*;

/*
    Layered Blob
*/

/// The builder object for blobs
pub struct LayeredBlobBuilder {
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
    pub radial_joint_builder: Box<dyn BlobJointBuilder>,
    /// The circumferential joint specification
    pub circumferential_joint_builder: Box<dyn BlobJointBuilder>,
    /// Whether to include circumferential joints in the blob
    pub include_circumferential_joints: bool,
    /// The number of cross layer connections
    pub cross_layer_connections: usize,
    /// External node builder
    pub external_node_builder: BlobBodyBuilder,
    /// Internal node builder
    pub internal_node_builder: BlobBodyBuilder,
}

impl Default for LayeredBlobBuilder {
    fn default() -> Self {
        Self {
            center: Vector2::new(0.0, 0.0),
            radius: 1.0,
            include_center: true,
            num_layers: 1,
            layer_size: NonZeroUsize::new(12).unwrap(),
            center_gap: 1.0,
            layer_gap: 0.1,
            radial_joint_builder: Box::new(PrismaticSpringJointBuilder::new(
                Vector2::zeros(),
                10.0,
                10.0,
            )),
            circumferential_joint_builder: Box::new(SpringJointBuilder::new(1.0, 10.0, 10.0)),
            include_circumferential_joints: false,
            cross_layer_connections: 1,
            external_node_builder: BlobBodyBuilder::default(),
            internal_node_builder: BlobBodyBuilder::default(),
        }
    }
}

impl BlobBuilder for LayeredBlobBuilder {
    type BlobType = LayeredBlob;
    fn build<T: BlobPhysics>(&mut self, phys: &mut T) -> LayeredBlob {
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
                    joints.push(connect_nodes(c, *n, &mut self.radial_joint_builder, phys));
                }
            }

            // connect adjacent nodes
            if self.include_circumferential_joints {
                for (h1, h2) in layer.iter().circular_tuple_windows() {
                    joints.push(connect_nodes(
                        *h1,
                        *h2,
                        &mut self.circumferential_joint_builder,
                        phys,
                    ));
                }
            }
            // connect to previous layer if present
            if let Some(prev_layer) = layers.last() {
                for offset in 0..self.cross_layer_connections {
                    for (p, n) in layer.iter().zip(prev_layer.iter().cycle().skip(offset)) {
                        joints.push(connect_nodes(
                            *p,
                            *n,
                            &mut self.circumferential_joint_builder,
                            phys,
                        ));
                    }
                }
            }

            // prep for next layer
            layers.push(layer);
            distance += self.layer_gap;
        }
        LayeredBlob {
            center,
            layers,
            joints,
        }
    }
}

/// A single blob layer
pub type Layer = Vec<Body>;

pub struct LayeredBlob {
    /// The central node of the blob
    pub center: Option<Body>,
    /// Blob layers surrounding the center
    pub layers: Vec<Layer>,
    /// Joints between nodes
    pub joints: Vec<Joint>,
}

impl BlobLike for LayeredBlob {
    fn all_bodies(&self) -> Vec<Body> {
        let layer_nodes = self.layers.clone().into_iter().flatten();
        if let Some(c) = self.center {
            layer_nodes.chain(vec![c].into_iter()).collect()
        } else {
            layer_nodes.collect()
        }
    }

    fn mass<T: BlobPhysics>(&self, phys: &T) -> f32 {
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
}

impl Drawable for LayeredBlob {
    fn draw<T: BlobPhysics>(&self, phys: &T) {
        for joint_handle in self.joints.iter() {
            let joint = phys.get_joint(joint_handle);
            let b1 = phys.get_body(&joint.body1);
            let b2 = phys.get_body(&joint.body2);
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
