use crate::blob::{BlobPhysics, Body};
use macroquad::prelude::*;
use rapier2d::prelude::*;

/// Draw a node based on its colliders
pub fn draw_node_from_colliders<T: BlobPhysics>(node: &Body, phys: &T, color: Color) {
    let body = phys.get_body(node);
    let position = body.position();
    for col in phys.get_colliders(node) {
        let shape = col.shape();
        let angle = col.position().rotation.angle();
        match shape.as_typed_shape() {
            TypedShape::Ball(ball) => {
                draw_circle(
                    position.translation.vector.x,
                    position.translation.vector.y,
                    ball.radius,
                    color,
                );
            }
            TypedShape::Cuboid(cuboid) => {
                let extents = cuboid.half_extents;
                draw_rectangle_ex(
                    position.translation.vector.x,
                    position.translation.vector.y,
                    extents.x * 2.0,
                    extents.y * 2.0,
                    DrawRectangleParams {
                        // offset is applied before width/height scaling, so
                        // this says move back by half the extent in both x and
                        // y directions
                        offset: vec2(0.5, 0.5),
                        rotation: angle,
                        color,
                    },
                );
            }
            _ => todo!(),
        }
    }
}

pub trait Drawable {
    fn draw<T: BlobPhysics>(&self, phys: &T);
}
