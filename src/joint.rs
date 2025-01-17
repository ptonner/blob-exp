use rapier2d::prelude::*;

use crate::blob::{BlobPhysics, Body, Joint};

/// A prismatic spring joint for building blob joints with valid motion along a
/// single axis.
#[derive(Copy, Clone, Debug, PartialEq)]
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

    /*
        from spring and/or prismatic joints
    */

    /// The underlying generic joint.
    pub fn data(&self) -> &GenericJoint {
        &self.data
    }

    /// Are contacts between the attached rigid-bodies enabled?
    pub fn contacts_enabled(&self) -> bool {
        self.data.contacts_enabled
    }

    /// Sets whether contacts between the attached rigid-bodies are enabled.
    pub fn set_contacts_enabled(&mut self, enabled: bool) -> &mut Self {
        self.data.set_contacts_enabled(enabled);
        self
    }

    /// The joint’s anchor, expressed in the local-space of the first rigid-body.
    #[must_use]
    pub fn local_anchor1(&self) -> Point<Real> {
        self.data.local_anchor1()
    }

    /// Sets the joint’s anchor, expressed in the local-space of the first rigid-body.
    pub fn set_local_anchor1(&mut self, anchor1: Point<Real>) -> &mut Self {
        self.data.set_local_anchor1(anchor1);
        self
    }

    /// The joint’s anchor, expressed in the local-space of the second rigid-body.
    #[must_use]
    pub fn local_anchor2(&self) -> Point<Real> {
        self.data.local_anchor2()
    }

    /// Sets the joint’s anchor, expressed in the local-space of the second rigid-body.
    pub fn set_local_anchor2(&mut self, anchor2: Point<Real>) -> &mut Self {
        self.data.set_local_anchor2(anchor2);
        self
    }

    /// The principal axis of the joint, expressed in the local-space of the first rigid-body.
    #[must_use]
    pub fn local_axis1(&self) -> UnitVector<Real> {
        self.data.local_axis1()
    }

    /// Sets the principal axis of the joint, expressed in the local-space of the first rigid-body.
    pub fn set_local_axis1(&mut self, axis1: UnitVector<Real>) -> &mut Self {
        self.data.set_local_axis1(axis1);
        self
    }

    /// The principal axis of the joint, expressed in the local-space of the second rigid-body.
    #[must_use]
    pub fn local_axis2(&self) -> UnitVector<Real> {
        self.data.local_axis2()
    }

    /// Sets the principal axis of the joint, expressed in the local-space of the second rigid-body.
    pub fn set_local_axis2(&mut self, axis2: UnitVector<Real>) -> &mut Self {
        self.data.set_local_axis2(axis2);
        self
    }

    /*
        from spring joint
    */

    /// Set the spring model used by this joint to reach the desired target velocity and position.
    ///
    /// Setting this to `MotorModel::ForceBased` (which is the default value for this joint) makes the spring constants
    /// (stiffness and damping) parameter understood as in the regular spring-mass-damper system. With
    /// `MotorModel::AccelerationBased`, the spring constants will be automatically scaled by the attached masses,
    /// making the spring more mass-independent.
    pub fn set_spring_model(&mut self, model: MotorModel) -> &mut Self {
        self.data.set_motor_model(JointAxis::LinX, model);
        self
    }

    /*
        from prismatic joint
    */

    /// The motor affecting the joint’s translational degree of freedom.
    #[must_use]
    pub fn motor(&self) -> Option<&JointMotor> {
        self.data.motor(JointAxis::LinX)
    }

    /// Set the spring-like model used by the motor to reach the desired target velocity and position.
    pub fn set_motor_model(&mut self, model: MotorModel) -> &mut Self {
        self.data.set_motor_model(JointAxis::LinX, model);
        self
    }

    /// Sets the target velocity this motor needs to reach.
    pub fn set_motor_velocity(&mut self, target_vel: Real, factor: Real) -> &mut Self {
        self.data
            .set_motor_velocity(JointAxis::LinX, target_vel, factor);
        self
    }

    /// Sets the target angle this motor needs to reach.
    pub fn set_motor_position(
        &mut self,
        target_pos: Real,
        stiffness: Real,
        damping: Real,
    ) -> &mut Self {
        self.data
            .set_motor_position(JointAxis::LinX, target_pos, stiffness, damping);
        self
    }

    /// Configure both the target angle and target velocity of the motor.
    pub fn set_motor(
        &mut self,
        target_pos: Real,
        target_vel: Real,
        stiffness: Real,
        damping: Real,
    ) -> &mut Self {
        self.data
            .set_motor(JointAxis::LinX, target_pos, target_vel, stiffness, damping);
        self
    }

    /// Sets the maximum force the motor can deliver.
    pub fn set_motor_max_force(&mut self, max_force: Real) -> &mut Self {
        self.data.set_motor_max_force(JointAxis::LinX, max_force);
        self
    }

    /// The limit distance attached bodies can translate along the joint’s principal axis.
    #[must_use]
    pub fn limits(&self) -> Option<&JointLimits<Real>> {
        self.data.limits(JointAxis::LinX)
    }

    /// Sets the `[min,max]` limit distances attached bodies can translate along the joint’s principal axis.
    pub fn set_limits(&mut self, limits: [Real; 2]) -> &mut Self {
        self.data.set_limits(JointAxis::LinX, limits);
        self
    }
}

impl From<PrismaticSpringJoint> for GenericJoint {
    fn from(val: PrismaticSpringJoint) -> GenericJoint {
        val.data
    }
}

/// Create prismatic spring joints using the builder pattern.
///
/// A prismatic spring joint locks all relative motion except for translations
/// along the joint’s principal axis, where motion is governed by a spring.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct PrismaticSpringJointBuilder(pub PrismaticSpringJoint);

impl PrismaticSpringJointBuilder {
    /// Creates a new builder for prismatic spring joints.
    ///
    /// This axis is expressed in the local-space of both rigid-bodies.
    pub fn new(direction: Vector<Real>, stiffness: Real, damping: Real) -> Self {
        Self(PrismaticSpringJoint::new(direction, stiffness, damping))
    }

    /// Sets whether contacts between the attached rigid-bodies are enabled.
    #[must_use]
    pub fn contacts_enabled(mut self, enabled: bool) -> Self {
        self.0.set_contacts_enabled(enabled);
        self
    }

    /// Sets the joint’s anchor, expressed in the local-space of the first rigid-body.
    #[must_use]
    pub fn local_anchor1(mut self, anchor1: Point<Real>) -> Self {
        self.0.set_local_anchor1(anchor1);
        self
    }

    /// Sets the joint’s anchor, expressed in the local-space of the second rigid-body.
    #[must_use]
    pub fn local_anchor2(mut self, anchor2: Point<Real>) -> Self {
        self.0.set_local_anchor2(anchor2);
        self
    }

    /// Sets the principal axis of the joint, expressed in the local-space of the first rigid-body.
    #[must_use]
    pub fn local_axis1(mut self, axis1: UnitVector<Real>) -> Self {
        self.0.set_local_axis1(axis1);
        self
    }

    /// Sets the principal axis of the joint, expressed in the local-space of the first rigid-body.
    pub fn set_local_axis1(&mut self, axis1: UnitVector<Real>) -> &mut Self {
        self.0.set_local_axis1(axis1);
        self
    }

    /// Sets the principal axis of the joint, expressed in the local-space of the second rigid-body.
    #[must_use]
    pub fn local_axis2(mut self, axis2: UnitVector<Real>) -> Self {
        self.0.set_local_axis2(axis2);
        self
    }

    /// Sets the principal axis of the joint, expressed in the local-space of the second rigid-body.
    pub fn set_local_axis2(&mut self, axis2: UnitVector<Real>) -> &mut Self {
        self.0.set_local_axis2(axis2);
        self
    }

    /// Set the spring-like model used by the motor to reach the desired target velocity and position.
    #[must_use]
    pub fn motor_model(mut self, model: MotorModel) -> Self {
        self.0.set_motor_model(model);
        self
    }

    /// Sets the target velocity this motor needs to reach.
    #[must_use]
    pub fn motor_velocity(mut self, target_vel: Real, factor: Real) -> Self {
        self.0.set_motor_velocity(target_vel, factor);
        self
    }

    /// Sets the target angle this motor needs to reach.
    #[must_use]
    pub fn motor_position(mut self, target_pos: Real, stiffness: Real, damping: Real) -> Self {
        self.0.set_motor_position(target_pos, stiffness, damping);
        self
    }

    /// Configure both the target angle and target velocity of the motor.
    #[must_use]
    pub fn set_motor(
        mut self,
        target_pos: Real,
        target_vel: Real,
        stiffness: Real,
        damping: Real,
    ) -> Self {
        self.0.set_motor(target_pos, target_vel, stiffness, damping);
        self
    }

    /// Sets the maximum force the motor can deliver.
    #[must_use]
    pub fn motor_max_force(mut self, max_force: Real) -> Self {
        self.0.set_motor_max_force(max_force);
        self
    }

    /// Sets the `[min,max]` limit distances attached bodies can translate along the joint’s principal axis.
    #[must_use]
    pub fn limits(mut self, limits: [Real; 2]) -> Self {
        self.0.set_limits(limits);
        self
    }

    /// Builds the prismatic joint.
    #[must_use]
    pub fn build(self) -> PrismaticSpringJoint {
        self.0
    }
}

impl From<PrismaticSpringJointBuilder> for GenericJoint {
    fn from(val: PrismaticSpringJointBuilder) -> GenericJoint {
        val.0.into()
    }
}

/// Trait for blob joint builders
pub trait BlobJointBuilder {
    /// Create generic joint from builder between two nodes
    fn build_blob_joint(&mut self, body1: &RigidBody, body2: &RigidBody) -> GenericJoint;
}

impl BlobJointBuilder for PrismaticSpringJointBuilder {
    fn build_blob_joint(&mut self, body1: &RigidBody, body2: &RigidBody) -> GenericJoint {
        let direction = Vector::from_homogeneous(
            (body2.position().translation.vector - body1.position().translation.vector)
                .to_homogeneous(),
        )
        .unwrap();
        let axis = UnitVector::new_normalize(direction);
        let mut motor = self.0.data.motor(JointAxis::LinX).unwrap().clone();
        motor.target_pos = direction.magnitude();
        self.0.data.set_motor(
            JointAxis::LinX,
            direction.magnitude(),
            motor.target_vel,
            motor.stiffness,
            motor.damping,
        );
        self.set_local_axis1(axis)
            .set_local_axis2(axis)
            .build()
            .into()
    }
}

impl BlobJointBuilder for SpringJointBuilder {
    fn build_blob_joint(&mut self, body1: &RigidBody, body2: &RigidBody) -> GenericJoint {
        let direction = Vector::from_homogeneous(
            (body2.position().translation.vector - body1.position().translation.vector)
                .to_homogeneous(),
        )
        .unwrap();
        let mut motor = self.0.data.motor(JointAxis::LinX).unwrap().clone();
        motor.target_pos = direction.magnitude();
        self.0.data.set_motor(
            JointAxis::LinX,
            direction.magnitude(),
            motor.target_vel,
            motor.stiffness,
            motor.damping,
        );

        self.build().into()
    }
}

/// Connect to blob nodes based on the joint builder
pub fn connect_nodes<T: BlobPhysics>(
    node1: Body,
    node2: Body,
    spec: &mut Box<dyn BlobJointBuilder>,
    phys: &mut T,
) -> Joint {
    let b1 = phys.get_body(&node1);
    let b2 = phys.get_body(&node2);
    let joint = spec.build_blob_joint(b1, b2);
    phys.add_joint(&node1, &node2, joint)
}
