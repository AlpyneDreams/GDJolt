
#ifndef JOLT_BODY_DIRECT_STATE_H
#define JOLT_BODY_DIRECT_STATE_H

#include "gdjolt.h"
#include "jolt_util.h"

#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/Body/MotionProperties.h>

class JoltDirectBodyState : public PhysicsDirectBodyState3D {
	GDCLASS(JoltDirectBodyState, PhysicsDirectBodyState3D);

public:
	JPH::Body* body;
	JPH::MotionProperties* motion;

	JoltDirectBodyState(JPH::Body& body) {
		this->body = &body;
		this->motion = body.GetMotionProperties();
	}

	virtual Vector3 get_total_gravity() const override {return Vector3(); } // TODO
	virtual real_t get_total_angular_damp() const override { return 0.0f; } // TODO
	virtual real_t get_total_linear_damp() const override { return 0.0f; } // TODO

	virtual Vector3 get_center_of_mass() const override {
        return to_godot(body->GetCenterOfMassPosition());
    }
	virtual Vector3 get_center_of_mass_local() const override {
        // TODO GetCenterOfMassTransform
        return to_godot(body->GetCenterOfMassPosition());
    }
	virtual Basis get_principal_inertia_axes() const override { return Basis(); } // TODO

	virtual real_t get_inverse_mass() const override {
        return motion->GetInverseMass();
    }
	virtual Vector3 get_inverse_inertia() const override { return Vector3(); } // TODO: GetInverseInertia
	virtual Basis get_inverse_inertia_tensor() const override { return Basis(); } // TODO: GetInverseInertia

	virtual void set_linear_velocity(const Vector3 &p_velocity) override {
        body->SetLinearVelocity(ToJolt(p_velocity));
    }
	virtual Vector3 get_linear_velocity() const override {
        return to_godot(body->GetLinearVelocity());
    }

	virtual void set_angular_velocity(const Vector3 &p_velocity) override {
        body->SetAngularVelocity(ToJolt(p_velocity));
    }
	virtual Vector3 get_angular_velocity() const override {
        return to_godot(body->GetAngularVelocity());
    }

	virtual void set_transform(const Transform3D &p_transform) override {} // TODO
	virtual Transform3D get_transform() const override {
		return Transform3D(
				to_godot(body->GetRotation()),
				to_godot(body->GetPosition()));
	}

	virtual Vector3 get_velocity_at_local_position(const Vector3 &p_position) const override { return Vector3(); } // TODO

	virtual void apply_central_impulse(const Vector3 &p_impulse) override {
		body->AddImpulse(ToJolt(p_impulse));
	}
	virtual void apply_impulse(const Vector3 &p_impulse, const Vector3 &p_position = Vector3()) override {
        body->AddImpulse(ToJolt(p_impulse), ToJolt(p_position));
	}
	virtual void apply_torque_impulse(const Vector3 &p_impulse) override {
        // TODO: Not AddTorque?
        body->AddAngularImpulse(ToJolt(p_impulse));
    }

	virtual void apply_central_force(const Vector3 &p_force) override {
        body->AddForce(ToJolt(p_force));
    }
	virtual void apply_force(const Vector3 &p_force, const Vector3 &p_position = Vector3()) override {
        body->AddForce(ToJolt(p_force), ToJolt(p_position));
    }
	virtual void apply_torque(const Vector3 &p_torque) override {
        // TODO: Not AddAngularImpulse?
        body->AddTorque(ToJolt(p_torque));
    }

	virtual void add_constant_central_force(const Vector3 &p_force) override {} // TODO
	virtual void add_constant_force(const Vector3 &p_force, const Vector3 &p_position = Vector3()) override {} // TODO
	virtual void add_constant_torque(const Vector3 &p_torque) override {} // TODO

	virtual void set_constant_force(const Vector3 &p_force) override {} // TODO
	virtual Vector3 get_constant_force() const override { return Vector3(); } // TODO

	virtual void set_constant_torque(const Vector3 &p_torque) override {} // TODO
	virtual Vector3 get_constant_torque() const override { return Vector3(); } // TODO

	virtual void set_sleep_state(bool p_sleep) override {} // TODO
	virtual bool is_sleeping() const override {
        return !body->IsActive();
    }

	virtual int get_contact_count() const override { return 0; } // TODO

	virtual Vector3 get_contact_local_position(int p_contact_idx) const override { return Vector3(); } // TODO
	virtual Vector3 get_contact_local_normal(int p_contact_idx) const override { return Vector3(); } // TODO
	virtual real_t get_contact_impulse(int p_contact_idx) const override { return 0.0f; } // TODO
	virtual int get_contact_local_shape(int p_contact_idx) const override { return 0; } // TODO

	virtual RID get_contact_collider(int p_contact_idx) const override { return RID(); } // TODO
	virtual Vector3 get_contact_collider_position(int p_contact_idx) const override { return Vector3(); } // TODO
	virtual ObjectID get_contact_collider_id(int p_contact_idx) const override { return ObjectID(); } // TODO
	virtual int get_contact_collider_shape(int p_contact_idx) const override { return 0; } // TODO
	virtual Vector3 get_contact_collider_velocity_at_position(int p_contact_idx) const override { return Vector3(); } // TODO

	virtual PhysicsDirectSpaceState3D *get_space_state() override { return nullptr; } // TODO

	virtual real_t get_step() const override { return 0.0f; } // TODO
};

#endif
