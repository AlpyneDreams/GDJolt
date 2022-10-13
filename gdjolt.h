
#ifndef GDJOLT_H
#define GDJOLT_H

#include "core/templates/hash_map.h"
#include "servers/physics_3d/godot_physics_server_3d.h"

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>

class JoltPhysicsServer3D : public GodotPhysicsServer3D {
	GDCLASS(JoltPhysicsServer3D, GodotPhysicsServer3D);

	// Using HashMap because we can't assign
	// custom RIDs in RID_Alloc/RID_PtrOwner.
	mutable HashMap<RID, uint32_t>		own_bodies;
	mutable HashMap<RID, JPH::Shape*>	own_shapes;

public:
	JoltPhysicsServer3D();
	~JoltPhysicsServer3D();

	/* SHAPE API */

	//virtual RID world_boundary_shape_create() override;
	//virtual RID separation_ray_shape_create() override;
	virtual RID sphere_shape_create() override;
	virtual RID box_shape_create() override;
	virtual RID capsule_shape_create() override;
	virtual RID cylinder_shape_create() override;
	virtual RID convex_polygon_shape_create() override;
	virtual RID concave_polygon_shape_create() override;
	virtual RID heightmap_shape_create() override;
	//virtual RID custom_shape_create() override;

	virtual void shape_set_data(RID p_shape, const Variant &p_data) override;
	//virtual void shape_set_custom_solver_bias(RID p_shape, real_t p_bias) override;

	virtual ShapeType shape_get_type(RID p_shape) const override;
	virtual Variant shape_get_data(RID p_shape) const override;

	//virtual void shape_set_margin(RID p_shape, real_t p_margin) override;
	//virtual real_t shape_get_margin(RID p_shape) const override;

	//virtual real_t shape_get_custom_solver_bias(RID p_shape) const override;

	/* SPACE API */

	/* AREA API */

	/* BODY API */

	virtual RID body_create() override;

	//virtual void body_set_space(RID p_body, RID p_space) override;
	//virtual RID body_get_space(RID p_body) const override;

	virtual void body_set_mode(RID p_body, BodyMode p_mode) override;
	virtual BodyMode body_get_mode(RID p_body) const override;

	virtual void body_add_shape(RID p_body, RID p_shape, const Transform3D &p_transform = Transform3D(), bool p_disabled = false) override;
	virtual void body_set_shape(RID p_body, int p_shape_idx, RID p_shape) override;
	//virtual void body_set_shape_transform(RID p_body, int p_shape_idx, const Transform3D &p_transform) override;

	virtual int body_get_shape_count(RID p_body) const override;
	virtual RID body_get_shape(RID p_body, int p_shape_idx) const override;
	//virtual Transform3D body_get_shape_transform(RID p_body, int p_shape_idx) const override;

	//virtual void body_set_shape_disabled(RID p_body, int p_shape_idx, bool p_disabled) override;

	virtual void body_remove_shape(RID p_body, int p_shape_idx) override;
	virtual void body_clear_shapes(RID p_body) override;

/*
	virtual void body_attach_object_instance_id(RID p_body, ObjectID p_id) override;
	virtual ObjectID body_get_object_instance_id(RID p_body) const override;

	virtual void body_set_enable_continuous_collision_detection(RID p_body, bool p_enable) override;
	virtual bool body_is_continuous_collision_detection_enabled(RID p_body) const override;

	virtual void body_set_collision_layer(RID p_body, uint32_t p_layer) override;
	virtual uint32_t body_get_collision_layer(RID p_body) const override;

	virtual void body_set_collision_mask(RID p_body, uint32_t p_mask) override;
	virtual uint32_t body_get_collision_mask(RID p_body) const override;

	virtual void body_set_collision_priority(RID p_body, real_t p_priority) override;
	virtual real_t body_get_collision_priority(RID p_body) const override;

	virtual void body_set_user_flags(RID p_body, uint32_t p_flags) override;
	virtual uint32_t body_get_user_flags(RID p_body) const override;

	virtual void body_set_param(RID p_body, BodyParameter p_param, const Variant &p_value) override;
	virtual Variant body_get_param(RID p_body, BodyParameter p_param) const override;

	virtual void body_reset_mass_properties(RID p_body) override;

	virtual void body_set_state(RID p_body, BodyState p_state, const Variant &p_variant) override;
	virtual Variant body_get_state(RID p_body, BodyState p_state) const override;

	virtual void body_apply_central_impulse(RID p_body, const Vector3 &p_impulse) override;
	virtual void body_apply_impulse(RID p_body, const Vector3 &p_impulse, const Vector3 &p_position = Vector3()) override;
	virtual void body_apply_torque_impulse(RID p_body, const Vector3 &p_impulse) override;

	virtual void body_apply_central_force(RID p_body, const Vector3 &p_force) override;
	virtual void body_apply_force(RID p_body, const Vector3 &p_force, const Vector3 &p_position = Vector3()) override;
	virtual void body_apply_torque(RID p_body, const Vector3 &p_torque) override;

	virtual void body_add_constant_central_force(RID p_body, const Vector3 &p_force) override;
	virtual void body_add_constant_force(RID p_body, const Vector3 &p_force, const Vector3 &p_position = Vector3()) override;
	virtual void body_add_constant_torque(RID p_body, const Vector3 &p_torque) override;

	virtual void body_set_constant_force(RID p_body, const Vector3 &p_force) override;
	virtual Vector3 body_get_constant_force(RID p_body) const override;

	virtual void body_set_constant_torque(RID p_body, const Vector3 &p_torque) override;
	virtual Vector3 body_get_constant_torque(RID p_body) const override;

	virtual void body_set_axis_velocity(RID p_body, const Vector3 &p_axis_velocity) override;

	virtual void body_set_axis_lock(RID p_body, BodyAxis p_axis, bool p_lock) override;
	virtual bool body_is_axis_locked(RID p_body, BodyAxis p_axis) const override;

	virtual void body_add_collision_exception(RID p_body, RID p_body_b) override;
	virtual void body_remove_collision_exception(RID p_body, RID p_body_b) override;
	virtual void body_get_collision_exceptions(RID p_body, List<RID> *p_exceptions) override;

	virtual void body_set_contacts_reported_depth_threshold(RID p_body, real_t p_threshold) override;
	virtual real_t body_get_contacts_reported_depth_threshold(RID p_body) const override;

	virtual void body_set_omit_force_integration(RID p_body, bool p_omit) override;
	virtual bool body_is_omitting_force_integration(RID p_body) const override;

	virtual void body_set_max_contacts_reported(RID p_body, int p_contacts) override;
	virtual int body_get_max_contacts_reported(RID p_body) const override;

	virtual void body_set_state_sync_callback(RID p_body, const Callable &p_callable) override;
	virtual void body_set_force_integration_callback(RID p_body, const Callable &p_callable, const Variant &p_udata = Variant()) override;

	virtual void body_set_ray_pickable(RID p_body, bool p_enable) override;

	virtual bool body_test_motion(RID p_body, const MotionParameters &p_parameters, MotionResult *r_result = nullptr) override;

	// this function only works on physics process, errors and returns null otherwise
	virtual PhysicsDirectBodyState3D *body_get_direct_state(RID p_body) override;
*/

	/* SOFT BODY */

	/* JOINT API */

	/* MISC */

	virtual void free(RID p_rid) override;

	//virtual void set_active(bool p_active) override;
	virtual void init() override;
	virtual void step(real_t p_step) override;
	//virtual void sync() override;
	//virtual void flush_queries() override;
	//virtual void end_sync() override;
	virtual void finish() override;

private:
	JPH::PhysicsSystem	Physics;
	JPH::JobSystem*		Jobs;
	JPH::TempAllocator*	TempAllocator;

	// Internals

	const JPH::BodyID get_body_id(RID p_rid) const {
		return own_bodies.has(p_rid) ? JPH::BodyID(own_bodies[p_rid]) : JPH::BodyID();
	}

	JPH::Shape* get_shape(RID p_rid) const {
		return own_shapes.has(p_rid) ? own_shapes[p_rid] : nullptr;
	}

	// TODO: Use GetBodyInterfaceNoLock where possible.
	JPH::BodyInterface& Bodies() { return Physics.GetBodyInterface(); }
	const JPH::BodyInterface& Bodies() const { return Physics.GetBodyInterface(); }
};

#endif // GDJOLT_H
