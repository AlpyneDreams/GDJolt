
#ifndef GDJOLT_H
#define GDJOLT_H

#include "core/templates/hash_map.h"
#include "servers/physics_3d/godot_physics_server_3d.h"

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>
#include <Jolt/Physics/PhysicsSystem.h>

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

	JPH::BodyInterface& Bodies() { return Physics.GetBodyInterfaceNoLock(); }
	const JPH::BodyInterface& Bodies() const { return Physics.GetBodyInterfaceNoLock(); }
	JPH::BodyInterface& BodiesLock() { return Physics.GetBodyInterface(); }
	const JPH::BodyInterface& BodiesLock() const { return Physics.GetBodyInterface(); }
};

#endif // GDJOLT_H
