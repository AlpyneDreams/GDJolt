
#ifndef GDJOLT_H
#define GDJOLT_H

#include "core/templates/rid_owner.h"
#include "servers/physics_3d/godot_physics_server_3d.h"

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>
#include <Jolt/Physics/PhysicsSystem.h>

class JoltPhysicsServer3D : public GodotPhysicsServer3D {
	GDCLASS(JoltPhysicsServer3D, GodotPhysicsServer3D);

	mutable RID_Alloc<uint32_t, true>  own_bodies;
	mutable RID_PtrOwner<JPH::Shape, true> own_shapes;

public:
	JoltPhysicsServer3D();
	~JoltPhysicsServer3D();

	/* SHAPE API */

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
		uint32_t* ptr = own_bodies.get_or_null(p_rid);
		return ptr == nullptr ? JPH::BodyID() : JPH::BodyID(*ptr);
	}

	JPH::BodyInterface& Bodies() { return Physics.GetBodyInterfaceNoLock(); }
	const JPH::BodyInterface& Bodies() const { return Physics.GetBodyInterfaceNoLock(); }
	JPH::BodyInterface& BodiesLock() { return Physics.GetBodyInterface(); }
	const JPH::BodyInterface& BodiesLock() const { return Physics.GetBodyInterface(); }
};

#endif // GDJOLT_H
