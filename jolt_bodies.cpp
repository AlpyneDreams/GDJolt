#include "gdjolt.h"
#include "jolt_util.h"
#include "jolt_layers.h"

#include <Jolt/Physics/Body/BodyCreationSettings.h>

#include <Jolt/Physics/Collision/Shape/MutableCompoundShape.h>

// TEMP
#include <Jolt/Physics/Collision/Shape/SphereShape.h>

using Jolt 		= JoltPhysicsServer3D;
using Base		= GodotPhysicsServer3D;
using Shapes	= JPH::MutableCompoundShape;

using namespace JPH;

// Jolt does not like CompoundShapes with zero shapes, so we add
// a dummy shape and set this userdata value to indicate a shapeless body.
#define NULL_SHAPE UINT32_MAX

// Locks a body to mutate its shapes.
struct ShapesLock {
	const BodyInterface& bodies;
	const BodyID id;
	BodyLockWrite lock;
	const bool succeeded;
	Body* body;
	Shapes* shapes;
	Vec3 oldCenter;

	ShapesLock(PhysicsSystem& physics, BodyID& id)
		: bodies(physics.GetBodyInterfaceNoLock())
		, id(id)
		, lock(physics.GetBodyLockInterface(), id)
		, succeeded(lock.Succeeded())
		, body(succeeded ? &lock.GetBody() : nullptr) {
		if (succeeded) {
			// This is what MutableCompoundShapeTest does.
			shapes = static_cast<Shapes *>(const_cast<Shape *>(body->GetShape()));
			oldCenter = shapes->GetCenterOfMass();
			
		}
	}

	operator bool () const { return succeeded; }
	Shapes* operator -> () const { return shapes; }
	Shapes* operator * () const { return shapes; }

	~ShapesLock() {
			
		if (succeeded) {

			// Last shape removed, add a null shape
			if (shapes->GetNumSubShapes() == 0) {
				printf("[Jolt] Removed all shapes from body %u, adding null shape.\n", id.GetIndex());
				shapes->AddShape(Vec3::sZero(), Quat::sIdentity(), new SphereShape(1.0f), NULL_SHAPE);
				shapes->SetUserData(NULL_SHAPE);
			}

			shapes->AdjustCenterOfMass();
			bodies.NotifyShapeChanged(id, oldCenter, true, EActivation::Activate);
		}
	}
};

/* BODY API */

RID Jolt::body_create() {
	using namespace JPH;
	printf("[Jolt] Body created!\n");
	
	RID rid = Base::body_create();

	MutableCompoundShapeSettings* shapes = new MutableCompoundShapeSettings;

	// TODO: Should start with no shapes! But JPH requires at least one shape.
	shapes->AddShape(Vec3::sZero(), Quat::sIdentity(), new SphereShape(1.0f), NULL_SHAPE);
	shapes->mUserData = NULL_SHAPE;
	
	// Create body
	BodyCreationSettings settings(
		shapes,
		Vec3(0.0f, 0.0f, 0.0f), Quat::sIdentity(),
		EMotionType::Dynamic, Layers::MOVING
	);
	
	// Create body and add it to the world
	BodyID body = Bodies().CreateAndAddBody(settings, EActivation::Activate); // TODO: Don't activate


	// Assign to RID
	own_bodies.insert(rid, BodyData { body.GetIndexAndSequenceNumber() });
	
	return rid;
}

using BodyMode = Base::BodyMode;

void Jolt::body_set_mode(RID p_body, BodyMode mode) {
	JPH::BodyID body = get_body_id(p_body);
	ERR_FAIL_COND(body.IsInvalid());

	Base::body_set_mode(p_body, mode);

	JPH::EActivation activation = JPH::EActivation::DontActivate;
	
	switch (mode)
	{
		case PhysicsServer3D::BODY_MODE_STATIC:
			Bodies().SetMotionType(body, JPH::EMotionType::Static, activation);
			break;
		case PhysicsServer3D::BODY_MODE_KINEMATIC:
			Bodies().SetMotionType(body, JPH::EMotionType::Kinematic, activation);
			break;
		case PhysicsServer3D::BODY_MODE_RIGID:
			Bodies().SetMotionType(body, JPH::EMotionType::Dynamic, activation);
			break;
		case PhysicsServer3D::BODY_MODE_RIGID_LINEAR:
			ERR_PRINT("[Jolt] body_set_mode: Rigid linear mode not implemented");
			Bodies().SetMotionType(body, JPH::EMotionType::Dynamic, activation);
			break;
	}
}

BodyMode Jolt::body_get_mode(RID p_body) const {
	JPH::BodyID body = get_body_id(p_body);
	ERR_FAIL_COND_V(body.IsInvalid(), Base::body_get_mode(p_body));
	return BodyMode(uint8_t(Bodies().GetMotionType(body)));
}

void Jolt::body_add_shape(RID p_body, RID p_shape, const Transform3D &p_transform, bool p_disabled) {
	printf("[Jolt] body_add_shape\n");

	Base::body_add_shape(p_body, p_shape, p_transform, p_disabled);

	BodyID id = get_body_id(p_body);
	ERR_FAIL_COND(id.IsInvalid());
	
	// Get shape
	Shape* shape = get_shape(p_shape);
	ERR_FAIL_COND(!shape);

	if (ShapesLock shapes = ShapesLock(Physics, id)) {
		Vec3 pos = ToJolt(p_transform.origin);
		Quat rot = Quat::sIdentity(); // TODO

		if (shapes->GetUserData() == NULL_SHAPE) {
			// First shape...
			shapes->RemoveShape(0);
			shapes->SetUserData(0);
		}
		shapes->AddShape(pos, rot, shape, p_shape.get_id());
	}
}

void Jolt::body_set_shape(RID p_body, int idx, RID p_shape) {
	printf("[Jolt] body_set_shape\n");

	Base::body_set_shape(p_body, idx, p_shape);

	BodyID id = get_body_id(p_body);
	ERR_FAIL_COND(id.IsInvalid());
	
	// Get shape
	Shape* shape = get_shape(p_shape);
	ERR_FAIL_COND(!shape);

	if (ShapesLock shapes = ShapesLock(Physics, id)) {
		Vec3 pos = Vec3(); // TODO: ?
		Quat rot = Quat::sIdentity(); // TODO: ?

		if (idx < 0 || uint(idx) >= shapes->GetNumSubShapes()) {
			ERR_FAIL_MSG("[Jolt] body_set_shape: Invalid index");
		}

		shapes->ModifyShape(idx, pos, rot, shape);
	}
}

void Jolt::body_set_shape_transform(RID p_body, int idx, const Transform3D &p_transform) {
	Base::body_set_shape_transform(p_body, idx, p_transform);

	BodyID id = get_body_id(p_body);
	ERR_FAIL_COND(id.IsInvalid());

	if (ShapesLock shapes = ShapesLock(Physics, id)) {
		Vec3 pos = ToJolt(p_transform.origin);
		Quat rot = ToJolt(p_transform.basis);

		if (idx < 0 || uint(idx) >= shapes->GetNumSubShapes()) {
			ERR_FAIL_MSG("[Jolt] body_set_shape_transform: Invalid index");
		}

		shapes->ModifyShape(idx, pos, rot);
	}

}

int Jolt::body_get_shape_count(RID p_body) const {
	BodyID body = get_body_id(p_body);
	ERR_FAIL_COND_V(body.IsInvalid(), Base::body_get_shape_count(p_body));
	
	const Shapes* shapes = get_body_shapes(body);
	return shapes->GetUserData() == NULL_SHAPE ? 0 : shapes->GetNumSubShapes();
}

RID Jolt::body_get_shape(RID p_body, int idx) const {
	BodyID body = get_body_id(p_body);
	ERR_FAIL_COND_V(body.IsInvalid(), Base::body_get_shape(p_body, idx));

	const Shapes* shapes = get_body_shapes(body);

	if (idx < 0 || uint(idx) >= shapes->GetNumSubShapes()) {
		ERR_FAIL_V_MSG(RID(), "[Jolt] body_get_shape: Invalid index");
	}

	return RID::from_uint64(shapes->GetSubShape(idx).mUserData);
}

void Jolt::body_remove_shape(RID p_body, int idx) {
	printf("[Jolt] body_remove_shape\n");

	Base::body_remove_shape(p_body, idx);

	BodyID body = get_body_id(p_body);
	ERR_FAIL_COND(body.IsInvalid());

	if (ShapesLock shapes = ShapesLock(Physics, body)) {
		uint numShapes = shapes->GetNumSubShapes();

		ERR_FAIL_COND_MSG(shapes->GetUserData() == NULL_SHAPE, "[Jolt] body_remove_shape: No shapes to remove");
		ERR_FAIL_COND_MSG(idx < 0 || uint(idx) >= numShapes, "[Jolt] body_remove_shape: Invalid index");
		
		shapes->RemoveShape(idx);
	}
}

void Jolt::body_clear_shapes(RID p_body) {
	printf("[Jolt] body_clear_shapes\n");
	
	Base::body_clear_shapes(p_body);

	JPH::BodyID body = get_body_id(p_body);
	ERR_FAIL_COND(body.IsInvalid());

	if (ShapesLock shapes = ShapesLock(Physics, body)) {
		uint numShapes = shapes->GetNumSubShapes();

		for (uint i = 0; i < numShapes; i++)
			shapes->RemoveShape(i);	
	}
}

void Jolt::body_set_state(RID p_body, BodyState p_state, const Variant &p_variant) {
	
	Base::body_set_state(p_body, p_state, p_variant);

	JPH::BodyID body = get_body_id(p_body);
	ERR_FAIL_COND(body.IsInvalid());

	switch (p_state) {
		case BODY_STATE_TRANSFORM: {
			Transform3D t = p_variant;
			Bodies().SetPositionAndRotation(body, ToJolt(t.origin), ToJolt(t.basis), EActivation::DontActivate);
		} break;
		case BODY_STATE_LINEAR_VELOCITY: {
			Bodies().SetLinearVelocity(body, ToJolt(Vector3(p_variant)));
		} break;
		case BODY_STATE_ANGULAR_VELOCITY: {
			Bodies().SetAngularVelocity(body, ToJolt(Vector3(p_variant)));
		} break;
		case BODY_STATE_SLEEPING: {
			bool sleeping = p_variant;
			if (sleeping) {
				Bodies().DeactivateBody(body);
			} else {
				Bodies().ActivateBody(body);
			}
		} break;
		case BODY_STATE_CAN_SLEEP: {
			bool can_sleep = p_variant;
			BodyLockWrite lock(Physics.GetBodyLockInterface(), body);
			lock.GetBody().SetAllowSleeping(can_sleep);
		} break;
		default: {
			Base::body_set_state(p_body, p_state, p_variant);
		} break;
	}
}

Variant Jolt::body_get_state(RID p_body, BodyState p_state) const {
	JPH::BodyID body = get_body_id(p_body);
	if (body.IsInvalid())
		return Base::body_get_state(p_body, p_state);

	switch (p_state) {
		case BODY_STATE_TRANSFORM:
			JPH::Vec3 pos;
			JPH::Quat rot;
			Bodies().GetPositionAndRotation(body, pos, rot);
			return Transform3D(to_godot(rot), to_godot(pos));
		case BODY_STATE_LINEAR_VELOCITY:
			return to_godot(Bodies().GetLinearVelocity(body));
		case BODY_STATE_ANGULAR_VELOCITY:
			return to_godot(Bodies().GetAngularVelocity(body));
		case BODY_STATE_SLEEPING:
			return !Bodies().IsActive(body);
		case BODY_STATE_CAN_SLEEP: {
			BodyLockRead lock(Physics.GetBodyLockInterface(), body);
			return lock.GetBody().GetAllowSleeping();
		}
		default:
			return Base::body_get_state(p_body, p_state);
	}
}

void Jolt::body_set_state_sync_callback(RID p_body, const Callable &p_callable) {
	printf("[Jolt] body_set_state_sync_callback\n");

	if (!own_bodies.has(p_body))
		Base::body_set_state_sync_callback(p_body, p_callable);
	
	BodyData& data = own_bodies[p_body];
	data.state_callback = p_callable;
}
