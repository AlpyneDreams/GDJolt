#include "gdjolt.h"
#include "jolt_util.h"
#include "jolt_layers.h"

#include <Jolt/Physics/Body/BodyCreationSettings.h>

#include <Jolt/Physics/Collision/Shape/MutableCompoundShape.h>

// TEMP
#include <Jolt/Physics/Collision/Shape/SphereShape.h>

using Jolt = JoltPhysicsServer3D;
using Base = GodotPhysicsServer3D;

/* BODY API */

RID Jolt::body_create() {
	using namespace JPH;
	printf("[Jolt] Body created!\n");
	
	RID rid = Base::body_create();

	// TODO: Should start out with no shapes! But JPH requires at least one shape.
	// Placeholder shape
	SphereShapeSettings shape = SphereShapeSettings(1.0f);
	
	// Create body
	BodyCreationSettings settings(shape.Create().Get(), Vec3(0.0f, 0.0f, 0.0f), Quat::sIdentity(), EMotionType::Dynamic, Layers::MOVING);
	
	Body* body = Bodies().CreateBody(settings);

	// Assign to RID
	own_bodies.insert(rid, body->GetID().GetIndexAndSequenceNumber());
	
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

	using namespace JPH;
	BodyID body = get_body_id(p_body);
	ERR_FAIL_COND(body.IsInvalid());
	
	// Get shape
	Shape* shape = get_shape(p_shape);
	ERR_FAIL_COND(!shape);

	// If body has no shapes yet...
	if (!Bodies().IsAdded(body)) {
		Bodies().SetShape(body, shape, true, EActivation::DontActivate);
		Bodies().AddBody(body, EActivation::Activate);
		// TODO: TEMP
		Bodies().SetLinearVelocity(body, Vec3(0.0f, -5.0f, 0.0f));
		return;
	}

	// Get current shape...
	RefConst<Shape> oldShape = Bodies().GetShape(body);
	if (oldShape->GetType() != EShapeType::Compound) {
		// TODO: Create compound shape...
		ERR_PRINT("[Jolt] Compound shapes not yet implemented.");
	} else {
		const CompoundShape* shapes = static_cast<const CompoundShape*>(oldShape.GetPtr());

		// TODO: Add shape to compound...
		ERR_PRINT("[Jolt] Compound shapes not yet implemented.");
	}
}

void Jolt::body_set_shape(RID p_body, int idx, RID p_shape) {
	printf("[Jolt] body_set_shape\n");

	Base::body_set_shape(p_body, idx, p_shape);

	using namespace JPH;
	BodyID body = get_body_id(p_body);
	ERR_FAIL_COND(body.IsInvalid());
	
	// Get shape
	Shape* shape = get_shape(p_shape);
	ERR_FAIL_COND(!shape);

	// If body has no shapes yet...
	if (!Bodies().IsAdded(body)) {
		ERR_PRINT("[Jolt] body_set_shape: Body has no shapes yet.");
		return;
	}

	// Get current shape...
	RefConst<Shape> oldShape = Bodies().GetShape(body);
	if (oldShape->GetType() != EShapeType::Compound) {
		if (idx == 0) {
			// Setting the one main shape
			Bodies().SetShape(body, shape, true, EActivation::Activate);
		} else {
			// TODO: Create compound shape...
			ERR_PRINT("[Jolt] Compound shapes not yet implemented.");
		}
	} else {
		const CompoundShape* shapes = static_cast<const CompoundShape*>(oldShape.GetPtr());
		
		if (idx < 0 || uint(idx) >= shapes->GetNumSubShapes()) {
			ERR_PRINT("[Jolt] body_set_shape: Invalid shape index");
			return;
		}

		// TODO: Add shape to compound...
		ERR_PRINT("[Jolt] Compound shapes not yet implemented.");
	}
}

int Jolt::body_get_shape_count(RID p_body) const {
	using namespace JPH;
	BodyID body = get_body_id(p_body);
	ERR_FAIL_COND_V(body.IsInvalid(), Base::body_get_shape_count(p_body));
	
	// If body has no shapes yet...
	if (!Bodies().IsAdded(body)) {
		return 0;
	}

	RefConst<Shape> shape = Bodies().GetShape(body);

	// Body is either compound or has one shape
	if (shape->GetType() == EShapeType::Compound)
		return static_cast<const CompoundShape*>(shape.GetPtr())->GetNumSubShapes();
	else
		return 1;
}

RID Jolt::body_get_shape(RID p_body, int idx) const {
	using namespace JPH;
	BodyID body = get_body_id(p_body);
	ERR_FAIL_COND_V(body.IsInvalid(), Base::body_get_shape(p_body, idx));

	RefConst<Shape> shape = Bodies().GetShape(body);

	if (shape->GetType() == EShapeType::Compound) {
		const CompoundShape* shapes = static_cast<const CompoundShape*>(shape.GetPtr());
		
		if (idx < 0 || uint(idx) >= shapes->GetNumSubShapes())
			return RID();
		
		return RID::from_uint64(
			shapes->GetSubShape(idx).mShape->GetUserData()
		);
	} else if (idx == 0)
		return RID::from_uint64(shape->GetUserData());
	else
		return RID();
}