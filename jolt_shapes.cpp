#include "gdjolt.h"
#include "jolt_util.h"

#include <Jolt/Physics/Collision/Shape/Shape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/CylinderShape.h>
#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>
#include <Jolt/Physics/Collision/Shape/HeightFieldShape.h>

using Jolt = JoltPhysicsServer3D;
using Base = GodotPhysicsServer3D;

/* SHAPE API */

#define JOLT_SHAPE_CREATE(name, Shape)         \
	RID Jolt::name##_shape_create() {          \
		RID rid = Base::name##_shape_create(); \
		Shape *shape = new Shape();            \
		own_shapes[rid] = shape;         	   \
		shape->SetUserData(rid.get_id());      \
		return rid;                            \
	}

// TODO: This may cause fragmentation. Jolt doesn't provide placement new...
// Implicitly unrefs (and, ideally, deletes) the previous shape.
#define JOLT_SHAPE_RECREATE(S, rid, ref, ...) \
	own_shapes[rid] = new S(__VA_ARGS__);

//JOLT_SHAPE_CREATE(world_boundary, ...)
//JOLT_SHAPE_CREATE(separation_ray, ...)
JOLT_SHAPE_CREATE(sphere,			JPH::SphereShape);
JOLT_SHAPE_CREATE(box,				JPH::BoxShape);
JOLT_SHAPE_CREATE(capsule,			JPH::CapsuleShape);
JOLT_SHAPE_CREATE(cylinder,			JPH::CylinderShape);
//JOLT_SHAPE_CREATE(convex_polygon,	JPH::ConvexHullShape);
//JOLT_SHAPE_CREATE(concave_polygon,	JPH::MeshShape);
//JOLT_SHAPE_CREATE(heightmap,		JPH::HeightFieldShape);
//JOLT_SHAPE_CREATE(custom, ...)

void Jolt::shape_set_data(RID p_shape, const Variant &p_data) {
	JPH::Shape* shape = get_shape(p_shape);
	if (!shape) {
		printf("[Jolt] Untracked Shape: %d\n", Base::shape_get_type(p_shape));
		return Base::shape_set_data(p_shape, p_data);
	}
	switch (shape->GetSubType())
	{
		// TODO: world boundary, separation ray
		case JPH::EShapeSubType::Sphere:
			JOLT_SHAPE_RECREATE(JPH::SphereShape, p_shape, shape, float(p_data));
			break;
		case JPH::EShapeSubType::Box:
			JOLT_SHAPE_RECREATE(JPH::BoxShape, p_shape, shape, ToJolt(Vector3(p_data)));
			break;
		case JPH::EShapeSubType::Capsule: {
			Dictionary d = p_data;
			ERR_FAIL_COND(!d.has("radius"));
			ERR_FAIL_COND(!d.has("height"));
			JOLT_SHAPE_RECREATE(JPH::CapsuleShape, p_shape, shape, float(d["height"]) * 0.5f, float(d["radius"]));
			break;
		}
		case JPH::EShapeSubType::Cylinder: {
			Dictionary d = p_data;
			ERR_FAIL_COND(!d.has("radius"));
			ERR_FAIL_COND(!d.has("height"));
			JOLT_SHAPE_RECREATE(JPH::CylinderShape, p_shape, shape, float(d["height"]) * 0.5f, float(d["radius"]));
			break;
		}
		case JPH::EShapeSubType::ConvexHull:
			ERR_PRINT("[Jolt] shape_set_data: ConvexHullShape not implemented");
			break;
		case JPH::EShapeSubType::Mesh:
			ERR_PRINT("[Jolt] shape_set_data: MeshShape not implemented");
			break;
		case JPH::EShapeSubType::HeightField:
			ERR_PRINT("[Jolt] shape_set_data: HeightFieldShape not implemented");
			break;
		default:
			ERR_PRINT("[Jolt] shape_set_data: Unrecognized shape type");
			break;
	}
	return Base::shape_set_data(p_shape, p_data);
}

PhysicsServer3D::ShapeType Jolt::shape_get_type(RID p_shape) const {
	const JPH::Shape* shape = get_shape(p_shape);
	if (!shape) {
		printf("[Jolt] Untracked Shape: %d\n", Base::shape_get_type(p_shape));
		return Base::shape_get_type(p_shape);
	}
	switch (shape->GetSubType())
	{
		// TODO: world boundary, separation ray
		case JPH::EShapeSubType::Sphere:		return PhysicsServer3D::SHAPE_SPHERE;
		case JPH::EShapeSubType::Box:			return PhysicsServer3D::SHAPE_BOX;
		case JPH::EShapeSubType::Capsule:		return PhysicsServer3D::SHAPE_CAPSULE;
		case JPH::EShapeSubType::Cylinder:		return PhysicsServer3D::SHAPE_CYLINDER;
		case JPH::EShapeSubType::ConvexHull:	return PhysicsServer3D::SHAPE_CONVEX_POLYGON;
		case JPH::EShapeSubType::Mesh:			return PhysicsServer3D::SHAPE_CONCAVE_POLYGON;
		case JPH::EShapeSubType::HeightField:	return PhysicsServer3D::SHAPE_HEIGHTMAP;
		default:								return PhysicsServer3D::SHAPE_CUSTOM;
	}
};

Variant Jolt::shape_get_data(RID p_shape) const {
	JPH::Shape* shape = get_shape(p_shape);
	if (!shape)
		return Base::shape_get_data(p_shape);
	switch (shape->GetSubType())
	{
		// TODO: world boundary, separation ray
		case JPH::EShapeSubType::Sphere:
			return static_cast<JPH::SphereShape*>(shape)->GetRadius();
		
		case JPH::EShapeSubType::Box:
			return to_godot(static_cast<JPH::BoxShape*>(shape)->GetHalfExtent());
		
		case JPH::EShapeSubType::Capsule: {
			Dictionary d;
			d["radius"] = static_cast<JPH::CapsuleShape*>(shape)->GetRadius();
			d["height"] = static_cast<JPH::CapsuleShape*>(shape)->GetHalfHeightOfCylinder() * 2.0f;
			return d;
		}
		case JPH::EShapeSubType::Cylinder: {
			Dictionary d;
			d["radius"] = static_cast<JPH::CylinderShape*>(shape)->GetRadius();
			d["height"] = static_cast<JPH::CylinderShape*>(shape)->GetHalfHeight() * 2.0f;
			return d;
		}
		case JPH::EShapeSubType::ConvexHull:
			break;
		case JPH::EShapeSubType::Mesh:
			break;
		case JPH::EShapeSubType::HeightField:
			break;
		default:
			break;
	}
	return Base::shape_get_data(p_shape);
}