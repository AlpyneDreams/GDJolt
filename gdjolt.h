
#ifndef GDJOLT_H
#define GDJOLT_H

#include "servers/physics_3d/godot_physics_server_3d.h"

#include <Jolt/Jolt.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/Body/BodyInterface.h>

class JoltPhysicsServer3D : public GodotPhysicsServer3D {
    GDCLASS(JoltPhysicsServer3D, GodotPhysicsServer3D);

    mutable RID_PtrOwner<JPH::Body, true> own_bodies;

public:

	/* SHAPE API */

	/* SPACE API */

	/* AREA API */

	/* BODY API */
    
    virtual RID body_create() override;

	/* SOFT BODY */

	/* JOINT API */

	/* MISC */

	virtual void free(RID p_rid) override;

private:
    JPH::PhysicsSystem Physics;

    // Internals

    const JPH::BodyID& get_body_id(RID p_rid) const {
        return own_bodies.get_or_null(p_rid)->GetID();
    }

    const JPH::BodyInterface& Bodies() {
        return Physics.GetBodyInterfaceNoLock();
    }

};

#endif // GDJOLT_H
