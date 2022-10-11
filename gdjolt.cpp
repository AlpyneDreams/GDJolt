#include "gdjolt.h"

using Jolt = JoltPhysicsServer3D;
using Base = GodotPhysicsServer3D;

#include <Jolt/Physics/Body/BodyCreationSettings.h>

RID Jolt::body_create() {
    using namespace JPH;
    //Bodies().CreateBody(...);
    printf("Body created!\n");

    return Base::body_create();
}

void Jolt::free(RID p_rid) {
    printf("Freeing RID %lu\n", p_rid.get_id());

    // Check each owner
    if (own_bodies.owns(p_rid)) {
        Physics.GetBodyInterfaceNoLock().DestroyBody(get_body_id(p_rid));
        own_bodies.free(p_rid);
    }

    return Base::free(p_rid);
}
