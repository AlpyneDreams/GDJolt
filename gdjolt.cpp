#include "gdjolt.h"

using Base = GodotPhysicsServer3D;

RID JoltPhysicsServer3D::body_create() {
    printf("Body created!\n");
    return Base::body_create();
}
