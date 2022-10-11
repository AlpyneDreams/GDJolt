
#ifndef GDJOLT_H
#define GDJOLT_H

#include "servers/physics_3d/godot_physics_server_3d.h"



class JoltPhysicsServer3D : public GodotPhysicsServer3D {
    GDCLASS(JoltPhysicsServer3D, GodotPhysicsServer3D);

    virtual RID body_create() override;

};

#endif // GDJOLT_H