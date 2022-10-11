#include "register_types.h"

#include "core/object/callable_method_pointer.h"
#include "core/object/class_db.h"
#include "core/string/print_string.h"
#include "gdjolt.h"


static PhysicsServer3D *_createJoltPhysics3DCallback() {
	PhysicsServer3D *physics_server_3d = memnew(JoltPhysicsServer3D());
	return physics_server_3d;
	//return memnew(PhysicsServer3DWrapMT(physics_server_3d, using_threads));
}


void initialize_jolt_module(ModuleInitializationLevel p_level) {
	if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
		return;
	}

	ClassDB::register_class<JoltPhysicsServer3D>();
	PhysicsServer3DManager::get_singleton()->register_server("Jolt", callable_mp_static(_createJoltPhysics3DCallback));
}

void uninitialize_jolt_module(ModuleInitializationLevel p_level) {
	if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
		return;
	}
   // Nothing to do here.
}
