#include "gdjolt.h"

using Jolt = JoltPhysicsServer3D;
using Base = GodotPhysicsServer3D;

#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/RegisterTypes.h>

#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>

#include "jolt_util.h"
#include "jolt_layers.h"

// Callback for traces, connect this to your own trace function if you have one
static void TraceImpl(const char *inFMT, ...)
{ 
	// Format the message
	va_list list;
	va_start(list, inFMT);
	vprintf(inFMT, list);
}

#ifdef JPH_ENABLE_ASSERTS

// Callback for asserts, connect this to your own assert handler if you have one
static bool AssertFailedImpl(const char *inExpression, const char *inMessage, const char *inFile, uint inLine)
{ 
	// Print to the TTY
	printf("[Jolt] Assert Failed: %s:%u: (%s) %s\n", inFile, inLine, inExpression, (inMessage != nullptr ? inMessage : ""));

	// Breakpoint
	return true;
};

#endif // JPH_ENABLE_ASSERTS


Jolt::JoltPhysicsServer3D() : Base(false) {
	// Register allocation hook
	JPH::RegisterDefaultAllocator();

	// Install callbacks
	JPH::Trace = TraceImpl;
	JPH_IF_ENABLE_ASSERTS(JPH::AssertFailed = AssertFailedImpl;)

	// Create a factory
	JPH::Factory::sInstance = new JPH::Factory();

	// Register all Jolt physics types
	JPH::RegisterTypes();

	// Jolt example jobs system implementation
	Jobs = new JPH::JobSystemThreadPool(JPH::cMaxPhysicsJobs, JPH::cMaxPhysicsBarriers);
	
	// Pre-allocate 64 MB
	TempAllocator = new JPH::TempAllocatorImpl(64 * 1024 * 1024);
}

Jolt::~JoltPhysicsServer3D() {
	delete Jobs;
	delete TempAllocator;
}

RID Jolt::body_create() {
	using namespace JPH;
	printf("Body created!\n");
	
	RID rid = Base::body_create();

	// Placeholder shape
	SphereShapeSettings shape = SphereShapeSettings(1.0f);
	
	// Create body
	BodyCreationSettings settings(shape.Create().Get(), Vec3(0.0f, 0.0f, 0.0f), Quat::sIdentity(), EMotionType::Dynamic, Layers::MOVING);
	
	BodyInterface& bodies = BodiesLock();
	BodyID body = bodies.CreateAndAddBody(settings, EActivation::Activate);

	// Assign to RID
	own_bodies.initialize_rid(rid, body.GetIndexAndSequenceNumber());

	// TEMP
	bodies.SetLinearVelocity(body, Vec3(0.0f, -5.0f, 0.0f));
	
	return rid;
}

void Jolt::free(RID p_rid) {
	printf("Freeing RID %lu\n", p_rid.get_id());

	// Check each owner!
	if (own_bodies.owns(p_rid)) {
		Bodies().DestroyBody(get_body_id(p_rid));
		own_bodies.free(p_rid);
	} // else if (own_shapes.owns(p_rid)) { ... }

	return Base::free(p_rid);
}


void Jolt::init() {
	printf("[Jolt] init()\n");

	// This is the max amount of rigid bodies that you can add to the physics system.
	const uint cMaxBodies = 65536;

	// This determines how many mutexes to allocate to protect rigid bodies from concurrent access. Set it to 0 for the default settings.
	const uint cNumBodyMutexes = 0;

	// This is the max amount of body pairs that can be queued at any time (the broad phase will detect overlapping
	// body pairs based on their bounding boxes and will insert them into a queue for the narrowphase). If you make this buffer
	// too small the queue will fill up and the broad phase jobs will start to do narrow phase work. This is slightly less efficient.
	const uint cMaxBodyPairs = 65536;

	// This is the maximum size of the contact constraint buffer. If more contacts (collisions between bodies) are detected than this
	// number then these contacts will be ignored and bodies will start interpenetrating / fall through the world.
	const uint cMaxContactConstraints = 10240;

	// Create mapping table from object layer to broadphase layer
	// Note: As this is an interface, PhysicsSystem will take a reference to this so this instance needs to stay alive!
	// TODO: put this somewhere else
	static BPLayerInterfaceImpl bpLayers;

	// Now we can create the actual physics system.
	Physics.Init(cMaxBodies, cNumBodyMutexes, cMaxBodyPairs, cMaxContactConstraints, bpLayers, GDBroadPhaseCanCollide, GDObjectCanCollide);

	// Optional step: Before starting the physics simulation you can optimize the broad phase. This improves collision detection performance (it's pointless here because we only have 2 bodies).
	// You should definitely not call this every frame or when e.g. streaming in a new level section as it is an expensive operation.
	// Instead insert all new objects in batches instead of 1 at a time to keep the broad phase efficient.
	Physics.OptimizeBroadPhase();
	
	return Base::init();
}

void Jolt::finish() {
	printf("[Jolt] finish()\n");
	return Base::finish();
}

void Jolt::step(real_t p_step) {

	// If you take larger steps than 1 / 60th of a second you need to do multiple collision steps in order to keep the simulation stable. Do 1 collision step per 1 / 60th of a second (round up).
	const int cCollisionSteps = 1;

	// If you want more accurate step results you can do multiple sub steps within a collision step. Usually you would set this to 1.
	const int cIntegrationSubSteps = 1;

	// Step the world
	Physics.Update(p_step, cCollisionSteps, cIntegrationSubSteps, TempAllocator, Jobs);
	
	return Base::step(0.1f * p_step);
}
