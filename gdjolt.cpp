#include "gdjolt.h"
#include "jolt_util.h"
#include "jolt_layers.h"
#include "jolt_body_direct_state.h"

#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/RegisterTypes.h>

using Jolt = JoltPhysicsServer3D;
using Base = GodotPhysicsServer3D;

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


// Function that determines if two object layers can collide
static bool GDObjectCanCollide(JPH::ObjectLayer inObject1, JPH::ObjectLayer inObject2)
{
	using namespace JPH;
	switch (inObject1)
	{
	case Layers::NON_MOVING:
		return inObject2 == Layers::MOVING; // Non moving only collides with moving
	case Layers::MOVING:
		return true; // Moving collides with everything
	default:
		JPH_ASSERT(false);
		return false;
	}
};


// Function that determines if two broadphase layers can collide
static bool GDBroadPhaseCanCollide(JPH::ObjectLayer inLayer1, JPH::BroadPhaseLayer inLayer2)
{
	using namespace JPH;
	switch (inLayer1)
	{
	case Layers::NON_MOVING:
		return inLayer2 == BroadPhaseLayers::MOVING;
	case Layers::MOVING:
		return true;	
	default:
		JPH_ASSERT(false);
		return false;
	}
}

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

void Jolt::free(RID p_rid) {
	printf("[Jolt] Freeing RID %lu\n", p_rid.get_id());

	// Check each owner!
	// For JPH::Ref<T>, simply erase them to release.
	if (own_bodies.has(p_rid)) {
		const JPH::BodyID id = get_body_id(p_rid);
		if (Bodies().IsAdded(id))
			Bodies().RemoveBody(id);
		Bodies().DestroyBody(id);
		own_bodies.erase(p_rid);
	} else if (own_shapes.has(p_rid)) {
		own_shapes.erase(p_rid);
	}

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

	if (!active)
		return;

	// If you take larger steps than 1 / 60th of a second you need to do multiple collision steps in order to keep the simulation stable. Do 1 collision step per 1 / 60th of a second (round up).
	const int cCollisionSteps = 1;

	// If you want more accurate step results you can do multiple sub steps within a collision step. Usually you would set this to 1.
	const int cIntegrationSubSteps = 1;

	// Step the world
	Physics.Update(p_step, cCollisionSteps, cIntegrationSubSteps, TempAllocator, Jobs);
	
	return Base::step(0.1f * p_step);
}

void Jolt::flush_queries() {
	
	if (!active)
		return;

	// Call all our callbacks

	// for each (active) space
		// for each (queried) body: call_queries
		// for each (queried) area: call_queries

	for (KeyValue<RID, BodyData>& it : own_bodies) {
		
		BodyData& body = it.value;
		
		if (body.fi_callback.get_object()) {
			// TODO
		}

		if (body.state_callback.get_object()) {

			JPH::BodyLockWrite lock(Physics.GetBodyLockInterface(), JPH::BodyID(body.id));
			if (lock.Succeeded()) {
				JoltDirectBodyState state(lock.GetBody());
				Variant direct_state_variant = &state;
				const Variant *vp[1] = { &direct_state_variant };
				Callable::CallError ce;
				Variant rv;
				body.state_callback.callp(vp, 1, rv, ce);
			}
		}
	}

	return Base::flush_queries();
}