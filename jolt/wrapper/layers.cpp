/*
 * Jolt Physics C Wrapper - Collision Layers, Contact Events, Filtered Raycast
 *
 * T-0103: Dynamic N-layer collision system, contact event polling,
 * and layer-filtered raycasting.
 */

#include "layers.h"
#include "core.h"
#include <Jolt/Jolt.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/NarrowPhaseQuery.h>
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>
#include <Jolt/Physics/Collision/ContactListener.h>
#include <vector>
#include <mutex>
#include <algorithm>
#include <memory>

using namespace JPH;

// ============================================================================
// Collision Matrix Data
// ============================================================================

struct CollisionMatrixData
{
	int numLayers;
	std::vector<bool> matrix; // [numLayers * numLayers], symmetric

	CollisionMatrixData(int n) : numLayers(n), matrix(n * n, false) {}

	bool ShouldCollide(int l1, int l2) const
	{
		if (l1 < 0 || l1 >= numLayers || l2 < 0 || l2 >= numLayers) return false;
		return matrix[l1 * numLayers + l2];
	}

	void Enable(int l1, int l2)
	{
		if (l1 < 0 || l1 >= numLayers || l2 < 0 || l2 >= numLayers) return;
		matrix[l1 * numLayers + l2] = true;
		matrix[l2 * numLayers + l1] = true;
	}

	void Disable(int l1, int l2)
	{
		if (l1 < 0 || l1 >= numLayers || l2 < 0 || l2 >= numLayers) return;
		matrix[l1 * numLayers + l2] = false;
		matrix[l2 * numLayers + l1] = false;
	}
};

// ============================================================================
// Dynamic N-Layer Interfaces
// ============================================================================

// 1:1 mapping: each object layer is its own broadphase layer
class DynamicBPLayerInterface final : public BroadPhaseLayerInterface
{
public:
	DynamicBPLayerInterface(int numLayers) : m_numLayers(numLayers) {}

	virtual uint GetNumBroadPhaseLayers() const override
	{
		return static_cast<uint>(m_numLayers);
	}

	virtual BroadPhaseLayer GetBroadPhaseLayer(ObjectLayer inLayer) const override
	{
		return BroadPhaseLayer(static_cast<BroadPhaseLayer::Type>(inLayer));
	}

#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
	virtual const char* GetBroadPhaseLayerName(BroadPhaseLayer inLayer) const override
	{
		return "DYNAMIC";
	}
#endif

private:
	int m_numLayers;
};

// ObjectVsBroadPhaseLayerFilter using collision matrix (owns a copy)
class DynamicObjectVsBPFilter final : public ObjectVsBroadPhaseLayerFilter
{
public:
	DynamicObjectVsBPFilter(const CollisionMatrixData& cm) : m_cm(cm) {}

	virtual bool ShouldCollide(ObjectLayer inLayer1, BroadPhaseLayer inLayer2) const override
	{
		// With 1:1 mapping, broadphase layer N == object layer N
		int bp = static_cast<int>((BroadPhaseLayer::Type)inLayer2);
		return m_cm.ShouldCollide(static_cast<int>(inLayer1), bp);
	}

private:
	CollisionMatrixData m_cm; // Owned copy
};

// ObjectLayerPairFilter using collision matrix (owns a copy)
class DynamicObjectLayerPairFilter final : public ObjectLayerPairFilter
{
public:
	DynamicObjectLayerPairFilter(const CollisionMatrixData& cm) : m_cm(cm) {}

	virtual bool ShouldCollide(ObjectLayer inObject1, ObjectLayer inObject2) const override
	{
		return m_cm.ShouldCollide(static_cast<int>(inObject1), static_cast<int>(inObject2));
	}

private:
	CollisionMatrixData m_cm; // Owned copy
};

// ============================================================================
// Contact Listener Implementation
// ============================================================================

class ContactListenerImpl : public ContactListener
{
public:
	virtual ValidateResult OnContactValidate(const Body &inBody1, const Body &inBody2,
	                                         RVec3Arg inBaseOffset,
	                                         const CollideShapeResult &inCollisionResult) override
	{
		return ValidateResult::AcceptAllContactsForThisBodyPair;
	}

	virtual void OnContactAdded(const Body &inBody1, const Body &inBody2,
	                            const ContactManifold &inManifold,
	                            ContactSettings &ioSettings) override
	{
		JoltContactEvent evt;
		evt.type = JoltContactAdded;
		evt.bodyID1 = inBody1.GetID().GetIndexAndSequenceNumber();
		evt.bodyID2 = inBody2.GetID().GetIndexAndSequenceNumber();

		// Use first contact point if available
		if (inManifold.mRelativeContactPointsOn1.size() > 0)
		{
			Vec3 cp = Vec3(inManifold.mRelativeContactPointsOn1[0]) + Vec3(inManifold.mBaseOffset);
			evt.contactPointX = cp.GetX();
			evt.contactPointY = cp.GetY();
			evt.contactPointZ = cp.GetZ();
		}
		else
		{
			evt.contactPointX = evt.contactPointY = evt.contactPointZ = 0;
		}

		Vec3 normal = inManifold.mWorldSpaceNormal;
		evt.normalX = normal.GetX();
		evt.normalY = normal.GetY();
		evt.normalZ = normal.GetZ();

		std::lock_guard<std::mutex> lock(m_mutex);
		m_events.push_back(evt);
	}

	virtual void OnContactPersisted(const Body &inBody1, const Body &inBody2,
	                                const ContactManifold &inManifold,
	                                ContactSettings &ioSettings) override
	{
		JoltContactEvent evt;
		evt.type = JoltContactPersisted;
		evt.bodyID1 = inBody1.GetID().GetIndexAndSequenceNumber();
		evt.bodyID2 = inBody2.GetID().GetIndexAndSequenceNumber();

		if (inManifold.mRelativeContactPointsOn1.size() > 0)
		{
			Vec3 cp = Vec3(inManifold.mRelativeContactPointsOn1[0]) + Vec3(inManifold.mBaseOffset);
			evt.contactPointX = cp.GetX();
			evt.contactPointY = cp.GetY();
			evt.contactPointZ = cp.GetZ();
		}
		else
		{
			evt.contactPointX = evt.contactPointY = evt.contactPointZ = 0;
		}

		Vec3 normal = inManifold.mWorldSpaceNormal;
		evt.normalX = normal.GetX();
		evt.normalY = normal.GetY();
		evt.normalZ = normal.GetZ();

		std::lock_guard<std::mutex> lock(m_mutex);
		m_events.push_back(evt);
	}

	virtual void OnContactRemoved(const SubShapeIDPair &inSubShapePair) override
	{
		JoltContactEvent evt;
		evt.type = JoltContactRemoved;
		evt.bodyID1 = inSubShapePair.GetBody1ID().GetIndexAndSequenceNumber();
		evt.bodyID2 = inSubShapePair.GetBody2ID().GetIndexAndSequenceNumber();
		evt.contactPointX = evt.contactPointY = evt.contactPointZ = 0;
		evt.normalX = evt.normalY = evt.normalZ = 0;

		std::lock_guard<std::mutex> lock(m_mutex);
		m_events.push_back(evt);
	}

	int Drain(JoltContactEvent* outEvents, int maxEvents)
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		int count = std::min(static_cast<int>(m_events.size()), maxEvents);
		for (int i = 0; i < count; i++)
		{
			outEvents[i] = m_events[i];
		}
		if (count == static_cast<int>(m_events.size()))
		{
			m_events.clear();
		}
		else
		{
			m_events.erase(m_events.begin(), m_events.begin() + count);
		}
		return count;
	}

private:
	std::mutex m_mutex;
	std::vector<JoltContactEvent> m_events;
};

// Helper for PhysicsSystemWrapper destructor (type is complete here)
void DestroyContactListener(ContactListenerImpl* listener)
{
	delete listener;
}

// ============================================================================
// Layer Mask Filters (for filtered raycast)
// ============================================================================

class LayerMaskBroadPhaseFilter : public BroadPhaseLayerFilter
{
public:
	LayerMaskBroadPhaseFilter(uint32_t mask) : m_mask(mask) {}

	virtual bool ShouldCollide(BroadPhaseLayer inLayer) const override
	{
		uint32_t layerBit = 1u << static_cast<uint32_t>((BroadPhaseLayer::Type)inLayer);
		return (m_mask & layerBit) != 0;
	}

private:
	uint32_t m_mask;
};

class LayerMaskObjectFilter : public ObjectLayerFilter
{
public:
	LayerMaskObjectFilter(uint32_t mask) : m_mask(mask) {}

	virtual bool ShouldCollide(ObjectLayer inLayer) const override
	{
		uint32_t layerBit = 1u << static_cast<uint32_t>(inLayer);
		return (m_mask & layerBit) != 0;
	}

private:
	uint32_t m_mask;
};

// ============================================================================
// Closest Ray Hit Collector (duplicated from query.cpp for self-containment)
// ============================================================================

class FilteredClosestRayHitCollector : public CastRayCollector
{
public:
	FilteredClosestRayHitCollector() : m_hasHit(false), m_closestFraction(1.0f) {}

	virtual void AddHit(const RayCastResult& inResult) override
	{
		if (inResult.mFraction < m_closestFraction)
		{
			m_hasHit = true;
			m_closestFraction = inResult.mFraction;
			m_closestHit = inResult;
		}
	}

	bool HasHit() const { return m_hasHit; }
	const RayCastResult& GetClosestHit() const { return m_closestHit; }

private:
	bool m_hasHit;
	float m_closestFraction;
	RayCastResult m_closestHit;
};

// ============================================================================
// C API Implementation
// ============================================================================

// --- Collision Matrix ---

JoltCollisionMatrix JoltCreateCollisionMatrix(int numLayers)
{
	if (numLayers <= 0) return nullptr;
	auto* cm = new CollisionMatrixData(numLayers);
	return static_cast<JoltCollisionMatrix>(cm);
}

void JoltCollisionMatrixEnableCollision(JoltCollisionMatrix matrix, int layer1, int layer2)
{
	auto* cm = static_cast<CollisionMatrixData*>(matrix);
	if (cm) cm->Enable(layer1, layer2);
}

void JoltCollisionMatrixDisableCollision(JoltCollisionMatrix matrix, int layer1, int layer2)
{
	auto* cm = static_cast<CollisionMatrixData*>(matrix);
	if (cm) cm->Disable(layer1, layer2);
}

void JoltDestroyCollisionMatrix(JoltCollisionMatrix matrix)
{
	auto* cm = static_cast<CollisionMatrixData*>(matrix);
	delete cm;
}

// --- N-Layer Physics System ---

JoltPhysicsSystem JoltCreatePhysicsSystemWithLayers(JoltCollisionMatrix matrix)
{
	auto* cm = static_cast<CollisionMatrixData*>(matrix);
	if (!cm) return nullptr;

	const uint cMaxBodies = 10240;
	const uint cNumBodyMutexes = 0;
	const uint cMaxBodyPairs = 65536;
	const uint cMaxContactConstraints = 20480;

	auto wrapper = std::make_unique<PhysicsSystemWrapper>();

	// Create dynamic layer interfaces — each filter copies the collision matrix data,
	// so the JoltCollisionMatrix can be freed after this call.
	wrapper->broad_phase_layer_interface = std::make_unique<DynamicBPLayerInterface>(cm->numLayers);
	wrapper->object_vs_broadphase_layer_filter = std::make_unique<DynamicObjectVsBPFilter>(*cm);
	wrapper->object_vs_object_layer_filter = std::make_unique<DynamicObjectLayerPairFilter>(*cm);

	wrapper->system = std::make_unique<PhysicsSystem>();
	wrapper->system->Init(cMaxBodies, cNumBodyMutexes, cMaxBodyPairs, cMaxContactConstraints,
	                      *wrapper->broad_phase_layer_interface,
	                      *wrapper->object_vs_broadphase_layer_filter,
	                      *wrapper->object_vs_object_layer_filter);

	return static_cast<JoltPhysicsSystem>(wrapper.release());
}

// --- Layer-Aware Body Creation ---

JoltBodyID JoltCreateBodyOnLayer(JoltBodyInterface bodyInterface,
                                 JoltShape shape,
                                 float x, float y, float z,
                                 JoltMotionType motionType,
                                 int isSensor,
                                 int layer)
{
	BodyInterface *bi = static_cast<BodyInterface *>(bodyInterface);
	const Shape *s = static_cast<const Shape *>(shape);

	EMotionType joltMotionType;
	switch (motionType)
	{
	case JoltMotionTypeStatic:
		joltMotionType = EMotionType::Static;
		break;
	case JoltMotionTypeKinematic:
		joltMotionType = EMotionType::Kinematic;
		break;
	case JoltMotionTypeDynamic:
		joltMotionType = EMotionType::Dynamic;
		break;
	default:
		joltMotionType = EMotionType::Static;
		break;
	}

	BodyCreationSettings body_settings(
		s,
		RVec3(x, y, z),
		Quat::sIdentity(),
		joltMotionType,
		static_cast<ObjectLayer>(layer));

	body_settings.mIsSensor = (isSensor != 0);

	Body *body = bi->CreateBody(body_settings);
	if (!body) return nullptr;

	bi->AddBody(body->GetID(), EActivation::DontActivate);

	auto bodyIDPtr = std::make_unique<BodyID>(body->GetID());
	return static_cast<JoltBodyID>(bodyIDPtr.release());
}

int JoltGetBodyLayer(JoltPhysicsSystem system, JoltBodyID bodyID)
{
	PhysicsSystemWrapper *wrapper = static_cast<PhysicsSystemWrapper *>(system);
	PhysicsSystem *ps = GetPhysicsSystem(wrapper);
	const BodyID *bid = static_cast<const BodyID *>(bodyID);

	BodyLockRead lock(ps->GetBodyLockInterface(), *bid);
	if (lock.Succeeded())
	{
		return static_cast<int>(lock.GetBody().GetObjectLayer());
	}
	return -1;
}

// --- Contact Events ---

void JoltEnableContactEvents(JoltPhysicsSystem system)
{
	PhysicsSystemWrapper *wrapper = static_cast<PhysicsSystemWrapper *>(system);
	PhysicsSystem *ps = GetPhysicsSystem(wrapper);

	if (!wrapper->contact_listener)
	{
		wrapper->contact_listener = new ContactListenerImpl();
		ps->SetContactListener(wrapper->contact_listener);
	}
}

int JoltDrainContactEvents(JoltPhysicsSystem system, JoltContactEvent* outEvents, int maxEvents)
{
	PhysicsSystemWrapper *wrapper = static_cast<PhysicsSystemWrapper *>(system);
	if (!wrapper->contact_listener || !outEvents || maxEvents <= 0) return 0;
	return wrapper->contact_listener->Drain(outEvents, maxEvents);
}

// --- Filtered Raycast ---

int JoltCastRayFiltered(JoltPhysicsSystem system,
                        float originX, float originY, float originZ,
                        float directionX, float directionY, float directionZ,
                        unsigned int layerMask,
                        JoltRaycastHit* outHit)
{
	PhysicsSystemWrapper *wrapper = static_cast<PhysicsSystemWrapper *>(system);
	PhysicsSystem *ps = GetPhysicsSystem(wrapper);

	const NarrowPhaseQuery& query = ps->GetNarrowPhaseQuery();

	RRayCast ray;
	ray.mOrigin = RVec3(originX, originY, originZ);
	ray.mDirection = Vec3(directionX, directionY, directionZ);

	// Use layer mask filters
	LayerMaskBroadPhaseFilter bpFilter(layerMask);
	LayerMaskObjectFilter objFilter(layerMask);

	FilteredClosestRayHitCollector collector;
	RayCastSettings settings;

	query.CastRay(ray, settings, collector, bpFilter, objFilter);

	if (collector.HasHit() && outHit != nullptr)
	{
		const RayCastResult& result = collector.GetClosestHit();

		BodyID* bodyIDCopy = new BodyID(result.mBodyID);
		outHit->bodyID = static_cast<JoltBodyID>(bodyIDCopy);

		RVec3 hitPoint = ray.GetPointOnRay(result.mFraction);
		outHit->hitPointX = static_cast<float>(hitPoint.GetX());
		outHit->hitPointY = static_cast<float>(hitPoint.GetY());
		outHit->hitPointZ = static_cast<float>(hitPoint.GetZ());

		Vec3 normal = Vec3::sZero();
		{
			const BodyLockInterface& bodyLock = ps->GetBodyLockInterface();
			BodyLockRead lock(bodyLock, result.mBodyID);
			if (lock.Succeeded())
			{
				const Body& body = lock.GetBody();
				normal = body.GetWorldSpaceSurfaceNormal(result.mSubShapeID2,
				         Vec3(ray.GetPointOnRay(result.mFraction)));
			}
		}
		outHit->normalX = normal.GetX();
		outHit->normalY = normal.GetY();
		outHit->normalZ = normal.GetZ();

		outHit->fraction = result.mFraction;
	}

	return collector.HasHit() ? 1 : 0;
}
