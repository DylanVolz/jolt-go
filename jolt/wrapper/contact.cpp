/*
 * Jolt Physics C Wrapper - Contact Listener and Body Activation Listener
 *
 * T-0121: callback-based ContactListener and BodyActivationListener that
 * marshal Jolt types into flat C structs and forward to Go via extern "C"
 * trampolines (defined as cgo //export functions in jolt/contact.go).
 */

#include "contact.h"
#include <Jolt/Jolt.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/Body/BodyActivationListener.h>
#include <Jolt/Physics/Collision/ContactListener.h>
#include <Jolt/Physics/Collision/CollideShape.h>
#include <Jolt/Physics/Collision/Shape/SubShapeIDPair.h>

using namespace JPH;

// ============================================================================
// cgo //export trampolines (resolved at final Go binary link time).
// ============================================================================
extern "C" {

int  goJoltOnContactValidate(unsigned int body1ID, unsigned int body2ID,
                              float baseOffsetX, float baseOffsetY, float baseOffsetZ,
                              float contactPointX, float contactPointY, float contactPointZ,
                              float normalX, float normalY, float normalZ,
                              float penetrationDepth,
                              uintptr_t userData);

void goJoltOnContactAdded(const JoltContactManifold* manifold,
                          JoltContactSettings* settings,
                          uintptr_t userData);

void goJoltOnContactPersisted(const JoltContactManifold* manifold,
                              JoltContactSettings* settings,
                              uintptr_t userData);

void goJoltOnContactRemoved(unsigned int body1ID, unsigned int body2ID,
                            uintptr_t userData);

void goJoltOnBodyActivated(unsigned int bodyID, unsigned long long bodyUserData,
                           uintptr_t userData);

void goJoltOnBodyDeactivated(unsigned int bodyID, unsigned long long bodyUserData,
                             uintptr_t userData);

} // extern "C"

// ============================================================================
// Contact Listener Implementation
// ============================================================================

class ContactCallbackListenerImpl : public ContactListener
{
public:
	explicit ContactCallbackListenerImpl(uintptr_t userData) : m_userData(userData) {}

	virtual ValidateResult OnContactValidate(const Body &inBody1, const Body &inBody2,
	                                         RVec3Arg inBaseOffset,
	                                         const CollideShapeResult &inCollisionResult) override
	{
		// JPH docs: contact normal is -mPenetrationAxis.Normalized()
		Vec3 normal = -inCollisionResult.mPenetrationAxis.NormalizedOr(Vec3::sZero());
		Vec3 cp1 = inCollisionResult.mContactPointOn1; // relative to inBaseOffset

		int r = goJoltOnContactValidate(
			inBody1.GetID().GetIndexAndSequenceNumber(),
			inBody2.GetID().GetIndexAndSequenceNumber(),
			static_cast<float>(inBaseOffset.GetX()),
			static_cast<float>(inBaseOffset.GetY()),
			static_cast<float>(inBaseOffset.GetZ()),
			cp1.GetX(), cp1.GetY(), cp1.GetZ(),
			normal.GetX(), normal.GetY(), normal.GetZ(),
			inCollisionResult.mPenetrationDepth,
			m_userData);

		switch (r)
		{
		case JoltAcceptAllContactsForThisBodyPair: return ValidateResult::AcceptAllContactsForThisBodyPair;
		case JoltAcceptContact:                    return ValidateResult::AcceptContact;
		case JoltRejectContact:                    return ValidateResult::RejectContact;
		case JoltRejectAllContactsForThisBodyPair: return ValidateResult::RejectAllContactsForThisBodyPair;
		default:                                   return ValidateResult::AcceptAllContactsForThisBodyPair;
		}
	}

	virtual void OnContactAdded(const Body &inBody1, const Body &inBody2,
	                            const ContactManifold &inManifold,
	                            ContactSettings &ioSettings) override
	{
		JoltContactManifold m;
		FillManifold(m, inBody1, inBody2, inManifold);
		JoltContactSettings s;
		FillSettings(s, ioSettings);
		goJoltOnContactAdded(&m, &s, m_userData);
		ApplySettings(ioSettings, s);
	}

	virtual void OnContactPersisted(const Body &inBody1, const Body &inBody2,
	                                const ContactManifold &inManifold,
	                                ContactSettings &ioSettings) override
	{
		JoltContactManifold m;
		FillManifold(m, inBody1, inBody2, inManifold);
		JoltContactSettings s;
		FillSettings(s, ioSettings);
		goJoltOnContactPersisted(&m, &s, m_userData);
		ApplySettings(ioSettings, s);
	}

	virtual void OnContactRemoved(const SubShapeIDPair &inSubShapePair) override
	{
		goJoltOnContactRemoved(
			inSubShapePair.GetBody1ID().GetIndexAndSequenceNumber(),
			inSubShapePair.GetBody2ID().GetIndexAndSequenceNumber(),
			m_userData);
	}

private:
	uintptr_t m_userData;

	static void FillManifold(JoltContactManifold &out, const Body &b1, const Body &b2,
	                         const ContactManifold &m)
	{
		out.body1ID = b1.GetID().GetIndexAndSequenceNumber();
		out.body2ID = b2.GetID().GetIndexAndSequenceNumber();

		out.baseOffsetX = static_cast<float>(m.mBaseOffset.GetX());
		out.baseOffsetY = static_cast<float>(m.mBaseOffset.GetY());
		out.baseOffsetZ = static_cast<float>(m.mBaseOffset.GetZ());

		out.normalX = m.mWorldSpaceNormal.GetX();
		out.normalY = m.mWorldSpaceNormal.GetY();
		out.normalZ = m.mWorldSpaceNormal.GetZ();

		out.penetrationDepth = m.mPenetrationDepth;
		out.numContactPoints = static_cast<int>(m.mRelativeContactPointsOn1.size());

		if (m.mRelativeContactPointsOn1.size() > 0)
		{
			Vec3 p1 = m.mRelativeContactPointsOn1[0];
			out.contactPoint1X = p1.GetX();
			out.contactPoint1Y = p1.GetY();
			out.contactPoint1Z = p1.GetZ();
		}
		else
		{
			out.contactPoint1X = out.contactPoint1Y = out.contactPoint1Z = 0.0f;
		}

		if (m.mRelativeContactPointsOn2.size() > 0)
		{
			Vec3 p2 = m.mRelativeContactPointsOn2[0];
			out.contactPoint2X = p2.GetX();
			out.contactPoint2Y = p2.GetY();
			out.contactPoint2Z = p2.GetZ();
		}
		else
		{
			out.contactPoint2X = out.contactPoint2Y = out.contactPoint2Z = 0.0f;
		}

		// Relative linear velocity (body2 - body1) at the body level — useful for
		// estimating impact strength in the Added callback before the solver runs.
		Vec3 v1 = b1.GetLinearVelocity();
		Vec3 v2 = b2.GetLinearVelocity();
		Vec3 rv = v2 - v1;
		out.relativeLinearVelocityX = rv.GetX();
		out.relativeLinearVelocityY = rv.GetY();
		out.relativeLinearVelocityZ = rv.GetZ();
	}

	static void FillSettings(JoltContactSettings &out, const ContactSettings &s)
	{
		out.combinedFriction = s.mCombinedFriction;
		out.combinedRestitution = s.mCombinedRestitution;
		out.invMassScale1 = s.mInvMassScale1;
		out.invInertiaScale1 = s.mInvInertiaScale1;
		out.invMassScale2 = s.mInvMassScale2;
		out.invInertiaScale2 = s.mInvInertiaScale2;
		out.isSensor = s.mIsSensor ? 1 : 0;
		out.relativeLinearSurfaceVelocityX = s.mRelativeLinearSurfaceVelocity.GetX();
		out.relativeLinearSurfaceVelocityY = s.mRelativeLinearSurfaceVelocity.GetY();
		out.relativeLinearSurfaceVelocityZ = s.mRelativeLinearSurfaceVelocity.GetZ();
		out.relativeAngularSurfaceVelocityX = s.mRelativeAngularSurfaceVelocity.GetX();
		out.relativeAngularSurfaceVelocityY = s.mRelativeAngularSurfaceVelocity.GetY();
		out.relativeAngularSurfaceVelocityZ = s.mRelativeAngularSurfaceVelocity.GetZ();
	}

	static void ApplySettings(ContactSettings &out, const JoltContactSettings &s)
	{
		out.mCombinedFriction = s.combinedFriction;
		out.mCombinedRestitution = s.combinedRestitution;
		out.mInvMassScale1 = s.invMassScale1;
		out.mInvInertiaScale1 = s.invInertiaScale1;
		out.mInvMassScale2 = s.invMassScale2;
		out.mInvInertiaScale2 = s.invInertiaScale2;
		out.mIsSensor = (s.isSensor != 0);
		out.mRelativeLinearSurfaceVelocity = Vec3(
			s.relativeLinearSurfaceVelocityX,
			s.relativeLinearSurfaceVelocityY,
			s.relativeLinearSurfaceVelocityZ);
		out.mRelativeAngularSurfaceVelocity = Vec3(
			s.relativeAngularSurfaceVelocityX,
			s.relativeAngularSurfaceVelocityY,
			s.relativeAngularSurfaceVelocityZ);
	}
};

// ============================================================================
// Body Activation Listener Implementation
// ============================================================================

class BodyActivationListenerImpl : public BodyActivationListener
{
public:
	explicit BodyActivationListenerImpl(uintptr_t userData) : m_userData(userData) {}

	virtual void OnBodyActivated(const BodyID &inBodyID, uint64 inBodyUserData) override
	{
		goJoltOnBodyActivated(inBodyID.GetIndexAndSequenceNumber(),
		                      static_cast<unsigned long long>(inBodyUserData),
		                      m_userData);
	}

	virtual void OnBodyDeactivated(const BodyID &inBodyID, uint64 inBodyUserData) override
	{
		goJoltOnBodyDeactivated(inBodyID.GetIndexAndSequenceNumber(),
		                        static_cast<unsigned long long>(inBodyUserData),
		                        m_userData);
	}

private:
	uintptr_t m_userData;
};

// ============================================================================
// Cleanup helpers (called from PhysicsSystemWrapper destructor)
// ============================================================================

void DestroyContactCallbackListener(ContactCallbackListenerImpl* listener)
{
	delete listener;
}

void DestroyBodyActivationListener(BodyActivationListenerImpl* listener)
{
	delete listener;
}

// ============================================================================
// C API Implementation
// ============================================================================

extern "C" {

int JoltInstallContactCallbacks(JoltPhysicsSystem system, uintptr_t userData)
{
	PhysicsSystemWrapper *wrapper = static_cast<PhysicsSystemWrapper *>(system);
	if (!wrapper) return -1;
	PhysicsSystem *ps = GetPhysicsSystem(wrapper);
	if (!ps) return -1;

	if (wrapper->contact_callback_listener)
	{
		delete wrapper->contact_callback_listener;
		wrapper->contact_callback_listener = nullptr;
	}
	wrapper->contact_callback_listener = new ContactCallbackListenerImpl(userData);
	ps->SetContactListener(wrapper->contact_callback_listener);
	return 0;
}

void JoltUninstallContactCallbacks(JoltPhysicsSystem system)
{
	PhysicsSystemWrapper *wrapper = static_cast<PhysicsSystemWrapper *>(system);
	if (!wrapper) return;
	PhysicsSystem *ps = GetPhysicsSystem(wrapper);
	if (ps) ps->SetContactListener(nullptr);
	delete wrapper->contact_callback_listener;
	wrapper->contact_callback_listener = nullptr;
}

int JoltInstallBodyActivationCallbacks(JoltPhysicsSystem system, uintptr_t userData)
{
	PhysicsSystemWrapper *wrapper = static_cast<PhysicsSystemWrapper *>(system);
	if (!wrapper) return -1;
	PhysicsSystem *ps = GetPhysicsSystem(wrapper);
	if (!ps) return -1;

	if (wrapper->body_activation_listener)
	{
		delete wrapper->body_activation_listener;
		wrapper->body_activation_listener = nullptr;
	}
	wrapper->body_activation_listener = new BodyActivationListenerImpl(userData);
	ps->SetBodyActivationListener(wrapper->body_activation_listener);
	return 0;
}

void JoltUninstallBodyActivationCallbacks(JoltPhysicsSystem system)
{
	PhysicsSystemWrapper *wrapper = static_cast<PhysicsSystemWrapper *>(system);
	if (!wrapper) return;
	PhysicsSystem *ps = GetPhysicsSystem(wrapper);
	if (ps) ps->SetBodyActivationListener(nullptr);
	delete wrapper->body_activation_listener;
	wrapper->body_activation_listener = nullptr;
}

} // extern "C"
