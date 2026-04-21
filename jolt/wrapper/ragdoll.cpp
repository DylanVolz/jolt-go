/*
 * Jolt Physics C Wrapper - Ragdoll System Implementation (T-0124)
 */

#include "ragdoll.h"
#include "physics.h"
#include <Jolt/Jolt.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>
#include <Jolt/Physics/Constraints/SwingTwistConstraint.h>
#include <Jolt/Physics/Ragdoll/Ragdoll.h>
#include <Jolt/Physics/EActivation.h>
#include <Jolt/Skeleton/Skeleton.h>
#include <vector>

using namespace JPH;

// --- Skeleton ---

JoltSkeleton JoltCreateSkeleton(void)
{
    Skeleton *s = new Skeleton();
    s->AddRef();
    return static_cast<JoltSkeleton>(s);
}

void JoltDestroySkeleton(JoltSkeleton skeleton)
{
    Skeleton *s = static_cast<Skeleton *>(skeleton);
    if (s) s->Release();
}

int JoltSkeletonAddJoint(JoltSkeleton skeleton, const char *name, int parentIndex)
{
    Skeleton *s = static_cast<Skeleton *>(skeleton);
    if (!s || !name) return -1;
    int count = s->GetJointCount();
    if (parentIndex >= count) return -1; // forward references not allowed
    if (parentIndex < -1) return -1;
    return (int)s->AddJoint(name, parentIndex);
}

int JoltSkeletonGetJointCount(JoltSkeleton skeleton)
{
    Skeleton *s = static_cast<Skeleton *>(skeleton);
    return s ? s->GetJointCount() : 0;
}

int JoltSkeletonGetParentIndex(JoltSkeleton skeleton, int jointIndex)
{
    Skeleton *s = static_cast<Skeleton *>(skeleton);
    if (!s || jointIndex < 0 || jointIndex >= s->GetJointCount()) return -1;
    return s->GetJoint(jointIndex).mParentJointIndex;
}

// --- RagdollSettings ---

JoltRagdollSettings JoltCreateRagdollSettings(JoltSkeleton skeleton)
{
    Skeleton *s = static_cast<Skeleton *>(skeleton);
    if (!s) return nullptr;

    RagdollSettings *rs = new RagdollSettings();
    rs->AddRef();
    rs->mSkeleton = s; // Ref<Skeleton> auto-AddRefs
    rs->mParts.resize(s->GetJointCount());
    for (RagdollSettings::Part &p : rs->mParts)
    {
        p.mRotation = Quat::sIdentity();
    }
    return static_cast<JoltRagdollSettings>(rs);
}

void JoltDestroyRagdollSettings(JoltRagdollSettings settings)
{
    RagdollSettings *rs = static_cast<RagdollSettings *>(settings);
    if (rs) rs->Release();
}

int JoltRagdollSettingsSetPart(
    JoltRagdollSettings settings, int jointIndex, JoltShape shape,
    float posX, float posY, float posZ,
    float rotX, float rotY, float rotZ, float rotW,
    int motionType, unsigned short objectLayer, float mass)
{
    RagdollSettings *rs = static_cast<RagdollSettings *>(settings);
    if (!rs || jointIndex < 0 || jointIndex >= (int)rs->mParts.size()) return -1;
    Shape *s = static_cast<Shape *>(shape);
    if (!s) return -1;

    RagdollSettings::Part &p = rs->mParts[jointIndex];
    p.SetShape(s);
    p.mPosition = RVec3(posX, posY, posZ);
    p.mRotation = Quat(rotX, rotY, rotZ, rotW);
    p.mMotionType = static_cast<EMotionType>(motionType);
    p.mObjectLayer = static_cast<ObjectLayer>(objectLayer);

    if (mass > 0.0f)
    {
        // Override the mass but keep inertia derived from the shape.
        p.mOverrideMassProperties = EOverrideMassProperties::CalculateInertia;
        p.mMassPropertiesOverride.mMass = mass;
    }
    return 0;
}

int JoltRagdollSettingsSetSwingTwistConstraint(
    JoltRagdollSettings settings, int jointIndex,
    float pivotX, float pivotY, float pivotZ,
    float twistAxisX, float twistAxisY, float twistAxisZ,
    float planeAxisX, float planeAxisY, float planeAxisZ,
    float normalHalfConeAngle, float planeHalfConeAngle,
    float twistMinAngle, float twistMaxAngle)
{
    RagdollSettings *rs = static_cast<RagdollSettings *>(settings);
    if (!rs || jointIndex < 0 || jointIndex >= (int)rs->mParts.size()) return -1;
    int parentIdx = rs->mSkeleton->GetJoint(jointIndex).mParentJointIndex;
    if (parentIdx < 0) return -1;

    SwingTwistConstraintSettings *st = new SwingTwistConstraintSettings();
    st->mSpace = EConstraintSpace::WorldSpace;
    RVec3 pivot(pivotX, pivotY, pivotZ);
    Vec3 twist(twistAxisX, twistAxisY, twistAxisZ);
    Vec3 plane(planeAxisX, planeAxisY, planeAxisZ);
    st->mPosition1 = pivot;
    st->mPosition2 = pivot;
    st->mTwistAxis1 = twist;
    st->mTwistAxis2 = twist;
    st->mPlaneAxis1 = plane;
    st->mPlaneAxis2 = plane;
    st->mNormalHalfConeAngle = normalHalfConeAngle;
    st->mPlaneHalfConeAngle = planeHalfConeAngle;
    st->mTwistMinAngle = twistMinAngle;
    st->mTwistMaxAngle = twistMaxAngle;

    rs->mParts[jointIndex].mToParent = st; // Ref<TwoBodyConstraintSettings> auto-AddRefs
    return 0;
}

int JoltRagdollSettingsFinalize(JoltRagdollSettings settings)
{
    RagdollSettings *rs = static_cast<RagdollSettings *>(settings);
    if (!rs) return -1;
    if (!rs->Stabilize()) return -1;
    rs->CalculateBodyIndexToConstraintIndex();
    rs->CalculateConstraintIndexToBodyIdxPair();
    rs->DisableParentChildCollisions();
    return 0;
}

// --- Ragdoll ---

JoltRagdoll JoltCreateRagdoll(JoltRagdollSettings settings,
                              JoltPhysicsSystem system,
                              unsigned int groupID,
                              unsigned long long userData)
{
    RagdollSettings *rs = static_cast<RagdollSettings *>(settings);
    PhysicsSystemWrapper *wrapper = static_cast<PhysicsSystemWrapper *>(system);
    if (!rs || !wrapper) return nullptr;
    PhysicsSystem *ps = GetPhysicsSystem(wrapper);

    Ragdoll *r = rs->CreateRagdoll(groupID, userData, ps);
    if (r) r->AddRef();
    return static_cast<JoltRagdoll>(r);
}

void JoltDestroyRagdoll(JoltRagdoll ragdoll)
{
    Ragdoll *r = static_cast<Ragdoll *>(ragdoll);
    if (r) r->Release();
}

void JoltRagdollAddToPhysicsSystem(JoltRagdoll ragdoll, int activate)
{
    Ragdoll *r = static_cast<Ragdoll *>(ragdoll);
    if (!r) return;
    r->AddToPhysicsSystem(activate ? EActivation::Activate : EActivation::DontActivate);
}

void JoltRagdollRemoveFromPhysicsSystem(JoltRagdoll ragdoll)
{
    Ragdoll *r = static_cast<Ragdoll *>(ragdoll);
    if (r) r->RemoveFromPhysicsSystem();
}

void JoltRagdollActivate(JoltRagdoll ragdoll)
{
    Ragdoll *r = static_cast<Ragdoll *>(ragdoll);
    if (r) r->Activate();
}

int JoltRagdollIsActive(JoltRagdoll ragdoll)
{
    Ragdoll *r = static_cast<Ragdoll *>(ragdoll);
    return (r && r->IsActive()) ? 1 : 0;
}

int JoltRagdollGetBodyCount(JoltRagdoll ragdoll)
{
    Ragdoll *r = static_cast<Ragdoll *>(ragdoll);
    return r ? (int)r->GetBodyCount() : 0;
}

JoltBodyID JoltRagdollGetBodyID(JoltRagdoll ragdoll, int index)
{
    Ragdoll *r = static_cast<Ragdoll *>(ragdoll);
    if (!r || index < 0 || index >= (int)r->GetBodyCount()) return nullptr;
    return static_cast<JoltBodyID>(new BodyID(r->GetBodyID(index)));
}

void JoltRagdollAddImpulse(JoltRagdoll ragdoll, float x, float y, float z)
{
    Ragdoll *r = static_cast<Ragdoll *>(ragdoll);
    if (r) r->AddImpulse(Vec3(x, y, z));
}

void JoltRagdollSetLinearVelocity(JoltRagdoll ragdoll, float x, float y, float z)
{
    Ragdoll *r = static_cast<Ragdoll *>(ragdoll);
    if (r) r->SetLinearVelocity(Vec3(x, y, z));
}

void JoltRagdollGetRootTransform(JoltRagdoll ragdoll,
                                 float *outPosX, float *outPosY, float *outPosZ,
                                 float *outRotX, float *outRotY, float *outRotZ,
                                 float *outRotW)
{
    Ragdoll *r = static_cast<Ragdoll *>(ragdoll);
    if (!r) return;
    RVec3 pos;
    Quat rot;
    r->GetRootTransform(pos, rot);
    *outPosX = (float)pos.GetX();
    *outPosY = (float)pos.GetY();
    *outPosZ = (float)pos.GetZ();
    *outRotX = rot.GetX();
    *outRotY = rot.GetY();
    *outRotZ = rot.GetZ();
    *outRotW = rot.GetW();
}

int JoltRagdollDriveToPoseUsingKinematics(
    JoltRagdoll ragdoll,
    float rootOffsetX, float rootOffsetY, float rootOffsetZ,
    const float *jointMatrices, int jointCount,
    float deltaTime)
{
    Ragdoll *r = static_cast<Ragdoll *>(ragdoll);
    if (!r || !jointMatrices) return -1;
    if (jointCount != (int)r->GetBodyCount()) return -1;

    std::vector<Mat44> mats((size_t)jointCount);
    for (int i = 0; i < jointCount; ++i)
    {
        const float *m = jointMatrices + i * 16;
        mats[i] = Mat44(
            Vec4(m[0], m[1], m[2], m[3]),
            Vec4(m[4], m[5], m[6], m[7]),
            Vec4(m[8], m[9], m[10], m[11]),
            Vec4(m[12], m[13], m[14], m[15]));
    }
    r->DriveToPoseUsingKinematics(RVec3(rootOffsetX, rootOffsetY, rootOffsetZ),
                                  mats.data(), deltaTime);
    return 0;
}
