/*
 * Jolt Physics C Wrapper - Constraints & Buoyancy Implementation (T-0105)
 */

#include "constraints.h"
#include "physics.h"
#include <Jolt/Jolt.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/Constraints/DistanceConstraint.h>
#include <Jolt/Physics/Constraints/HingeConstraint.h>
#include <Jolt/Physics/Constraints/SliderConstraint.h>
#include <Jolt/Physics/Constraints/FixedConstraint.h>

using namespace JPH;

// --- Helper: resolve bodies from system + body IDs ---
// Returns false if either body lock fails.
static bool LockTwoBodies(PhysicsSystem *ps, const BodyID &bid1, const BodyID &bid2,
                          BodyLockWrite &lock1, BodyLockWrite &lock2)
{
    // Note: lock1 and lock2 are constructed by caller as stack variables
    // We just check their success.
    return lock1.Succeeded() && lock2.Succeeded();
}

// --- PhysicsSystem constraint management ---

void JoltAddConstraint(JoltPhysicsSystem system, JoltConstraint constraint)
{
    PhysicsSystemWrapper *wrapper = static_cast<PhysicsSystemWrapper *>(system);
    PhysicsSystem *ps = GetPhysicsSystem(wrapper);
    Constraint *c = static_cast<Constraint *>(constraint);
    ps->AddConstraint(c);
}

void JoltRemoveConstraint(JoltPhysicsSystem system, JoltConstraint constraint)
{
    PhysicsSystemWrapper *wrapper = static_cast<PhysicsSystemWrapper *>(system);
    PhysicsSystem *ps = GetPhysicsSystem(wrapper);
    Constraint *c = static_cast<Constraint *>(constraint);
    ps->RemoveConstraint(c);
}

void JoltDestroyConstraint(JoltConstraint constraint)
{
    // Constraints are ref-counted in Jolt. We Release our reference.
    Constraint *c = static_cast<Constraint *>(constraint);
    if (c)
    {
        c->Release();
    }
}

// --- Distance Constraint ---

JoltConstraint JoltCreateDistanceConstraint(
    JoltPhysicsSystem system,
    JoltBodyID bodyID1, JoltBodyID bodyID2,
    float point1X, float point1Y, float point1Z,
    float point2X, float point2Y, float point2Z,
    float minDistance, float maxDistance)
{
    PhysicsSystemWrapper *wrapper = static_cast<PhysicsSystemWrapper *>(system);
    PhysicsSystem *ps = GetPhysicsSystem(wrapper);
    const BodyID *bid1 = static_cast<const BodyID *>(bodyID1);
    const BodyID *bid2 = static_cast<const BodyID *>(bodyID2);

    DistanceConstraintSettings settings;
    settings.mSpace = EConstraintSpace::WorldSpace;
    settings.mPoint1 = RVec3(point1X, point1Y, point1Z);
    settings.mPoint2 = RVec3(point2X, point2Y, point2Z);
    settings.mMinDistance = minDistance;
    settings.mMaxDistance = maxDistance;

    BodyLockWrite lock1(ps->GetBodyLockInterface(), *bid1);
    BodyLockWrite lock2(ps->GetBodyLockInterface(), *bid2);
    if (!lock1.Succeeded() || !lock2.Succeeded())
        return nullptr;

    TwoBodyConstraint *constraint = settings.Create(lock1.GetBody(), lock2.GetBody());
    constraint->AddRef(); // We own a reference
    return static_cast<JoltConstraint>(constraint);
}

void JoltDistanceConstraintSetDistance(JoltConstraint constraint,
                                        float minDistance, float maxDistance)
{
    DistanceConstraint *c = static_cast<DistanceConstraint *>(constraint);
    c->SetDistance(minDistance, maxDistance);
}

float JoltDistanceConstraintGetMinDistance(JoltConstraint constraint)
{
    DistanceConstraint *c = static_cast<DistanceConstraint *>(constraint);
    return c->GetMinDistance();
}

float JoltDistanceConstraintGetMaxDistance(JoltConstraint constraint)
{
    DistanceConstraint *c = static_cast<DistanceConstraint *>(constraint);
    return c->GetMaxDistance();
}

void JoltDistanceConstraintSetSpring(JoltConstraint constraint,
                                      float frequency, float damping)
{
    DistanceConstraint *c = static_cast<DistanceConstraint *>(constraint);
    SpringSettings spring(ESpringMode::FrequencyAndDamping, frequency, damping);
    c->SetLimitsSpringSettings(spring);
}

float JoltDistanceConstraintGetTotalLambdaPosition(JoltConstraint constraint)
{
    DistanceConstraint *c = static_cast<DistanceConstraint *>(constraint);
    return c->GetTotalLambdaPosition();
}

// --- Hinge Constraint ---

JoltConstraint JoltCreateHingeConstraint(
    JoltPhysicsSystem system,
    JoltBodyID bodyID1, JoltBodyID bodyID2,
    float pointX, float pointY, float pointZ,
    float hingeAxisX, float hingeAxisY, float hingeAxisZ,
    float normalAxisX, float normalAxisY, float normalAxisZ,
    float limitsMin, float limitsMax)
{
    PhysicsSystemWrapper *wrapper = static_cast<PhysicsSystemWrapper *>(system);
    PhysicsSystem *ps = GetPhysicsSystem(wrapper);
    const BodyID *bid1 = static_cast<const BodyID *>(bodyID1);
    const BodyID *bid2 = static_cast<const BodyID *>(bodyID2);

    HingeConstraintSettings settings;
    settings.mSpace = EConstraintSpace::WorldSpace;
    settings.mPoint1 = RVec3(pointX, pointY, pointZ);
    settings.mPoint2 = RVec3(pointX, pointY, pointZ);
    settings.mHingeAxis1 = Vec3(hingeAxisX, hingeAxisY, hingeAxisZ);
    settings.mHingeAxis2 = Vec3(hingeAxisX, hingeAxisY, hingeAxisZ);
    settings.mNormalAxis1 = Vec3(normalAxisX, normalAxisY, normalAxisZ);
    settings.mNormalAxis2 = Vec3(normalAxisX, normalAxisY, normalAxisZ);

    if (limitsMin != 0.0f || limitsMax != 0.0f)
    {
        settings.mLimitsMin = limitsMin;
        settings.mLimitsMax = limitsMax;
    }

    BodyLockWrite lock1(ps->GetBodyLockInterface(), *bid1);
    BodyLockWrite lock2(ps->GetBodyLockInterface(), *bid2);
    if (!lock1.Succeeded() || !lock2.Succeeded())
        return nullptr;

    TwoBodyConstraint *constraint = settings.Create(lock1.GetBody(), lock2.GetBody());
    constraint->AddRef();
    return static_cast<JoltConstraint>(constraint);
}

float JoltHingeConstraintGetCurrentAngle(JoltConstraint constraint)
{
    HingeConstraint *c = static_cast<HingeConstraint *>(constraint);
    return c->GetCurrentAngle();
}

void JoltHingeConstraintSetLimits(JoltConstraint constraint,
                                   float limitsMin, float limitsMax)
{
    HingeConstraint *c = static_cast<HingeConstraint *>(constraint);
    c->SetLimits(limitsMin, limitsMax);
}

void JoltHingeConstraintSetMotorState(JoltConstraint constraint, JoltMotorState state)
{
    HingeConstraint *c = static_cast<HingeConstraint *>(constraint);
    c->SetMotorState(static_cast<EMotorState>(state));
}

int JoltHingeConstraintGetMotorState(JoltConstraint constraint)
{
    HingeConstraint *c = static_cast<HingeConstraint *>(constraint);
    return static_cast<int>(c->GetMotorState());
}

void JoltHingeConstraintSetTargetAngularVelocity(JoltConstraint constraint, float radPerSec)
{
    HingeConstraint *c = static_cast<HingeConstraint *>(constraint);
    c->SetTargetAngularVelocity(radPerSec);
}

float JoltHingeConstraintGetTargetAngularVelocity(JoltConstraint constraint)
{
    HingeConstraint *c = static_cast<HingeConstraint *>(constraint);
    return c->GetTargetAngularVelocity();
}

void JoltHingeConstraintSetTargetAngle(JoltConstraint constraint, float radians)
{
    HingeConstraint *c = static_cast<HingeConstraint *>(constraint);
    c->SetTargetAngle(radians);
}

float JoltHingeConstraintGetTargetAngle(JoltConstraint constraint)
{
    HingeConstraint *c = static_cast<HingeConstraint *>(constraint);
    return c->GetTargetAngle();
}

void JoltHingeConstraintSetMotorSettings(JoltConstraint constraint,
                                          float frequency, float damping,
                                          float forceLimit, float torqueLimit)
{
    HingeConstraint *c = static_cast<HingeConstraint *>(constraint);
    MotorSettings &ms = c->GetMotorSettings();
    ms.mSpringSettings = SpringSettings(ESpringMode::FrequencyAndDamping, frequency, damping);
    ms.SetForceLimit(forceLimit);
    ms.SetTorqueLimit(torqueLimit);
}

void JoltHingeConstraintSetMaxFrictionTorque(JoltConstraint constraint, float torque)
{
    HingeConstraint *c = static_cast<HingeConstraint *>(constraint);
    c->SetMaxFrictionTorque(torque);
}

void JoltHingeConstraintGetTotalLambdaPosition(JoltConstraint constraint,
                                                float *x, float *y, float *z)
{
    HingeConstraint *c = static_cast<HingeConstraint *>(constraint);
    Vec3 lambda = c->GetTotalLambdaPosition();
    *x = lambda.GetX();
    *y = lambda.GetY();
    *z = lambda.GetZ();
}

float JoltHingeConstraintGetTotalLambdaRotationLimits(JoltConstraint constraint)
{
    HingeConstraint *c = static_cast<HingeConstraint *>(constraint);
    return c->GetTotalLambdaRotationLimits();
}

float JoltHingeConstraintGetTotalLambdaMotor(JoltConstraint constraint)
{
    HingeConstraint *c = static_cast<HingeConstraint *>(constraint);
    return c->GetTotalLambdaMotor();
}

// --- Slider Constraint ---

JoltConstraint JoltCreateSliderConstraint(
    JoltPhysicsSystem system,
    JoltBodyID bodyID1, JoltBodyID bodyID2,
    float pointX, float pointY, float pointZ,
    float sliderAxisX, float sliderAxisY, float sliderAxisZ,
    float limitsMin, float limitsMax)
{
    PhysicsSystemWrapper *wrapper = static_cast<PhysicsSystemWrapper *>(system);
    PhysicsSystem *ps = GetPhysicsSystem(wrapper);
    const BodyID *bid1 = static_cast<const BodyID *>(bodyID1);
    const BodyID *bid2 = static_cast<const BodyID *>(bodyID2);

    SliderConstraintSettings settings;
    settings.mSpace = EConstraintSpace::WorldSpace;
    settings.mAutoDetectPoint = false;
    settings.mPoint1 = RVec3(pointX, pointY, pointZ);
    settings.mPoint2 = RVec3(pointX, pointY, pointZ);
    settings.SetSliderAxis(Vec3(sliderAxisX, sliderAxisY, sliderAxisZ));

    if (limitsMin != 0.0f || limitsMax != 0.0f)
    {
        settings.mLimitsMin = limitsMin;
        settings.mLimitsMax = limitsMax;
    }

    BodyLockWrite lock1(ps->GetBodyLockInterface(), *bid1);
    BodyLockWrite lock2(ps->GetBodyLockInterface(), *bid2);
    if (!lock1.Succeeded() || !lock2.Succeeded())
        return nullptr;

    TwoBodyConstraint *constraint = settings.Create(lock1.GetBody(), lock2.GetBody());
    constraint->AddRef();
    return static_cast<JoltConstraint>(constraint);
}

float JoltSliderConstraintGetCurrentPosition(JoltConstraint constraint)
{
    SliderConstraint *c = static_cast<SliderConstraint *>(constraint);
    return c->GetCurrentPosition();
}

void JoltSliderConstraintSetLimits(JoltConstraint constraint,
                                    float limitsMin, float limitsMax)
{
    SliderConstraint *c = static_cast<SliderConstraint *>(constraint);
    c->SetLimits(limitsMin, limitsMax);
}

void JoltSliderConstraintSetMotorState(JoltConstraint constraint, JoltMotorState state)
{
    SliderConstraint *c = static_cast<SliderConstraint *>(constraint);
    c->SetMotorState(static_cast<EMotorState>(state));
}

int JoltSliderConstraintGetMotorState(JoltConstraint constraint)
{
    SliderConstraint *c = static_cast<SliderConstraint *>(constraint);
    return static_cast<int>(c->GetMotorState());
}

void JoltSliderConstraintSetTargetVelocity(JoltConstraint constraint, float velocity)
{
    SliderConstraint *c = static_cast<SliderConstraint *>(constraint);
    c->SetTargetVelocity(velocity);
}

float JoltSliderConstraintGetTargetVelocity(JoltConstraint constraint)
{
    SliderConstraint *c = static_cast<SliderConstraint *>(constraint);
    return c->GetTargetVelocity();
}

void JoltSliderConstraintSetTargetPosition(JoltConstraint constraint, float position)
{
    SliderConstraint *c = static_cast<SliderConstraint *>(constraint);
    c->SetTargetPosition(position);
}

float JoltSliderConstraintGetTargetPosition(JoltConstraint constraint)
{
    SliderConstraint *c = static_cast<SliderConstraint *>(constraint);
    return c->GetTargetPosition();
}

void JoltSliderConstraintSetMotorSettings(JoltConstraint constraint,
                                           float frequency, float damping,
                                           float forceLimit, float torqueLimit)
{
    SliderConstraint *c = static_cast<SliderConstraint *>(constraint);
    MotorSettings &ms = c->GetMotorSettings();
    ms.mSpringSettings = SpringSettings(ESpringMode::FrequencyAndDamping, frequency, damping);
    ms.SetForceLimit(forceLimit);
    ms.SetTorqueLimit(torqueLimit);
}

void JoltSliderConstraintSetMaxFrictionForce(JoltConstraint constraint, float force)
{
    SliderConstraint *c = static_cast<SliderConstraint *>(constraint);
    c->SetMaxFrictionForce(force);
}

float JoltSliderConstraintGetTotalLambdaPositionLimits(JoltConstraint constraint)
{
    SliderConstraint *c = static_cast<SliderConstraint *>(constraint);
    return c->GetTotalLambdaPositionLimits();
}

float JoltSliderConstraintGetTotalLambdaMotor(JoltConstraint constraint)
{
    SliderConstraint *c = static_cast<SliderConstraint *>(constraint);
    return c->GetTotalLambdaMotor();
}

// --- Fixed Constraint ---

JoltConstraint JoltCreateFixedConstraint(
    JoltPhysicsSystem system,
    JoltBodyID bodyID1, JoltBodyID bodyID2,
    float pointX, float pointY, float pointZ)
{
    PhysicsSystemWrapper *wrapper = static_cast<PhysicsSystemWrapper *>(system);
    PhysicsSystem *ps = GetPhysicsSystem(wrapper);
    const BodyID *bid1 = static_cast<const BodyID *>(bodyID1);
    const BodyID *bid2 = static_cast<const BodyID *>(bodyID2);

    FixedConstraintSettings settings;
    settings.mSpace = EConstraintSpace::WorldSpace;
    settings.mAutoDetectPoint = false;
    settings.mPoint1 = RVec3(pointX, pointY, pointZ);
    settings.mPoint2 = RVec3(pointX, pointY, pointZ);

    BodyLockWrite lock1(ps->GetBodyLockInterface(), *bid1);
    BodyLockWrite lock2(ps->GetBodyLockInterface(), *bid2);
    if (!lock1.Succeeded() || !lock2.Succeeded())
        return nullptr;

    TwoBodyConstraint *constraint = settings.Create(lock1.GetBody(), lock2.GetBody());
    constraint->AddRef();
    return static_cast<JoltConstraint>(constraint);
}

void JoltFixedConstraintGetTotalLambdaPosition(JoltConstraint constraint,
                                                float *x, float *y, float *z)
{
    FixedConstraint *c = static_cast<FixedConstraint *>(constraint);
    Vec3 lambda = c->GetTotalLambdaPosition();
    *x = lambda.GetX();
    *y = lambda.GetY();
    *z = lambda.GetZ();
}

void JoltFixedConstraintGetTotalLambdaRotation(JoltConstraint constraint,
                                                float *x, float *y, float *z)
{
    FixedConstraint *c = static_cast<FixedConstraint *>(constraint);
    Vec3 lambda = c->GetTotalLambdaRotation();
    *x = lambda.GetX();
    *y = lambda.GetY();
    *z = lambda.GetZ();
}

// --- Buoyancy ---

int JoltApplyBuoyancyImpulse(
    JoltPhysicsSystem system, JoltBodyID bodyID,
    float surfacePosX, float surfacePosY, float surfacePosZ,
    float surfaceNormalX, float surfaceNormalY, float surfaceNormalZ,
    float buoyancy, float linearDrag, float angularDrag,
    float fluidVelX, float fluidVelY, float fluidVelZ,
    float gravityX, float gravityY, float gravityZ,
    float deltaTime)
{
    PhysicsSystemWrapper *wrapper = static_cast<PhysicsSystemWrapper *>(system);
    PhysicsSystem *ps = GetPhysicsSystem(wrapper);
    const BodyID *bid = static_cast<const BodyID *>(bodyID);

    BodyLockWrite lock(ps->GetBodyLockInterface(), *bid);
    if (!lock.Succeeded())
        return 0;

    Body &body = lock.GetBody();
    bool applied = body.ApplyBuoyancyImpulse(
        RVec3(surfacePosX, surfacePosY, surfacePosZ),
        Vec3(surfaceNormalX, surfaceNormalY, surfaceNormalZ),
        buoyancy, linearDrag, angularDrag,
        Vec3(fluidVelX, fluidVelY, fluidVelZ),
        Vec3(gravityX, gravityY, gravityZ),
        deltaTime);

    return applied ? 1 : 0;
}

int JoltApplyBuoyancyImpulseWithVolume(
    JoltPhysicsSystem system, JoltBodyID bodyID,
    float totalVolume, float submergedVolume,
    float relativeCoBX, float relativeCoBY, float relativeCoBZ,
    float buoyancy, float linearDrag, float angularDrag,
    float fluidVelX, float fluidVelY, float fluidVelZ,
    float gravityX, float gravityY, float gravityZ,
    float deltaTime)
{
    PhysicsSystemWrapper *wrapper = static_cast<PhysicsSystemWrapper *>(system);
    PhysicsSystem *ps = GetPhysicsSystem(wrapper);
    const BodyID *bid = static_cast<const BodyID *>(bodyID);

    BodyLockWrite lock(ps->GetBodyLockInterface(), *bid);
    if (!lock.Succeeded())
        return 0;

    Body &body = lock.GetBody();
    bool applied = body.ApplyBuoyancyImpulse(
        totalVolume, submergedVolume,
        Vec3(relativeCoBX, relativeCoBY, relativeCoBZ),
        buoyancy, linearDrag, angularDrag,
        Vec3(fluidVelX, fluidVelY, fluidVelZ),
        Vec3(gravityX, gravityY, gravityZ),
        deltaTime);

    return applied ? 1 : 0;
}

void JoltGetSubmergedVolume(
    JoltPhysicsSystem system, JoltBodyID bodyID,
    float surfacePosX, float surfacePosY, float surfacePosZ,
    float surfaceNormalX, float surfaceNormalY, float surfaceNormalZ,
    float *outTotalVolume, float *outSubmergedVolume,
    float *outRelCoBX, float *outRelCoBY, float *outRelCoBZ)
{
    PhysicsSystemWrapper *wrapper = static_cast<PhysicsSystemWrapper *>(system);
    PhysicsSystem *ps = GetPhysicsSystem(wrapper);
    const BodyID *bid = static_cast<const BodyID *>(bodyID);

    BodyLockRead lock(ps->GetBodyLockInterface(), *bid);
    if (!lock.Succeeded())
    {
        *outTotalVolume = 0;
        *outSubmergedVolume = 0;
        *outRelCoBX = 0;
        *outRelCoBY = 0;
        *outRelCoBZ = 0;
        return;
    }

    const Body &body = lock.GetBody();
    float totalVol = 0, submergedVol = 0;
    Vec3 relCoB = Vec3::sZero();

    body.GetSubmergedVolume(
        RVec3(surfacePosX, surfacePosY, surfacePosZ),
        Vec3(surfaceNormalX, surfaceNormalY, surfaceNormalZ),
        totalVol, submergedVol, relCoB);

    *outTotalVolume = totalVol;
    *outSubmergedVolume = submergedVol;
    *outRelCoBX = relCoB.GetX();
    *outRelCoBY = relCoB.GetY();
    *outRelCoBZ = relCoB.GetZ();
}
