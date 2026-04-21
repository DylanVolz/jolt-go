/*
 * Jolt Physics C Wrapper - Constraints & Buoyancy Implementation (T-0105, T-0122, T-0123)
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
#include <Jolt/Physics/Constraints/PointConstraint.h>
#include <Jolt/Physics/Constraints/ConeConstraint.h>
#include <Jolt/Physics/Constraints/SwingTwistConstraint.h>
#include <Jolt/Physics/Constraints/SixDOFConstraint.h>

using namespace JPH;

#include <Jolt/Physics/Constraints/PulleyConstraint.h>
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

// --- SwingTwist Constraint (T-0123) ---

JoltConstraint JoltCreateSwingTwistConstraint(
    JoltPhysicsSystem system,
    JoltBodyID bodyID1, JoltBodyID bodyID2,
    float positionX, float positionY, float positionZ,
    float twistAxisX, float twistAxisY, float twistAxisZ,
    float planeAxisX, float planeAxisY, float planeAxisZ,
    float normalHalfConeAngle, float planeHalfConeAngle,
    float twistMinAngle, float twistMaxAngle,
    JoltSwingType swingType)
{
    PhysicsSystemWrapper *wrapper = static_cast<PhysicsSystemWrapper *>(system);
    PhysicsSystem *ps = GetPhysicsSystem(wrapper);
    const BodyID *bid1 = static_cast<const BodyID *>(bodyID1);
    const BodyID *bid2 = static_cast<const BodyID *>(bodyID2);

    SwingTwistConstraintSettings settings;
    settings.mSpace = EConstraintSpace::WorldSpace;
    RVec3 pos(positionX, positionY, positionZ);
    Vec3 twist(twistAxisX, twistAxisY, twistAxisZ);
    Vec3 plane(planeAxisX, planeAxisY, planeAxisZ);
    settings.mPosition1 = pos;
    settings.mPosition2 = pos;
    settings.mTwistAxis1 = twist;
    settings.mTwistAxis2 = twist;
    settings.mPlaneAxis1 = plane;
    settings.mPlaneAxis2 = plane;
    settings.mNormalHalfConeAngle = normalHalfConeAngle;
    settings.mPlaneHalfConeAngle = planeHalfConeAngle;
    settings.mTwistMinAngle = twistMinAngle;
    settings.mTwistMaxAngle = twistMaxAngle;
    settings.mSwingType = static_cast<ESwingType>(swingType);

    BodyLockWrite lock1(ps->GetBodyLockInterface(), *bid1);
    BodyLockWrite lock2(ps->GetBodyLockInterface(), *bid2);
    if (!lock1.Succeeded() || !lock2.Succeeded())
        return nullptr;

    TwoBodyConstraint *constraint = settings.Create(lock1.GetBody(), lock2.GetBody());
    constraint->AddRef();
    return static_cast<JoltConstraint>(constraint);
}

void JoltSwingTwistSetNormalHalfConeAngle(JoltConstraint c, float angle)
{
    static_cast<SwingTwistConstraint *>(c)->SetNormalHalfConeAngle(angle);
}

float JoltSwingTwistGetNormalHalfConeAngle(JoltConstraint c)
{
    return static_cast<SwingTwistConstraint *>(c)->GetNormalHalfConeAngle();
}

void JoltSwingTwistSetPlaneHalfConeAngle(JoltConstraint c, float angle)
{
    static_cast<SwingTwistConstraint *>(c)->SetPlaneHalfConeAngle(angle);
}

float JoltSwingTwistGetPlaneHalfConeAngle(JoltConstraint c)
{
    return static_cast<SwingTwistConstraint *>(c)->GetPlaneHalfConeAngle();
}

void JoltSwingTwistSetTwistMinAngle(JoltConstraint c, float angle)
{
    static_cast<SwingTwistConstraint *>(c)->SetTwistMinAngle(angle);
}

float JoltSwingTwistGetTwistMinAngle(JoltConstraint c)
{
    return static_cast<SwingTwistConstraint *>(c)->GetTwistMinAngle();
}

void JoltSwingTwistSetTwistMaxAngle(JoltConstraint c, float angle)
{
    static_cast<SwingTwistConstraint *>(c)->SetTwistMaxAngle(angle);
}

float JoltSwingTwistGetTwistMaxAngle(JoltConstraint c)
{
    return static_cast<SwingTwistConstraint *>(c)->GetTwistMaxAngle();
}

void JoltSwingTwistSetMaxFrictionTorque(JoltConstraint c, float torque)
{
    static_cast<SwingTwistConstraint *>(c)->SetMaxFrictionTorque(torque);
}

float JoltSwingTwistGetMaxFrictionTorque(JoltConstraint c)
{
    return static_cast<SwingTwistConstraint *>(c)->GetMaxFrictionTorque();
}

void JoltSwingTwistSetSwingMotorState(JoltConstraint c, JoltMotorState state)
{
    static_cast<SwingTwistConstraint *>(c)->SetSwingMotorState(static_cast<EMotorState>(state));
}

int JoltSwingTwistGetSwingMotorState(JoltConstraint c)
{
    return static_cast<int>(static_cast<SwingTwistConstraint *>(c)->GetSwingMotorState());
}

void JoltSwingTwistSetTwistMotorState(JoltConstraint c, JoltMotorState state)
{
    static_cast<SwingTwistConstraint *>(c)->SetTwistMotorState(static_cast<EMotorState>(state));
}

int JoltSwingTwistGetTwistMotorState(JoltConstraint c)
{
    return static_cast<int>(static_cast<SwingTwistConstraint *>(c)->GetTwistMotorState());
}

void JoltSwingTwistSetTargetAngularVelocityCS(JoltConstraint c, float x, float y, float z)
{
    static_cast<SwingTwistConstraint *>(c)->SetTargetAngularVelocityCS(Vec3(x, y, z));
}

void JoltSwingTwistGetTargetAngularVelocityCS(JoltConstraint c, float *x, float *y, float *z)
{
    Vec3 v = static_cast<SwingTwistConstraint *>(c)->GetTargetAngularVelocityCS();
    *x = v.GetX(); *y = v.GetY(); *z = v.GetZ();
}

void JoltSwingTwistSetTargetOrientationCS(JoltConstraint c, float qx, float qy, float qz, float qw)
{
    static_cast<SwingTwistConstraint *>(c)->SetTargetOrientationCS(Quat(qx, qy, qz, qw));
}

void JoltSwingTwistGetTargetOrientationCS(JoltConstraint c, float *qx, float *qy, float *qz, float *qw)
{
    Quat q = static_cast<SwingTwistConstraint *>(c)->GetTargetOrientationCS();
    *qx = q.GetX(); *qy = q.GetY(); *qz = q.GetZ(); *qw = q.GetW();
}

void JoltSwingTwistGetRotationInConstraintSpace(JoltConstraint c,
                                                 float *qx, float *qy, float *qz, float *qw)
{
    Quat q = static_cast<SwingTwistConstraint *>(c)->GetRotationInConstraintSpace();
    *qx = q.GetX(); *qy = q.GetY(); *qz = q.GetZ(); *qw = q.GetW();
}

void JoltSwingTwistSetSwingMotorSettings(JoltConstraint c,
                                          float frequency, float damping,
                                          float forceLimit, float torqueLimit)
{
    SwingTwistConstraint *st = static_cast<SwingTwistConstraint *>(c);
    MotorSettings &ms = st->GetSwingMotorSettings();
    ms.mSpringSettings = SpringSettings(ESpringMode::FrequencyAndDamping, frequency, damping);
    ms.SetForceLimit(forceLimit);
    ms.SetTorqueLimit(torqueLimit);
}

void JoltSwingTwistSetTwistMotorSettings(JoltConstraint c,
                                          float frequency, float damping,
                                          float forceLimit, float torqueLimit)
{
    SwingTwistConstraint *st = static_cast<SwingTwistConstraint *>(c);
    MotorSettings &ms = st->GetTwistMotorSettings();
    ms.mSpringSettings = SpringSettings(ESpringMode::FrequencyAndDamping, frequency, damping);
    ms.SetForceLimit(forceLimit);
    ms.SetTorqueLimit(torqueLimit);
}

void JoltSwingTwistGetTotalLambdaPosition(JoltConstraint c, float *x, float *y, float *z)
{
    Vec3 v = static_cast<SwingTwistConstraint *>(c)->GetTotalLambdaPosition();
    *x = v.GetX(); *y = v.GetY(); *z = v.GetZ();
}

float JoltSwingTwistGetTotalLambdaTwist(JoltConstraint c)
{
    return static_cast<SwingTwistConstraint *>(c)->GetTotalLambdaTwist();
}

float JoltSwingTwistGetTotalLambdaSwingY(JoltConstraint c)
{
    return static_cast<SwingTwistConstraint *>(c)->GetTotalLambdaSwingY();
}

float JoltSwingTwistGetTotalLambdaSwingZ(JoltConstraint c)
{
    return static_cast<SwingTwistConstraint *>(c)->GetTotalLambdaSwingZ();
}

void JoltSwingTwistGetTotalLambdaMotor(JoltConstraint c, float *x, float *y, float *z)
{
    Vec3 v = static_cast<SwingTwistConstraint *>(c)->GetTotalLambdaMotor();
    *x = v.GetX(); *y = v.GetY(); *z = v.GetZ();
}

// --- SixDOF Constraint (T-0123) ---

static SixDOFConstraintSettings::EAxis ToSixDOFAxis(JoltSixDOFAxis axis)
{
    return static_cast<SixDOFConstraintSettings::EAxis>(axis);
}

JoltConstraint JoltCreateSixDOFConstraint(
    JoltPhysicsSystem system,
    JoltBodyID bodyID1, JoltBodyID bodyID2,
    float positionX, float positionY, float positionZ,
    float axisXx, float axisXy, float axisXz,
    float axisYx, float axisYy, float axisYz,
    const float *limitMin,
    const float *limitMax,
    JoltSwingType swingType)
{
    PhysicsSystemWrapper *wrapper = static_cast<PhysicsSystemWrapper *>(system);
    PhysicsSystem *ps = GetPhysicsSystem(wrapper);
    const BodyID *bid1 = static_cast<const BodyID *>(bodyID1);
    const BodyID *bid2 = static_cast<const BodyID *>(bodyID2);

    SixDOFConstraintSettings settings;
    settings.mSpace = EConstraintSpace::WorldSpace;
    RVec3 pos(positionX, positionY, positionZ);
    Vec3 ax(axisXx, axisXy, axisXz);
    Vec3 ay(axisYx, axisYy, axisYz);
    settings.mPosition1 = pos;
    settings.mPosition2 = pos;
    settings.mAxisX1 = ax;
    settings.mAxisX2 = ax;
    settings.mAxisY1 = ay;
    settings.mAxisY2 = ay;
    settings.mSwingType = static_cast<ESwingType>(swingType);

    for (int i = 0; i < SixDOFConstraintSettings::EAxis::Num; ++i)
    {
        settings.mLimitMin[i] = limitMin[i];
        settings.mLimitMax[i] = limitMax[i];
    }

    BodyLockWrite lock1(ps->GetBodyLockInterface(), *bid1);
    BodyLockWrite lock2(ps->GetBodyLockInterface(), *bid2);
    if (!lock1.Succeeded() || !lock2.Succeeded())
        return nullptr;

    TwoBodyConstraint *constraint = settings.Create(lock1.GetBody(), lock2.GetBody());
    constraint->AddRef();
    return static_cast<JoltConstraint>(constraint);
}

void JoltSixDOFSetTranslationLimits(JoltConstraint c,
                                     float minX, float minY, float minZ,
                                     float maxX, float maxY, float maxZ)
{
    static_cast<SixDOFConstraint *>(c)->SetTranslationLimits(
        Vec3(minX, minY, minZ), Vec3(maxX, maxY, maxZ));
}

void JoltSixDOFSetRotationLimits(JoltConstraint c,
                                  float minX, float minY, float minZ,
                                  float maxX, float maxY, float maxZ)
{
    static_cast<SixDOFConstraint *>(c)->SetRotationLimits(
        Vec3(minX, minY, minZ), Vec3(maxX, maxY, maxZ));
}

float JoltSixDOFGetLimitsMin(JoltConstraint c, JoltSixDOFAxis axis)
{
    return static_cast<SixDOFConstraint *>(c)->GetLimitsMin(ToSixDOFAxis(axis));
}

float JoltSixDOFGetLimitsMax(JoltConstraint c, JoltSixDOFAxis axis)
{
    return static_cast<SixDOFConstraint *>(c)->GetLimitsMax(ToSixDOFAxis(axis));
}

int JoltSixDOFIsFixedAxis(JoltConstraint c, JoltSixDOFAxis axis)
{
    return static_cast<SixDOFConstraint *>(c)->IsFixedAxis(ToSixDOFAxis(axis)) ? 1 : 0;
}

int JoltSixDOFIsFreeAxis(JoltConstraint c, JoltSixDOFAxis axis)
{
    return static_cast<SixDOFConstraint *>(c)->IsFreeAxis(ToSixDOFAxis(axis)) ? 1 : 0;
}

void JoltSixDOFSetMaxFriction(JoltConstraint c, JoltSixDOFAxis axis, float friction)
{
    static_cast<SixDOFConstraint *>(c)->SetMaxFriction(ToSixDOFAxis(axis), friction);
}

float JoltSixDOFGetMaxFriction(JoltConstraint c, JoltSixDOFAxis axis)
{
    return static_cast<SixDOFConstraint *>(c)->GetMaxFriction(ToSixDOFAxis(axis));
}

void JoltSixDOFSetMotorState(JoltConstraint c, JoltSixDOFAxis axis, JoltMotorState state)
{
    static_cast<SixDOFConstraint *>(c)->SetMotorState(ToSixDOFAxis(axis),
                                                       static_cast<EMotorState>(state));
}

int JoltSixDOFGetMotorState(JoltConstraint c, JoltSixDOFAxis axis)
{
    return static_cast<int>(static_cast<SixDOFConstraint *>(c)->GetMotorState(ToSixDOFAxis(axis)));
}

void JoltSixDOFSetMotorSettings(JoltConstraint c, JoltSixDOFAxis axis,
                                 float frequency, float damping,
                                 float forceLimit, float torqueLimit)
{
    SixDOFConstraint *sd = static_cast<SixDOFConstraint *>(c);
    MotorSettings &ms = sd->GetMotorSettings(ToSixDOFAxis(axis));
    ms.mSpringSettings = SpringSettings(ESpringMode::FrequencyAndDamping, frequency, damping);
    ms.SetForceLimit(forceLimit);
    ms.SetTorqueLimit(torqueLimit);
}

void JoltSixDOFSetTargetVelocityCS(JoltConstraint c, float x, float y, float z)
{
    static_cast<SixDOFConstraint *>(c)->SetTargetVelocityCS(Vec3(x, y, z));
}

void JoltSixDOFGetTargetVelocityCS(JoltConstraint c, float *x, float *y, float *z)
{
    Vec3 v = static_cast<SixDOFConstraint *>(c)->GetTargetVelocityCS();
    *x = v.GetX(); *y = v.GetY(); *z = v.GetZ();
}

void JoltSixDOFSetTargetAngularVelocityCS(JoltConstraint c, float x, float y, float z)
{
    static_cast<SixDOFConstraint *>(c)->SetTargetAngularVelocityCS(Vec3(x, y, z));
}

void JoltSixDOFGetTargetAngularVelocityCS(JoltConstraint c, float *x, float *y, float *z)
{
    Vec3 v = static_cast<SixDOFConstraint *>(c)->GetTargetAngularVelocityCS();
    *x = v.GetX(); *y = v.GetY(); *z = v.GetZ();
}

void JoltSixDOFSetTargetPositionCS(JoltConstraint c, float x, float y, float z)
{
    static_cast<SixDOFConstraint *>(c)->SetTargetPositionCS(Vec3(x, y, z));
}

void JoltSixDOFGetTargetPositionCS(JoltConstraint c, float *x, float *y, float *z)
{
    Vec3 v = static_cast<SixDOFConstraint *>(c)->GetTargetPositionCS();
    *x = v.GetX(); *y = v.GetY(); *z = v.GetZ();
}

void JoltSixDOFSetTargetOrientationCS(JoltConstraint c,
                                       float qx, float qy, float qz, float qw)
{
    static_cast<SixDOFConstraint *>(c)->SetTargetOrientationCS(Quat(qx, qy, qz, qw));
}

void JoltSixDOFGetTargetOrientationCS(JoltConstraint c,
                                       float *qx, float *qy, float *qz, float *qw)
{
    Quat q = static_cast<SixDOFConstraint *>(c)->GetTargetOrientationCS();
    *qx = q.GetX(); *qy = q.GetY(); *qz = q.GetZ(); *qw = q.GetW();
}

void JoltSixDOFGetRotationInConstraintSpace(JoltConstraint c,
                                             float *qx, float *qy, float *qz, float *qw)
{
    Quat q = static_cast<SixDOFConstraint *>(c)->GetRotationInConstraintSpace();
    *qx = q.GetX(); *qy = q.GetY(); *qz = q.GetZ(); *qw = q.GetW();
}

void JoltSixDOFGetTotalLambdaPosition(JoltConstraint c, float *x, float *y, float *z)
{
    Vec3 v = static_cast<SixDOFConstraint *>(c)->GetTotalLambdaPosition();
    *x = v.GetX(); *y = v.GetY(); *z = v.GetZ();
}

void JoltSixDOFGetTotalLambdaRotation(JoltConstraint c, float *x, float *y, float *z)
{
    Vec3 v = static_cast<SixDOFConstraint *>(c)->GetTotalLambdaRotation();
    *x = v.GetX(); *y = v.GetY(); *z = v.GetZ();
}

void JoltSixDOFGetTotalLambdaMotorTranslation(JoltConstraint c, float *x, float *y, float *z)
{
    Vec3 v = static_cast<SixDOFConstraint *>(c)->GetTotalLambdaMotorTranslation();
    *x = v.GetX(); *y = v.GetY(); *z = v.GetZ();
}

void JoltSixDOFGetTotalLambdaMotorRotation(JoltConstraint c, float *x, float *y, float *z)
{
    Vec3 v = static_cast<SixDOFConstraint *>(c)->GetTotalLambdaMotorRotation();
    *x = v.GetX(); *y = v.GetY(); *z = v.GetZ();
}

// --- Point Constraint (T-0122) ---

JoltConstraint JoltCreatePointConstraint(
    JoltPhysicsSystem system,
    JoltBodyID bodyID1, JoltBodyID bodyID2,
    float pointX, float pointY, float pointZ)
{
    PhysicsSystemWrapper *wrapper = static_cast<PhysicsSystemWrapper *>(system);
    PhysicsSystem *ps = GetPhysicsSystem(wrapper);
    const BodyID *bid1 = static_cast<const BodyID *>(bodyID1);
    const BodyID *bid2 = static_cast<const BodyID *>(bodyID2);

    PointConstraintSettings settings;
    settings.mSpace = EConstraintSpace::WorldSpace;
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

void JoltPointConstraintGetTotalLambdaPosition(JoltConstraint constraint,
                                                float *x, float *y, float *z)
{
    PointConstraint *c = static_cast<PointConstraint *>(constraint);
    Vec3 lambda = c->GetTotalLambdaPosition();
    *x = lambda.GetX();
    *y = lambda.GetY();
    *z = lambda.GetZ();
}

// --- Cone Constraint (T-0122) ---

JoltConstraint JoltCreateConeConstraint(
    JoltPhysicsSystem system,
    JoltBodyID bodyID1, JoltBodyID bodyID2,
    float pointX, float pointY, float pointZ,
    float twistAxisX, float twistAxisY, float twistAxisZ,
    float halfAngleMax)
{
    PhysicsSystemWrapper *wrapper = static_cast<PhysicsSystemWrapper *>(system);
    PhysicsSystem *ps = GetPhysicsSystem(wrapper);
    const BodyID *bid1 = static_cast<const BodyID *>(bodyID1);
    const BodyID *bid2 = static_cast<const BodyID *>(bodyID2);

    ConeConstraintSettings settings;
    settings.mSpace = EConstraintSpace::WorldSpace;
    settings.mPoint1 = RVec3(pointX, pointY, pointZ);
    settings.mPoint2 = RVec3(pointX, pointY, pointZ);
    settings.mTwistAxis1 = Vec3(twistAxisX, twistAxisY, twistAxisZ);
    settings.mTwistAxis2 = Vec3(twistAxisX, twistAxisY, twistAxisZ);
    settings.mHalfConeAngle = halfAngleMax;

    BodyLockWrite lock1(ps->GetBodyLockInterface(), *bid1);
    BodyLockWrite lock2(ps->GetBodyLockInterface(), *bid2);
    if (!lock1.Succeeded() || !lock2.Succeeded())
        return nullptr;

    TwoBodyConstraint *constraint = settings.Create(lock1.GetBody(), lock2.GetBody());
    constraint->AddRef();
    return static_cast<JoltConstraint>(constraint);
}

void JoltConeConstraintSetHalfConeAngle(JoltConstraint constraint, float halfAngle)
{
    ConeConstraint *c = static_cast<ConeConstraint *>(constraint);
    c->SetHalfConeAngle(halfAngle);
}

float JoltConeConstraintGetCosHalfConeAngle(JoltConstraint constraint)
{
    ConeConstraint *c = static_cast<ConeConstraint *>(constraint);
    return c->GetCosHalfConeAngle();
}

void JoltConeConstraintGetTotalLambdaPosition(JoltConstraint constraint,
                                               float *x, float *y, float *z)
{
    ConeConstraint *c = static_cast<ConeConstraint *>(constraint);
    Vec3 lambda = c->GetTotalLambdaPosition();
    *x = lambda.GetX();
    *y = lambda.GetY();
    *z = lambda.GetZ();
}

float JoltConeConstraintGetTotalLambdaRotation(JoltConstraint constraint)
{
    ConeConstraint *c = static_cast<ConeConstraint *>(constraint);
    return c->GetTotalLambdaRotation();
}

// --- Pulley Constraint ---

JoltConstraint JoltCreatePulleyConstraint(
    JoltPhysicsSystem system,
    JoltBodyID bodyID1, JoltBodyID bodyID2,
    float bodyPoint1X, float bodyPoint1Y, float bodyPoint1Z,
    float bodyPoint2X, float bodyPoint2Y, float bodyPoint2Z,
    float fixedPoint1X, float fixedPoint1Y, float fixedPoint1Z,
    float fixedPoint2X, float fixedPoint2Y, float fixedPoint2Z,
    float ratio,
    float minLength, float maxLength)
{
    PhysicsSystemWrapper *wrapper = static_cast<PhysicsSystemWrapper *>(system);
    PhysicsSystem *ps = GetPhysicsSystem(wrapper);
    const BodyID *bid1 = static_cast<const BodyID *>(bodyID1);
    const BodyID *bid2 = static_cast<const BodyID *>(bodyID2);

    PulleyConstraintSettings settings;
    settings.mSpace = EConstraintSpace::WorldSpace;
    settings.mBodyPoint1 = RVec3(bodyPoint1X, bodyPoint1Y, bodyPoint1Z);
    settings.mBodyPoint2 = RVec3(bodyPoint2X, bodyPoint2Y, bodyPoint2Z);
    settings.mFixedPoint1 = RVec3(fixedPoint1X, fixedPoint1Y, fixedPoint1Z);
    settings.mFixedPoint2 = RVec3(fixedPoint2X, fixedPoint2Y, fixedPoint2Z);
    settings.mRatio = ratio;
    settings.mMinLength = minLength;
    settings.mMaxLength = maxLength;

    BodyLockWrite lock1(ps->GetBodyLockInterface(), *bid1);
    BodyLockWrite lock2(ps->GetBodyLockInterface(), *bid2);
    if (!lock1.Succeeded() || !lock2.Succeeded())
        return nullptr;

    TwoBodyConstraint *constraint = settings.Create(lock1.GetBody(), lock2.GetBody());
    constraint->AddRef();
    return static_cast<JoltConstraint>(constraint);
}

void JoltPulleyConstraintSetLength(JoltConstraint constraint,
                                    float minLength, float maxLength)
{
    PulleyConstraint *c = static_cast<PulleyConstraint *>(constraint);
    c->SetLength(minLength, maxLength);
}

float JoltPulleyConstraintGetMinLength(JoltConstraint constraint)
{
    PulleyConstraint *c = static_cast<PulleyConstraint *>(constraint);
    return c->GetMinLength();
}

float JoltPulleyConstraintGetMaxLength(JoltConstraint constraint)
{
    PulleyConstraint *c = static_cast<PulleyConstraint *>(constraint);
    return c->GetMaxLength();
}

float JoltPulleyConstraintGetCurrentLength(JoltConstraint constraint)
{
    PulleyConstraint *c = static_cast<PulleyConstraint *>(constraint);
    return c->GetCurrentLength();
}

float JoltPulleyConstraintGetTotalLambdaPosition(JoltConstraint constraint)
{
    PulleyConstraint *c = static_cast<PulleyConstraint *>(constraint);
    return c->GetTotalLambdaPosition();
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
