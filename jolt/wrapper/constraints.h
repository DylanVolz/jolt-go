/*
 * Jolt Physics C Wrapper - Constraints & Buoyancy (T-0105, T-0122, T-0123)
 *
 * Constraint types: Distance, Hinge, Slider, Fixed, SwingTwist, SixDOF
 * Motor control for Hinge, Slider, SwingTwist, and SixDOF constraints
 * Buoyancy impulse application via Body
 */

#ifndef JOLT_WRAPPER_CONSTRAINTS_H
#define JOLT_WRAPPER_CONSTRAINTS_H

#include "physics.h"
#include "body.h"

#ifdef __cplusplus
extern "C" {
#endif

// --- Opaque pointer types ---
typedef void* JoltConstraint;
typedef void* JoltConstraintSettings;

// --- Motor state enum (matches Jolt EMotorState) ---
typedef enum {
    JoltMotorStateOff = 0,
    JoltMotorStateVelocity = 1,
    JoltMotorStatePosition = 2
} JoltMotorState;

// --- PhysicsSystem constraint management ---

void JoltAddConstraint(JoltPhysicsSystem system, JoltConstraint constraint);
void JoltRemoveConstraint(JoltPhysicsSystem system, JoltConstraint constraint);
void JoltDestroyConstraint(JoltConstraint constraint);

// --- Distance Constraint ---

// Create a distance constraint between two bodies.
// point1/point2 are world-space attachment points.
// minDistance/maxDistance: set both to -1 for auto-detect from initial positions.
JoltConstraint JoltCreateDistanceConstraint(
    JoltPhysicsSystem system,
    JoltBodyID bodyID1, JoltBodyID bodyID2,
    float point1X, float point1Y, float point1Z,
    float point2X, float point2Y, float point2Z,
    float minDistance, float maxDistance);

void JoltDistanceConstraintSetDistance(JoltConstraint constraint,
                                        float minDistance, float maxDistance);
float JoltDistanceConstraintGetMinDistance(JoltConstraint constraint);
float JoltDistanceConstraintGetMaxDistance(JoltConstraint constraint);

// Spring settings for distance constraint limits
void JoltDistanceConstraintSetSpring(JoltConstraint constraint,
                                      float frequency, float damping);

// Force readback (impulse applied this step)
float JoltDistanceConstraintGetTotalLambdaPosition(JoltConstraint constraint);

// --- Hinge Constraint ---

// Create a hinge constraint between two bodies.
// point: world-space hinge pivot
// hingeAxis: world-space rotation axis (normalized)
// normalAxis: world-space normal axis perpendicular to hinge (normalized)
// limitsMin/limitsMax: angle limits in radians (-PI to PI). Set both to 0 for no limits.
JoltConstraint JoltCreateHingeConstraint(
    JoltPhysicsSystem system,
    JoltBodyID bodyID1, JoltBodyID bodyID2,
    float pointX, float pointY, float pointZ,
    float hingeAxisX, float hingeAxisY, float hingeAxisZ,
    float normalAxisX, float normalAxisY, float normalAxisZ,
    float limitsMin, float limitsMax);

float JoltHingeConstraintGetCurrentAngle(JoltConstraint constraint);

void JoltHingeConstraintSetLimits(JoltConstraint constraint,
                                   float limitsMin, float limitsMax);

// Motor control
void JoltHingeConstraintSetMotorState(JoltConstraint constraint, JoltMotorState state);
int JoltHingeConstraintGetMotorState(JoltConstraint constraint);
void JoltHingeConstraintSetTargetAngularVelocity(JoltConstraint constraint, float radPerSec);
float JoltHingeConstraintGetTargetAngularVelocity(JoltConstraint constraint);
void JoltHingeConstraintSetTargetAngle(JoltConstraint constraint, float radians);
float JoltHingeConstraintGetTargetAngle(JoltConstraint constraint);

// Motor settings
void JoltHingeConstraintSetMotorSettings(JoltConstraint constraint,
                                          float frequency, float damping,
                                          float forceLimit, float torqueLimit);
void JoltHingeConstraintSetMaxFrictionTorque(JoltConstraint constraint, float torque);

// Force readback
void JoltHingeConstraintGetTotalLambdaPosition(JoltConstraint constraint,
                                                float* x, float* y, float* z);
float JoltHingeConstraintGetTotalLambdaRotationLimits(JoltConstraint constraint);
float JoltHingeConstraintGetTotalLambdaMotor(JoltConstraint constraint);

// --- Slider Constraint ---

// Create a slider (prismatic) constraint between two bodies.
// sliderAxis: world-space axis along which sliding occurs (normalized)
// limitsMin/limitsMax: position limits along axis. Set both to 0 for no limits.
JoltConstraint JoltCreateSliderConstraint(
    JoltPhysicsSystem system,
    JoltBodyID bodyID1, JoltBodyID bodyID2,
    float pointX, float pointY, float pointZ,
    float sliderAxisX, float sliderAxisY, float sliderAxisZ,
    float limitsMin, float limitsMax);

float JoltSliderConstraintGetCurrentPosition(JoltConstraint constraint);

void JoltSliderConstraintSetLimits(JoltConstraint constraint,
                                    float limitsMin, float limitsMax);

// Motor control
void JoltSliderConstraintSetMotorState(JoltConstraint constraint, JoltMotorState state);
int JoltSliderConstraintGetMotorState(JoltConstraint constraint);
void JoltSliderConstraintSetTargetVelocity(JoltConstraint constraint, float velocity);
float JoltSliderConstraintGetTargetVelocity(JoltConstraint constraint);
void JoltSliderConstraintSetTargetPosition(JoltConstraint constraint, float position);
float JoltSliderConstraintGetTargetPosition(JoltConstraint constraint);

// Motor settings
void JoltSliderConstraintSetMotorSettings(JoltConstraint constraint,
                                           float frequency, float damping,
                                           float forceLimit, float torqueLimit);
void JoltSliderConstraintSetMaxFrictionForce(JoltConstraint constraint, float force);

// Force readback
float JoltSliderConstraintGetTotalLambdaPositionLimits(JoltConstraint constraint);
float JoltSliderConstraintGetTotalLambdaMotor(JoltConstraint constraint);

// --- Fixed Constraint ---

// Create a fixed constraint between two bodies at a world-space point.
JoltConstraint JoltCreateFixedConstraint(
    JoltPhysicsSystem system,
    JoltBodyID bodyID1, JoltBodyID bodyID2,
    float pointX, float pointY, float pointZ);

// Force readback
void JoltFixedConstraintGetTotalLambdaPosition(JoltConstraint constraint,
                                                float* x, float* y, float* z);
void JoltFixedConstraintGetTotalLambdaRotation(JoltConstraint constraint,
                                                float* x, float* y, float* z);

// --- SwingTwist Constraint (T-0123) ---

// Matches Jolt ESwingType
typedef enum {
    JoltSwingTypeCone = 0,
    JoltSwingTypePyramid = 1
} JoltSwingType;

// Create a swing-twist constraint between two bodies (humanoid joint).
// position: world-space attachment point (used for both bodies)
// twistAxis: world-space twist axis (normalized) - shared by both bodies
// planeAxis: world-space plane axis perpendicular to twist (normalized) - shared
// normalHalfConeAngle/planeHalfConeAngle: swing limits in radians
// twistMinAngle/twistMaxAngle: twist limits in radians, in [-PI, PI]
JoltConstraint JoltCreateSwingTwistConstraint(
    JoltPhysicsSystem system,
    JoltBodyID bodyID1, JoltBodyID bodyID2,
    float positionX, float positionY, float positionZ,
    float twistAxisX, float twistAxisY, float twistAxisZ,
    float planeAxisX, float planeAxisY, float planeAxisZ,
    float normalHalfConeAngle, float planeHalfConeAngle,
    float twistMinAngle, float twistMaxAngle,
    JoltSwingType swingType);

// Limit accessors
void JoltSwingTwistSetNormalHalfConeAngle(JoltConstraint c, float angle);
float JoltSwingTwistGetNormalHalfConeAngle(JoltConstraint c);
void JoltSwingTwistSetPlaneHalfConeAngle(JoltConstraint c, float angle);
float JoltSwingTwistGetPlaneHalfConeAngle(JoltConstraint c);
void JoltSwingTwistSetTwistMinAngle(JoltConstraint c, float angle);
float JoltSwingTwistGetTwistMinAngle(JoltConstraint c);
void JoltSwingTwistSetTwistMaxAngle(JoltConstraint c, float angle);
float JoltSwingTwistGetTwistMaxAngle(JoltConstraint c);

// Friction (when motor is off)
void JoltSwingTwistSetMaxFrictionTorque(JoltConstraint c, float torque);
float JoltSwingTwistGetMaxFrictionTorque(JoltConstraint c);

// Motor controls (mirrors hinge motor pattern from T-0105)
void JoltSwingTwistSetSwingMotorState(JoltConstraint c, JoltMotorState state);
int JoltSwingTwistGetSwingMotorState(JoltConstraint c);
void JoltSwingTwistSetTwistMotorState(JoltConstraint c, JoltMotorState state);
int JoltSwingTwistGetTwistMotorState(JoltConstraint c);

// Target angular velocity in body 2 constraint space
void JoltSwingTwistSetTargetAngularVelocityCS(JoltConstraint c, float x, float y, float z);
void JoltSwingTwistGetTargetAngularVelocityCS(JoltConstraint c, float *x, float *y, float *z);

// Target orientation in constraint space (drives constraint to inOrientation)
void JoltSwingTwistSetTargetOrientationCS(JoltConstraint c, float qx, float qy, float qz, float qw);
void JoltSwingTwistGetTargetOrientationCS(JoltConstraint c, float *qx, float *qy, float *qz, float *qw);

// Current rotation of body 2 relative to body 1 in constraint space
void JoltSwingTwistGetRotationInConstraintSpace(JoltConstraint c,
                                                 float *qx, float *qy, float *qz, float *qw);

// Motor settings
void JoltSwingTwistSetSwingMotorSettings(JoltConstraint c,
                                          float frequency, float damping,
                                          float forceLimit, float torqueLimit);
void JoltSwingTwistSetTwistMotorSettings(JoltConstraint c,
                                          float frequency, float damping,
                                          float forceLimit, float torqueLimit);

// Force readback
void JoltSwingTwistGetTotalLambdaPosition(JoltConstraint c, float *x, float *y, float *z);
float JoltSwingTwistGetTotalLambdaTwist(JoltConstraint c);
float JoltSwingTwistGetTotalLambdaSwingY(JoltConstraint c);
float JoltSwingTwistGetTotalLambdaSwingZ(JoltConstraint c);
void JoltSwingTwistGetTotalLambdaMotor(JoltConstraint c, float *x, float *y, float *z);

// --- SixDOF Constraint (T-0123) ---

// Matches Jolt SixDOFConstraintSettings::EAxis
typedef enum {
    JoltSixDOFAxisTranslationX = 0,
    JoltSixDOFAxisTranslationY = 1,
    JoltSixDOFAxisTranslationZ = 2,
    JoltSixDOFAxisRotationX = 3,
    JoltSixDOFAxisRotationY = 4,
    JoltSixDOFAxisRotationZ = 5
} JoltSixDOFAxis;

// Create a 6-DOF constraint between two bodies.
// position: world-space attachment point (used for both bodies)
// axisX: world-space X axis of the constraint frame
// axisY: world-space Y axis of the constraint frame (must be perpendicular to axisX)
// limitMin/limitMax: arrays of 6 floats indexed by JoltSixDOFAxis.
//   - Free axis: min = -FLT_MAX, max = FLT_MAX
//   - Fixed axis: min = FLT_MAX, max = -FLT_MAX (driven to 0)
//   - Limited axis: any other valid range
JoltConstraint JoltCreateSixDOFConstraint(
    JoltPhysicsSystem system,
    JoltBodyID bodyID1, JoltBodyID bodyID2,
    float positionX, float positionY, float positionZ,
    float axisXx, float axisXy, float axisXz,
    float axisYx, float axisYy, float axisYz,
    const float *limitMin,
    const float *limitMax,
    JoltSwingType swingType);

// Limits
void JoltSixDOFSetTranslationLimits(JoltConstraint c,
                                     float minX, float minY, float minZ,
                                     float maxX, float maxY, float maxZ);
void JoltSixDOFSetRotationLimits(JoltConstraint c,
                                  float minX, float minY, float minZ,
                                  float maxX, float maxY, float maxZ);
float JoltSixDOFGetLimitsMin(JoltConstraint c, JoltSixDOFAxis axis);
float JoltSixDOFGetLimitsMax(JoltConstraint c, JoltSixDOFAxis axis);
int JoltSixDOFIsFixedAxis(JoltConstraint c, JoltSixDOFAxis axis);
int JoltSixDOFIsFreeAxis(JoltConstraint c, JoltSixDOFAxis axis);

// Friction
void JoltSixDOFSetMaxFriction(JoltConstraint c, JoltSixDOFAxis axis, float friction);
float JoltSixDOFGetMaxFriction(JoltConstraint c, JoltSixDOFAxis axis);

// Motor controls (per-axis state)
void JoltSixDOFSetMotorState(JoltConstraint c, JoltSixDOFAxis axis, JoltMotorState state);
int JoltSixDOFGetMotorState(JoltConstraint c, JoltSixDOFAxis axis);

// Motor settings (per-axis)
void JoltSixDOFSetMotorSettings(JoltConstraint c, JoltSixDOFAxis axis,
                                 float frequency, float damping,
                                 float forceLimit, float torqueLimit);

// Target velocity in body 1 constraint space
void JoltSixDOFSetTargetVelocityCS(JoltConstraint c, float x, float y, float z);
void JoltSixDOFGetTargetVelocityCS(JoltConstraint c, float *x, float *y, float *z);

// Target angular velocity in body 2 constraint space (NB: body 2, not body 1)
void JoltSixDOFSetTargetAngularVelocityCS(JoltConstraint c, float x, float y, float z);
void JoltSixDOFGetTargetAngularVelocityCS(JoltConstraint c, float *x, float *y, float *z);

// Target position in body 1 constraint space
void JoltSixDOFSetTargetPositionCS(JoltConstraint c, float x, float y, float z);
void JoltSixDOFGetTargetPositionCS(JoltConstraint c, float *x, float *y, float *z);

// Target orientation in body 1 constraint space
void JoltSixDOFSetTargetOrientationCS(JoltConstraint c,
                                       float qx, float qy, float qz, float qw);
void JoltSixDOFGetTargetOrientationCS(JoltConstraint c,
                                       float *qx, float *qy, float *qz, float *qw);

// Current rotation in constraint space
void JoltSixDOFGetRotationInConstraintSpace(JoltConstraint c,
                                             float *qx, float *qy, float *qz, float *qw);

// Force readback
void JoltSixDOFGetTotalLambdaPosition(JoltConstraint c, float *x, float *y, float *z);
void JoltSixDOFGetTotalLambdaRotation(JoltConstraint c, float *x, float *y, float *z);
void JoltSixDOFGetTotalLambdaMotorTranslation(JoltConstraint c, float *x, float *y, float *z);
void JoltSixDOFGetTotalLambdaMotorRotation(JoltConstraint c, float *x, float *y, float *z);

// --- Point Constraint (T-0122) ---

// Create a point (ball-joint) constraint between two bodies.
// Constrains 3 linear DOFs while leaving all rotational DOFs free.
// point: world-space attachment point shared by both bodies.
JoltConstraint JoltCreatePointConstraint(
    JoltPhysicsSystem system,
    JoltBodyID bodyID1, JoltBodyID bodyID2,
    float pointX, float pointY, float pointZ);

// Force readback (position impulse this step)
void JoltPointConstraintGetTotalLambdaPosition(JoltConstraint constraint,
                                                float* x, float* y, float* z);

// --- Cone Constraint (T-0122) ---

// Create a cone (swing-limited ball-joint) constraint between two bodies.
// Constrains 3 linear DOFs and limits the swing between the two bodies'
// twist axes to within halfAngleMax radians (allowed range: [0, PI]).
// point: world-space attachment point shared by both bodies.
// twistAxis: world-space twist axis shared by both bodies (normalized).
// halfAngleMax: maximum allowed swing half-angle, in radians.
JoltConstraint JoltCreateConeConstraint(
    JoltPhysicsSystem system,
    JoltBodyID bodyID1, JoltBodyID bodyID2,
    float pointX, float pointY, float pointZ,
    float twistAxisX, float twistAxisY, float twistAxisZ,
    float halfAngleMax);

// Runtime mutation of the cone's half-angle (radians, [0, PI]).
void JoltConeConstraintSetHalfConeAngle(JoltConstraint constraint, float halfAngle);

// Returns cos(halfConeAngle) stored on the constraint.
float JoltConeConstraintGetCosHalfConeAngle(JoltConstraint constraint);

// Force readback
void JoltConeConstraintGetTotalLambdaPosition(JoltConstraint constraint,
                                               float* x, float* y, float* z);
float JoltConeConstraintGetTotalLambdaRotation(JoltConstraint constraint);

// --- Pulley Constraint ---

// Create a pulley constraint between two bodies.
// bodyPoint1/bodyPoint2: world-space attachment points on each body (e.g. top-of-bucket).
// fixedPoint1/fixedPoint2: world-space fixed pulley anchor points. Use the same value
//   for both to model a single pulley wheel; use different values for compound pulleys.
// ratio: multiplier applied to segment 2 length. 1 = simple pulley, >1 = block-and-tackle
//   mechanical advantage where moving body 1 by X moves body 2 by X/ratio.
// minLength/maxLength: constraint on (Length1 + ratio * Length2). Use -1 for either to
//   auto-detect from the initial configuration. Set min == max for a rigid rope.
JoltConstraint JoltCreatePulleyConstraint(
    JoltPhysicsSystem system,
    JoltBodyID bodyID1, JoltBodyID bodyID2,
    float bodyPoint1X, float bodyPoint1Y, float bodyPoint1Z,
    float bodyPoint2X, float bodyPoint2Y, float bodyPoint2Z,
    float fixedPoint1X, float fixedPoint1Y, float fixedPoint1Z,
    float fixedPoint2X, float fixedPoint2Y, float fixedPoint2Z,
    float ratio,
    float minLength, float maxLength);

// Update the min/max length bounds. Both must be >= 0 and min <= max.
void JoltPulleyConstraintSetLength(JoltConstraint constraint,
                                    float minLength, float maxLength);
float JoltPulleyConstraintGetMinLength(JoltConstraint constraint);
float JoltPulleyConstraintGetMaxLength(JoltConstraint constraint);

// Current effective length: |body1 - fixed1| + ratio * |body2 - fixed2|.
float JoltPulleyConstraintGetCurrentLength(JoltConstraint constraint);

// Force readback (impulse applied this step)
float JoltPulleyConstraintGetTotalLambdaPosition(JoltConstraint constraint);

// --- Buoyancy (Body-level API) ---

// Apply buoyancy impulse using surface plane detection (simple overload).
// surfacePosition: a point on the water surface plane
// surfaceNormal: normal of the water surface (should point up, typically 0,1,0)
// buoyancy: buoyancy factor (1 = neutral, >1 = floats, <1 = sinks)
// linearDrag: linear drag (~0.5)
// angularDrag: angular drag (~0.01)
// fluidVelocity: water current velocity
// gravity: gravity vector (typically 0,-9.81,0)
// deltaTime: simulation timestep
// Returns 1 if impulse was applied (body in fluid), 0 otherwise.
int JoltApplyBuoyancyImpulse(
    JoltPhysicsSystem system, JoltBodyID bodyID,
    float surfacePosX, float surfacePosY, float surfacePosZ,
    float surfaceNormalX, float surfaceNormalY, float surfaceNormalZ,
    float buoyancy, float linearDrag, float angularDrag,
    float fluidVelX, float fluidVelY, float fluidVelZ,
    float gravityX, float gravityY, float gravityZ,
    float deltaTime);

// Apply buoyancy impulse using pre-computed submerged volume (advanced overload).
// Use JoltGetSubmergedVolume first to compute volumes, then pass results here.
int JoltApplyBuoyancyImpulseWithVolume(
    JoltPhysicsSystem system, JoltBodyID bodyID,
    float totalVolume, float submergedVolume,
    float relativeCoBX, float relativeCoBY, float relativeCoBZ,
    float buoyancy, float linearDrag, float angularDrag,
    float fluidVelX, float fluidVelY, float fluidVelZ,
    float gravityX, float gravityY, float gravityZ,
    float deltaTime);

// Compute submerged volume of a body relative to a water surface plane.
// surfacePosition: a point on the water surface plane
// surfaceNormal: normal of the water surface (should point up)
// Returns total volume, submerged volume, and relative center of buoyancy.
void JoltGetSubmergedVolume(
    JoltPhysicsSystem system, JoltBodyID bodyID,
    float surfacePosX, float surfacePosY, float surfacePosZ,
    float surfaceNormalX, float surfaceNormalY, float surfaceNormalZ,
    float* outTotalVolume, float* outSubmergedVolume,
    float* outRelCoBX, float* outRelCoBY, float* outRelCoBZ);

#ifdef __cplusplus
}
#endif

#endif // JOLT_WRAPPER_CONSTRAINTS_H
