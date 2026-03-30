/*
 * Jolt Physics C Wrapper - Constraints & Buoyancy (T-0105)
 *
 * Constraint types: Distance, Hinge, Slider, Fixed
 * Motor control for Hinge and Slider constraints
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
