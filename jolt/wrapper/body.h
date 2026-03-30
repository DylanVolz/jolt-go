/*
 * Jolt Physics C Wrapper - Body Operations
 *
 * Handles rigid body creation and manipulation.
 */

#ifndef JOLT_WRAPPER_BODY_H
#define JOLT_WRAPPER_BODY_H

#include "physics.h"

#ifdef __cplusplus
extern "C" {
#endif

// Opaque pointer types
typedef void* JoltBodyInterface;
typedef void* JoltBodyID;
typedef void* JoltShape;

// Motion type enum (matches Jolt's EMotionType)
typedef enum {
    JoltMotionTypeStatic = 0,    // Immovable, zero velocity
    JoltMotionTypeKinematic = 1, // Movable by user, zero velocity response to forces
    JoltMotionTypeDynamic = 2    // Affected by forces
} JoltMotionType;

// Get the body interface for creating/manipulating bodies
JoltBodyInterface JoltPhysicsSystemGetBodyInterface(JoltPhysicsSystem system);

// Get the position of a body
void JoltGetBodyPosition(const JoltBodyInterface bodyInterface,
                        const JoltBodyID bodyID,
                        float* x, float* y, float* z);

// Set the position of a body
void JoltSetBodyPosition(JoltBodyInterface bodyInterface,
                        JoltBodyID bodyID,
                        float x, float y, float z);

// Create a body with specific motion type and sensor flag
JoltBodyID JoltCreateBody(JoltBodyInterface bodyInterface,
                          JoltShape shape,
                          float x, float y, float z,
                          JoltMotionType motionType,
                          int isSensor);

// Activate a body (makes it participate in simulation)
void JoltActivateBody(JoltBodyInterface bodyInterface, JoltBodyID bodyID);

// Deactivate a body (removes from active simulation)
void JoltDeactivateBody(JoltBodyInterface bodyInterface, JoltBodyID bodyID);

// Set the shape of a body
void JoltSetBodyShape(JoltBodyInterface bodyInterface,
                     JoltBodyID bodyID,
                     JoltShape shape,
                     int updateMassProperties);

// Destroy a body ID
void JoltDestroyBodyID(JoltBodyID bodyID);

// --- Extended Body API (T-0102) ---

// Linear velocity
void JoltGetLinearVelocity(JoltBodyInterface bodyInterface, JoltBodyID bodyID,
                           float* x, float* y, float* z);
void JoltSetLinearVelocity(JoltBodyInterface bodyInterface, JoltBodyID bodyID,
                           float x, float y, float z);

// Angular velocity
void JoltGetAngularVelocity(JoltBodyInterface bodyInterface, JoltBodyID bodyID,
                            float* x, float* y, float* z);
void JoltSetAngularVelocity(JoltBodyInterface bodyInterface, JoltBodyID bodyID,
                            float x, float y, float z);

// Forces
void JoltAddImpulse(JoltBodyInterface bodyInterface, JoltBodyID bodyID,
                    float x, float y, float z);

// Material properties
void JoltSetFriction(JoltBodyInterface bodyInterface, JoltBodyID bodyID, float friction);
void JoltSetRestitution(JoltBodyInterface bodyInterface, JoltBodyID bodyID, float restitution);

// Damping (needs body lock, takes PhysicsSystem instead of BodyInterface)
void JoltSetLinearDamping(JoltPhysicsSystem system, JoltBodyID bodyID, float damping);
void JoltSetAngularDamping(JoltPhysicsSystem system, JoltBodyID bodyID, float damping);

// Gravity factor
void JoltSetGravityFactor(JoltBodyInterface bodyInterface, JoltBodyID bodyID, float factor);

// Rotation (quaternion)
void JoltGetRotation(JoltBodyInterface bodyInterface, JoltBodyID bodyID,
                     float* x, float* y, float* z, float* w);
void JoltSetRotation(JoltBodyInterface bodyInterface, JoltBodyID bodyID,
                     float x, float y, float z, float w);

// Body state
int JoltIsBodyActive(JoltBodyInterface bodyInterface, JoltBodyID bodyID);

// Remove from broadphase + destroy body. Do NOT call JoltDestroyBodyID after this.
void JoltRemoveAndDestroyBody(JoltBodyInterface bodyInterface, JoltBodyID bodyID);

#ifdef __cplusplus
}
#endif

#endif // JOLT_WRAPPER_BODY_H
