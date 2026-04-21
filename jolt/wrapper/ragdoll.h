/*
 * Jolt Physics C Wrapper - Ragdoll System (T-0124)
 *
 * Wraps JPH::Skeleton, JPH::RagdollSettings, JPH::Ragdoll for skeletal
 * physics characters. Constraints between bones use SwingTwistConstraintSettings
 * (the standard humanoid joint).
 *
 * Lifecycle:
 *   1. JoltCreateSkeleton() -> add joints in parent-before-child order
 *   2. JoltCreateRagdollSettings(skel) -> for each joint set part shape +
 *      (for non-root) swing/twist constraint
 *   3. JoltRagdollSettingsFinalize() -> stabilize + index tables +
 *      disable parent/child collisions
 *   4. JoltCreateRagdoll(settings, system, groupID) -> instance
 *   5. JoltRagdollAddToPhysicsSystem(ragdoll, activate)
 *   6. ... simulate ...
 *   7. JoltRagdollRemoveFromPhysicsSystem(ragdoll)
 *   8. JoltDestroyRagdoll / JoltDestroyRagdollSettings / JoltDestroySkeleton
 */

#ifndef JOLT_WRAPPER_RAGDOLL_H
#define JOLT_WRAPPER_RAGDOLL_H

#include "physics.h"
#include "body.h"
#include "shape.h"

#ifdef __cplusplus
extern "C" {
#endif

// --- Opaque pointer types ---
typedef void* JoltSkeleton;
typedef void* JoltRagdollSettings;
typedef void* JoltRagdoll;

// --- Skeleton ---

// Create an empty skeleton. Returns an owned handle (Release with JoltDestroySkeleton).
JoltSkeleton JoltCreateSkeleton(void);
void JoltDestroySkeleton(JoltSkeleton skeleton);

// Add a joint. parentIndex must be < current joint count, or -1 for the root.
// Joints must be added in parent-before-child order.
// Returns the new joint's index, or -1 on error.
int JoltSkeletonAddJoint(JoltSkeleton skeleton, const char* name, int parentIndex);

// Number of joints currently in the skeleton.
int JoltSkeletonGetJointCount(JoltSkeleton skeleton);

// Parent index of a joint, or -1 for root / out-of-range.
int JoltSkeletonGetParentIndex(JoltSkeleton skeleton, int jointIndex);

// --- RagdollSettings ---

// Build a RagdollSettings sized for the given skeleton's joint count.
// All parts default to identity rotation, zero position, and dynamic motion.
JoltRagdollSettings JoltCreateRagdollSettings(JoltSkeleton skeleton);
void JoltDestroyRagdollSettings(JoltRagdollSettings settings);

// Configure the rigid body for one joint.
// motionType: 0 static, 1 kinematic, 2 dynamic (matches JoltMotionType).
// objectLayer: physics layer for this body.
// mass: kg. If <= 0, mass is calculated from shape density.
// Returns 0 on success, -1 on out-of-range index.
int JoltRagdollSettingsSetPart(
    JoltRagdollSettings settings, int jointIndex, JoltShape shape,
    float posX, float posY, float posZ,
    float rotX, float rotY, float rotZ, float rotW,
    int motionType, unsigned short objectLayer, float mass);

// Attach a SwingTwistConstraint between this joint's body and its parent's
// body. World-space pivot/axes.
//   twistAxis  = bone forward axis (the long axis of the limb)
//   planeAxis  = perpendicular to twistAxis (defines the swing plane)
//   normalHalfConeAngle / planeHalfConeAngle = swing limits in radians
//   twistMin / twistMax = twist limits in radians (in [-PI, PI])
// Returns 0 on success, -1 if jointIndex is invalid or has no parent.
int JoltRagdollSettingsSetSwingTwistConstraint(
    JoltRagdollSettings settings, int jointIndex,
    float pivotX, float pivotY, float pivotZ,
    float twistAxisX, float twistAxisY, float twistAxisZ,
    float planeAxisX, float planeAxisY, float planeAxisZ,
    float normalHalfConeAngle, float planeHalfConeAngle,
    float twistMinAngle, float twistMaxAngle);

// Run Stabilize() + CalculateBodyIndexToConstraintIndex() +
// DisableParentChildCollisions(). Call once after all parts and constraints
// are configured, before CreateRagdoll.
// Returns 0 on success, -1 if Stabilize failed.
int JoltRagdollSettingsFinalize(JoltRagdollSettings settings);

// --- Ragdoll instance ---

// Materialize a ragdoll instance. Bodies are created but NOT yet added to the
// physics system — call JoltRagdollAddToPhysicsSystem next.
// groupID: collision group; use a unique value per ragdoll.
// Returns NULL if the system is out of body slots.
JoltRagdoll JoltCreateRagdoll(JoltRagdollSettings settings,
                              JoltPhysicsSystem system,
                              unsigned int groupID,
                              unsigned long long userData);
void JoltDestroyRagdoll(JoltRagdoll ragdoll);

// Insert all bodies and constraints into the physics system.
// activate: 1 to wake the bodies immediately, 0 to leave them asleep.
void JoltRagdollAddToPhysicsSystem(JoltRagdoll ragdoll, int activate);

// Remove all bodies and constraints from the physics system. Safe to call
// multiple times.
void JoltRagdollRemoveFromPhysicsSystem(JoltRagdoll ragdoll);

// Wake all bodies in the ragdoll.
void JoltRagdollActivate(JoltRagdoll ragdoll);

// 1 if any body in the ragdoll is currently active, else 0.
int JoltRagdollIsActive(JoltRagdoll ragdoll);

// Number of bodies in the ragdoll (== number of skeleton joints).
int JoltRagdollGetBodyCount(JoltRagdoll ragdoll);

// Returns a newly-allocated JoltBodyID for the body at index.
// Caller owns it and must call JoltDestroyBodyID. Returns NULL on out-of-range.
JoltBodyID JoltRagdollGetBodyID(JoltRagdoll ragdoll, int index);

// Apply an instantaneous impulse to every body in the ragdoll
// (e.g. a one-shot blast).
void JoltRagdollAddImpulse(JoltRagdoll ragdoll, float x, float y, float z);

// Set the linear velocity of every body in the ragdoll.
void JoltRagdollSetLinearVelocity(JoltRagdoll ragdoll, float x, float y, float z);

// World-space position + rotation of the root body.
void JoltRagdollGetRootTransform(JoltRagdoll ragdoll,
                                 float* outPosX, float* outPosY, float* outPosZ,
                                 float* outRotX, float* outRotY, float* outRotZ,
                                 float* outRotW);

// Drive the ragdoll toward a target pose by setting kinematic velocities on
// each body so that it reaches the pose in deltaTime seconds.
//   rootOffset: world-space root translation
//   jointMatrices: jointCount * 16 floats. Each block is a 4x4 column-major
//                  world-space transform for the corresponding joint.
// jointCount must equal the number of bodies in the ragdoll.
// Returns 0 on success, -1 on size mismatch.
int JoltRagdollDriveToPoseUsingKinematics(
    JoltRagdoll ragdoll,
    float rootOffsetX, float rootOffsetY, float rootOffsetZ,
    const float* jointMatrices, int jointCount,
    float deltaTime);

#ifdef __cplusplus
}
#endif

#endif // JOLT_WRAPPER_RAGDOLL_H
