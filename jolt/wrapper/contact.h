/*
 * Jolt Physics C Wrapper - Contact Listener and Body Activation Listener
 *
 * T-0121: Establishes the cgo callback trampoline pattern.
 * Go installs a userData token (cgo.Handle) on a C++ ContactListener /
 * BodyActivationListener implementation. The C++ virtual overrides marshal
 * Jolt types into flat C structs (JoltContactManifold / JoltContactSettings)
 * and invoke extern "C" trampolines (goJolt*) that cgo //exports from Go.
 *
 * Callbacks fire from multiple Jolt worker threads simultaneously — Go
 * handlers must be threadsafe.
 */

#ifndef JOLT_WRAPPER_CONTACT_H
#define JOLT_WRAPPER_CONTACT_H

#include "physics.h"
#include "body.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Mirrors JPH::ValidateResult.
typedef enum {
    JoltAcceptAllContactsForThisBodyPair = 0,
    JoltAcceptContact = 1,
    JoltRejectContact = 2,
    JoltRejectAllContactsForThisBodyPair = 3
} JoltValidateResult;

// Flat-C view of JPH::ContactManifold passed to OnContactAdded / OnContactPersisted.
// Contact points are reported relative to baseOffset; world-space point = baseOffset + relative.
typedef struct {
    unsigned int  body1ID;
    unsigned int  body2ID;
    float         baseOffsetX, baseOffsetY, baseOffsetZ;
    float         normalX, normalY, normalZ;          // Direction to push body 2 out of body 1
    float         penetrationDepth;
    int           numContactPoints;                   // total points in the manifold
    float         contactPoint1X, contactPoint1Y, contactPoint1Z; // first point on shape 1, relative to baseOffset
    float         contactPoint2X, contactPoint2Y, contactPoint2Z; // first point on shape 2, relative to baseOffset
    float         relativeLinearVelocityX, relativeLinearVelocityY, relativeLinearVelocityZ; // body2.linearVel - body1.linearVel
} JoltContactManifold;

// Mutable view of JPH::ContactSettings; callbacks may write back to adjust the
// contact constraint (friction, restitution, sensor flag, etc.).
typedef struct {
    float combinedFriction;
    float combinedRestitution;
    float invMassScale1, invInertiaScale1;
    float invMassScale2, invInertiaScale2;
    int   isSensor;
    float relativeLinearSurfaceVelocityX, relativeLinearSurfaceVelocityY, relativeLinearSurfaceVelocityZ;
    float relativeAngularSurfaceVelocityX, relativeAngularSurfaceVelocityY, relativeAngularSurfaceVelocityZ;
} JoltContactSettings;

// Install the callback-based contact listener. Replaces any prior contact listener
// on the system (including the queue-based one set by JoltEnableContactEvents).
// userData is opaque (typically a cgo.Handle) and is passed verbatim to callbacks.
// Returns 0 on success, -1 on failure.
int  JoltInstallContactCallbacks(JoltPhysicsSystem system, uintptr_t userData);

// Uninstall and free the callback-based contact listener (no-op if not installed).
void JoltUninstallContactCallbacks(JoltPhysicsSystem system);

// Install the body activation listener. Replaces any prior activation listener.
// Returns 0 on success, -1 on failure.
int  JoltInstallBodyActivationCallbacks(JoltPhysicsSystem system, uintptr_t userData);

// Uninstall and free the body activation listener (no-op if not installed).
void JoltUninstallBodyActivationCallbacks(JoltPhysicsSystem system);

#ifdef __cplusplus
}

// Forward declarations referenced by PhysicsSystemWrapper destructor.
class ContactCallbackListenerImpl;
class BodyActivationListenerImpl;

void DestroyContactCallbackListener(ContactCallbackListenerImpl* listener);
void DestroyBodyActivationListener(BodyActivationListenerImpl* listener);

#endif

#endif // JOLT_WRAPPER_CONTACT_H
