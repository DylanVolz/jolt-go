/*
 * Jolt Physics C Wrapper - Physics System
 *
 * Handles physics world creation, management, and collision layers.
 */

#ifndef JOLT_WRAPPER_PHYSICS_H
#define JOLT_WRAPPER_PHYSICS_H

#ifdef __cplusplus
extern "C" {
#endif

// Opaque pointer types
typedef void* JoltPhysicsSystem;

// Create a new physics world
JoltPhysicsSystem JoltCreatePhysicsSystem();

// Destroy a physics world
void JoltDestroyPhysicsSystem(JoltPhysicsSystem system);

// Step the physics simulation by deltaTime seconds
void JoltPhysicsSystemUpdate(JoltPhysicsSystem system, float deltaTime);

#ifdef __cplusplus
}

// C++ only: Accessor functions for wrapper internals
#include <memory>

namespace JPH {
    class PhysicsSystem;
    class BroadPhaseLayerInterface;
    class ObjectVsBroadPhaseLayerFilter;
    class ObjectLayerPairFilter;
    class ContactListener;
}

// Forward declaration for contact listener implementation
class ContactListenerImpl;

// Wrapper to keep layer interfaces alive (PhysicsSystem stores references to them)
struct PhysicsSystemWrapper
{
    std::unique_ptr<JPH::PhysicsSystem> system;
    std::unique_ptr<JPH::BroadPhaseLayerInterface> broad_phase_layer_interface;
    std::unique_ptr<JPH::ObjectVsBroadPhaseLayerFilter> object_vs_broadphase_layer_filter;
    std::unique_ptr<JPH::ObjectLayerPairFilter> object_vs_object_layer_filter;

    // Optional: contact event listener (T-0103)
    ContactListenerImpl* contact_listener = nullptr;

    ~PhysicsSystemWrapper();
};

// Accessor functions
JPH::PhysicsSystem* GetPhysicsSystem(PhysicsSystemWrapper* wrapper);
const JPH::ObjectVsBroadPhaseLayerFilter* GetObjectVsBroadPhaseLayerFilter(PhysicsSystemWrapper* wrapper);
const JPH::ObjectLayerPairFilter* GetObjectLayerPairFilter(PhysicsSystemWrapper* wrapper);

// Defined in layers.cpp where ContactListenerImpl is complete
void DestroyContactListener(ContactListenerImpl* listener);

#endif

#endif // JOLT_WRAPPER_PHYSICS_H
