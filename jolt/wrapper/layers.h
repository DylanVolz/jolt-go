/*
 * Jolt Physics C Wrapper - Collision Layers, Contact Events, Filtered Raycast
 *
 * T-0103: Configurable N-layer collision filtering, contact event polling,
 * and layer-filtered raycasting.
 */

#ifndef JOLT_WRAPPER_LAYERS_H
#define JOLT_WRAPPER_LAYERS_H

#include "physics.h"
#include "body.h"
#include "query.h"

#ifdef __cplusplus
extern "C" {
#endif

// --- Collision Matrix ---

typedef void* JoltCollisionMatrix;

// Create a collision matrix for numLayers layers. All collisions disabled by default.
JoltCollisionMatrix JoltCreateCollisionMatrix(int numLayers);

// Enable collision between two layers (symmetric)
void JoltCollisionMatrixEnableCollision(JoltCollisionMatrix matrix, int layer1, int layer2);

// Disable collision between two layers (symmetric)
void JoltCollisionMatrixDisableCollision(JoltCollisionMatrix matrix, int layer1, int layer2);

// Destroy a collision matrix
void JoltDestroyCollisionMatrix(JoltCollisionMatrix matrix);

// --- N-Layer Physics System ---

// Create a physics system with configurable collision layers
JoltPhysicsSystem JoltCreatePhysicsSystemWithLayers(JoltCollisionMatrix matrix);

// --- Layer-Aware Body Creation ---

// Create a body on a specific collision layer
JoltBodyID JoltCreateBodyOnLayer(JoltBodyInterface bodyInterface,
                                 JoltShape shape,
                                 float x, float y, float z,
                                 JoltMotionType motionType,
                                 int isSensor,
                                 int layer);

// Get the collision layer of a body (needs body lock, takes PhysicsSystem)
int JoltGetBodyLayer(JoltPhysicsSystem system, JoltBodyID bodyID);

// --- Contact Events ---

typedef enum {
    JoltContactAdded = 0,
    JoltContactPersisted = 1,
    JoltContactRemoved = 2
} JoltContactEventType;

typedef struct {
    JoltContactEventType type;
    unsigned int bodyID1;
    unsigned int bodyID2;
    float contactPointX;
    float contactPointY;
    float contactPointZ;
    float normalX;
    float normalY;
    float normalZ;
} JoltContactEvent;

// Enable contact event collection on a physics system
void JoltEnableContactEvents(JoltPhysicsSystem system);

// Drain queued contact events into outEvents array
// Returns number of events written (may be less than maxEvents)
int JoltDrainContactEvents(JoltPhysicsSystem system, JoltContactEvent* outEvents, int maxEvents);

// --- Filtered Raycast ---

// Cast a ray filtered by a layer bitmask
// layerMask: bitmask of layers to include (e.g., 0b101 = layers 0 and 2)
// Returns 1 if hit, 0 if miss
int JoltCastRayFiltered(JoltPhysicsSystem system,
                        float originX, float originY, float originZ,
                        float directionX, float directionY, float directionZ,
                        unsigned int layerMask,
                        JoltRaycastHit* outHit);

#ifdef __cplusplus
}
#endif

#endif // JOLT_WRAPPER_LAYERS_H
