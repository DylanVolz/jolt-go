/*
 * Jolt Physics C Wrapper - Soft Body Operations (T-0129)
 *
 * Binds SoftBodySharedSettings + SoftBodyCreationSettings, plus runtime
 * vertex access used for driving cloth, flag, and rope-like simulations.
 *
 * Shared settings are ref-counted on the C++ side; every
 * JoltCreateSoftBodySharedSettings / JoltCreateClothGridSettings call must
 * be matched by JoltDestroySoftBodySharedSettings. The same settings pointer
 * may be passed to multiple JoltCreateSoftBody calls before being destroyed —
 * each body acquires its own ref.
 */

#ifndef JOLT_WRAPPER_SOFTBODY_H
#define JOLT_WRAPPER_SOFTBODY_H

#include "body.h"
#include "physics.h"

#ifdef __cplusplus
extern "C" {
#endif

// Opaque handle to a ref-counted SoftBodySharedSettings
typedef void* JoltSoftBodySharedSettings;

// Bend constraint type (matches SoftBodySharedSettings::EBendType)
typedef enum {
    JoltBendTypeNone     = 0, // No bend constraints
    JoltBendTypeDistance = 1, // Simple distance-based bend springs
    JoltBendTypeDihedral = 2  // Dihedral bend constraints
} JoltBendType;

// --- Shared settings construction -------------------------------------------

// Create an empty SoftBodySharedSettings. Must be destroyed with
// JoltDestroySoftBodySharedSettings (which releases the owning ref).
JoltSoftBodySharedSettings JoltCreateSoftBodySharedSettings(void);

// Append a vertex (particle). invMass=0 marks a kinematic (pinned) vertex.
// Returns the index of the newly added vertex.
int JoltSoftBodySharedSettingsAddVertex(JoltSoftBodySharedSettings settings,
                                        float x, float y, float z,
                                        float invMass);

// Append a triangular face referencing three existing vertex indices.
// Returns 0 on success, -1 if indices are degenerate.
int JoltSoftBodySharedSettingsAddFace(JoltSoftBodySharedSettings settings,
                                      unsigned int v0,
                                      unsigned int v1,
                                      unsigned int v2);

// Append an explicit edge (spring) constraint between two vertices.
// compliance is the inverse stiffness; 0 = perfectly rigid spring.
// The rest length is computed from the current vertex positions when
// JoltSoftBodySharedSettingsCreateConstraints or Optimize is invoked.
void JoltSoftBodySharedSettingsAddEdge(JoltSoftBodySharedSettings settings,
                                       unsigned int v0,
                                       unsigned int v1,
                                       float compliance);

// Auto-build edge, shear, and bend constraints from the face list. Calls
// CreateConstraints(vertexAttrs, 1, bendType) followed by the internal
// calculations for edge lengths, LRA lengths and bend constants.
void JoltSoftBodySharedSettingsCreateConstraints(JoltSoftBodySharedSettings settings,
                                                 float compliance,
                                                 float shearCompliance,
                                                 float bendCompliance,
                                                 JoltBendType bendType);

// Reorder the constraint arrays for parallel solver execution. Must be
// called before the settings are handed to JoltCreateSoftBody.
void JoltSoftBodySharedSettingsOptimize(JoltSoftBodySharedSettings settings);

// Release the caller's ref on the shared settings. The underlying object
// is freed once no live soft body retains a ref.
void JoltDestroySoftBodySharedSettings(JoltSoftBodySharedSettings settings);

// Convenience: build a flat cloth grid laid out in the XZ plane (Y = 0,
// centered around the origin), auto-generate faces, edge and bend
// constraints, and optimize. invMasses is a row-major array of
// gridSizeX * gridSizeZ floats (0 = pinned vertex, >0 = dynamic).
// Pass invMasses=NULL to default to 1.0 (all dynamic). bendType selects
// the bend-constraint type. Returns NULL on invalid arguments.
JoltSoftBodySharedSettings JoltCreateClothGridSettings(int gridSizeX,
                                                       int gridSizeZ,
                                                       float spacing,
                                                       const float* invMasses,
                                                       int numInvMasses,
                                                       float compliance,
                                                       JoltBendType bendType);

// --- Soft body creation -----------------------------------------------------

// Create a soft body and add it to the simulation. Caller must ensure the
// shared settings have been Optimized. Pressure models an internal gas
// (see SoftBodyCreationSettings::mPressure); pass 0 for cloth. numIterations
// is the XPBD solver iteration count (5 is Jolt's default). Returns NULL on
// failure.
JoltBodyID JoltCreateSoftBody(JoltBodyInterface bodyInterface,
                              JoltSoftBodySharedSettings settings,
                              float x, float y, float z,
                              float rotX, float rotY, float rotZ, float rotW,
                              int layer,
                              float pressure,
                              int numIterations,
                              float linearDamping,
                              float gravityFactor);

// --- Soft body runtime queries ----------------------------------------------

// Number of simulated vertices on the soft body. Returns -1 if the body is
// not a soft body or the lock fails.
int JoltGetSoftBodyVertexCount(JoltPhysicsSystem system, JoltBodyID bodyID);

// Copy the world-space positions of all vertices into outXYZ (3 floats per
// vertex). bufLen is the capacity in floats (must be >= 3 * vertexCount).
// Returns the number of vertices written, or -1 on error.
int JoltGetSoftBodyVertexPositions(JoltPhysicsSystem system,
                                   JoltBodyID bodyID,
                                   float* outXYZ,
                                   int bufLen);

// Read a single vertex's inverse mass. Returns <0 on error.
float JoltGetSoftBodyVertexInvMass(JoltPhysicsSystem system,
                                   JoltBodyID bodyID,
                                   int index);

// Set a single vertex's inverse mass. 0 makes the vertex kinematic (pinned).
// Returns 0 on success, -1 on error.
int JoltSetSoftBodyVertexInvMass(JoltPhysicsSystem system,
                                 JoltBodyID bodyID,
                                 int index,
                                 float invMass);

// Apply a continuous force (N) to the soft body for the current step. Works
// on any body but exposed here for soft-body wind fields.
void JoltAddSoftBodyForce(JoltBodyInterface bodyInterface,
                          JoltBodyID bodyID,
                          float fx, float fy, float fz);

#ifdef __cplusplus
}
#endif

#endif // JOLT_WRAPPER_SOFTBODY_H
