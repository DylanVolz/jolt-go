/*
 * Jolt Physics C Wrapper - Decorator and MutableCompound Shapes (T-0128)
 *
 * Exposes runtime-mutable compound shapes plus the three decorator shapes
 * (Scaled, RotatedTranslated, OffsetCenterOfMass). MutableCompoundShape
 * allows add/remove/modify of sub-shapes without rebuilding the shape tree,
 * which is needed for animated machinery and breakable attachments.
 *
 * Thread-safety note: MutableCompoundShape mutations are NOT thread-safe.
 * Callers must ensure any bodies using the shape are locked during mutation,
 * and call BodyInterface::NotifyShapeChanged afterwards. This wrapper does
 * not perform that coordination — it is the caller's responsibility.
 */

#ifndef JOLT_WRAPPER_SHAPE_DECORATORS_H
#define JOLT_WRAPPER_SHAPE_DECORATORS_H

#ifdef __cplusplus
extern "C" {
#endif

// Opaque pointer type (matches the typedef in shape.h)
typedef void* JoltShape;

// --- Decorator shapes ---

// Create a ScaledShape wrapping `innerShape` with the given non-uniform scale.
// The inner shape's ref count is incremented; the returned shape owns its own ref.
// Returns NULL on failure (e.g. zero scale).
JoltShape JoltCreateScaledShape(JoltShape innerShape,
                                float scaleX, float scaleY, float scaleZ);

// Create a RotatedTranslatedShape wrapping `innerShape` with the given offset
// and rotation (as a quaternion x,y,z,w).
// Returns NULL on failure.
JoltShape JoltCreateRotatedTranslatedShape(JoltShape innerShape,
                                           float posX, float posY, float posZ,
                                           float rotX, float rotY, float rotZ, float rotW);

// Create an OffsetCenterOfMassShape wrapping `innerShape`, shifting the
// center of mass by the given offset without changing collision geometry.
// Returns NULL on failure.
JoltShape JoltCreateOffsetCenterOfMassShape(JoltShape innerShape,
                                            float offsetX, float offsetY, float offsetZ);

// --- MutableCompound shape ---

// Create a MutableCompoundShape from an initial set of sub-shapes.
// shapes/positions/rotations may be NULL if count == 0 (empty compound).
// positions: interleaved x,y,z (3 floats per sub-shape)
// rotations: interleaved x,y,z,w (4 floats per sub-shape)
// Returns NULL on failure.
JoltShape JoltCreateMutableCompound(const JoltShape* shapes,
                                    const float* positions,
                                    const float* rotations,
                                    int count);

// Add a sub-shape to a MutableCompound at the end.
// Returns the new sub-shape index on success, or 0xFFFFFFFF on failure
// (e.g. `compound` is not a MutableCompoundShape).
unsigned int JoltMutableCompoundAddShape(JoltShape compound,
                                         float posX, float posY, float posZ,
                                         float rotX, float rotY, float rotZ, float rotW,
                                         JoltShape subShape);

// Remove a sub-shape by index. Returns 0 on success, -1 on failure.
int JoltMutableCompoundRemoveShape(JoltShape compound, unsigned int index);

// Modify the position and rotation of a sub-shape. Returns 0 on success, -1 on failure.
int JoltMutableCompoundModifyShape(JoltShape compound, unsigned int index,
                                   float posX, float posY, float posZ,
                                   float rotX, float rotY, float rotZ, float rotW);

// Modify the position, rotation, and underlying shape of a sub-shape.
// Returns 0 on success, -1 on failure.
int JoltMutableCompoundModifyShapeWithShape(JoltShape compound, unsigned int index,
                                            float posX, float posY, float posZ,
                                            float rotX, float rotY, float rotZ, float rotW,
                                            JoltShape subShape);

// Recalculate the center of mass and shift all sub-shapes accordingly.
// Must be called after significant topology changes on dynamic bodies.
// Returns 0 on success, -1 on failure.
int JoltMutableCompoundAdjustCenterOfMass(JoltShape compound);

// Number of sub-shapes currently in a compound (Static or Mutable).
// Returns -1 if `compound` is not a compound shape.
int JoltCompoundGetNumSubShapes(JoltShape compound);

#ifdef __cplusplus
}
#endif

#endif // JOLT_WRAPPER_SHAPE_DECORATORS_H
