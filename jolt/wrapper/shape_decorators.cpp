/*
 * Jolt Physics C Wrapper - Decorator and MutableCompound Shapes (T-0128)
 */

#include "shape_decorators.h"

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>
#include <Jolt/Physics/Collision/Shape/CompoundShape.h>
#include <Jolt/Physics/Collision/Shape/MutableCompoundShape.h>
#include <Jolt/Physics/Collision/Shape/ScaledShape.h>
#include <Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h>
#include <Jolt/Physics/Collision/Shape/OffsetCenterOfMassShape.h>

#include <climits>

using namespace JPH;

namespace {

// Finalize a ShapeSettings::ShapeResult into a raw Shape* with a held reference.
// Returns nullptr on failure (mirrors the pattern used in shape.cpp).
static JoltShape FinalizeShapeResult(ShapeSettings::ShapeResult &result)
{
	if (result.HasError()) return nullptr;
	ShapeRefC shape = result.Get();
	if (shape == nullptr) return nullptr;
	shape->AddRef();
	return static_cast<JoltShape>(const_cast<Shape*>(shape.GetPtr()));
}

// Cast to MutableCompoundShape if and only if the handle is actually one.
// Returns nullptr otherwise (so mutations are a safe no-op / error).
static MutableCompoundShape *AsMutableCompound(JoltShape handle)
{
	if (handle == nullptr) return nullptr;
	Shape *s = static_cast<Shape *>(handle);
	if (s->GetSubType() != EShapeSubType::MutableCompound) return nullptr;
	return static_cast<MutableCompoundShape *>(s);
}

} // namespace

// --- Decorator shapes ---

JoltShape JoltCreateScaledShape(JoltShape innerShape,
                                float scaleX, float scaleY, float scaleZ)
{
	if (innerShape == nullptr) return nullptr;
	const Shape *inner = static_cast<const Shape *>(innerShape);

	ScaledShapeSettings settings(inner, Vec3(scaleX, scaleY, scaleZ));
	ShapeSettings::ShapeResult result = settings.Create();
	return FinalizeShapeResult(result);
}

JoltShape JoltCreateRotatedTranslatedShape(JoltShape innerShape,
                                           float posX, float posY, float posZ,
                                           float rotX, float rotY, float rotZ, float rotW)
{
	if (innerShape == nullptr) return nullptr;
	const Shape *inner = static_cast<const Shape *>(innerShape);

	RotatedTranslatedShapeSettings settings(
		Vec3(posX, posY, posZ),
		Quat(rotX, rotY, rotZ, rotW),
		inner);
	ShapeSettings::ShapeResult result = settings.Create();
	return FinalizeShapeResult(result);
}

JoltShape JoltCreateOffsetCenterOfMassShape(JoltShape innerShape,
                                            float offsetX, float offsetY, float offsetZ)
{
	if (innerShape == nullptr) return nullptr;
	const Shape *inner = static_cast<const Shape *>(innerShape);

	OffsetCenterOfMassShapeSettings settings(Vec3(offsetX, offsetY, offsetZ), inner);
	ShapeSettings::ShapeResult result = settings.Create();
	return FinalizeShapeResult(result);
}

// --- MutableCompound shape ---

JoltShape JoltCreateMutableCompound(const JoltShape *shapes,
                                    const float *positions,
                                    const float *rotations,
                                    int count)
{
	if (count < 0) return nullptr;
	if (count > 0 && (shapes == nullptr || positions == nullptr || rotations == nullptr))
		return nullptr;

	MutableCompoundShapeSettings settings;
	for (int i = 0; i < count; ++i)
	{
		const Shape *sub = static_cast<const Shape *>(shapes[i]);
		Vec3 pos(positions[i * 3 + 0], positions[i * 3 + 1], positions[i * 3 + 2]);
		Quat rot(rotations[i * 4 + 0], rotations[i * 4 + 1],
		         rotations[i * 4 + 2], rotations[i * 4 + 3]);
		settings.AddShape(pos, rot, sub);
	}

	ShapeSettings::ShapeResult result = settings.Create();
	return FinalizeShapeResult(result);
}

unsigned int JoltMutableCompoundAddShape(JoltShape compound,
                                         float posX, float posY, float posZ,
                                         float rotX, float rotY, float rotZ, float rotW,
                                         JoltShape subShape)
{
	MutableCompoundShape *mc = AsMutableCompound(compound);
	if (mc == nullptr || subShape == nullptr) return UINT_MAX;

	const Shape *sub = static_cast<const Shape *>(subShape);
	return static_cast<unsigned int>(
		mc->AddShape(Vec3(posX, posY, posZ),
		             Quat(rotX, rotY, rotZ, rotW),
		             sub));
}

int JoltMutableCompoundRemoveShape(JoltShape compound, unsigned int index)
{
	MutableCompoundShape *mc = AsMutableCompound(compound);
	if (mc == nullptr) return -1;
	if (index >= mc->GetNumSubShapes()) return -1;
	mc->RemoveShape(index);
	return 0;
}

int JoltMutableCompoundModifyShape(JoltShape compound, unsigned int index,
                                   float posX, float posY, float posZ,
                                   float rotX, float rotY, float rotZ, float rotW)
{
	MutableCompoundShape *mc = AsMutableCompound(compound);
	if (mc == nullptr) return -1;
	if (index >= mc->GetNumSubShapes()) return -1;
	mc->ModifyShape(index,
	                Vec3(posX, posY, posZ),
	                Quat(rotX, rotY, rotZ, rotW));
	return 0;
}

int JoltMutableCompoundModifyShapeWithShape(JoltShape compound, unsigned int index,
                                            float posX, float posY, float posZ,
                                            float rotX, float rotY, float rotZ, float rotW,
                                            JoltShape subShape)
{
	MutableCompoundShape *mc = AsMutableCompound(compound);
	if (mc == nullptr || subShape == nullptr) return -1;
	if (index >= mc->GetNumSubShapes()) return -1;
	const Shape *sub = static_cast<const Shape *>(subShape);
	mc->ModifyShape(index,
	                Vec3(posX, posY, posZ),
	                Quat(rotX, rotY, rotZ, rotW),
	                sub);
	return 0;
}

int JoltMutableCompoundAdjustCenterOfMass(JoltShape compound)
{
	MutableCompoundShape *mc = AsMutableCompound(compound);
	if (mc == nullptr) return -1;
	mc->AdjustCenterOfMass();
	return 0;
}

int JoltCompoundGetNumSubShapes(JoltShape compound)
{
	if (compound == nullptr) return -1;
	Shape *s = static_cast<Shape *>(compound);
	if (s->GetType() != EShapeType::Compound) return -1;
	CompoundShape *cs = static_cast<CompoundShape *>(s);
	return static_cast<int>(cs->GetNumSubShapes());
}
