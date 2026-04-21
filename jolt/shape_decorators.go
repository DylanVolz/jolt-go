package jolt

// #include "wrapper/shape_decorators.h"
import "C"

import (
	"errors"
	"math"
)

// ErrNotMutableCompound is returned by MutableCompound operations when the
// receiver is not actually a MutableCompound shape, when the requested sub-shape
// index is out of range, or when the underlying mutation otherwise fails.
var ErrNotMutableCompound = errors.New("jolt: shape is not a MutableCompound or index is out of range")

// invalidSubShapeIndex is the sentinel the C wrapper returns for failed
// MutableCompoundAddShape calls (matches UINT_MAX).
const invalidSubShapeIndex = math.MaxUint32

// CreateScaledShape wraps `inner` in a ScaledShape that applies a non-uniform
// scale in the inner shape's local space. Zero components are rejected by Jolt
// and will cause the call to return nil.
//
// The returned shape holds its own reference. Callers must Destroy() it
// independently of the inner shape.
func CreateScaledShape(inner *Shape, scale Vec3) *Shape {
	if inner == nil {
		return nil
	}
	handle := C.JoltCreateScaledShape(
		inner.handle,
		C.float(scale.X), C.float(scale.Y), C.float(scale.Z),
	)
	if handle == nil {
		return nil
	}
	return &Shape{handle: handle}
}

// CreateRotatedTranslatedShape wraps `inner` in a decorator that applies a
// translation and rotation to it. Useful for offsetting convex primitives
// inside a compound without paying the compound overhead.
func CreateRotatedTranslatedShape(inner *Shape, position Vec3, rotation Quat) *Shape {
	if inner == nil {
		return nil
	}
	handle := C.JoltCreateRotatedTranslatedShape(
		inner.handle,
		C.float(position.X), C.float(position.Y), C.float(position.Z),
		C.float(rotation.X), C.float(rotation.Y), C.float(rotation.Z), C.float(rotation.W),
	)
	if handle == nil {
		return nil
	}
	return &Shape{handle: handle}
}

// CreateOffsetCenterOfMassShape wraps `inner` with a shifted center of mass.
// Collision geometry is unchanged; only the mass distribution is offset. Use
// this to stabilize top-heavy objects (e.g. lower the COM of a boat hull).
func CreateOffsetCenterOfMassShape(inner *Shape, offset Vec3) *Shape {
	if inner == nil {
		return nil
	}
	handle := C.JoltCreateOffsetCenterOfMassShape(
		inner.handle,
		C.float(offset.X), C.float(offset.Y), C.float(offset.Z),
	)
	if handle == nil {
		return nil
	}
	return &Shape{handle: handle}
}

// CreateMutableCompound creates a compound shape whose sub-shapes may be
// added, removed, and re-posed at runtime without rebuilding the tree.
// Query performance is lower than StaticCompound; prefer StaticCompound for
// read-only geometry.
//
// Slices may be empty to create an empty compound that sub-shapes are added
// to later via MutableCompoundAddShape. When non-empty, all three slices must
// have the same length.
//
// Thread-safety: mutations are NOT thread-safe. The caller is responsible for
// holding a BodyLockWrite on any body using the shape during mutation and
// calling PhysicsSystem.NotifyShapeChanged after.
func CreateMutableCompound(shapes []*Shape, positions []Vec3, rotations []Quat) *Shape {
	n := len(shapes)
	if n != len(positions) || n != len(rotations) {
		return nil
	}

	var (
		cShapesPtr *C.JoltShape
		cPosPtr    *C.float
		cRotPtr    *C.float
	)

	if n > 0 {
		cShapes := make([]C.JoltShape, n)
		cPos := make([]C.float, n*3)
		cRot := make([]C.float, n*4)
		for i := range n {
			if shapes[i] == nil {
				return nil
			}
			cShapes[i] = shapes[i].handle
			cPos[i*3+0] = C.float(positions[i].X)
			cPos[i*3+1] = C.float(positions[i].Y)
			cPos[i*3+2] = C.float(positions[i].Z)
			cRot[i*4+0] = C.float(rotations[i].X)
			cRot[i*4+1] = C.float(rotations[i].Y)
			cRot[i*4+2] = C.float(rotations[i].Z)
			cRot[i*4+3] = C.float(rotations[i].W)
		}
		cShapesPtr = &cShapes[0]
		cPosPtr = &cPos[0]
		cRotPtr = &cRot[0]
	}

	handle := C.JoltCreateMutableCompound(cShapesPtr, cPosPtr, cRotPtr, C.int(n))
	if handle == nil {
		return nil
	}
	return &Shape{handle: handle}
}

// AddSubShape appends a sub-shape to a MutableCompound and returns the new
// sub-shape index. Returns ErrNotMutableCompound if the receiver is not a
// MutableCompound.
//
// The sub-shape's ref count is incremented by Jolt; callers are still free to
// Destroy() their local reference.
func (s *Shape) AddSubShape(position Vec3, rotation Quat, sub *Shape) (uint32, error) {
	if s == nil || sub == nil {
		return 0, ErrNotMutableCompound
	}
	idx := C.JoltMutableCompoundAddShape(
		s.handle,
		C.float(position.X), C.float(position.Y), C.float(position.Z),
		C.float(rotation.X), C.float(rotation.Y), C.float(rotation.Z), C.float(rotation.W),
		sub.handle,
	)
	if uint32(idx) == invalidSubShapeIndex {
		return 0, ErrNotMutableCompound
	}
	return uint32(idx), nil
}

// RemoveSubShape removes the sub-shape at the given index from a MutableCompound.
// Note: removing a sub-shape renumbers subsequent shapes, so stored indices into
// the same compound may shift.
func (s *Shape) RemoveSubShape(index uint32) error {
	if s == nil {
		return ErrNotMutableCompound
	}
	if C.JoltMutableCompoundRemoveShape(s.handle, C.uint(index)) != 0 {
		return ErrNotMutableCompound
	}
	return nil
}

// ModifySubShape updates the position and rotation of an existing sub-shape
// without changing the underlying geometry.
func (s *Shape) ModifySubShape(index uint32, position Vec3, rotation Quat) error {
	if s == nil {
		return ErrNotMutableCompound
	}
	rc := C.JoltMutableCompoundModifyShape(
		s.handle, C.uint(index),
		C.float(position.X), C.float(position.Y), C.float(position.Z),
		C.float(rotation.X), C.float(rotation.Y), C.float(rotation.Z), C.float(rotation.W),
	)
	if rc != 0 {
		return ErrNotMutableCompound
	}
	return nil
}

// ModifySubShapeWithShape updates the position, rotation, and underlying
// geometry of a sub-shape in one call.
func (s *Shape) ModifySubShapeWithShape(index uint32, position Vec3, rotation Quat, sub *Shape) error {
	if s == nil || sub == nil {
		return ErrNotMutableCompound
	}
	rc := C.JoltMutableCompoundModifyShapeWithShape(
		s.handle, C.uint(index),
		C.float(position.X), C.float(position.Y), C.float(position.Z),
		C.float(rotation.X), C.float(rotation.Y), C.float(rotation.Z), C.float(rotation.W),
		sub.handle,
	)
	if rc != 0 {
		return ErrNotMutableCompound
	}
	return nil
}

// AdjustCenterOfMass recalculates and shifts the center of mass of a
// MutableCompound. Call this after significant topology changes on dynamic
// bodies to keep simulation stable. Callers must also call
// PhysicsSystem.NotifyShapeChanged and Constraint.NotifyShapeChanged on any
// affected bodies/constraints.
func (s *Shape) AdjustCenterOfMass() error {
	if s == nil {
		return ErrNotMutableCompound
	}
	if C.JoltMutableCompoundAdjustCenterOfMass(s.handle) != 0 {
		return ErrNotMutableCompound
	}
	return nil
}

// NumSubShapes returns the number of sub-shapes in a compound shape (static
// or mutable). Returns -1 if the receiver is not a compound shape.
func (s *Shape) NumSubShapes() int {
	if s == nil {
		return -1
	}
	return int(C.JoltCompoundGetNumSubShapes(s.handle))
}
