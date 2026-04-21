package jolt

// #include <stdlib.h>
// #include "wrapper/ragdoll.h"
import "C"

import (
	"errors"
	"unsafe"
)

// Skeleton is a hierarchy of named joints used to build a ragdoll.
// Joints must be added in parent-before-child order.
type Skeleton struct {
	handle C.JoltSkeleton
}

// NewSkeleton creates an empty skeleton.
func NewSkeleton() *Skeleton {
	return &Skeleton{handle: C.JoltCreateSkeleton()}
}

// Destroy releases the skeleton. Safe to call after the RagdollSettings
// that referenced it has been destroyed; Jolt ref-counts the skeleton.
func (s *Skeleton) Destroy() {
	if s.handle != nil {
		C.JoltDestroySkeleton(s.handle)
		s.handle = nil
	}
}

// AddJoint appends a joint to the skeleton.
// parentIndex is the index of an already-added joint, or -1 for the root.
// Returns the new joint's index, or -1 on error.
func (s *Skeleton) AddJoint(name string, parentIndex int) int {
	cName := C.CString(name)
	defer C.free(unsafe.Pointer(cName))
	return int(C.JoltSkeletonAddJoint(s.handle, cName, C.int(parentIndex)))
}

// JointCount returns the number of joints added so far.
func (s *Skeleton) JointCount() int {
	return int(C.JoltSkeletonGetJointCount(s.handle))
}

// ParentIndex returns the parent joint index of the given joint, or -1 for root / out-of-range.
func (s *Skeleton) ParentIndex(jointIndex int) int {
	return int(C.JoltSkeletonGetParentIndex(s.handle, C.int(jointIndex)))
}

// RagdollSettings is the build-time description of a ragdoll: one rigid body
// per joint, plus a constraint connecting each non-root body to its parent.
type RagdollSettings struct {
	handle C.JoltRagdollSettings
}

// NewRagdollSettings allocates a settings object sized for the skeleton's
// current joint count. Skeleton joints added after this call are ignored.
func NewRagdollSettings(skeleton *Skeleton) *RagdollSettings {
	if skeleton == nil || skeleton.handle == nil {
		return nil
	}
	h := C.JoltCreateRagdollSettings(skeleton.handle)
	if h == nil {
		return nil
	}
	return &RagdollSettings{handle: h}
}

// Destroy releases the settings.
func (rs *RagdollSettings) Destroy() {
	if rs.handle != nil {
		C.JoltDestroyRagdollSettings(rs.handle)
		rs.handle = nil
	}
}

// SetPart configures the rigid body for one joint.
// mass <= 0 falls back to the shape's calculated mass.
func (rs *RagdollSettings) SetPart(
	jointIndex int,
	shape *Shape,
	position Vec3,
	rotation Quat,
	motionType MotionType,
	objectLayer uint16,
	mass float32,
) error {
	if shape == nil {
		return errors.New("ragdoll: nil shape")
	}
	rc := C.JoltRagdollSettingsSetPart(
		rs.handle, C.int(jointIndex), shape.handle,
		C.float(position.X), C.float(position.Y), C.float(position.Z),
		C.float(rotation.X), C.float(rotation.Y), C.float(rotation.Z), C.float(rotation.W),
		C.int(motionType), C.ushort(objectLayer), C.float(mass),
	)
	if rc != 0 {
		return errors.New("ragdoll: SetPart failed (bad joint index?)")
	}
	return nil
}

// SwingTwistLimits describes the swing/twist range of a SwingTwistConstraint
// in radians. Pivot and axes are in world space at bind time.
//
//	TwistAxis   bone "forward" axis (the limb's long axis)
//	PlaneAxis   perpendicular to TwistAxis (defines the swing plane)
//	NormalHalfConeAngle / PlaneHalfConeAngle  swing limits
//	TwistMin / TwistMax                        twist limits
type SwingTwistLimits struct {
	Pivot               Vec3
	TwistAxis           Vec3
	PlaneAxis           Vec3
	NormalHalfConeAngle float32
	PlaneHalfConeAngle  float32
	TwistMin            float32
	TwistMax            float32
}

// SetSwingTwistConstraint installs a SwingTwistConstraint between this joint's
// body and its parent's body. Errors if the joint has no parent.
func (rs *RagdollSettings) SetSwingTwistConstraint(jointIndex int, limits SwingTwistLimits) error {
	rc := C.JoltRagdollSettingsSetSwingTwistConstraint(
		rs.handle, C.int(jointIndex),
		C.float(limits.Pivot.X), C.float(limits.Pivot.Y), C.float(limits.Pivot.Z),
		C.float(limits.TwistAxis.X), C.float(limits.TwistAxis.Y), C.float(limits.TwistAxis.Z),
		C.float(limits.PlaneAxis.X), C.float(limits.PlaneAxis.Y), C.float(limits.PlaneAxis.Z),
		C.float(limits.NormalHalfConeAngle), C.float(limits.PlaneHalfConeAngle),
		C.float(limits.TwistMin), C.float(limits.TwistMax),
	)
	if rc != 0 {
		return errors.New("ragdoll: SetSwingTwistConstraint failed (root joint or bad index?)")
	}
	return nil
}

// Finalize stabilizes mass ratios, builds index tables, and disables
// parent/child collisions. Call once after all parts and constraints are set,
// before NewRagdoll.
func (rs *RagdollSettings) Finalize() error {
	if C.JoltRagdollSettingsFinalize(rs.handle) != 0 {
		return errors.New("ragdoll: Finalize/Stabilize failed")
	}
	return nil
}

// Ragdoll is a runtime instance of a ragdoll, owning a body per skeleton joint
// plus the connecting constraints.
type Ragdoll struct {
	handle C.JoltRagdoll
}

// NewRagdoll instantiates a ragdoll in the given physics system.
// groupID should be unique per ragdoll so the parent/child collision filter
// only excludes within-ragdoll pairs.
// Returns nil if the system is out of body slots.
func NewRagdoll(settings *RagdollSettings, system *PhysicsSystem, groupID uint32) *Ragdoll {
	if settings == nil || system == nil {
		return nil
	}
	h := C.JoltCreateRagdoll(settings.handle, system.handle, C.uint(groupID), C.ulonglong(0))
	if h == nil {
		return nil
	}
	return &Ragdoll{handle: h}
}

// Destroy releases the ragdoll. RemoveFromPhysicsSystem first if it was added.
func (r *Ragdoll) Destroy() {
	if r.handle != nil {
		C.JoltDestroyRagdoll(r.handle)
		r.handle = nil
	}
}

// AddToPhysicsSystem inserts all bodies and constraints into the physics system.
func (r *Ragdoll) AddToPhysicsSystem(activate bool) {
	a := C.int(0)
	if activate {
		a = 1
	}
	C.JoltRagdollAddToPhysicsSystem(r.handle, a)
}

// RemoveFromPhysicsSystem removes all bodies and constraints from the system.
func (r *Ragdoll) RemoveFromPhysicsSystem() {
	C.JoltRagdollRemoveFromPhysicsSystem(r.handle)
}

// Activate wakes all bodies in the ragdoll.
func (r *Ragdoll) Activate() {
	C.JoltRagdollActivate(r.handle)
}

// IsActive reports whether at least one body in the ragdoll is currently active.
func (r *Ragdoll) IsActive() bool {
	return C.JoltRagdollIsActive(r.handle) != 0
}

// BodyCount returns the number of bodies (== skeleton joint count).
func (r *Ragdoll) BodyCount() int {
	return int(C.JoltRagdollGetBodyCount(r.handle))
}

// BodyID returns a newly-allocated BodyID for the given body index.
// Caller must Destroy() the returned BodyID when done. Returns nil on out-of-range.
func (r *Ragdoll) BodyID(index int) *BodyID {
	h := C.JoltRagdollGetBodyID(r.handle, C.int(index))
	if h == nil {
		return nil
	}
	return &BodyID{handle: h}
}

// AddImpulse applies an instantaneous impulse to every body in the ragdoll.
func (r *Ragdoll) AddImpulse(impulse Vec3) {
	C.JoltRagdollAddImpulse(r.handle, C.float(impulse.X), C.float(impulse.Y), C.float(impulse.Z))
}

// SetLinearVelocity sets the linear velocity of every body in the ragdoll.
func (r *Ragdoll) SetLinearVelocity(velocity Vec3) {
	C.JoltRagdollSetLinearVelocity(r.handle, C.float(velocity.X), C.float(velocity.Y), C.float(velocity.Z))
}

// RootTransform returns the position and rotation of the root body.
func (r *Ragdoll) RootTransform() (Vec3, Quat) {
	var px, py, pz, qx, qy, qz, qw C.float
	C.JoltRagdollGetRootTransform(r.handle, &px, &py, &pz, &qx, &qy, &qz, &qw)
	return Vec3{X: float32(px), Y: float32(py), Z: float32(pz)},
		Quat{X: float32(qx), Y: float32(qy), Z: float32(qz), W: float32(qw)}
}

// DriveToPose drives the ragdoll toward a target pose using kinematic
// velocities so that each body reaches its target transform in deltaTime
// seconds.
//
// jointMatrices must contain BodyCount() entries; each is a 4x4 column-major
// world-space transform: [col0(xyz,0), col1(xyz,0), col2(xyz,0), translation(xyz,1)].
// Returns an error if the matrix count does not match BodyCount().
func (r *Ragdoll) DriveToPose(rootOffset Vec3, jointMatrices [][16]float32, deltaTime float32) error {
	count := len(jointMatrices)
	if count == 0 {
		return errors.New("ragdoll: empty jointMatrices")
	}
	flat := make([]C.float, count*16)
	for i := range jointMatrices {
		for j := range 16 {
			flat[i*16+j] = C.float(jointMatrices[i][j])
		}
	}
	rc := C.JoltRagdollDriveToPoseUsingKinematics(
		r.handle,
		C.float(rootOffset.X), C.float(rootOffset.Y), C.float(rootOffset.Z),
		&flat[0], C.int(count),
		C.float(deltaTime),
	)
	if rc != 0 {
		return errors.New("ragdoll: DriveToPose joint count mismatch")
	}
	return nil
}
