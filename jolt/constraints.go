package jolt

// #include "wrapper/constraints.h"
// #include <stdlib.h>
import "C"

import "unsafe"

// MotorState controls how a constraint motor behaves
type MotorState int

const (
	MotorStateOff      MotorState = C.JoltMotorStateOff      // Motor is off
	MotorStateVelocity MotorState = C.JoltMotorStateVelocity // Motor drives to target velocity
	MotorStatePosition MotorState = C.JoltMotorStatePosition // Motor drives to target position
)

// Constraint represents a physics constraint between two bodies.
// Call Destroy() when done to release the constraint.
type Constraint struct {
	handle C.JoltConstraint
}

// Destroy releases the constraint. Must be called after RemoveConstraint.
func (c *Constraint) Destroy() {
	if c.handle != nil {
		C.JoltDestroyConstraint(c.handle)
		c.handle = nil
	}
}

// AddConstraint registers a constraint with the physics system.
func (ps *PhysicsSystem) AddConstraint(c *Constraint) {
	C.JoltAddConstraint(ps.handle, c.handle)
}

// RemoveConstraint unregisters a constraint from the physics system.
// Call Destroy() on the constraint afterward to free it.
func (ps *PhysicsSystem) RemoveConstraint(c *Constraint) {
	C.JoltRemoveConstraint(ps.handle, c.handle)
}

// --- Distance Constraint ---

// CreateDistanceConstraint creates a constraint that maintains a distance between two bodies.
// point1/point2 are world-space attachment points on each body.
// Set minDistance and maxDistance to -1 to auto-detect from initial positions.
func (ps *PhysicsSystem) CreateDistanceConstraint(
	bodyID1, bodyID2 *BodyID,
	point1, point2 Vec3,
	minDistance, maxDistance float32,
) *Constraint {
	handle := C.JoltCreateDistanceConstraint(
		ps.handle,
		bodyID1.handle, bodyID2.handle,
		C.float(point1.X), C.float(point1.Y), C.float(point1.Z),
		C.float(point2.X), C.float(point2.Y), C.float(point2.Z),
		C.float(minDistance), C.float(maxDistance),
	)
	if handle == nil {
		return nil
	}
	return &Constraint{handle: handle}
}

// DistanceConstraintSetDistance updates the min/max distance of a distance constraint
func (c *Constraint) DistanceConstraintSetDistance(minDistance, maxDistance float32) {
	C.JoltDistanceConstraintSetDistance(c.handle, C.float(minDistance), C.float(maxDistance))
}

// DistanceConstraintGetMinDistance returns the minimum distance
func (c *Constraint) DistanceConstraintGetMinDistance() float32 {
	return float32(C.JoltDistanceConstraintGetMinDistance(c.handle))
}

// DistanceConstraintGetMaxDistance returns the maximum distance
func (c *Constraint) DistanceConstraintGetMaxDistance() float32 {
	return float32(C.JoltDistanceConstraintGetMaxDistance(c.handle))
}

// DistanceConstraintSetSpring configures the spring for the distance limits
func (c *Constraint) DistanceConstraintSetSpring(frequency, damping float32) {
	C.JoltDistanceConstraintSetSpring(c.handle, C.float(frequency), C.float(damping))
}

// DistanceConstraintGetTotalLambdaPosition returns the constraint impulse magnitude
func (c *Constraint) DistanceConstraintGetTotalLambdaPosition() float32 {
	return float32(C.JoltDistanceConstraintGetTotalLambdaPosition(c.handle))
}

// --- Hinge Constraint ---

// CreateHingeConstraint creates a rotational constraint around an axis.
// point: world-space pivot point
// hingeAxis: world-space rotation axis (normalized)
// normalAxis: perpendicular to hinge axis (normalized)
// limitsMin/limitsMax: angle limits in radians. Set both to 0 for no limits.
func (ps *PhysicsSystem) CreateHingeConstraint(
	bodyID1, bodyID2 *BodyID,
	point Vec3,
	hingeAxis, normalAxis Vec3,
	limitsMin, limitsMax float32,
) *Constraint {
	handle := C.JoltCreateHingeConstraint(
		ps.handle,
		bodyID1.handle, bodyID2.handle,
		C.float(point.X), C.float(point.Y), C.float(point.Z),
		C.float(hingeAxis.X), C.float(hingeAxis.Y), C.float(hingeAxis.Z),
		C.float(normalAxis.X), C.float(normalAxis.Y), C.float(normalAxis.Z),
		C.float(limitsMin), C.float(limitsMax),
	)
	if handle == nil {
		return nil
	}
	return &Constraint{handle: handle}
}

// HingeGetCurrentAngle returns the current angle of the hinge in radians
func (c *Constraint) HingeGetCurrentAngle() float32 {
	return float32(C.JoltHingeConstraintGetCurrentAngle(c.handle))
}

// HingeSetLimits updates the angular limits of the hinge
func (c *Constraint) HingeSetLimits(limitsMin, limitsMax float32) {
	C.JoltHingeConstraintSetLimits(c.handle, C.float(limitsMin), C.float(limitsMax))
}

// HingeSetMotorState sets the motor mode (Off, Velocity, Position)
func (c *Constraint) HingeSetMotorState(state MotorState) {
	C.JoltHingeConstraintSetMotorState(c.handle, C.JoltMotorState(state))
}

// HingeGetMotorState returns the current motor state
func (c *Constraint) HingeGetMotorState() MotorState {
	return MotorState(C.JoltHingeConstraintGetMotorState(c.handle))
}

// HingeSetTargetAngularVelocity sets the target angular velocity in rad/s (Velocity mode)
func (c *Constraint) HingeSetTargetAngularVelocity(radPerSec float32) {
	C.JoltHingeConstraintSetTargetAngularVelocity(c.handle, C.float(radPerSec))
}

// HingeGetTargetAngularVelocity returns the target angular velocity
func (c *Constraint) HingeGetTargetAngularVelocity() float32 {
	return float32(C.JoltHingeConstraintGetTargetAngularVelocity(c.handle))
}

// HingeSetTargetAngle sets the target angle in radians (Position mode)
func (c *Constraint) HingeSetTargetAngle(radians float32) {
	C.JoltHingeConstraintSetTargetAngle(c.handle, C.float(radians))
}

// HingeGetTargetAngle returns the target angle
func (c *Constraint) HingeGetTargetAngle() float32 {
	return float32(C.JoltHingeConstraintGetTargetAngle(c.handle))
}

// HingeSetMotorSettings configures the motor spring and force/torque limits
func (c *Constraint) HingeSetMotorSettings(frequency, damping, forceLimit, torqueLimit float32) {
	C.JoltHingeConstraintSetMotorSettings(c.handle,
		C.float(frequency), C.float(damping),
		C.float(forceLimit), C.float(torqueLimit))
}

// HingeSetMaxFrictionTorque sets the maximum friction torque for the hinge
func (c *Constraint) HingeSetMaxFrictionTorque(torque float32) {
	C.JoltHingeConstraintSetMaxFrictionTorque(c.handle, C.float(torque))
}

// HingeGetTotalLambdaPosition returns the position constraint impulse vector
func (c *Constraint) HingeGetTotalLambdaPosition() Vec3 {
	var x, y, z C.float
	C.JoltHingeConstraintGetTotalLambdaPosition(c.handle, &x, &y, &z)
	return Vec3{X: float32(x), Y: float32(y), Z: float32(z)}
}

// HingeGetTotalLambdaRotationLimits returns the rotation limits impulse
func (c *Constraint) HingeGetTotalLambdaRotationLimits() float32 {
	return float32(C.JoltHingeConstraintGetTotalLambdaRotationLimits(c.handle))
}

// HingeGetTotalLambdaMotor returns the motor impulse
func (c *Constraint) HingeGetTotalLambdaMotor() float32 {
	return float32(C.JoltHingeConstraintGetTotalLambdaMotor(c.handle))
}

// --- Slider Constraint ---

// CreateSliderConstraint creates a prismatic (sliding) constraint along an axis.
// point: world-space reference point
// sliderAxis: world-space axis along which bodies can slide (normalized)
// limitsMin/limitsMax: position limits along axis. Set both to 0 for no limits.
func (ps *PhysicsSystem) CreateSliderConstraint(
	bodyID1, bodyID2 *BodyID,
	point Vec3,
	sliderAxis Vec3,
	limitsMin, limitsMax float32,
) *Constraint {
	handle := C.JoltCreateSliderConstraint(
		ps.handle,
		bodyID1.handle, bodyID2.handle,
		C.float(point.X), C.float(point.Y), C.float(point.Z),
		C.float(sliderAxis.X), C.float(sliderAxis.Y), C.float(sliderAxis.Z),
		C.float(limitsMin), C.float(limitsMax),
	)
	if handle == nil {
		return nil
	}
	return &Constraint{handle: handle}
}

// SliderGetCurrentPosition returns the current sliding position
func (c *Constraint) SliderGetCurrentPosition() float32 {
	return float32(C.JoltSliderConstraintGetCurrentPosition(c.handle))
}

// SliderSetLimits updates the position limits
func (c *Constraint) SliderSetLimits(limitsMin, limitsMax float32) {
	C.JoltSliderConstraintSetLimits(c.handle, C.float(limitsMin), C.float(limitsMax))
}

// SliderSetMotorState sets the motor mode (Off, Velocity, Position)
func (c *Constraint) SliderSetMotorState(state MotorState) {
	C.JoltSliderConstraintSetMotorState(c.handle, C.JoltMotorState(state))
}

// SliderGetMotorState returns the current motor state
func (c *Constraint) SliderGetMotorState() MotorState {
	return MotorState(C.JoltSliderConstraintGetMotorState(c.handle))
}

// SliderSetTargetVelocity sets the target velocity in m/s (Velocity mode)
func (c *Constraint) SliderSetTargetVelocity(velocity float32) {
	C.JoltSliderConstraintSetTargetVelocity(c.handle, C.float(velocity))
}

// SliderGetTargetVelocity returns the target velocity
func (c *Constraint) SliderGetTargetVelocity() float32 {
	return float32(C.JoltSliderConstraintGetTargetVelocity(c.handle))
}

// SliderSetTargetPosition sets the target position (Position mode)
func (c *Constraint) SliderSetTargetPosition(position float32) {
	C.JoltSliderConstraintSetTargetPosition(c.handle, C.float(position))
}

// SliderGetTargetPosition returns the target position
func (c *Constraint) SliderGetTargetPosition() float32 {
	return float32(C.JoltSliderConstraintGetTargetPosition(c.handle))
}

// SliderSetMotorSettings configures the motor spring and force/torque limits
func (c *Constraint) SliderSetMotorSettings(frequency, damping, forceLimit, torqueLimit float32) {
	C.JoltSliderConstraintSetMotorSettings(c.handle,
		C.float(frequency), C.float(damping),
		C.float(forceLimit), C.float(torqueLimit))
}

// SliderSetMaxFrictionForce sets the maximum friction force for the slider
func (c *Constraint) SliderSetMaxFrictionForce(force float32) {
	C.JoltSliderConstraintSetMaxFrictionForce(c.handle, C.float(force))
}

// SliderGetTotalLambdaPositionLimits returns the position limits impulse
func (c *Constraint) SliderGetTotalLambdaPositionLimits() float32 {
	return float32(C.JoltSliderConstraintGetTotalLambdaPositionLimits(c.handle))
}

// SliderGetTotalLambdaMotor returns the motor impulse
func (c *Constraint) SliderGetTotalLambdaMotor() float32 {
	return float32(C.JoltSliderConstraintGetTotalLambdaMotor(c.handle))
}

// --- Fixed Constraint ---

// CreateFixedConstraint creates a constraint that rigidly connects two bodies.
// point: world-space connection point
func (ps *PhysicsSystem) CreateFixedConstraint(
	bodyID1, bodyID2 *BodyID,
	point Vec3,
) *Constraint {
	handle := C.JoltCreateFixedConstraint(
		ps.handle,
		bodyID1.handle, bodyID2.handle,
		C.float(point.X), C.float(point.Y), C.float(point.Z),
	)
	if handle == nil {
		return nil
	}
	return &Constraint{handle: handle}
}

// FixedGetTotalLambdaPosition returns the position constraint impulse vector
func (c *Constraint) FixedGetTotalLambdaPosition() Vec3 {
	var x, y, z C.float
	C.JoltFixedConstraintGetTotalLambdaPosition(c.handle, &x, &y, &z)
	return Vec3{X: float32(x), Y: float32(y), Z: float32(z)}
}

// FixedGetTotalLambdaRotation returns the rotation constraint impulse vector
func (c *Constraint) FixedGetTotalLambdaRotation() Vec3 {
	var x, y, z C.float
	C.JoltFixedConstraintGetTotalLambdaRotation(c.handle, &x, &y, &z)
	return Vec3{X: float32(x), Y: float32(y), Z: float32(z)}
}

// --- SwingTwist Constraint (T-0123) ---

// SwingType selects how the swing limit is shaped
type SwingType int

const (
	SwingTypeCone    SwingType = C.JoltSwingTypeCone    // Symmetric elliptical cone (Y/Z half-angles)
	SwingTypePyramid SwingType = C.JoltSwingTypePyramid // Independent Y/Z limits forming a pyramid
)

// CreateSwingTwistConstraint creates a humanoid-style joint with swing and twist limits.
// position: world-space attachment point
// twistAxis: world-space twist axis (normalized) - the bone direction
// planeAxis: world-space plane axis (normalized) - perpendicular to twistAxis
// normalHalfConeAngle/planeHalfConeAngle: swing limits in radians (0 = no swing)
// twistMinAngle/twistMaxAngle: twist limits in radians, in [-PI, PI]
func (ps *PhysicsSystem) CreateSwingTwistConstraint(
	bodyID1, bodyID2 *BodyID,
	position Vec3,
	twistAxis, planeAxis Vec3,
	normalHalfConeAngle, planeHalfConeAngle float32,
	twistMinAngle, twistMaxAngle float32,
	swingType SwingType,
) *Constraint {
	handle := C.JoltCreateSwingTwistConstraint(
		ps.handle,
		bodyID1.handle, bodyID2.handle,
		C.float(position.X), C.float(position.Y), C.float(position.Z),
		C.float(twistAxis.X), C.float(twistAxis.Y), C.float(twistAxis.Z),
		C.float(planeAxis.X), C.float(planeAxis.Y), C.float(planeAxis.Z),
		C.float(normalHalfConeAngle), C.float(planeHalfConeAngle),
		C.float(twistMinAngle), C.float(twistMaxAngle),
		C.JoltSwingType(swingType),
	)
	if handle == nil {
		return nil
	}
	return &Constraint{handle: handle}
}

// SwingTwistSetNormalHalfConeAngle updates the normal (Y) swing half-angle
func (c *Constraint) SwingTwistSetNormalHalfConeAngle(angle float32) {
	C.JoltSwingTwistSetNormalHalfConeAngle(c.handle, C.float(angle))
}

// SwingTwistGetNormalHalfConeAngle returns the normal swing half-angle
func (c *Constraint) SwingTwistGetNormalHalfConeAngle() float32 {
	return float32(C.JoltSwingTwistGetNormalHalfConeAngle(c.handle))
}

// SwingTwistSetPlaneHalfConeAngle updates the plane (Z) swing half-angle
func (c *Constraint) SwingTwistSetPlaneHalfConeAngle(angle float32) {
	C.JoltSwingTwistSetPlaneHalfConeAngle(c.handle, C.float(angle))
}

// SwingTwistGetPlaneHalfConeAngle returns the plane swing half-angle
func (c *Constraint) SwingTwistGetPlaneHalfConeAngle() float32 {
	return float32(C.JoltSwingTwistGetPlaneHalfConeAngle(c.handle))
}

// SwingTwistSetTwistMinAngle updates the lower twist limit (radians)
func (c *Constraint) SwingTwistSetTwistMinAngle(angle float32) {
	C.JoltSwingTwistSetTwistMinAngle(c.handle, C.float(angle))
}

// SwingTwistGetTwistMinAngle returns the lower twist limit (radians)
func (c *Constraint) SwingTwistGetTwistMinAngle() float32 {
	return float32(C.JoltSwingTwistGetTwistMinAngle(c.handle))
}

// SwingTwistSetTwistMaxAngle updates the upper twist limit (radians)
func (c *Constraint) SwingTwistSetTwistMaxAngle(angle float32) {
	C.JoltSwingTwistSetTwistMaxAngle(c.handle, C.float(angle))
}

// SwingTwistGetTwistMaxAngle returns the upper twist limit (radians)
func (c *Constraint) SwingTwistGetTwistMaxAngle() float32 {
	return float32(C.JoltSwingTwistGetTwistMaxAngle(c.handle))
}

// SwingTwistSetMaxFrictionTorque sets the maximum friction torque (N·m) when motors are off
func (c *Constraint) SwingTwistSetMaxFrictionTorque(torque float32) {
	C.JoltSwingTwistSetMaxFrictionTorque(c.handle, C.float(torque))
}

// SwingTwistGetMaxFrictionTorque returns the friction torque
func (c *Constraint) SwingTwistGetMaxFrictionTorque() float32 {
	return float32(C.JoltSwingTwistGetMaxFrictionTorque(c.handle))
}

// SwingTwistSetSwingMotorState sets the swing motor mode (Off, Velocity, Position)
func (c *Constraint) SwingTwistSetSwingMotorState(state MotorState) {
	C.JoltSwingTwistSetSwingMotorState(c.handle, C.JoltMotorState(state))
}

// SwingTwistGetSwingMotorState returns the swing motor mode
func (c *Constraint) SwingTwistGetSwingMotorState() MotorState {
	return MotorState(C.JoltSwingTwistGetSwingMotorState(c.handle))
}

// SwingTwistSetTwistMotorState sets the twist motor mode (Off, Velocity, Position)
func (c *Constraint) SwingTwistSetTwistMotorState(state MotorState) {
	C.JoltSwingTwistSetTwistMotorState(c.handle, C.JoltMotorState(state))
}

// SwingTwistGetTwistMotorState returns the twist motor mode
func (c *Constraint) SwingTwistGetTwistMotorState() MotorState {
	return MotorState(C.JoltSwingTwistGetTwistMotorState(c.handle))
}

// SwingTwistSetTargetAngularVelocityCS sets the target angular velocity (rad/s)
// in body 2 constraint space (for Velocity-mode motors)
func (c *Constraint) SwingTwistSetTargetAngularVelocityCS(v Vec3) {
	C.JoltSwingTwistSetTargetAngularVelocityCS(c.handle,
		C.float(v.X), C.float(v.Y), C.float(v.Z))
}

// SwingTwistGetTargetAngularVelocityCS returns the target angular velocity
func (c *Constraint) SwingTwistGetTargetAngularVelocityCS() Vec3 {
	var x, y, z C.float
	C.JoltSwingTwistGetTargetAngularVelocityCS(c.handle, &x, &y, &z)
	return Vec3{X: float32(x), Y: float32(y), Z: float32(z)}
}

// SwingTwistSetTargetOrientationCS sets the target orientation in constraint space
// (for Position-mode motors)
func (c *Constraint) SwingTwistSetTargetOrientationCS(q Quat) {
	C.JoltSwingTwistSetTargetOrientationCS(c.handle,
		C.float(q.X), C.float(q.Y), C.float(q.Z), C.float(q.W))
}

// SwingTwistGetTargetOrientationCS returns the target orientation
func (c *Constraint) SwingTwistGetTargetOrientationCS() Quat {
	var x, y, z, w C.float
	C.JoltSwingTwistGetTargetOrientationCS(c.handle, &x, &y, &z, &w)
	return Quat{X: float32(x), Y: float32(y), Z: float32(z), W: float32(w)}
}

// SwingTwistGetRotationInConstraintSpace returns body 2's rotation relative to
// body 1, expressed in constraint space. Useful for inspecting swing/twist state.
func (c *Constraint) SwingTwistGetRotationInConstraintSpace() Quat {
	var x, y, z, w C.float
	C.JoltSwingTwistGetRotationInConstraintSpace(c.handle, &x, &y, &z, &w)
	return Quat{X: float32(x), Y: float32(y), Z: float32(z), W: float32(w)}
}

// SwingTwistSetSwingMotorSettings configures the swing motor spring/limits
func (c *Constraint) SwingTwistSetSwingMotorSettings(frequency, damping, forceLimit, torqueLimit float32) {
	C.JoltSwingTwistSetSwingMotorSettings(c.handle,
		C.float(frequency), C.float(damping),
		C.float(forceLimit), C.float(torqueLimit))
}

// SwingTwistSetTwistMotorSettings configures the twist motor spring/limits
func (c *Constraint) SwingTwistSetTwistMotorSettings(frequency, damping, forceLimit, torqueLimit float32) {
	C.JoltSwingTwistSetTwistMotorSettings(c.handle,
		C.float(frequency), C.float(damping),
		C.float(forceLimit), C.float(torqueLimit))
}

// SwingTwistGetTotalLambdaPosition returns the position constraint impulse vector
func (c *Constraint) SwingTwistGetTotalLambdaPosition() Vec3 {
	var x, y, z C.float
	C.JoltSwingTwistGetTotalLambdaPosition(c.handle, &x, &y, &z)
	return Vec3{X: float32(x), Y: float32(y), Z: float32(z)}
}

// SwingTwistGetTotalLambdaTwist returns the twist limit impulse magnitude
func (c *Constraint) SwingTwistGetTotalLambdaTwist() float32 {
	return float32(C.JoltSwingTwistGetTotalLambdaTwist(c.handle))
}

// SwingTwistGetTotalLambdaSwingY returns the swing-Y limit impulse magnitude
func (c *Constraint) SwingTwistGetTotalLambdaSwingY() float32 {
	return float32(C.JoltSwingTwistGetTotalLambdaSwingY(c.handle))
}

// SwingTwistGetTotalLambdaSwingZ returns the swing-Z limit impulse magnitude
func (c *Constraint) SwingTwistGetTotalLambdaSwingZ() float32 {
	return float32(C.JoltSwingTwistGetTotalLambdaSwingZ(c.handle))
}

// SwingTwistGetTotalLambdaMotor returns the motor impulse vector (twist, swingY, swingZ)
func (c *Constraint) SwingTwistGetTotalLambdaMotor() Vec3 {
	var x, y, z C.float
	C.JoltSwingTwistGetTotalLambdaMotor(c.handle, &x, &y, &z)
	return Vec3{X: float32(x), Y: float32(y), Z: float32(z)}
}

// --- SixDOF Constraint (T-0123) ---

// SixDOFAxis indexes the 6 degrees of freedom (3 translation + 3 rotation)
type SixDOFAxis int

const (
	SixDOFAxisTranslationX SixDOFAxis = C.JoltSixDOFAxisTranslationX
	SixDOFAxisTranslationY SixDOFAxis = C.JoltSixDOFAxisTranslationY
	SixDOFAxisTranslationZ SixDOFAxis = C.JoltSixDOFAxisTranslationZ
	SixDOFAxisRotationX    SixDOFAxis = C.JoltSixDOFAxisRotationX
	SixDOFAxisRotationY    SixDOFAxis = C.JoltSixDOFAxisRotationY
	SixDOFAxisRotationZ    SixDOFAxis = C.JoltSixDOFAxisRotationZ
)

// SixDOFAxisCount is the number of axes (6)
const SixDOFAxisCount = 6

// SixDOFFreeMin is the sentinel for a free axis (use with SixDOFFreeMax)
const SixDOFFreeMin float32 = -3.4028235e+38 // -FLT_MAX

// SixDOFFreeMax is the sentinel for a free axis (use with SixDOFFreeMin)
const SixDOFFreeMax float32 = 3.4028235e+38 // FLT_MAX

// SixDOFLimits describes per-axis limits for a 6-DOF constraint.
//   - Free axis: min = SixDOFFreeMin, max = SixDOFFreeMax
//   - Fixed axis: min = SixDOFFreeMax, max = SixDOFFreeMin (any min > max → driven to 0)
//   - Limited axis: any other valid range
type SixDOFLimits struct {
	Min [SixDOFAxisCount]float32
	Max [SixDOFAxisCount]float32
}

// MakeFreeAxis marks the axis as unconstrained
func (l *SixDOFLimits) MakeFreeAxis(axis SixDOFAxis) {
	l.Min[axis] = SixDOFFreeMin
	l.Max[axis] = SixDOFFreeMax
}

// MakeFixedAxis locks the axis at value 0
func (l *SixDOFLimits) MakeFixedAxis(axis SixDOFAxis) {
	l.Min[axis] = SixDOFFreeMax
	l.Max[axis] = SixDOFFreeMin
}

// SetLimitedAxis sets a finite range for the axis
func (l *SixDOFLimits) SetLimitedAxis(axis SixDOFAxis, min, max float32) {
	l.Min[axis] = min
	l.Max[axis] = max
}

// AllFreeSixDOFLimits returns a SixDOFLimits with every axis free
func AllFreeSixDOFLimits() SixDOFLimits {
	var l SixDOFLimits
	for i := range SixDOFAxisCount {
		l.Min[i] = SixDOFFreeMin
		l.Max[i] = SixDOFFreeMax
	}
	return l
}

// CreateSixDOFConstraint creates a 6 degree-of-freedom constraint between two bodies.
// position: world-space attachment point
// axisX/axisY: orthonormal axes defining the constraint frame (axisZ = axisX × axisY)
// limits: per-axis min/max (use SixDOFLimits helpers to mark free/fixed axes)
// swingType: how the swing-Y/Z limits combine (Cone or Pyramid)
func (ps *PhysicsSystem) CreateSixDOFConstraint(
	bodyID1, bodyID2 *BodyID,
	position Vec3,
	axisX, axisY Vec3,
	limits SixDOFLimits,
	swingType SwingType,
) *Constraint {
	minPtr := (*C.float)(&limits.Min[0])
	maxPtr := (*C.float)(&limits.Max[0])
	handle := C.JoltCreateSixDOFConstraint(
		ps.handle,
		bodyID1.handle, bodyID2.handle,
		C.float(position.X), C.float(position.Y), C.float(position.Z),
		C.float(axisX.X), C.float(axisX.Y), C.float(axisX.Z),
		C.float(axisY.X), C.float(axisY.Y), C.float(axisY.Z),
		minPtr, maxPtr,
		C.JoltSwingType(swingType),
	)
	if handle == nil {
		return nil
	}
	return &Constraint{handle: handle}
}

// SixDOFSetTranslationLimits updates per-axis translation limits
func (c *Constraint) SixDOFSetTranslationLimits(min, max Vec3) {
	C.JoltSixDOFSetTranslationLimits(c.handle,
		C.float(min.X), C.float(min.Y), C.float(min.Z),
		C.float(max.X), C.float(max.Y), C.float(max.Z))
}

// SixDOFSetRotationLimits updates per-axis rotation limits (radians)
func (c *Constraint) SixDOFSetRotationLimits(min, max Vec3) {
	C.JoltSixDOFSetRotationLimits(c.handle,
		C.float(min.X), C.float(min.Y), C.float(min.Z),
		C.float(max.X), C.float(max.Y), C.float(max.Z))
}

// SixDOFGetLimitsMin returns the lower limit for an axis
func (c *Constraint) SixDOFGetLimitsMin(axis SixDOFAxis) float32 {
	return float32(C.JoltSixDOFGetLimitsMin(c.handle, C.JoltSixDOFAxis(axis)))
}

// SixDOFGetLimitsMax returns the upper limit for an axis
func (c *Constraint) SixDOFGetLimitsMax(axis SixDOFAxis) float32 {
	return float32(C.JoltSixDOFGetLimitsMax(c.handle, C.JoltSixDOFAxis(axis)))
}

// SixDOFIsFixedAxis reports whether an axis is fixed (driven to 0)
func (c *Constraint) SixDOFIsFixedAxis(axis SixDOFAxis) bool {
	return C.JoltSixDOFIsFixedAxis(c.handle, C.JoltSixDOFAxis(axis)) != 0
}

// SixDOFIsFreeAxis reports whether an axis is free (no limits)
func (c *Constraint) SixDOFIsFreeAxis(axis SixDOFAxis) bool {
	return C.JoltSixDOFIsFreeAxis(c.handle, C.JoltSixDOFAxis(axis)) != 0
}

// SixDOFSetMaxFriction sets the per-axis friction (N for translation, N·m for rotation)
func (c *Constraint) SixDOFSetMaxFriction(axis SixDOFAxis, friction float32) {
	C.JoltSixDOFSetMaxFriction(c.handle, C.JoltSixDOFAxis(axis), C.float(friction))
}

// SixDOFGetMaxFriction returns the per-axis friction
func (c *Constraint) SixDOFGetMaxFriction(axis SixDOFAxis) float32 {
	return float32(C.JoltSixDOFGetMaxFriction(c.handle, C.JoltSixDOFAxis(axis)))
}

// SixDOFSetMotorState sets the per-axis motor mode (Off, Velocity, Position)
func (c *Constraint) SixDOFSetMotorState(axis SixDOFAxis, state MotorState) {
	C.JoltSixDOFSetMotorState(c.handle, C.JoltSixDOFAxis(axis), C.JoltMotorState(state))
}

// SixDOFGetMotorState returns the per-axis motor mode
func (c *Constraint) SixDOFGetMotorState(axis SixDOFAxis) MotorState {
	return MotorState(C.JoltSixDOFGetMotorState(c.handle, C.JoltSixDOFAxis(axis)))
}

// SixDOFSetMotorSettings configures the per-axis motor spring and force/torque limits
func (c *Constraint) SixDOFSetMotorSettings(axis SixDOFAxis,
	frequency, damping, forceLimit, torqueLimit float32) {
	C.JoltSixDOFSetMotorSettings(c.handle, C.JoltSixDOFAxis(axis),
		C.float(frequency), C.float(damping),
		C.float(forceLimit), C.float(torqueLimit))
}

// SixDOFSetTargetVelocityCS sets the linear target velocity in body 1 constraint space
func (c *Constraint) SixDOFSetTargetVelocityCS(v Vec3) {
	C.JoltSixDOFSetTargetVelocityCS(c.handle, C.float(v.X), C.float(v.Y), C.float(v.Z))
}

// SixDOFGetTargetVelocityCS returns the linear target velocity
func (c *Constraint) SixDOFGetTargetVelocityCS() Vec3 {
	var x, y, z C.float
	C.JoltSixDOFGetTargetVelocityCS(c.handle, &x, &y, &z)
	return Vec3{X: float32(x), Y: float32(y), Z: float32(z)}
}

// SixDOFSetTargetAngularVelocityCS sets the angular target velocity in body 2 constraint space
func (c *Constraint) SixDOFSetTargetAngularVelocityCS(v Vec3) {
	C.JoltSixDOFSetTargetAngularVelocityCS(c.handle, C.float(v.X), C.float(v.Y), C.float(v.Z))
}

// SixDOFGetTargetAngularVelocityCS returns the angular target velocity
func (c *Constraint) SixDOFGetTargetAngularVelocityCS() Vec3 {
	var x, y, z C.float
	C.JoltSixDOFGetTargetAngularVelocityCS(c.handle, &x, &y, &z)
	return Vec3{X: float32(x), Y: float32(y), Z: float32(z)}
}

// SixDOFSetTargetPositionCS sets the linear target position in body 1 constraint space
func (c *Constraint) SixDOFSetTargetPositionCS(v Vec3) {
	C.JoltSixDOFSetTargetPositionCS(c.handle, C.float(v.X), C.float(v.Y), C.float(v.Z))
}

// SixDOFGetTargetPositionCS returns the linear target position
func (c *Constraint) SixDOFGetTargetPositionCS() Vec3 {
	var x, y, z C.float
	C.JoltSixDOFGetTargetPositionCS(c.handle, &x, &y, &z)
	return Vec3{X: float32(x), Y: float32(y), Z: float32(z)}
}

// SixDOFSetTargetOrientationCS sets the orientation target in body 1 constraint space
func (c *Constraint) SixDOFSetTargetOrientationCS(q Quat) {
	C.JoltSixDOFSetTargetOrientationCS(c.handle,
		C.float(q.X), C.float(q.Y), C.float(q.Z), C.float(q.W))
}

// SixDOFGetTargetOrientationCS returns the orientation target
func (c *Constraint) SixDOFGetTargetOrientationCS() Quat {
	var x, y, z, w C.float
	C.JoltSixDOFGetTargetOrientationCS(c.handle, &x, &y, &z, &w)
	return Quat{X: float32(x), Y: float32(y), Z: float32(z), W: float32(w)}
}

// SixDOFGetRotationInConstraintSpace returns body 2's rotation in constraint space
func (c *Constraint) SixDOFGetRotationInConstraintSpace() Quat {
	var x, y, z, w C.float
	C.JoltSixDOFGetRotationInConstraintSpace(c.handle, &x, &y, &z, &w)
	return Quat{X: float32(x), Y: float32(y), Z: float32(z), W: float32(w)}
}

// SixDOFGetTotalLambdaPosition returns the position constraint impulse vector
func (c *Constraint) SixDOFGetTotalLambdaPosition() Vec3 {
	var x, y, z C.float
	C.JoltSixDOFGetTotalLambdaPosition(c.handle, &x, &y, &z)
	return Vec3{X: float32(x), Y: float32(y), Z: float32(z)}
}

// SixDOFGetTotalLambdaRotation returns the rotation constraint impulse vector
func (c *Constraint) SixDOFGetTotalLambdaRotation() Vec3 {
	var x, y, z C.float
	C.JoltSixDOFGetTotalLambdaRotation(c.handle, &x, &y, &z)
	return Vec3{X: float32(x), Y: float32(y), Z: float32(z)}
}

// SixDOFGetTotalLambdaMotorTranslation returns the translation motor impulse vector
func (c *Constraint) SixDOFGetTotalLambdaMotorTranslation() Vec3 {
	var x, y, z C.float
	C.JoltSixDOFGetTotalLambdaMotorTranslation(c.handle, &x, &y, &z)
	return Vec3{X: float32(x), Y: float32(y), Z: float32(z)}
}

// SixDOFGetTotalLambdaMotorRotation returns the rotation motor impulse vector
func (c *Constraint) SixDOFGetTotalLambdaMotorRotation() Vec3 {
	var x, y, z C.float
	C.JoltSixDOFGetTotalLambdaMotorRotation(c.handle, &x, &y, &z)
	return Vec3{X: float32(x), Y: float32(y), Z: float32(z)}
}

// --- Point Constraint (T-0122) ---

// CreatePointConstraint creates a ball-joint constraint that removes
// 3 linear DOFs between two bodies while leaving all rotational DOFs free.
// point: world-space attachment point shared by both bodies.
func (ps *PhysicsSystem) CreatePointConstraint(
	bodyID1, bodyID2 *BodyID,
	point Vec3,
) *Constraint {
	handle := C.JoltCreatePointConstraint(
		ps.handle,
		bodyID1.handle, bodyID2.handle,
		C.float(point.X), C.float(point.Y), C.float(point.Z),
	)
	if handle == nil {
		return nil
	}
	return &Constraint{handle: handle}
}

// PointGetTotalLambdaPosition returns the position constraint impulse vector
func (c *Constraint) PointGetTotalLambdaPosition() Vec3 {
	var x, y, z C.float
	C.JoltPointConstraintGetTotalLambdaPosition(c.handle, &x, &y, &z)
	return Vec3{X: float32(x), Y: float32(y), Z: float32(z)}
}

// --- Cone Constraint (T-0122) ---

// CreateConeConstraint creates a swing-limited ball-joint between two bodies.
// It removes 3 linear DOFs and clamps the swing between the two bodies'
// twist axes to within halfAngleMax radians ([0, pi]).
// point: world-space attachment point shared by both bodies.
// twistAxis: world-space twist axis shared by both bodies (should be normalized).
// halfAngleMax: maximum allowed swing half-angle, in radians.
func (ps *PhysicsSystem) CreateConeConstraint(
	bodyID1, bodyID2 *BodyID,
	point Vec3,
	twistAxis Vec3,
	halfAngleMax float32,
) *Constraint {
	handle := C.JoltCreateConeConstraint(
		ps.handle,
		bodyID1.handle, bodyID2.handle,
		C.float(point.X), C.float(point.Y), C.float(point.Z),
		C.float(twistAxis.X), C.float(twistAxis.Y), C.float(twistAxis.Z),
		C.float(halfAngleMax),
	)
	if handle == nil {
		return nil
	}
	return &Constraint{handle: handle}
}

// ConeSetHalfConeAngle updates the maximum allowed swing half-angle (radians, [0, pi]).
func (c *Constraint) ConeSetHalfConeAngle(halfAngle float32) {
	C.JoltConeConstraintSetHalfConeAngle(c.handle, C.float(halfAngle))
}

// ConeGetCosHalfConeAngle returns the cosine of the configured half-cone angle.
func (c *Constraint) ConeGetCosHalfConeAngle() float32 {
	return float32(C.JoltConeConstraintGetCosHalfConeAngle(c.handle))
}

// ConeGetTotalLambdaPosition returns the position constraint impulse vector
func (c *Constraint) ConeGetTotalLambdaPosition() Vec3 {
	var x, y, z C.float
	C.JoltConeConstraintGetTotalLambdaPosition(c.handle, &x, &y, &z)
	return Vec3{X: float32(x), Y: float32(y), Z: float32(z)}
}

// ConeGetTotalLambdaRotation returns the swing-limit rotation impulse magnitude
func (c *Constraint) ConeGetTotalLambdaRotation() float32 {
	return float32(C.JoltConeConstraintGetTotalLambdaRotation(c.handle))
}

// --- Pulley Constraint ---

// CreatePulleyConstraint creates a rope-over-pulley coupling between two bodies.
//
// The constraint enforces MinLength <= L1 + ratio*L2 <= MaxLength, where
// L1 = |bodyPoint1 - fixedPoint1| and L2 = |bodyPoint2 - fixedPoint2|. This
// models a rope that passes over one or two fixed pulley anchors.
//
// Parameters:
//   - bodyPoint1/bodyPoint2: world-space attachment points on each body
//   - fixedPoint1/fixedPoint2: world-space fixed pulley anchor positions.
//     Use the same value for both to model a single pulley wheel.
//   - ratio: segment-2 length multiplier. 1 = plain pulley (Atwood machine);
//     >1 = block-and-tackle — moving body 1 by X moves body 2 by X/ratio,
//     trading speed for mechanical advantage.
//   - minLength/maxLength: bounds on the combined length. Pass -1 to
//     auto-detect from the initial configuration; set min == max for a
//     rigid rope.
//
// Typical use cases: well buckets with counterweight, portcullis chains,
// drawbridge lifts, trebuchet counterweights.
func (ps *PhysicsSystem) CreatePulleyConstraint(
	bodyID1, bodyID2 *BodyID,
	bodyPoint1, bodyPoint2 Vec3,
	fixedPoint1, fixedPoint2 Vec3,
	ratio float32,
	minLength, maxLength float32,
) *Constraint {
	handle := C.JoltCreatePulleyConstraint(
		ps.handle,
		bodyID1.handle, bodyID2.handle,
		C.float(bodyPoint1.X), C.float(bodyPoint1.Y), C.float(bodyPoint1.Z),
		C.float(bodyPoint2.X), C.float(bodyPoint2.Y), C.float(bodyPoint2.Z),
		C.float(fixedPoint1.X), C.float(fixedPoint1.Y), C.float(fixedPoint1.Z),
		C.float(fixedPoint2.X), C.float(fixedPoint2.Y), C.float(fixedPoint2.Z),
		C.float(ratio),
		C.float(minLength), C.float(maxLength),
	)
	if handle == nil {
		return nil
	}
	return &Constraint{handle: handle}
}

// PulleySetLength updates the min/max length bounds. Both must be non-negative
// and min <= max.
func (c *Constraint) PulleySetLength(minLength, maxLength float32) {
	C.JoltPulleyConstraintSetLength(c.handle, C.float(minLength), C.float(maxLength))
}

// PulleyGetMinLength returns the minimum allowed combined length.
func (c *Constraint) PulleyGetMinLength() float32 {
	return float32(C.JoltPulleyConstraintGetMinLength(c.handle))
}

// PulleyGetMaxLength returns the maximum allowed combined length.
func (c *Constraint) PulleyGetMaxLength() float32 {
	return float32(C.JoltPulleyConstraintGetMaxLength(c.handle))
}

// PulleyGetCurrentLength returns the current combined length
// (|body1 - fixed1| + ratio * |body2 - fixed2|).
func (c *Constraint) PulleyGetCurrentLength() float32 {
	return float32(C.JoltPulleyConstraintGetCurrentLength(c.handle))
}

// PulleyGetTotalLambdaPosition returns the constraint impulse magnitude from
// the last solver step.
func (c *Constraint) PulleyGetTotalLambdaPosition() float32 {
	return float32(C.JoltPulleyConstraintGetTotalLambdaPosition(c.handle))
}

// --- Path Constraint (Hermite spline) ---

// PathRotationConstraintType determines how body 2's rotation is constrained along the path
type PathRotationConstraintType int

const (
	PathRotationFree                    PathRotationConstraintType = C.JoltPathRotationFree
	PathRotationConstrainAroundTangent  PathRotationConstraintType = C.JoltPathRotationConstrainAroundTangent
	PathRotationConstrainAroundNormal   PathRotationConstraintType = C.JoltPathRotationConstrainAroundNormal
	PathRotationConstrainAroundBinormal PathRotationConstraintType = C.JoltPathRotationConstrainAroundBinormal
	PathRotationConstrainToPath         PathRotationConstraintType = C.JoltPathRotationConstrainToPath
	PathRotationFullyConstrained        PathRotationConstraintType = C.JoltPathRotationFullyConstrained
)

// HermitePathPoint is a single control point on a Hermite-spline path.
// Tangent points along the direction of travel and does not need to be normalized.
// Normal is perpendicular to Tangent and together they form the constraint basis
// (useful for constraining body orientation along the path).
type HermitePathPoint struct {
	Position Vec3
	Tangent  Vec3
	Normal   Vec3
}

// HermitePath is a reference-counted Hermite-spline path for PathConstraint.
// Call Destroy() when done — the path ref is shared with any constraint that uses it,
// so Jolt keeps it alive until both the Go-side and all constraints release their refs.
type HermitePath struct {
	handle C.JoltPathConstraintPath
}

// CreateHermitePath builds a Hermite spline from at least 2 control points.
// If looping is true, the first and last point are auto-connected (they must not be identical).
func CreateHermitePath(points []HermitePathPoint, looping bool) *HermitePath {
	n := len(points)
	if n < 2 {
		return nil
	}

	positions := make([]C.float, 3*n)
	tangents := make([]C.float, 3*n)
	normals := make([]C.float, 3*n)
	for i, p := range points {
		positions[3*i+0] = C.float(p.Position.X)
		positions[3*i+1] = C.float(p.Position.Y)
		positions[3*i+2] = C.float(p.Position.Z)
		tangents[3*i+0] = C.float(p.Tangent.X)
		tangents[3*i+1] = C.float(p.Tangent.Y)
		tangents[3*i+2] = C.float(p.Tangent.Z)
		normals[3*i+0] = C.float(p.Normal.X)
		normals[3*i+1] = C.float(p.Normal.Y)
		normals[3*i+2] = C.float(p.Normal.Z)
	}

	loop := C.int(0)
	if looping {
		loop = 1
	}
	handle := C.JoltCreateHermitePath(
		(*C.float)(unsafe.Pointer(&positions[0])),
		(*C.float)(unsafe.Pointer(&tangents[0])),
		(*C.float)(unsafe.Pointer(&normals[0])),
		C.int(n), loop,
	)
	if handle == nil {
		return nil
	}
	return &HermitePath{handle: handle}
}

// Destroy releases this Go-side reference to the path.
// Safe to call after the path has been passed to CreatePathConstraint.
func (p *HermitePath) Destroy() {
	if p.handle != nil {
		C.JoltDestroyPath(p.handle)
		p.handle = nil
	}
}

// MaxFraction returns the highest valid fraction for this path.
// For a non-looping N-point path this is N-1; for a looping path it is N.
func (p *HermitePath) MaxFraction() float32 {
	return float32(C.JoltPathGetMaxFraction(p.handle))
}

// PointOnPath evaluates the position and tangent at a given fraction along the path.
func (p *HermitePath) PointOnPath(fraction float32) (position, tangent Vec3) {
	var px, py, pz, tx, ty, tz C.float
	C.JoltPathGetPointOnPath(p.handle, C.float(fraction),
		&px, &py, &pz, &tx, &ty, &tz)
	return Vec3{X: float32(px), Y: float32(py), Z: float32(pz)},
		Vec3{X: float32(tx), Y: float32(ty), Z: float32(tz)}
}

// CreatePathConstraint attaches body2 to a Hermite path anchored on body1.
//
// pathPosition / pathRotation give the path's origin in body1's local space
// (pass {0,0,0} and QuatIdentity() to anchor at body1's origin).
// pathFraction is the starting fraction for body2 along the path (0 = beginning).
// rotationConstraintType controls how body2's orientation is tied to the path.
// maxFrictionForce is the force clamp applied along the path when the motor is off.
func (ps *PhysicsSystem) CreatePathConstraint(
	bodyID1, bodyID2 *BodyID,
	path *HermitePath,
	pathPosition Vec3,
	pathRotation Quat,
	pathFraction float32,
	rotationConstraintType PathRotationConstraintType,
	maxFrictionForce float32,
) *Constraint {
	if path == nil || path.handle == nil {
		return nil
	}
	handle := C.JoltCreatePathConstraint(
		ps.handle,
		bodyID1.handle, bodyID2.handle,
		path.handle,
		C.float(pathPosition.X), C.float(pathPosition.Y), C.float(pathPosition.Z),
		C.float(pathRotation.X), C.float(pathRotation.Y), C.float(pathRotation.Z), C.float(pathRotation.W),
		C.float(pathFraction),
		C.JoltPathRotationConstraintType(rotationConstraintType),
		C.float(maxFrictionForce),
	)
	if handle == nil {
		return nil
	}
	return &Constraint{handle: handle}
}

// PathSetPath swaps in a new Hermite path, re-anchoring body2 at pathFraction.
func (c *Constraint) PathSetPath(path *HermitePath, pathFraction float32) {
	var h C.JoltPathConstraintPath
	if path != nil {
		h = path.handle
	}
	C.JoltPathConstraintSetPath(c.handle, h, C.float(pathFraction))
}

// PathGetPathFraction returns the current fraction of body2 along the path.
func (c *Constraint) PathGetPathFraction() float32 {
	return float32(C.JoltPathConstraintGetPathFraction(c.handle))
}

// PathSetMaxFrictionForce sets the friction clamp applied along the path when
// the motor is off.
func (c *Constraint) PathSetMaxFrictionForce(force float32) {
	C.JoltPathConstraintSetMaxFrictionForce(c.handle, C.float(force))
}

// PathGetMaxFrictionForce returns the current friction force clamp.
func (c *Constraint) PathGetMaxFrictionForce() float32 {
	return float32(C.JoltPathConstraintGetMaxFrictionForce(c.handle))
}

// PathSetPositionMotorState sets the motor mode (Off, Velocity, Position).
func (c *Constraint) PathSetPositionMotorState(state MotorState) {
	C.JoltPathConstraintSetPositionMotorState(c.handle, C.JoltMotorState(state))
}

// PathGetPositionMotorState returns the current motor state.
func (c *Constraint) PathGetPositionMotorState() MotorState {
	return MotorState(C.JoltPathConstraintGetPositionMotorState(c.handle))
}

// PathSetTargetVelocity sets the target velocity along the path (Velocity mode, fraction/sec).
func (c *Constraint) PathSetTargetVelocity(velocity float32) {
	C.JoltPathConstraintSetTargetVelocity(c.handle, C.float(velocity))
}

// PathGetTargetVelocity returns the target velocity along the path.
func (c *Constraint) PathGetTargetVelocity() float32 {
	return float32(C.JoltPathConstraintGetTargetVelocity(c.handle))
}

// PathSetTargetPathFraction sets the target fraction along the path (Position mode).
func (c *Constraint) PathSetTargetPathFraction(fraction float32) {
	C.JoltPathConstraintSetTargetPathFraction(c.handle, C.float(fraction))
}

// PathGetTargetPathFraction returns the target fraction along the path.
func (c *Constraint) PathGetTargetPathFraction() float32 {
	return float32(C.JoltPathConstraintGetTargetPathFraction(c.handle))
}

// PathSetPositionMotorSettings configures the position motor spring and force limits.
// forceLimit clamps the motor's linear impulse per step; torqueLimit clamps its rotational.
func (c *Constraint) PathSetPositionMotorSettings(frequency, damping, forceLimit, torqueLimit float32) {
	C.JoltPathConstraintSetPositionMotorSettings(c.handle,
		C.float(frequency), C.float(damping),
		C.float(forceLimit), C.float(torqueLimit))
}

// PathGetTotalLambdaMotor returns the motor impulse from the last step.
func (c *Constraint) PathGetTotalLambdaMotor() float32 {
	return float32(C.JoltPathConstraintGetTotalLambdaMotor(c.handle))
}

// PathGetTotalLambdaPositionLimits returns the path-endpoint limit impulse from the last step.
func (c *Constraint) PathGetTotalLambdaPositionLimits() float32 {
	return float32(C.JoltPathConstraintGetTotalLambdaPositionLimits(c.handle))
}

// --- Gear Constraint (T-0127) ---

// CreateGearConstraint couples the rotation of two bodies via a gear ratio.
// Must be paired with two HingeConstraints (one anchoring each body) so each
// gear has a defined rotation axis. Use GearSetConstraints to register those
// hinges with the gear so Jolt can correct numerical drift.
//
// hingeAxis1 / hingeAxis2: world-space rotation axes of the two gears (normalized).
// ratio: defines Gear1Rotation(t) = -ratio * Gear2Rotation(t).
// For a gear pair, ratio = numTeethGear2 / numTeethGear1.
func (ps *PhysicsSystem) CreateGearConstraint(
	bodyID1, bodyID2 *BodyID,
	hingeAxis1, hingeAxis2 Vec3,
	ratio float32,
) *Constraint {
	handle := C.JoltCreateGearConstraint(
		ps.handle,
		bodyID1.handle, bodyID2.handle,
		C.float(hingeAxis1.X), C.float(hingeAxis1.Y), C.float(hingeAxis1.Z),
		C.float(hingeAxis2.X), C.float(hingeAxis2.Y), C.float(hingeAxis2.Z),
		C.float(ratio),
	)
	if handle == nil {
		return nil
	}
	return &Constraint{handle: handle}
}

// GearSetConstraints associates the two hinge constraints constraining the
// gears so Jolt can fix numerical rotation drift. Pass nil to clear a side.
func (c *Constraint) GearSetConstraints(hinge1, hinge2 *Constraint) {
	var h1, h2 C.JoltConstraint
	if hinge1 != nil {
		h1 = hinge1.handle
	}
	if hinge2 != nil {
		h2 = hinge2.handle
	}
	C.JoltGearConstraintSetConstraints(c.handle, h1, h2)
}

// GearGetTotalLambda returns the angular impulse applied by the gear last step.
func (c *Constraint) GearGetTotalLambda() float32 {
	return float32(C.JoltGearConstraintGetTotalLambda(c.handle))
}

// --- Rack and Pinion Constraint (T-0127) ---

// CreateRackAndPinionConstraint couples the rotation of a pinion (body 1) to
// the translation of a rack (body 2). Must be paired with a HingeConstraint on
// the pinion and a SliderConstraint on the rack; register them via
// RackAndPinionSetConstraints so Jolt can correct numerical drift.
//
// hingeAxis: world-space rotation axis of the pinion (normalized).
// sliderAxis: world-space sliding axis of the rack (normalized).
// ratio: defines PinionRotation(t) = ratio * RackTranslation(t).
// Compute as 2*PI * numTeethRack / (rackLength * numTeethPinion), or use
// RackAndPinionRatio.
func (ps *PhysicsSystem) CreateRackAndPinionConstraint(
	pinionBodyID, rackBodyID *BodyID,
	hingeAxis, sliderAxis Vec3,
	ratio float32,
) *Constraint {
	handle := C.JoltCreateRackAndPinionConstraint(
		ps.handle,
		pinionBodyID.handle, rackBodyID.handle,
		C.float(hingeAxis.X), C.float(hingeAxis.Y), C.float(hingeAxis.Z),
		C.float(sliderAxis.X), C.float(sliderAxis.Y), C.float(sliderAxis.Z),
		C.float(ratio),
	)
	if handle == nil {
		return nil
	}
	return &Constraint{handle: handle}
}

// RackAndPinionSetConstraints associates the pinion hinge and rack slider so
// Jolt can fix numerical position drift. Pass nil to clear a side.
func (c *Constraint) RackAndPinionSetConstraints(pinionHinge, rackSlider *Constraint) {
	var p, r C.JoltConstraint
	if pinionHinge != nil {
		p = pinionHinge.handle
	}
	if rackSlider != nil {
		r = rackSlider.handle
	}
	C.JoltRackAndPinionConstraintSetConstraints(c.handle, p, r)
}

// RackAndPinionGetTotalLambda returns the impulse applied by the constraint last step.
func (c *Constraint) RackAndPinionGetTotalLambda() float32 {
	return float32(C.JoltRackAndPinionConstraintGetTotalLambda(c.handle))
}

// --- Buoyancy ---

// ApplyBuoyancyImpulse applies buoyancy and drag forces to a body using surface plane detection.
// surfacePosition: a point on the water surface plane
// surfaceNormal: normal of the water surface (should point up, typically {0,1,0})
// buoyancy: buoyancy factor (1 = neutral, >1 = floats, <1 = sinks)
// linearDrag: linear drag coefficient (~0.5)
// angularDrag: angular drag coefficient (~0.01)
// fluidVelocity: water current velocity
// gravity: gravity vector (typically {0,-9.81,0})
// deltaTime: simulation timestep
// Returns true if impulse was applied (body is in fluid).
func (ps *PhysicsSystem) ApplyBuoyancyImpulse(
	bodyID *BodyID,
	surfacePosition, surfaceNormal Vec3,
	buoyancy, linearDrag, angularDrag float32,
	fluidVelocity, gravity Vec3,
	deltaTime float32,
) bool {
	result := C.JoltApplyBuoyancyImpulse(
		ps.handle, bodyID.handle,
		C.float(surfacePosition.X), C.float(surfacePosition.Y), C.float(surfacePosition.Z),
		C.float(surfaceNormal.X), C.float(surfaceNormal.Y), C.float(surfaceNormal.Z),
		C.float(buoyancy), C.float(linearDrag), C.float(angularDrag),
		C.float(fluidVelocity.X), C.float(fluidVelocity.Y), C.float(fluidVelocity.Z),
		C.float(gravity.X), C.float(gravity.Y), C.float(gravity.Z),
		C.float(deltaTime),
	)
	return result != 0
}

// ApplyBuoyancyImpulseWithVolume applies buoyancy using pre-computed submerged volume data.
// Use GetSubmergedVolume first to compute totalVolume, submergedVolume, and relativeCenterOfBuoyancy,
// then pass them here for multi-point buoyancy probes.
func (ps *PhysicsSystem) ApplyBuoyancyImpulseWithVolume(
	bodyID *BodyID,
	totalVolume, submergedVolume float32,
	relativeCenterOfBuoyancy Vec3,
	buoyancy, linearDrag, angularDrag float32,
	fluidVelocity, gravity Vec3,
	deltaTime float32,
) bool {
	result := C.JoltApplyBuoyancyImpulseWithVolume(
		ps.handle, bodyID.handle,
		C.float(totalVolume), C.float(submergedVolume),
		C.float(relativeCenterOfBuoyancy.X), C.float(relativeCenterOfBuoyancy.Y), C.float(relativeCenterOfBuoyancy.Z),
		C.float(buoyancy), C.float(linearDrag), C.float(angularDrag),
		C.float(fluidVelocity.X), C.float(fluidVelocity.Y), C.float(fluidVelocity.Z),
		C.float(gravity.X), C.float(gravity.Y), C.float(gravity.Z),
		C.float(deltaTime),
	)
	return result != 0
}

// SubmergedVolumeResult holds the output of GetSubmergedVolume
type SubmergedVolumeResult struct {
	TotalVolume              float32
	SubmergedVolume          float32
	RelativeCenterOfBuoyancy Vec3
}

// GetSubmergedVolume computes how much of a body is below a water surface plane.
// surfacePosition: a point on the water surface
// surfaceNormal: normal of the water surface (should point up)
func (ps *PhysicsSystem) GetSubmergedVolume(
	bodyID *BodyID,
	surfacePosition, surfaceNormal Vec3,
) SubmergedVolumeResult {
	var totalVol, submergedVol C.float
	var relX, relY, relZ C.float
	C.JoltGetSubmergedVolume(
		ps.handle, bodyID.handle,
		C.float(surfacePosition.X), C.float(surfacePosition.Y), C.float(surfacePosition.Z),
		C.float(surfaceNormal.X), C.float(surfaceNormal.Y), C.float(surfaceNormal.Z),
		&totalVol, &submergedVol,
		&relX, &relY, &relZ,
	)
	return SubmergedVolumeResult{
		TotalVolume:              float32(totalVol),
		SubmergedVolume:          float32(submergedVol),
		RelativeCenterOfBuoyancy: Vec3{X: float32(relX), Y: float32(relY), Z: float32(relZ)},
	}
}
