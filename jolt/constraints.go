package jolt

// #include "wrapper/constraints.h"
import "C"

// MotorState controls how a constraint motor behaves
type MotorState int

const (
	MotorStateOff      MotorState = C.JoltMotorStateOff      // Motor is off
	MotorStateVelocity MotorState = C.JoltMotorStateVelocity // Motor drives to target velocity
	MotorStatePosition MotorState = C.JoltMotorStatePosition  // Motor drives to target position
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
