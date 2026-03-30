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
