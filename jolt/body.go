package jolt

// #include "wrapper/body.h"
import "C"

// MotionType determines how a body responds to forces
type MotionType int

const (
	MotionTypeStatic    MotionType = C.JoltMotionTypeStatic    // Immovable, zero velocity
	MotionTypeKinematic MotionType = C.JoltMotionTypeKinematic // Movable by user, doesn't respond to forces
	MotionTypeDynamic   MotionType = C.JoltMotionTypeDynamic   // Affected by forces
)

// BodyInterface provides methods to create and manipulate physics bodies
type BodyInterface struct {
	handle C.JoltBodyInterface
}

// GetBodyInterface returns the interface for creating/manipulating bodies
func (ps *PhysicsSystem) GetBodyInterface() *BodyInterface {
	handle := C.JoltPhysicsSystemGetBodyInterface(ps.handle)
	return &BodyInterface{handle: handle}
}

// BodyID uniquely identifies a physics body
type BodyID struct {
	handle C.JoltBodyID
}

// Destroy frees the body ID
func (b *BodyID) Destroy() {
	C.JoltDestroyBodyID(b.handle)
}

// IndexAndSequenceNumber returns the packed (index | sequence-number) value of
// this body ID. This is the same uint32 reported through ContactListener and
// BodyActivationListener callbacks, allowing tests/handlers to match callback
// IDs back to the BodyIDs they hold.
func (b *BodyID) IndexAndSequenceNumber() uint32 {
	return uint32(C.JoltBodyIDGetIndexAndSequenceNumber(b.handle))
}

// GetPosition returns the current position of a body
func (bi *BodyInterface) GetPosition(bodyID *BodyID) Vec3 {
	var x, y, z C.float
	C.JoltGetBodyPosition(bi.handle, bodyID.handle, &x, &y, &z)
	return Vec3{
		X: float32(x),
		Y: float32(y),
		Z: float32(z),
	}
}

// CreateBody creates a body with specific motion type and sensor flag.
//
// Parameters:
//   - shape: The collision shape
//   - position: Initial position
//   - motionType: MotionTypeStatic, MotionTypeKinematic, or MotionTypeDynamic
//   - isSensor: If true, body is detected by queries but doesn't generate contact forces
//
// Examples:
//
//	// Create static ground
//	box := jolt.CreateBox(jolt.Vec3{X: 10, Y: 0.5, Z: 10})
//	ground := bi.CreateBody(box, jolt.Vec3{X: 0, Y: 0, Z: 0}, jolt.MotionTypeStatic, false)
//
//	// Create dynamic sphere
//	sphere := jolt.CreateSphere(1.0)
//	ball := bi.CreateBody(sphere, jolt.Vec3{X: 0, Y: 10, Z: 0}, jolt.MotionTypeDynamic, false)
//	bi.ActivateBody(ball)
//
//	// Create kinematic sensor
//	capsule := jolt.CreateCapsule(0.5, 1.8)
//	sensor := bi.CreateBody(capsule, jolt.Vec3{X: 0, Y: 1, Z: 0}, jolt.MotionTypeKinematic, true)
//	bi.ActivateBody(sensor)
func (bi *BodyInterface) CreateBody(shape *Shape, position Vec3, motionType MotionType, isSensor bool) *BodyID {
	sensor := C.int(0)
	if isSensor {
		sensor = C.int(1)
	}

	handle := C.JoltCreateBody(
		bi.handle,
		shape.handle,
		C.float(position.X),
		C.float(position.Y),
		C.float(position.Z),
		C.JoltMotionType(motionType),
		sensor,
	)

	return &BodyID{handle: handle}
}

// SetPosition updates the position of a body
func (bi *BodyInterface) SetPosition(bodyID *BodyID, position Vec3) {
	C.JoltSetBodyPosition(
		bi.handle,
		bodyID.handle,
		C.float(position.X),
		C.float(position.Y),
		C.float(position.Z),
	)
}

// ActivateBody makes a body participate in the simulation
func (bi *BodyInterface) ActivateBody(bodyID *BodyID) {
	C.JoltActivateBody(bi.handle, bodyID.handle)
}

// DeactivateBody removes a body from active simulation
func (bi *BodyInterface) DeactivateBody(bodyID *BodyID) {
	C.JoltDeactivateBody(bi.handle, bodyID.handle)
}

// SetShape changes the collision shape of a body
//
// Parameters:
//   - bodyID: The body to modify
//   - shape: The new collision shape
//   - updateMassProperties: If true, recalculates mass/inertia from the new shape
//
// Note: This automatically activates the body
func (bi *BodyInterface) SetShape(bodyID *BodyID, shape *Shape, updateMassProperties bool) {
	update := C.int(0)
	if updateMassProperties {
		update = C.int(1)
	}
	C.JoltSetBodyShape(bi.handle, bodyID.handle, shape.handle, update)
}

// --- Extended Body API (T-0102) ---

// GetLinearVelocity returns the current linear velocity of a body
func (bi *BodyInterface) GetLinearVelocity(bodyID *BodyID) Vec3 {
	var x, y, z C.float
	C.JoltGetLinearVelocity(bi.handle, bodyID.handle, &x, &y, &z)
	return Vec3{X: float32(x), Y: float32(y), Z: float32(z)}
}

// SetLinearVelocity sets the linear velocity of a body
func (bi *BodyInterface) SetLinearVelocity(bodyID *BodyID, velocity Vec3) {
	C.JoltSetLinearVelocity(bi.handle, bodyID.handle,
		C.float(velocity.X), C.float(velocity.Y), C.float(velocity.Z))
}

// GetAngularVelocity returns the current angular velocity of a body
func (bi *BodyInterface) GetAngularVelocity(bodyID *BodyID) Vec3 {
	var x, y, z C.float
	C.JoltGetAngularVelocity(bi.handle, bodyID.handle, &x, &y, &z)
	return Vec3{X: float32(x), Y: float32(y), Z: float32(z)}
}

// SetAngularVelocity sets the angular velocity of a body
func (bi *BodyInterface) SetAngularVelocity(bodyID *BodyID, velocity Vec3) {
	C.JoltSetAngularVelocity(bi.handle, bodyID.handle,
		C.float(velocity.X), C.float(velocity.Y), C.float(velocity.Z))
}

// AddImpulse applies an instantaneous velocity change to a body
func (bi *BodyInterface) AddImpulse(bodyID *BodyID, impulse Vec3) {
	C.JoltAddImpulse(bi.handle, bodyID.handle,
		C.float(impulse.X), C.float(impulse.Y), C.float(impulse.Z))
}

// SetFriction sets the surface friction coefficient of a body
func (bi *BodyInterface) SetFriction(bodyID *BodyID, friction float32) {
	C.JoltSetFriction(bi.handle, bodyID.handle, C.float(friction))
}

// SetRestitution sets the bounciness (elasticity) of a body
func (bi *BodyInterface) SetRestitution(bodyID *BodyID, restitution float32) {
	C.JoltSetRestitution(bi.handle, bodyID.handle, C.float(restitution))
}

// SetLinearDamping sets the linear damping on a body (requires body lock via PhysicsSystem)
func (ps *PhysicsSystem) SetLinearDamping(bodyID *BodyID, damping float32) {
	C.JoltSetLinearDamping(ps.handle, bodyID.handle, C.float(damping))
}

// SetAngularDamping sets the angular damping on a body (requires body lock via PhysicsSystem)
func (ps *PhysicsSystem) SetAngularDamping(bodyID *BodyID, damping float32) {
	C.JoltSetAngularDamping(ps.handle, bodyID.handle, C.float(damping))
}

// SetGravityFactor sets the gravity factor for a body (0 = no gravity, 1 = normal)
func (bi *BodyInterface) SetGravityFactor(bodyID *BodyID, factor float32) {
	C.JoltSetGravityFactor(bi.handle, bodyID.handle, C.float(factor))
}

// GetRotation returns the current rotation quaternion of a body
func (bi *BodyInterface) GetRotation(bodyID *BodyID) Quat {
	var x, y, z, w C.float
	C.JoltGetRotation(bi.handle, bodyID.handle, &x, &y, &z, &w)
	return Quat{X: float32(x), Y: float32(y), Z: float32(z), W: float32(w)}
}

// SetRotation sets the rotation of a body
func (bi *BodyInterface) SetRotation(bodyID *BodyID, rotation Quat) {
	C.JoltSetRotation(bi.handle, bodyID.handle,
		C.float(rotation.X), C.float(rotation.Y), C.float(rotation.Z), C.float(rotation.W))
}

// IsActive returns whether a body is currently active in the simulation
func (bi *BodyInterface) IsActive(bodyID *BodyID) bool {
	return C.JoltIsBodyActive(bi.handle, bodyID.handle) != 0
}

// RemoveAndDestroyBody removes a body from the simulation and frees it.
// Do NOT call bodyID.Destroy() after this — the body ID is already freed.
func (bi *BodyInterface) RemoveAndDestroyBody(bodyID *BodyID) {
	C.JoltRemoveAndDestroyBody(bi.handle, bodyID.handle)
}
