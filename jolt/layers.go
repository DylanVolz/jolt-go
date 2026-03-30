package jolt

// #include "wrapper/layers.h"
import "C"

// CollisionMatrix configures which collision layers interact with each other
type CollisionMatrix struct {
	handle C.JoltCollisionMatrix
}

// NewCollisionMatrix creates a collision matrix for the specified number of layers.
// All collisions are disabled by default.
func NewCollisionMatrix(numLayers int) *CollisionMatrix {
	handle := C.JoltCreateCollisionMatrix(C.int(numLayers))
	return &CollisionMatrix{handle: handle}
}

// EnableCollision enables collision between two layers (symmetric)
func (cm *CollisionMatrix) EnableCollision(layer1, layer2 int) {
	C.JoltCollisionMatrixEnableCollision(cm.handle, C.int(layer1), C.int(layer2))
}

// DisableCollision disables collision between two layers (symmetric)
func (cm *CollisionMatrix) DisableCollision(layer1, layer2 int) {
	C.JoltCollisionMatrixDisableCollision(cm.handle, C.int(layer1), C.int(layer2))
}

// Destroy frees the collision matrix
func (cm *CollisionMatrix) Destroy() {
	C.JoltDestroyCollisionMatrix(cm.handle)
}

// NewPhysicsSystemWithLayers creates a physics system with configurable collision layers.
// The collision matrix data is copied — the CollisionMatrix can be destroyed after this call.
func NewPhysicsSystemWithLayers(cm *CollisionMatrix) *PhysicsSystem {
	handle := C.JoltCreatePhysicsSystemWithLayers(cm.handle)
	if handle == nil {
		return nil
	}
	return &PhysicsSystem{handle: handle}
}

// ContactEventType identifies the type of contact event
type ContactEventType int

const (
	ContactAdded     ContactEventType = 0
	ContactPersisted ContactEventType = 1
	ContactRemoved   ContactEventType = 2
)

// ContactEvent represents a physics contact event between two bodies
type ContactEvent struct {
	Type         ContactEventType
	BodyID1      uint32
	BodyID2      uint32
	ContactPoint Vec3
	Normal       Vec3
}

// EnableContactEvents enables contact event collection on this physics system
func (ps *PhysicsSystem) EnableContactEvents() {
	C.JoltEnableContactEvents(ps.handle)
}

// DrainContactEvents returns and removes all queued contact events (up to maxEvents)
func (ps *PhysicsSystem) DrainContactEvents(maxEvents int) []ContactEvent {
	if maxEvents <= 0 {
		return nil
	}

	cEvents := make([]C.JoltContactEvent, maxEvents)
	n := C.JoltDrainContactEvents(ps.handle, &cEvents[0], C.int(maxEvents))

	events := make([]ContactEvent, int(n))
	for i := 0; i < int(n); i++ {
		ce := cEvents[i]
		events[i] = ContactEvent{
			Type:    ContactEventType(ce._type),
			BodyID1: uint32(ce.bodyID1),
			BodyID2: uint32(ce.bodyID2),
			ContactPoint: Vec3{
				X: float32(ce.contactPointX),
				Y: float32(ce.contactPointY),
				Z: float32(ce.contactPointZ),
			},
			Normal: Vec3{
				X: float32(ce.normalX),
				Y: float32(ce.normalY),
				Z: float32(ce.normalZ),
			},
		}
	}

	return events
}

// GetBodyLayer returns the collision layer of a body
func (ps *PhysicsSystem) GetBodyLayer(bodyID *BodyID) int {
	return int(C.JoltGetBodyLayer(ps.handle, bodyID.handle))
}

// CastRayFiltered casts a ray filtered by a layer bitmask
// layerMask: bitmask of layers to include (use LayerMask helper)
func (ps *PhysicsSystem) CastRayFiltered(origin, direction Vec3, layerMask uint32) (RaycastHit, bool) {
	var cHit C.JoltRaycastHit

	result := C.JoltCastRayFiltered(
		ps.handle,
		C.float(origin.X), C.float(origin.Y), C.float(origin.Z),
		C.float(direction.X), C.float(direction.Y), C.float(direction.Z),
		C.uint(layerMask),
		&cHit,
	)

	if result == 0 {
		return RaycastHit{}, false
	}

	hit := RaycastHit{
		BodyID: &BodyID{handle: cHit.bodyID},
		HitPoint: Vec3{
			X: float32(cHit.hitPointX),
			Y: float32(cHit.hitPointY),
			Z: float32(cHit.hitPointZ),
		},
		Normal: Vec3{
			X: float32(cHit.normalX),
			Y: float32(cHit.normalY),
			Z: float32(cHit.normalZ),
		},
		Fraction: float32(cHit.fraction),
	}

	return hit, true
}

// CreateBodyOnLayer creates a body on a specific collision layer
func (bi *BodyInterface) CreateBodyOnLayer(shape *Shape, position Vec3, motionType MotionType, isSensor bool, layer int) *BodyID {
	sensor := C.int(0)
	if isSensor {
		sensor = C.int(1)
	}

	handle := C.JoltCreateBodyOnLayer(
		bi.handle,
		shape.handle,
		C.float(position.X),
		C.float(position.Y),
		C.float(position.Z),
		C.JoltMotionType(motionType),
		sensor,
		C.int(layer),
	)

	return &BodyID{handle: handle}
}

// LayerMask creates a bitmask from layer indices
func LayerMask(layers ...int) uint32 {
	var mask uint32
	for _, l := range layers {
		mask |= 1 << uint(l)
	}
	return mask
}
