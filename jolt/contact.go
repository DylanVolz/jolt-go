package jolt

// #include "wrapper/contact.h"
import "C"

import (
	"runtime/cgo"
)

// ValidateResult is the verdict returned from OnContactValidate, mirroring
// JPH::ValidateResult. Strongest accept has the lowest value.
type ValidateResult int

const (
	AcceptAllContactsForThisBodyPair ValidateResult = C.JoltAcceptAllContactsForThisBodyPair
	AcceptContact                    ValidateResult = C.JoltAcceptContact
	RejectContact                    ValidateResult = C.JoltRejectContact
	RejectAllContactsForThisBodyPair ValidateResult = C.JoltRejectAllContactsForThisBodyPair
)

// ContactManifold is the read-only collision data passed to OnContactAdded and
// OnContactPersisted. Contact points are presented in world space.
type ContactManifold struct {
	Body1ID                uint32
	Body2ID                uint32
	BaseOffset             Vec3
	Normal                 Vec3 // direction along which to push body 2 out of body 1
	PenetrationDepth       float32
	NumContactPoints       int  // total points in the manifold
	ContactPointOn1        Vec3 // first point on shape 1, world space
	ContactPointOn2        Vec3 // first point on shape 2, world space
	RelativeLinearVelocity Vec3 // body2 linear velocity minus body1 linear velocity
}

// ContactSettings is the mutable contact-constraint state. Callbacks may write to
// override the simulation's combined friction/restitution, mass scaling, sensor
// flag, or surface velocities (e.g., a conveyor-belt effect).
type ContactSettings struct {
	CombinedFriction               float32
	CombinedRestitution            float32
	InvMassScale1                  float32
	InvInertiaScale1               float32
	InvMassScale2                  float32
	InvInertiaScale2               float32
	IsSensor                       bool
	RelativeLinearSurfaceVelocity  Vec3
	RelativeAngularSurfaceVelocity Vec3
}

// ContactCallbacks bundles the optional contact event handlers. Any field may be nil.
//
// Callbacks fire from Jolt worker threads (potentially many in parallel) during
// PhysicsSystem.Update. Handlers MUST be threadsafe and MUST NOT call back into
// the physics system. OnContactRemoved cannot read body state.
type ContactCallbacks struct {
	// OnContactValidate is called before a contact constraint is created. Return
	// AcceptAllContactsForThisBodyPair to keep the contact, or a Reject* value to
	// drop it. baseOffset, contactPoint, and normal are world space.
	OnContactValidate func(body1, body2 uint32, baseOffset, contactPoint, normal Vec3, penetrationDepth float32) ValidateResult
	// OnContactAdded fires the first frame two bodies start touching. Velocities
	// in the manifold are pre-solver — useful for impact-strength estimation.
	OnContactAdded func(manifold *ContactManifold, settings *ContactSettings)
	// OnContactPersisted fires every frame two bodies remain in contact.
	OnContactPersisted func(manifold *ContactManifold, settings *ContactSettings)
	// OnContactRemoved fires when a previous contact is no longer detected. Body
	// state is NOT readable here — cache anything you need in Added/Persisted.
	OnContactRemoved func(body1, body2 uint32)
}

// ContactListener is an installed callback-based contact listener. Call Destroy()
// to uninstall it from the system and release its cgo handle.
type ContactListener struct {
	handle cgo.Handle
	system *PhysicsSystem
}

// NewContactListener installs cb as the contact listener on this physics system,
// replacing any prior contact listener (queue-based or callback-based).
// Returns nil on failure.
func (ps *PhysicsSystem) NewContactListener(cb ContactCallbacks) *ContactListener {
	h := cgo.NewHandle(&cb)
	rc := C.JoltInstallContactCallbacks(ps.handle, C.uintptr_t(h))
	if rc != 0 {
		h.Delete()
		return nil
	}
	return &ContactListener{handle: h, system: ps}
}

// Destroy uninstalls the listener and frees its cgo handle.
func (cl *ContactListener) Destroy() {
	if cl == nil || cl.system == nil {
		return
	}
	C.JoltUninstallContactCallbacks(cl.system.handle)
	cl.handle.Delete()
	cl.system = nil
}

// BodyActivationCallbacks bundles the optional body activation handlers.
// Both fire from Jolt worker threads — handlers must be threadsafe.
type BodyActivationCallbacks struct {
	OnBodyActivated   func(bodyID uint32, bodyUserData uint64)
	OnBodyDeactivated func(bodyID uint32, bodyUserData uint64)
}

// BodyActivationListener is an installed callback-based activation listener.
type BodyActivationListener struct {
	handle cgo.Handle
	system *PhysicsSystem
}

// NewBodyActivationListener installs cb on this physics system, replacing any
// prior activation listener. Returns nil on failure.
func (ps *PhysicsSystem) NewBodyActivationListener(cb BodyActivationCallbacks) *BodyActivationListener {
	h := cgo.NewHandle(&cb)
	rc := C.JoltInstallBodyActivationCallbacks(ps.handle, C.uintptr_t(h))
	if rc != 0 {
		h.Delete()
		return nil
	}
	return &BodyActivationListener{handle: h, system: ps}
}

// Destroy uninstalls the listener and frees its cgo handle.
func (bl *BodyActivationListener) Destroy() {
	if bl == nil || bl.system == nil {
		return
	}
	C.JoltUninstallBodyActivationCallbacks(bl.system.handle)
	bl.handle.Delete()
	bl.system = nil
}

// --- cgo trampolines (called from C++ ContactCallbackListenerImpl / BodyActivationListenerImpl) ---

func contactCallbacks(ud C.uintptr_t) *ContactCallbacks {
	return cgo.Handle(ud).Value().(*ContactCallbacks)
}

func bodyActivationCallbacks(ud C.uintptr_t) *BodyActivationCallbacks {
	return cgo.Handle(ud).Value().(*BodyActivationCallbacks)
}

//export goJoltOnContactValidate
func goJoltOnContactValidate(
	body1, body2 C.uint,
	baseOffsetX, baseOffsetY, baseOffsetZ C.float,
	contactPointX, contactPointY, contactPointZ C.float,
	normalX, normalY, normalZ C.float,
	penetrationDepth C.float,
	ud C.uintptr_t,
) C.int {
	cb := contactCallbacks(ud)
	if cb.OnContactValidate == nil {
		return C.int(AcceptAllContactsForThisBodyPair)
	}
	baseOffset := Vec3{X: float32(baseOffsetX), Y: float32(baseOffsetY), Z: float32(baseOffsetZ)}
	worldContactPoint := Vec3{
		X: float32(contactPointX) + baseOffset.X,
		Y: float32(contactPointY) + baseOffset.Y,
		Z: float32(contactPointZ) + baseOffset.Z,
	}
	normal := Vec3{X: float32(normalX), Y: float32(normalY), Z: float32(normalZ)}
	return C.int(cb.OnContactValidate(uint32(body1), uint32(body2),
		baseOffset, worldContactPoint, normal, float32(penetrationDepth)))
}

//export goJoltOnContactAdded
func goJoltOnContactAdded(m *C.JoltContactManifold, s *C.JoltContactSettings, ud C.uintptr_t) {
	cb := contactCallbacks(ud)
	if cb.OnContactAdded == nil {
		return
	}
	gm := convertManifold(m)
	gs := convertSettings(s)
	cb.OnContactAdded(&gm, &gs)
	writeBackSettings(s, &gs)
}

//export goJoltOnContactPersisted
func goJoltOnContactPersisted(m *C.JoltContactManifold, s *C.JoltContactSettings, ud C.uintptr_t) {
	cb := contactCallbacks(ud)
	if cb.OnContactPersisted == nil {
		return
	}
	gm := convertManifold(m)
	gs := convertSettings(s)
	cb.OnContactPersisted(&gm, &gs)
	writeBackSettings(s, &gs)
}

//export goJoltOnContactRemoved
func goJoltOnContactRemoved(body1, body2 C.uint, ud C.uintptr_t) {
	cb := contactCallbacks(ud)
	if cb.OnContactRemoved == nil {
		return
	}
	cb.OnContactRemoved(uint32(body1), uint32(body2))
}

//export goJoltOnBodyActivated
func goJoltOnBodyActivated(bodyID C.uint, bodyUserData C.ulonglong, ud C.uintptr_t) {
	cb := bodyActivationCallbacks(ud)
	if cb.OnBodyActivated == nil {
		return
	}
	cb.OnBodyActivated(uint32(bodyID), uint64(bodyUserData))
}

//export goJoltOnBodyDeactivated
func goJoltOnBodyDeactivated(bodyID C.uint, bodyUserData C.ulonglong, ud C.uintptr_t) {
	cb := bodyActivationCallbacks(ud)
	if cb.OnBodyDeactivated == nil {
		return
	}
	cb.OnBodyDeactivated(uint32(bodyID), uint64(bodyUserData))
}

func convertManifold(m *C.JoltContactManifold) ContactManifold {
	base := Vec3{X: float32(m.baseOffsetX), Y: float32(m.baseOffsetY), Z: float32(m.baseOffsetZ)}
	return ContactManifold{
		Body1ID:          uint32(m.body1ID),
		Body2ID:          uint32(m.body2ID),
		BaseOffset:       base,
		Normal:           Vec3{X: float32(m.normalX), Y: float32(m.normalY), Z: float32(m.normalZ)},
		PenetrationDepth: float32(m.penetrationDepth),
		NumContactPoints: int(m.numContactPoints),
		ContactPointOn1: Vec3{
			X: float32(m.contactPoint1X) + base.X,
			Y: float32(m.contactPoint1Y) + base.Y,
			Z: float32(m.contactPoint1Z) + base.Z,
		},
		ContactPointOn2: Vec3{
			X: float32(m.contactPoint2X) + base.X,
			Y: float32(m.contactPoint2Y) + base.Y,
			Z: float32(m.contactPoint2Z) + base.Z,
		},
		RelativeLinearVelocity: Vec3{
			X: float32(m.relativeLinearVelocityX),
			Y: float32(m.relativeLinearVelocityY),
			Z: float32(m.relativeLinearVelocityZ),
		},
	}
}

func convertSettings(s *C.JoltContactSettings) ContactSettings {
	return ContactSettings{
		CombinedFriction:    float32(s.combinedFriction),
		CombinedRestitution: float32(s.combinedRestitution),
		InvMassScale1:       float32(s.invMassScale1),
		InvInertiaScale1:    float32(s.invInertiaScale1),
		InvMassScale2:       float32(s.invMassScale2),
		InvInertiaScale2:    float32(s.invInertiaScale2),
		IsSensor:            s.isSensor != 0,
		RelativeLinearSurfaceVelocity: Vec3{
			X: float32(s.relativeLinearSurfaceVelocityX),
			Y: float32(s.relativeLinearSurfaceVelocityY),
			Z: float32(s.relativeLinearSurfaceVelocityZ),
		},
		RelativeAngularSurfaceVelocity: Vec3{
			X: float32(s.relativeAngularSurfaceVelocityX),
			Y: float32(s.relativeAngularSurfaceVelocityY),
			Z: float32(s.relativeAngularSurfaceVelocityZ),
		},
	}
}

func writeBackSettings(c *C.JoltContactSettings, g *ContactSettings) {
	c.combinedFriction = C.float(g.CombinedFriction)
	c.combinedRestitution = C.float(g.CombinedRestitution)
	c.invMassScale1 = C.float(g.InvMassScale1)
	c.invInertiaScale1 = C.float(g.InvInertiaScale1)
	c.invMassScale2 = C.float(g.InvMassScale2)
	c.invInertiaScale2 = C.float(g.InvInertiaScale2)
	if g.IsSensor {
		c.isSensor = 1
	} else {
		c.isSensor = 0
	}
	c.relativeLinearSurfaceVelocityX = C.float(g.RelativeLinearSurfaceVelocity.X)
	c.relativeLinearSurfaceVelocityY = C.float(g.RelativeLinearSurfaceVelocity.Y)
	c.relativeLinearSurfaceVelocityZ = C.float(g.RelativeLinearSurfaceVelocity.Z)
	c.relativeAngularSurfaceVelocityX = C.float(g.RelativeAngularSurfaceVelocity.X)
	c.relativeAngularSurfaceVelocityY = C.float(g.RelativeAngularSurfaceVelocity.Y)
	c.relativeAngularSurfaceVelocityZ = C.float(g.RelativeAngularSurfaceVelocity.Z)
}
