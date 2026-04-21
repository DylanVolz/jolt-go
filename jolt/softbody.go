package jolt

// #include <stdlib.h>
// #include "wrapper/softbody.h"
import "C"

import "unsafe"

// BendType selects the type of bend constraint generated when building
// cloth-like soft bodies from a face list.
type BendType int

const (
	BendTypeNone     BendType = C.JoltBendTypeNone     // No bend constraints
	BendTypeDistance BendType = C.JoltBendTypeDistance // Simple distance spring
	BendTypeDihedral BendType = C.JoltBendTypeDihedral // Dihedral bend (more accurate)
)

// SoftBodySharedSettings describes the particle layout and constraints that
// define a soft body. The underlying object is ref-counted on the C++ side:
// the same settings may be used to spawn multiple soft bodies, and the
// shared settings survive until every soft body referencing them is
// destroyed AND Destroy() has been called once on the Go-side handle.
type SoftBodySharedSettings struct {
	handle C.JoltSoftBodySharedSettings
}

// NewSoftBodySharedSettings creates an empty shared settings object.
// Append vertices, faces and constraints to it, then call
// CreateConstraints / Optimize before passing it to NewSoftBody.
func NewSoftBodySharedSettings() *SoftBodySharedSettings {
	h := C.JoltCreateSoftBodySharedSettings()
	return &SoftBodySharedSettings{handle: h}
}

// Destroy releases the caller's reference. The underlying object is freed
// once no live soft body still references it.
func (s *SoftBodySharedSettings) Destroy() {
	if s.handle != nil {
		C.JoltDestroySoftBodySharedSettings(s.handle)
		s.handle = nil
	}
}

// AddVertex appends a particle at the given position. invMass = 0 pins the
// vertex (kinematic); invMass > 0 makes it dynamic (typically 1.0).
// Returns the new vertex index.
func (s *SoftBodySharedSettings) AddVertex(position Vec3, invMass float32) int {
	return int(C.JoltSoftBodySharedSettingsAddVertex(
		s.handle,
		C.float(position.X), C.float(position.Y), C.float(position.Z),
		C.float(invMass),
	))
}

// AddFace appends a triangular face referencing three existing vertex
// indices. Returns an error-free bool: false if the indices are degenerate.
func (s *SoftBodySharedSettings) AddFace(v0, v1, v2 uint32) bool {
	return C.JoltSoftBodySharedSettingsAddFace(
		s.handle, C.uint(v0), C.uint(v1), C.uint(v2),
	) == 0
}

// AddEdge appends an explicit edge (spring) constraint. compliance is the
// inverse stiffness — 0 for a perfectly rigid spring.
func (s *SoftBodySharedSettings) AddEdge(v0, v1 uint32, compliance float32) {
	C.JoltSoftBodySharedSettingsAddEdge(s.handle, C.uint(v0), C.uint(v1), C.float(compliance))
}

// CreateConstraints auto-builds edge, shear and bend constraints from the
// face list using the provided per-vertex attributes. Must be invoked
// before Optimize() unless you've added all constraints manually.
func (s *SoftBodySharedSettings) CreateConstraints(compliance, shearCompliance, bendCompliance float32, bendType BendType) {
	C.JoltSoftBodySharedSettingsCreateConstraints(
		s.handle,
		C.float(compliance),
		C.float(shearCompliance),
		C.float(bendCompliance),
		C.JoltBendType(bendType),
	)
}

// Optimize reorders the constraint arrays for parallel solver execution.
// Call this once, after all vertices, faces and constraints have been
// populated, and before handing the settings to NewSoftBody.
func (s *SoftBodySharedSettings) Optimize() {
	C.JoltSoftBodySharedSettingsOptimize(s.handle)
}

// NewClothGridSettings builds a flat cloth grid in the XZ plane (Y = 0,
// centered around the origin) with ready-to-simulate edge+bend constraints.
// Pass invMasses as a row-major slice of length gridSizeX*gridSizeZ (0 =
// pinned, >0 = dynamic). Pass nil for invMasses to leave every vertex
// dynamic with invMass = 1.0.
//
// The returned settings have already been optimized.
func NewClothGridSettings(gridSizeX, gridSizeZ int, spacing float32, invMasses []float32, compliance float32, bendType BendType) *SoftBodySharedSettings {
	var (
		invMassPtr *C.float
		invMassLen C.int
	)
	if len(invMasses) > 0 {
		invMassPtr = (*C.float)(unsafe.Pointer(&invMasses[0]))
		invMassLen = C.int(len(invMasses))
	}
	h := C.JoltCreateClothGridSettings(
		C.int(gridSizeX), C.int(gridSizeZ),
		C.float(spacing),
		invMassPtr, invMassLen,
		C.float(compliance),
		C.JoltBendType(bendType),
	)
	if h == nil {
		return nil
	}
	return &SoftBodySharedSettings{handle: h}
}

// SoftBodyParams configures a soft body at creation time.
type SoftBodyParams struct {
	Position       Vec3
	Rotation       Quat    // Identity if zero-value W=0 is passed; callers should pass QuatIdentity()
	Layer          int     // Collision layer (typically MOVING)
	Pressure       float32 // Internal gas pressure; 0 for cloth
	NumIterations  int     // XPBD solver iterations (5 is Jolt's default; 0 keeps the default)
	LinearDamping  float32 // Per-step velocity damping (0.1 is Jolt's default)
	GravityFactor  float32 // Multiplier applied to world gravity (1 is default)
}

// DefaultSoftBodyParams returns a sensible starting set of parameters for a
// cloth-like body on the MOVING (layer 1) collision layer.
func DefaultSoftBodyParams() SoftBodyParams {
	return SoftBodyParams{
		Position:      Vec3{},
		Rotation:      QuatIdentity(),
		Layer:         1,
		Pressure:      0,
		NumIterations: 5,
		LinearDamping: 0.1,
		GravityFactor: 1.0,
	}
}

// NewSoftBody creates a soft body from the given shared settings and adds
// it (activated) to the physics system. The shared settings must already
// have been Optimized. Returns nil on failure.
func (bi *BodyInterface) NewSoftBody(settings *SoftBodySharedSettings, params SoftBodyParams) *BodyID {
	handle := C.JoltCreateSoftBody(
		bi.handle,
		settings.handle,
		C.float(params.Position.X), C.float(params.Position.Y), C.float(params.Position.Z),
		C.float(params.Rotation.X), C.float(params.Rotation.Y), C.float(params.Rotation.Z), C.float(params.Rotation.W),
		C.int(params.Layer),
		C.float(params.Pressure),
		C.int(params.NumIterations),
		C.float(params.LinearDamping),
		C.float(params.GravityFactor),
	)
	if handle == nil {
		return nil
	}
	return &BodyID{handle: handle}
}

// GetSoftBodyVertexCount returns the number of simulated vertices on a
// soft body, or -1 if the body is not a soft body.
func (ps *PhysicsSystem) GetSoftBodyVertexCount(bodyID *BodyID) int {
	return int(C.JoltGetSoftBodyVertexCount(ps.handle, bodyID.handle))
}

// GetSoftBodyVertexPositions copies world-space vertex positions into the
// supplied slice. The slice must be large enough to hold
// 3 * GetSoftBodyVertexCount floats. Returns the number of vertices
// written, or -1 on error.
func (ps *PhysicsSystem) GetSoftBodyVertexPositions(bodyID *BodyID, out []float32) int {
	if len(out) == 0 {
		return 0
	}
	return int(C.JoltGetSoftBodyVertexPositions(
		ps.handle, bodyID.handle,
		(*C.float)(unsafe.Pointer(&out[0])),
		C.int(len(out)),
	))
}

// GetSoftBodyVertexInvMass reads the inverse mass of a single vertex. A
// value of 0 indicates a pinned (kinematic) vertex. Returns a negative
// value on error.
func (ps *PhysicsSystem) GetSoftBodyVertexInvMass(bodyID *BodyID, index int) float32 {
	return float32(C.JoltGetSoftBodyVertexInvMass(ps.handle, bodyID.handle, C.int(index)))
}

// SetSoftBodyVertexInvMass overrides the inverse mass of a single vertex.
// Use 0 to pin a vertex, a positive value to make it dynamic again.
// Returns true on success.
func (ps *PhysicsSystem) SetSoftBodyVertexInvMass(bodyID *BodyID, index int, invMass float32) bool {
	return C.JoltSetSoftBodyVertexInvMass(ps.handle, bodyID.handle, C.int(index), C.float(invMass)) == 0
}

// AddSoftBodyForce applies a force (N) to a soft body for the current
// simulation step. Equivalent to BodyInterface::AddForce on a soft body —
// useful for driving wind fields.
func (bi *BodyInterface) AddSoftBodyForce(bodyID *BodyID, force Vec3) {
	C.JoltAddSoftBodyForce(
		bi.handle, bodyID.handle,
		C.float(force.X), C.float(force.Y), C.float(force.Z),
	)
}
