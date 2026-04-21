package jolt

import (
	"math"
	"testing"
)

// Run Jolt's global init once for every test in this package. Shape-only
// tests don't require it but tolerate it; soft-body and physics-system
// tests need the allocator, factory, and job pool.

func TestSoftBodySharedSettingsBasics(t *testing.T) {
	s := NewSoftBodySharedSettings()
	defer s.Destroy()

	// Three-vertex triangle
	a := s.AddVertex(Vec3{X: 0, Y: 0, Z: 0}, 1.0)
	b := s.AddVertex(Vec3{X: 1, Y: 0, Z: 0}, 1.0)
	c := s.AddVertex(Vec3{X: 0, Y: 0, Z: 1}, 1.0)

	if a != 0 || b != 1 || c != 2 {
		t.Fatalf("unexpected vertex indices %d %d %d", a, b, c)
	}

	if !s.AddFace(uint32(a), uint32(b), uint32(c)) {
		t.Fatal("AddFace should succeed for non-degenerate indices")
	}

	if s.AddFace(uint32(a), uint32(a), uint32(c)) {
		t.Fatal("AddFace should reject degenerate indices")
	}

	s.CreateConstraints(0, 0, 0, BendTypeNone)
	s.Optimize()
}

func TestClothGridSettings(t *testing.T) {
	const n = 5
	s := NewClothGridSettings(n, n, 0.5, nil, 1e-5, BendTypeNone)
	if s == nil {
		t.Fatal("NewClothGridSettings returned nil")
	}
	defer s.Destroy()

	// Invalid grid should return nil.
	if bad := NewClothGridSettings(1, 1, 0.5, nil, 0, BendTypeNone); bad != nil {
		bad.Destroy()
		t.Fatal("grid with <2 rows/cols should fail")
	}
}

// Cloth-flag acceptance test: 20x20 cloth with two pinned corners.
// Verifies that the cloth deforms under external force, settles when the
// force is removed, and no vertex ever tunnels through the static ground.
func TestSoftBodyClothFlag(t *testing.T) {
	// Match the per-test Init/Shutdown pattern used by other tests that
	// step the physics sim (path_constraint_test, pulley_constraint_test,
	// ragdoll_test). Earlier tests in the suite call defer Shutdown()
	// which tears down the shared Factory / TempAllocator / JobSystem;
	// without a fresh Init here any Update() call would deref freed
	// globals. Init is idempotent-by-pattern when paired with Shutdown.
	if err := Init(); err != nil {
		t.Fatal(err)
	}
	defer Shutdown()

	const (
		gridSize    = 20
		spacing     = float32(0.25)
		flagHeight  = float32(6.0)
		groundY     = float32(-6.0)
		windSteps   = 120
		settleSteps = 240
	)

	ps := NewPhysicsSystem()
	defer ps.Destroy()

	bi := ps.GetBodyInterface()

	// Static ground plane well below the cloth — any tunneling shows up
	// as vertices dropping past its top face.
	groundShape := CreateBox(Vec3{X: 30, Y: 0.25, Z: 30})
	defer groundShape.Destroy()
	ground := bi.CreateBody(groundShape, Vec3{X: 0, Y: groundY, Z: 0}, MotionTypeStatic, false)
	defer bi.RemoveAndDestroyBody(ground)

	// Pin the two corners along the first row (x varying, z=0) so the
	// cloth hangs like a flag attached to a horizontal pole.
	invMasses := make([]float32, gridSize*gridSize)
	for i := range invMasses {
		invMasses[i] = 1.0
	}
	invMasses[0] = 0          // (x=0, z=0)
	invMasses[gridSize-1] = 0 // (x=gridSize-1, z=0)

	settings := NewClothGridSettings(gridSize, gridSize, spacing, invMasses, 1e-4, BendTypeDistance)
	if settings == nil {
		t.Fatal("NewClothGridSettings returned nil")
	}
	defer settings.Destroy()

	params := DefaultSoftBodyParams()
	params.Position = Vec3{X: 0, Y: flagHeight, Z: 0}

	bodyID := bi.NewSoftBody(settings, params)
	if bodyID == nil {
		t.Fatal("NewSoftBody returned nil")
	}
	defer bi.RemoveAndDestroyBody(bodyID)

	vertexCount := ps.GetSoftBodyVertexCount(bodyID)
	if vertexCount != gridSize*gridSize {
		t.Fatalf("vertex count = %d, want %d", vertexCount, gridSize*gridSize)
	}
	if m := ps.GetSoftBodyVertexInvMass(bodyID, 0); m != 0 {
		t.Errorf("pinned corner invMass = %v, want 0", m)
	}

	positions := make([]float32, vertexCount*3)
	dt := float32(1.0 / 60.0)

	// Capture initial positions.
	if n := ps.GetSoftBodyVertexPositions(bodyID, positions); n != vertexCount {
		t.Fatalf("initial GetSoftBodyVertexPositions wrote %d, want %d", n, vertexCount)
	}
	initial := make([]float32, vertexCount*3)
	copy(initial, positions)

	// Phase 1: gravity + steady horizontal wind along +Z. Mass scales
	// with vertex count (~1 kg each), so the force must scale too. We
	// want ~5 m/s² of wind-driven acceleration.
	windAccel := float32(5.0)
	totalMass := float32(vertexCount) // roughly, since invMass=1 per free vertex
	wind := Vec3{X: 0, Y: 0, Z: windAccel * totalMass}
	minY := float32(math.Inf(1))

	for range windSteps {
		bi.AddSoftBodyForce(bodyID, wind)
		ps.Update(dt)

		if n := ps.GetSoftBodyVertexPositions(bodyID, positions); n != vertexCount {
			t.Fatalf("GetSoftBodyVertexPositions mid-wind wrote %d", n)
		}
		for i := range vertexCount {
			if y := positions[i*3+1]; y < minY {
				minY = y
			}
		}
	}

	// Cloth should have deformed — total displacement across all free
	// vertices should be well above zero.
	var totalDisp float32
	for i := range vertexCount {
		if invMasses[i] == 0 {
			continue
		}
		dx := positions[i*3+0] - initial[i*3+0]
		dy := positions[i*3+1] - initial[i*3+1]
		dz := positions[i*3+2] - initial[i*3+2]
		totalDisp += float32(math.Sqrt(float64(dx*dx + dy*dy + dz*dz)))
	}
	if totalDisp < 1.0 {
		t.Errorf("cloth did not deform under wind+gravity: totalDisp = %.3f", totalDisp)
	}

	// Pinned vertex 0 must still be near its initial world position.
	pinDX := positions[0] - initial[0]
	pinDY := positions[1] - initial[1]
	pinDZ := positions[2] - initial[2]
	pinDrift := float32(math.Sqrt(float64(pinDX*pinDX + pinDY*pinDY + pinDZ*pinDZ)))
	if pinDrift > 0.01 {
		t.Errorf("pinned vertex drifted: %.4f", pinDrift)
	}

	// No tunneling: cloth vertices must stay above the ground's top face.
	groundTop := groundY + 0.25
	if minY < groundTop-0.5 {
		t.Errorf("vertex tunneled through ground: minY = %.3f, groundTop = %.3f", minY, groundTop)
	}

	// Phase 2: cut the wind and let the cloth settle. Measure how much
	// each vertex moves per frame early vs. late — settling should shrink
	// the drift.
	prev := make([]float32, vertexCount*3)
	copy(prev, positions)
	var driftEarly, driftLate float32
	const sampleWindow = 10

	for step := range settleSteps {
		ps.Update(dt)
		if n := ps.GetSoftBodyVertexPositions(bodyID, positions); n != vertexCount {
			t.Fatalf("GetSoftBodyVertexPositions mid-settle wrote %d", n)
		}

		var drift float32
		for i := range len(positions) {
			d := positions[i] - prev[i]
			drift += d * d
		}
		drift = float32(math.Sqrt(float64(drift)))

		if step < sampleWindow {
			driftEarly += drift
		}
		if step >= settleSteps-sampleWindow {
			driftLate += drift
		}
		copy(prev, positions)

		for i := range vertexCount {
			if y := positions[i*3+1]; y < minY {
				minY = y
			}
		}
	}

	// Drift must measurably shrink by the end of the settle phase.
	if driftLate >= 0.5*driftEarly {
		t.Errorf("cloth failed to settle: early drift %.4f, late drift %.4f", driftEarly, driftLate)
	}

	// Still no tunneling by the end of the settle phase.
	if minY < groundTop-0.5 {
		t.Errorf("vertex tunneled through ground during settle: minY = %.3f", minY)
	}
}

// Verifies that toggling invMass at runtime re-pins / un-pins a vertex.
func TestSoftBodyVertexInvMassToggle(t *testing.T) {
	ps := NewPhysicsSystem()
	defer ps.Destroy()

	bi := ps.GetBodyInterface()

	settings := NewClothGridSettings(3, 3, 0.5, nil, 1e-4, BendTypeNone)
	if settings == nil {
		t.Fatal("NewClothGridSettings returned nil")
	}
	defer settings.Destroy()

	params := DefaultSoftBodyParams()
	params.Position = Vec3{X: 0, Y: 5, Z: 0}
	bodyID := bi.NewSoftBody(settings, params)
	if bodyID == nil {
		t.Fatal("NewSoftBody returned nil")
	}
	defer bi.RemoveAndDestroyBody(bodyID)

	// All vertices start dynamic.
	if m := ps.GetSoftBodyVertexInvMass(bodyID, 4); m <= 0 {
		t.Fatalf("expected center vertex dynamic, got invMass %v", m)
	}

	// Pin the center vertex.
	if !ps.SetSoftBodyVertexInvMass(bodyID, 4, 0) {
		t.Fatal("SetSoftBodyVertexInvMass returned false")
	}
	if m := ps.GetSoftBodyVertexInvMass(bodyID, 4); m != 0 {
		t.Fatalf("expected center vertex pinned, got invMass %v", m)
	}

	// Un-pin it.
	if !ps.SetSoftBodyVertexInvMass(bodyID, 4, 1.0) {
		t.Fatal("SetSoftBodyVertexInvMass returned false")
	}
	if m := ps.GetSoftBodyVertexInvMass(bodyID, 4); m <= 0 {
		t.Fatalf("expected center vertex dynamic again, got invMass %v", m)
	}

	// Invalid index should fail.
	if ps.SetSoftBodyVertexInvMass(bodyID, 9999, 0) {
		t.Error("SetSoftBodyVertexInvMass should fail on out-of-range index")
	}
	if ps.GetSoftBodyVertexInvMass(bodyID, 9999) >= 0 {
		t.Error("GetSoftBodyVertexInvMass on out-of-range index should return <0")
	}
}
