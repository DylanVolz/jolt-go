package jolt

import (
	"math"
	"sync"
	"testing"
)

// initOnce wraps jolt.Init() for tests so multiple test funcs in the package can
// share a single initialization. (Init/Shutdown manage process-global state.)
var initOnce sync.Once

func ensureInit(t *testing.T) {
	t.Helper()
	initOnce.Do(func() {
		if err := Init(); err != nil {
			t.Fatalf("jolt.Init failed: %v", err)
		}
	})
}

// stepUntil runs PhysicsSystem.Update at 60Hz until the predicate returns true,
// or maxSteps is reached. Returns the number of steps actually run.
func stepUntil(ps *PhysicsSystem, maxSteps int, pred func() bool) int {
	const dt = float32(1.0 / 60.0)
	for i := 0; i < maxSteps; i++ {
		ps.Update(dt)
		if pred() {
			return i + 1
		}
	}
	return maxSteps
}

// TestContactListener_TwoBodyCollision drops a dynamic sphere onto a static box
// and asserts the callback-based ContactListener observes a contact whose
// position, normal, and pre-solver relative velocity match the expected impact.
func TestContactListener_TwoBodyCollision(t *testing.T) {
	ensureInit(t)

	ps := NewPhysicsSystem()
	defer ps.Destroy()

	bi := ps.GetBodyInterface()

	// Static ground at Y=0, half-extents 5x0.5x5.
	groundShape := CreateBox(Vec3{X: 5, Y: 0.5, Z: 5})
	defer groundShape.Destroy()
	ground := bi.CreateBody(groundShape, Vec3{X: 0, Y: 0, Z: 0}, MotionTypeStatic, false)
	defer ground.Destroy()

	// Dynamic sphere starting just above the ground so it impacts within a few steps.
	sphereShape := CreateSphere(0.5)
	defer sphereShape.Destroy()
	startY := float32(2.0)
	sphere := bi.CreateBody(sphereShape, Vec3{X: 0, Y: startY, Z: 0}, MotionTypeDynamic, false)
	defer sphere.Destroy()
	bi.ActivateBody(sphere)

	type observation struct {
		manifold ContactManifold
	}
	var (
		mu              sync.Mutex
		validateCount   int
		addedCount      int
		persistedCount  int
		removedCount    int
		firstAdded      *observation
		validatedPair01 bool
	)

	cb := ContactCallbacks{
		OnContactValidate: func(b1, b2 uint32, baseOffset, contactPoint, normal Vec3, depth float32) ValidateResult {
			mu.Lock()
			defer mu.Unlock()
			validateCount++
			if (b1 == sphere.IndexAndSequenceNumber() && b2 == ground.IndexAndSequenceNumber()) ||
				(b2 == sphere.IndexAndSequenceNumber() && b1 == ground.IndexAndSequenceNumber()) {
				validatedPair01 = true
			}
			return AcceptAllContactsForThisBodyPair
		},
		OnContactAdded: func(m *ContactManifold, _ *ContactSettings) {
			mu.Lock()
			defer mu.Unlock()
			addedCount++
			if firstAdded == nil {
				firstAdded = &observation{manifold: *m}
			}
		},
		OnContactPersisted: func(m *ContactManifold, _ *ContactSettings) {
			mu.Lock()
			defer mu.Unlock()
			persistedCount++
		},
		OnContactRemoved: func(b1, b2 uint32) {
			mu.Lock()
			defer mu.Unlock()
			removedCount++
		},
	}

	listener := ps.NewContactListener(cb)
	if listener == nil {
		t.Fatal("NewContactListener returned nil")
	}
	defer listener.Destroy()

	steps := stepUntil(ps, 240, func() bool {
		mu.Lock()
		defer mu.Unlock()
		return firstAdded != nil
	})

	mu.Lock()
	defer mu.Unlock()

	if firstAdded == nil {
		t.Fatalf("OnContactAdded never fired after %d steps (validate=%d added=%d persisted=%d removed=%d)",
			steps, validateCount, addedCount, persistedCount, removedCount)
	}
	if !validatedPair01 {
		t.Errorf("OnContactValidate never saw the sphere/ground pair (validate=%d)", validateCount)
	}

	m := firstAdded.manifold

	// Body IDs must reference the two bodies we created.
	groundID := ground.IndexAndSequenceNumber()
	sphereID := sphere.IndexAndSequenceNumber()
	pair := [2]uint32{m.Body1ID, m.Body2ID}
	if !((pair[0] == groundID && pair[1] == sphereID) || (pair[0] == sphereID && pair[1] == groundID)) {
		t.Errorf("contact body IDs %v do not match {ground=%d, sphere=%d}", pair, groundID, sphereID)
	}

	// Contact point should be at the top of the ground / bottom of the sphere — Y ≈ 0.5.
	expectedY := float32(0.5)
	if math.Abs(float64(m.ContactPointOn1.Y-expectedY)) > 0.1 {
		t.Errorf("contact point Y = %.3f, expected ~%.3f (manifold=%+v)", m.ContactPointOn1.Y, expectedY, m)
	}

	// Normal must be vertical. Jolt may report it either as +Y or -Y depending on
	// which body it ordered as body1 — both are physically the same contact.
	if math.Abs(float64(m.Normal.X)) > 0.05 || math.Abs(float64(m.Normal.Z)) > 0.05 {
		t.Errorf("contact normal has horizontal component: %+v", m.Normal)
	}
	if absf(m.Normal.Y) < 0.95 {
		t.Errorf("contact normal Y magnitude %.3f, expected ~1.0 (normal=%+v)", absf(m.Normal.Y), m.Normal)
	}

	// Sphere is dynamic and falling under gravity from Y=2 to Y≈0.5 (drop ≈ 1.5m).
	// Pre-solver relative velocity (body2 - body1) should be vertical and non-trivial.
	if math.Abs(float64(m.RelativeLinearVelocity.X)) > 0.5 || math.Abs(float64(m.RelativeLinearVelocity.Z)) > 0.5 {
		t.Errorf("relative linear velocity has horizontal component: %+v", m.RelativeLinearVelocity)
	}
	if absf(m.RelativeLinearVelocity.Y) < 1.0 {
		t.Errorf("relative linear velocity Y magnitude %.3f, expected > 1.0 m/s for a free-fall impact (rv=%+v)",
			absf(m.RelativeLinearVelocity.Y), m.RelativeLinearVelocity)
	}

	if m.PenetrationDepth < 0 {
		t.Errorf("expected non-negative penetration depth, got %.4f", m.PenetrationDepth)
	}
	if m.NumContactPoints <= 0 {
		t.Errorf("expected at least one contact point, got %d", m.NumContactPoints)
	}
}

// TestContactListener_DestroyDoesNotLeak verifies Destroy is idempotent and a
// listener can be installed and removed without crashing the wrapper.
func TestContactListener_DestroyDoesNotLeak(t *testing.T) {
	ensureInit(t)

	ps := NewPhysicsSystem()
	defer ps.Destroy()

	for i := 0; i < 3; i++ {
		listener := ps.NewContactListener(ContactCallbacks{})
		if listener == nil {
			t.Fatal("NewContactListener returned nil")
		}
		listener.Destroy()
		listener.Destroy() // idempotent
	}
}

// TestBodyActivationListener_FiresOnActivate creates a dynamic sphere, activates
// it, and asserts OnBodyActivated fires for it.
func TestBodyActivationListener_FiresOnActivate(t *testing.T) {
	ensureInit(t)

	ps := NewPhysicsSystem()
	defer ps.Destroy()

	bi := ps.GetBodyInterface()

	sphereShape := CreateSphere(0.5)
	defer sphereShape.Destroy()
	sphere := bi.CreateBody(sphereShape, Vec3{X: 0, Y: 5, Z: 0}, MotionTypeDynamic, false)
	defer sphere.Destroy()

	var (
		mu             sync.Mutex
		activatedIDs   []uint32
		deactivatedIDs []uint32
	)

	cb := BodyActivationCallbacks{
		OnBodyActivated: func(bodyID uint32, _ uint64) {
			mu.Lock()
			defer mu.Unlock()
			activatedIDs = append(activatedIDs, bodyID)
		},
		OnBodyDeactivated: func(bodyID uint32, _ uint64) {
			mu.Lock()
			defer mu.Unlock()
			deactivatedIDs = append(deactivatedIDs, bodyID)
		},
	}

	listener := ps.NewBodyActivationListener(cb)
	if listener == nil {
		t.Fatal("NewBodyActivationListener returned nil")
	}
	defer listener.Destroy()

	bi.ActivateBody(sphere)

	// One step is enough for the activation event to fire (events are queued and
	// flushed during PhysicsSystem.Update).
	ps.Update(1.0 / 60.0)

	mu.Lock()
	got := append([]uint32(nil), activatedIDs...)
	mu.Unlock()

	want := sphere.IndexAndSequenceNumber()
	found := false
	for _, id := range got {
		if id == want {
			found = true
			break
		}
	}
	if !found {
		t.Errorf("expected OnBodyActivated for body %d; got %v", want, got)
	}
}

func absf(x float32) float32 {
	if x < 0 {
		return -x
	}
	return x
}
